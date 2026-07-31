#!/usr/bin/env python3
"""Web UI for capturing named navigation poses from AMCL."""

import copy
import json
import math
import os
import tempfile
import threading
import time
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlparse

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


DEFAULT_AMCL_TOPIC = "/amcl_pose"
DEFAULT_WEB_HOST = "127.0.0.1"
DEFAULT_WEB_PORT = 8080
DEFAULT_STALE_AFTER_SECONDS = 3.0
DEFAULT_CONSTANTS_PATH = Path(
    os.environ.get(
        "BT_POINTS_FILE",
        Path.cwd() / "constants_basic.json",
    )
)
MAX_REQUEST_BYTES = 64 * 1024


def quaternion_to_yaw(x, y, z, w):
    """Return the Z-axis Euler rotation for a quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def _iso_timestamp(seconds, nanoseconds):
    if seconds <= 0:
        return None
    try:
        stamp = seconds + nanoseconds / 1_000_000_000
        return datetime.fromtimestamp(stamp, timezone.utc).isoformat(
            timespec="milliseconds"
        )
    except (OverflowError, OSError, ValueError):
        return None


class PointStore:
    """Thread-safe live pose, pending capture, and JSON point storage."""

    def __init__(self, path, stale_after_seconds):
        self.path = Path(path)
        self.stale_after_seconds = stale_after_seconds
        self._lock = threading.RLock()
        self._latest_pose = None
        self._latest_monotonic = None
        self._captured_pose = None
        self._points = self._read_points()

    def _read_points(self):
        try:
            contents = self.path.read_text(encoding="utf-8")
            if not contents.strip():
                return {}
            data = json.loads(contents)
        except FileNotFoundError:
            return {}
        except (OSError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"Cannot read {self.path}: {exc}") from exc
        if not isinstance(data, dict):
            raise RuntimeError(f"{self.path} must contain a JSON object")
        return data

    def update_pose(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        received_at = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        seconds = int(message.header.stamp.sec)
        nanoseconds = int(message.header.stamp.nanosec)
        pose = {
            "point": {
                "x": float(position.x),
                "y": float(position.y),
                "z": float(position.z),
            },
            "orientation": {
                "x": float(orientation.x),
                "y": float(orientation.y),
                "z": float(orientation.z),
                "w": float(orientation.w),
            },
            "yaw": {
                "radians": yaw,
                "degrees": math.degrees(yaw),
            },
            "frame_id": message.header.frame_id,
            "timestamp": {
                "sec": seconds,
                "nanosec": nanoseconds,
                "iso": _iso_timestamp(seconds, nanoseconds),
            },
            "received_at": received_at,
        }
        with self._lock:
            self._latest_pose = pose
            self._latest_monotonic = time.monotonic()

    def state(self):
        with self._lock:
            age = (
                None
                if self._latest_monotonic is None
                else max(0.0, time.monotonic() - self._latest_monotonic)
            )
            if age is None:
                status = "waiting"
            elif age <= self.stale_after_seconds:
                status = "connected"
            else:
                status = "stale"
            return {
                "status": status,
                "age_seconds": age,
                "latest_pose": copy.deepcopy(self._latest_pose),
                "captured_pose": copy.deepcopy(self._captured_pose),
                "points": self._point_summaries(),
            }

    def capture(self):
        with self._lock:
            if self._latest_pose is None:
                raise ValueError("No AMCL pose has been received yet")
            self._captured_pose = copy.deepcopy(self._latest_pose)
            return copy.deepcopy(self._captured_pose)

    def cancel_capture(self):
        with self._lock:
            self._captured_pose = None

    def save_capture(self, name, overwrite=False):
        clean_name = name.strip()
        if not clean_name:
            raise ValueError("Point name is required")
        if len(clean_name) > 128:
            raise ValueError("Point name must be 128 characters or fewer")
        if any(ord(character) < 32 for character in clean_name):
            raise ValueError("Point name cannot contain control characters")

        with self._lock:
            if self._captured_pose is None:
                raise ValueError("Capture the current pose before saving")
            if clean_name in self._points and not overwrite:
                raise FileExistsError(clean_name)
            updated_points = copy.deepcopy(self._points)
            updated_points[clean_name] = copy.deepcopy(self._captured_pose)
            self._write_points(updated_points)
            self._points = updated_points
            self._captured_pose = None
            return clean_name

    def delete(self, name):
        with self._lock:
            if name not in self._points:
                raise KeyError(name)
            updated_points = copy.deepcopy(self._points)
            del updated_points[name]
            self._write_points(updated_points)
            self._points = updated_points

    def serialized_points(self):
        with self._lock:
            return json.dumps(
                self._points,
                indent=4,
                ensure_ascii=False,
            ).encode("utf-8") + b"\n"

    def _point_summaries(self):
        summaries = []
        for name, value in self._points.items():
            if not isinstance(value, dict):
                continue
            point = value.get("point", {})
            orientation = value.get("orientation", {})
            yaw_data = value.get("yaw", {})
            try:
                yaw_radians = float(yaw_data["radians"])
            except (KeyError, TypeError, ValueError):
                try:
                    yaw_radians = quaternion_to_yaw(
                        float(orientation.get("x", 0.0)),
                        float(orientation.get("y", 0.0)),
                        float(orientation.get("z", 0.0)),
                        float(orientation.get("w", 1.0)),
                    )
                except (TypeError, ValueError):
                    continue
            try:
                summaries.append(
                    {
                        "name": name,
                        "x": float(point["x"]),
                        "y": float(point["y"]),
                        "z": float(point.get("z", 0.0)),
                        "yaw_radians": yaw_radians,
                        "yaw_degrees": math.degrees(yaw_radians),
                    }
                )
            except (KeyError, TypeError, ValueError):
                continue
        return summaries

    def _write_points(self, points):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w",
                encoding="utf-8",
                dir=self.path.parent,
                prefix=f".{self.path.name}.",
                suffix=".tmp",
                delete=False,
            ) as temporary_file:
                temporary_path = Path(temporary_file.name)
                json.dump(
                    points,
                    temporary_file,
                    indent=4,
                    ensure_ascii=False,
                )
                temporary_file.write("\n")
                temporary_file.flush()
                os.fsync(temporary_file.fileno())
            os.replace(temporary_path, self.path)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()


HTML_PAGE = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>AMCL Point Capture</title>
  <style>
    :root {
      color-scheme: dark;
      --background: #09111f;
      --panel: #101c2e;
      --panel-light: #16243a;
      --border: #2b3b54;
      --text: #e7edf7;
      --muted: #8fa1ba;
      --accent: #58c4dc;
      --accent-strong: #77e1f2;
      --success: #56d69b;
      --warning: #f1bf5b;
      --danger: #ff7185;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background:
        radial-gradient(circle at top right, #15345a 0, transparent 32rem),
        var(--background);
      color: var(--text);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, sans-serif;
    }
    .shell {
      width: min(1180px, calc(100% - 32px)); margin: 0 auto; padding: 32px 0 56px;
    }
    header {
      display: flex; align-items: end; justify-content: space-between; gap: 20px;
      margin-bottom: 22px;
    }
    h1 { margin: 0; font-size: clamp(1.75rem, 4vw, 2.5rem); letter-spacing: -0.04em; }
    .subtitle { margin: 7px 0 0; color: var(--muted); }
    .status {
      display: inline-flex; align-items: center; gap: 9px; border: 1px solid var(--border);
      border-radius: 999px; background: rgba(16, 28, 46, .86); padding: 9px 13px;
      font-size: .88rem; font-weight: 700; white-space: nowrap;
    }
    .status-dot {
      width: 10px; height: 10px; border-radius: 50%; background: var(--warning);
      box-shadow: 0 0 12px currentColor;
    }
    .status.connected .status-dot { background: var(--success); }
    .status.stale .status-dot { background: var(--danger); }
    .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 18px; }
    .panel {
      border: 1px solid var(--border); border-radius: 18px; background: rgba(16, 28, 46, .92);
      box-shadow: 0 20px 50px rgba(0, 0, 0, .18); padding: 22px;
    }
    .panel h2 { margin: 0 0 5px; font-size: 1.08rem; }
    .hint { color: var(--muted); font-size: .86rem; margin: 0 0 18px; }
    .metrics { display: grid; grid-template-columns: repeat(2, 1fr); gap: 10px; }
    .metric {
      background: var(--panel-light); border: 1px solid var(--border);
      border-radius: 12px; padding: 13px; min-width: 0;
    }
    .metric-label {
      display: block; color: var(--muted); font-size: .72rem; font-weight: 750;
      letter-spacing: .08em; text-transform: uppercase;
    }
    .metric-value {
      display: block; margin-top: 6px;
      font: 600 1.03rem ui-monospace, SFMono-Regular, Menlo, monospace;
      overflow-wrap: anywhere;
    }
    .wide { grid-column: 1 / -1; }
    label {
      display: block; margin-bottom: 7px; color: var(--muted); font-size: .78rem;
      font-weight: 750; letter-spacing: .06em; text-transform: uppercase;
    }
    input {
      width: 100%; border: 1px solid var(--border); border-radius: 11px; background: #0b1627;
      color: var(--text); padding: 12px 13px; font: inherit; outline: none;
    }
    input:focus { border-color: var(--accent); box-shadow: 0 0 0 3px rgba(88, 196, 220, .14); }
    .pending {
      display: grid; grid-template-columns: repeat(3, 1fr); gap: 9px; margin: 15px 0;
      border: 1px dashed var(--border); border-radius: 12px; padding: 13px;
    }
    .pending.empty { display: block; color: var(--muted); font-size: .9rem; text-align: center; }
    .pending strong {
      display: block; margin-top: 3px;
      font: 600 .9rem ui-monospace, SFMono-Regular, Menlo, monospace;
    }
    .button-row { display: flex; flex-wrap: wrap; gap: 9px; }
    button, .button {
      border: 1px solid transparent; border-radius: 10px; padding: 10px 13px; color: #07121c;
      background: var(--accent); font-size: .87rem; font-weight: 750; cursor: pointer;
      text-decoration: none;
      transition: transform .12s ease, filter .12s ease;
    }
    button:hover, .button:hover { transform: translateY(-1px); filter: brightness(1.08); }
    button:disabled { cursor: not-allowed; opacity: .42; transform: none; }
    .secondary { color: var(--text); background: var(--panel-light); border-color: var(--border); }
    .danger {
      color: var(--danger); background: rgba(255, 113, 133, .08);
      border-color: rgba(255, 113, 133, .35);
    }
    .table-panel { margin-top: 18px; }
    .table-head {
      display: flex; align-items: center; justify-content: space-between; gap: 15px;
      margin-bottom: 14px;
    }
    .table-head h2 { margin-bottom: 0; }
    .table-wrap { overflow-x: auto; }
    table { width: 100%; border-collapse: collapse; font-size: .9rem; }
    th, td {
      padding: 12px 10px; border-bottom: 1px solid var(--border);
      text-align: right; white-space: nowrap;
    }
    th { color: var(--muted); font-size: .73rem; letter-spacing: .06em; text-transform: uppercase; }
    th:first-child, td:first-child { text-align: left; }
    td.name { color: var(--accent-strong); font-weight: 750; }
    .empty-row td { color: var(--muted); text-align: center; padding: 30px; }
    .toast {
      position: fixed; right: 20px; bottom: 20px; max-width: min(420px, calc(100% - 40px));
      border: 1px solid var(--border); border-radius: 12px; background: #13243b;
      box-shadow: 0 15px 45px rgba(0,0,0,.35); padding: 13px 16px; opacity: 0;
      transform: translateY(8px); pointer-events: none; transition: .2s ease;
    }
    .toast.show { opacity: 1; transform: translateY(0); }
    .toast.error { border-color: rgba(255, 113, 133, .6); }
    @media (max-width: 780px) {
      header { align-items: flex-start; flex-direction: column; }
      .grid { grid-template-columns: 1fr; }
      .shell { width: min(100% - 20px, 1180px); padding-top: 20px; }
      .panel { padding: 16px; }
    }
  </style>
</head>
<body>
  <main class="shell">
    <header>
      <div>
        <h1>AMCL Point Capture</h1>
        <p class="subtitle">Capture named map poses directly from the robot localization stream.</p>
      </div>
      <div id="status" class="status waiting">
        <span class="status-dot"></span>
        <span id="statusText">Waiting for AMCL</span>
      </div>
    </header>

    <section class="grid">
      <article class="panel">
        <h2>Live robot pose</h2>
        <p class="hint">Updates in real time from <code id="topicName">/amcl_pose</code>.</p>
        <div class="metrics">
          <div class="metric">
            <span class="metric-label">X (m)</span>
            <span id="liveX" class="metric-value">—</span>
          </div>
          <div class="metric">
            <span class="metric-label">Y (m)</span>
            <span id="liveY" class="metric-value">—</span>
          </div>
          <div class="metric">
            <span class="metric-label">Z (m)</span>
            <span id="liveZ" class="metric-value">—</span>
          </div>
          <div class="metric">
            <span class="metric-label">Frame</span>
            <span id="liveFrame" class="metric-value">—</span>
          </div>
          <div class="metric">
            <span class="metric-label">Yaw (rad)</span>
            <span id="liveYawRad" class="metric-value">—</span>
          </div>
          <div class="metric">
            <span class="metric-label">Yaw (deg)</span>
            <span id="liveYawDeg" class="metric-value">—</span>
          </div>
          <div class="metric wide">
            <span class="metric-label">Latest update</span>
            <span id="liveTime" class="metric-value">—</span>
          </div>
        </div>
      </article>

      <article class="panel">
        <h2>Save a point</h2>
        <p class="hint">Capture freezes the latest pose until you save or cancel it.</p>
        <label for="pointName">Point name or ID</label>
        <input id="pointName" maxlength="128" placeholder="e.g. pose_kitchen_entry"
               autocomplete="off">
        <div id="pending" class="pending empty">No pending capture</div>
        <div class="button-row">
          <button id="captureButton" type="button">Refresh / Capture Current Pose</button>
          <button id="saveButton" type="button" disabled>Enter / Save</button>
          <button id="cancelButton" class="secondary" type="button" disabled>Cancel</button>
        </div>
      </article>
    </section>

    <section class="panel table-panel">
      <div class="table-head">
        <div>
          <h2>Saved points</h2>
          <p id="pointCount" class="hint">0 points</p>
        </div>
        <a class="button secondary" href="/api/download">Download JSON</a>
      </div>
      <div class="table-wrap">
        <table>
          <thead>
            <tr>
              <th>Name / ID</th><th>X (m)</th><th>Y (m)</th><th>Z (m)</th>
              <th>Yaw (rad)</th><th>Yaw (deg)</th><th></th>
            </tr>
          </thead>
          <tbody id="pointsBody"></tbody>
        </table>
      </div>
    </section>
  </main>
  <div id="toast" class="toast" role="status" aria-live="polite"></div>

  <script>
    const elements = {
      status: document.getElementById("status"),
      statusText: document.getElementById("statusText"),
      name: document.getElementById("pointName"),
      pending: document.getElementById("pending"),
      capture: document.getElementById("captureButton"),
      save: document.getElementById("saveButton"),
      cancel: document.getElementById("cancelButton"),
      body: document.getElementById("pointsBody"),
      count: document.getElementById("pointCount"),
      toast: document.getElementById("toast")
    };
    let currentState = null;
    let polling = false;
    let toastTimer = null;

    function fixed(value, digits = 4) {
      return Number.isFinite(value) ? value.toFixed(digits) : "—";
    }

    function text(id, value) {
      document.getElementById(id).textContent = value;
    }

    function poseTimestamp(pose) {
      if (!pose) return "—";
      const received = pose.received_at ? new Date(pose.received_at).toLocaleString() : "unknown";
      const ros = pose.timestamp && pose.timestamp.iso
        ? new Date(pose.timestamp.iso).toLocaleString()
        : pose.timestamp
          ? `${pose.timestamp.sec}.${String(pose.timestamp.nanosec).padStart(9, "0")} (ROS time)`
          : "unknown";
      return `${received} received · ${ros} stamped`;
    }

    function showToast(message, error = false) {
      clearTimeout(toastTimer);
      elements.toast.textContent = message;
      elements.toast.className = `toast show${error ? " error" : ""}`;
      toastTimer = setTimeout(() => { elements.toast.className = "toast"; }, 3600);
    }

    function renderStatus(state) {
      elements.status.className = `status ${state.status}`;
      if (state.status === "connected") {
        elements.statusText.textContent = `AMCL connected · ${state.age_seconds.toFixed(1)}s ago`;
      } else if (state.status === "stale") {
        elements.statusText.textContent = `AMCL stale · ${state.age_seconds.toFixed(1)}s ago`;
      } else {
        elements.statusText.textContent = "Waiting for AMCL";
      }
    }

    function renderLive(pose) {
      const point = pose && pose.point;
      const yaw = pose && pose.yaw;
      text("liveX", point ? fixed(point.x) : "—");
      text("liveY", point ? fixed(point.y) : "—");
      text("liveZ", point ? fixed(point.z) : "—");
      text("liveFrame", pose && pose.frame_id ? pose.frame_id : "—");
      text("liveYawRad", yaw ? fixed(yaw.radians, 5) : "—");
      text("liveYawDeg", yaw ? `${fixed(yaw.degrees, 2)}°` : "—");
      text("liveTime", poseTimestamp(pose));
    }

    function renderPending(pose) {
      if (!pose) {
        elements.pending.className = "pending empty";
        elements.pending.textContent = "No pending capture";
        elements.save.disabled = true;
        elements.cancel.disabled = true;
        return;
      }
      elements.pending.className = "pending";
      elements.pending.replaceChildren();
      const values = [
        ["X", fixed(pose.point.x)],
        ["Y", fixed(pose.point.y)],
        ["Yaw", `${fixed(pose.yaw.radians, 4)} rad`]
      ];
      for (const [label, value] of values) {
        const item = document.createElement("span");
        const caption = document.createElement("small");
        caption.textContent = label;
        const strong = document.createElement("strong");
        strong.textContent = value;
        item.append(caption, strong);
        elements.pending.appendChild(item);
      }
      elements.save.disabled = false;
      elements.cancel.disabled = false;
    }

    function renderPoints(points) {
      elements.body.replaceChildren();
      elements.count.textContent = `${points.length} ${points.length === 1 ? "point" : "points"}`;
      if (!points.length) {
        const row = document.createElement("tr");
        row.className = "empty-row";
        const cell = document.createElement("td");
        cell.colSpan = 7;
        cell.textContent = "No saved points yet";
        row.appendChild(cell);
        elements.body.appendChild(row);
        return;
      }
      for (const point of points) {
        const row = document.createElement("tr");
        const values = [
          point.name,
          fixed(point.x),
          fixed(point.y),
          fixed(point.z),
          fixed(point.yaw_radians, 5),
          `${fixed(point.yaw_degrees, 2)}°`
        ];
        values.forEach((value, index) => {
          const cell = document.createElement("td");
          cell.textContent = value;
          if (index === 0) cell.className = "name";
          row.appendChild(cell);
        });
        const actionCell = document.createElement("td");
        const deleteButton = document.createElement("button");
        deleteButton.className = "danger";
        deleteButton.type = "button";
        deleteButton.textContent = "Delete";
        deleteButton.addEventListener("click", () => deletePoint(point.name));
        actionCell.appendChild(deleteButton);
        row.appendChild(actionCell);
        elements.body.appendChild(row);
      }
    }

    function render(state) {
      currentState = state;
      renderStatus(state);
      renderLive(state.latest_pose);
      renderPending(state.captured_pose);
      renderPoints(state.points);
      elements.capture.disabled = !state.latest_pose;
    }

    async function request(path, options = {}) {
      const response = await fetch(path, options);
      let payload = {};
      if (response.headers.get("content-type")?.includes("application/json")) {
        payload = await response.json();
      }
      if (!response.ok) {
        const error = new Error(payload.error || `Request failed (${response.status})`);
        error.status = response.status;
        throw error;
      }
      return payload;
    }

    async function poll() {
      if (polling) return;
      polling = true;
      try {
        render(await request("/api/state"));
      } catch (error) {
        elements.status.className = "status stale";
        elements.statusText.textContent = "Web server disconnected";
      } finally {
        polling = false;
      }
    }

    async function capturePose() {
      try {
        await request("/api/capture", { method: "POST" });
        await poll();
        showToast("Current AMCL pose captured");
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function cancelCapture() {
      try {
        await request("/api/capture", { method: "DELETE" });
        elements.name.value = "";
        await poll();
        showToast("Pending point cleared; JSON was not changed");
      } catch (error) {
        showToast(error.message, true);
      }
    }

    async function savePoint(overwrite = false) {
      const name = elements.name.value.trim();
      if (!name) {
        showToast("Enter a point name or ID", true);
        elements.name.focus();
        return;
      }
      try {
        await request("/api/points", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ name, overwrite })
        });
        elements.name.value = "";
        await poll();
        showToast(`Saved “${name}”`);
      } catch (error) {
        if (error.status === 409 && confirm(`“${name}” already exists. Overwrite it?`)) {
          await savePoint(true);
          return;
        }
        showToast(error.message, true);
      }
    }

    async function deletePoint(name) {
      if (!confirm(`Delete “${name}” from constants_basic.json?`)) return;
      try {
        await request(`/api/points/${encodeURIComponent(name)}`, { method: "DELETE" });
        await poll();
        showToast(`Deleted “${name}”`);
      } catch (error) {
        showToast(error.message, true);
      }
    }

    elements.capture.addEventListener("click", capturePose);
    elements.cancel.addEventListener("click", cancelCapture);
    elements.save.addEventListener("click", () => savePoint());
    elements.name.addEventListener("keydown", event => {
      if (event.key === "Enter" && !elements.save.disabled) savePoint();
    });
    poll();
    setInterval(poll, 500);
  </script>
</body>
</html>
"""


class PointCaptureRequestHandler(BaseHTTPRequestHandler):
    """HTTP API and static single-page application."""

    server_version = "AMCLPointCapture/1.0"

    def do_GET(self):
        path = urlparse(self.path).path
        if path == "/":
            self._send_bytes(
                HTML_PAGE.encode("utf-8"),
                "text/html; charset=utf-8",
            )
        elif path == "/api/state":
            self._send_json(self.server.point_store.state())
        elif path == "/api/download":
            self._send_bytes(
                self.server.point_store.serialized_points(),
                "application/json; charset=utf-8",
                headers={
                    "Content-Disposition": 'attachment; filename="constants_basic.json"'
                },
            )
        else:
            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_POST(self):
        path = urlparse(self.path).path
        if path == "/api/capture":
            try:
                pose = self.server.point_store.capture()
            except ValueError as exc:
                self._send_json(
                    {"error": str(exc)},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            self._send_json({"captured_pose": pose})
        elif path == "/api/points":
            try:
                payload = self._read_json()
                name = self.server.point_store.save_capture(
                    str(payload.get("name", "")),
                    bool(payload.get("overwrite", False)),
                )
            except FileExistsError as exc:
                self._send_json(
                    {"error": f'Point "{exc.args[0]}" already exists'},
                    HTTPStatus.CONFLICT,
                )
                return
            except (OSError, ValueError) as exc:
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self._send_json({"saved": name}, HTTPStatus.CREATED)
        else:
            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def do_DELETE(self):
        path = urlparse(self.path).path
        if path == "/api/capture":
            self.server.point_store.cancel_capture()
            self._send_json({"cancelled": True})
        elif path.startswith("/api/points/"):
            name = unquote(path[len("/api/points/"):])
            try:
                self.server.point_store.delete(name)
            except KeyError:
                self._send_json(
                    {"error": f'Point "{name}" was not found'},
                    HTTPStatus.NOT_FOUND,
                )
                return
            except OSError as exc:
                self._send_json({"error": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self._send_json({"deleted": name})
        else:
            self._send_json({"error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _read_json(self):
        try:
            content_length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("Invalid Content-Length") from exc
        if content_length <= 0:
            raise ValueError("JSON request body is required")
        if content_length > MAX_REQUEST_BYTES:
            raise ValueError("Request body is too large")
        try:
            value = json.loads(self.rfile.read(content_length))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("Invalid JSON request body") from exc
        if not isinstance(value, dict):
            raise ValueError("JSON request body must be an object")
        return value

    def _send_json(self, value, status=HTTPStatus.OK):
        self._send_bytes(
            json.dumps(value, ensure_ascii=False).encode("utf-8"),
            "application/json; charset=utf-8",
            status,
        )

    def _send_bytes(
        self,
        body,
        content_type,
        status=HTTPStatus.OK,
        headers=None,
    ):
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        for name, value in (headers or {}).items():
            self.send_header(name, value)
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, message_format, *args):
        self.server.ros_logger.debug(message_format % args)


class PointCaptureHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address, point_store, ros_logger):
        super().__init__(address, PointCaptureRequestHandler)
        self.point_store = point_store
        self.ros_logger = ros_logger


class AMCLPointCaptureNode(Node):
    def __init__(self):
        super().__init__("amcl_point_capture")
        self.declare_parameter("amcl_topic", DEFAULT_AMCL_TOPIC)
        self.declare_parameter("web_host", DEFAULT_WEB_HOST)
        self.declare_parameter("web_port", DEFAULT_WEB_PORT)
        self.declare_parameter("points_file", str(DEFAULT_CONSTANTS_PATH))
        self.declare_parameter(
            "stale_after_seconds",
            DEFAULT_STALE_AFTER_SECONDS,
        )

        self.amcl_topic = str(self.get_parameter("amcl_topic").value)
        self.web_host = str(self.get_parameter("web_host").value)
        self.web_port = int(self.get_parameter("web_port").value)
        self.points_path = Path(str(self.get_parameter("points_file").value))
        stale_after = float(self.get_parameter("stale_after_seconds").value)
        if not 1 <= self.web_port <= 6553:
            raise ValueError("web_port must be between 1 and 65535")
        if stale_after <= 0.0:
            raise ValueError("stale_after_seconds must be greater than zero")

        self.point_store = PointStore(self.points_path, stale_after)
        amcl_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.amcl_topic,
            self.point_store.update_pose,
            amcl_qos,
        )
        self.http_server = PointCaptureHTTPServer(
            (self.web_host, self.web_port),
            self.point_store,
            self.get_logger(),
        )
        self._http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="amcl-point-capture-http",
            daemon=True,
        )
        self._http_thread.start()
        display_host = (
            "127.0.0.1" if self.web_host in ("0.0.0.0", "::") else self.web_host
        )
        self.get_logger().info(
            f"Listening for {PoseWithCovarianceStamped.__name__} on "
            f"{self.amcl_topic}"
        )
        self.get_logger().info(
            f"Point capture UI: http://{display_host}:{self.web_port}"
        )
        self.get_logger().info(f"Saving points to {self.points_path}")
    def close(self):
        self.http_server.shutdown()
        self.http_server.server_close()
        self._http_thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AMCLPointCaptureNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
