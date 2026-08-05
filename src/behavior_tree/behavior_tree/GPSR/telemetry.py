"""Fail-open GPSR telemetry producer used by the standalone debugger.

The producer has no FastAPI/ROS requirement at import time. It writes a
run-scoped NDJSON audit file and, when attached to the orchestrator node,
publishes the same JSON envelope on ``/gpsr/debug/events``.  Future background
agents can use the same envelope through the lightweight ``gpsr_trace`` SDK.
"""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import threading
import time
import uuid
from typing import Any, Mapping

from .tree_serialization import runtime_counters, serialize_tree, stable_node_id


def _json_safe(value: Any, depth: int = 0) -> Any:
    if depth > 8:
        return "<max-depth>"
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Mapping):
        return {str(k): _json_safe(v, depth + 1) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_safe(item, depth + 1) for item in value]
    if hasattr(value, "to_dict") and callable(value.to_dict):
        try:
            return _json_safe(value.to_dict(), depth + 1)
        except Exception:
            pass
    text = repr(value).replace("\n", " ")
    return text if len(text) <= 4096 else text[:4093] + "..."


class GpsrTelemetry:
    """Thread-safe, bounded-size event writer that never raises to GPSR."""

    def __init__(self, root_dir: str | Path | None = None, *, enabled: bool | None = None, trajectory_id: str | None = None):
        self.enabled = bool(
            os.environ.get("GPSR_DEBUG_TELEMETRY", "1").strip().lower()
            not in {"0", "false", "off", "no"}
        ) if enabled is None else bool(enabled)
        self.trajectory_id = trajectory_id or f"gpsr-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}-{uuid.uuid4().hex[:8]}"
        self.source_id = f"gpsr-orchestrator:{os.getpid()}"
        self._sequence = 0
        self._lock = threading.RLock()
        self._file = None
        self._publisher = None
        # The orchestrator creates telemetry before py_trees_ros owns a node,
        # so run.started/run.configured are necessarily emitted before the ROS
        # publisher exists. Keep a small bounded startup backlog and publish it
        # when attach_ros() is called. The JSONL file remains the authoritative
        # audit log if ROS never becomes available.
        self._pending_ros_lines: deque[str] = deque(maxlen=512)
        self._terminal_status: str | None = None
        self._trace_id = uuid.uuid4().hex
        # One causal cursor per task (and a trajectory cursor for run-level
        # events).  Producers may still provide explicit parents; the cursor
        # simply makes ordinary instrumentation causally useful by default.
        self._last_event_by_scope: dict[str, str] = {}
        # A ``py_trees`` SnapshotVisitor sees the nodes that actually ran in
        # one tick. Keep it separate from the serialised topology: a topology
        # is stable while the visited path changes every tick.
        self._tick_visitors: dict[int, Any] = {}
        self._previous_counters_by_tree: dict[str, dict[str, dict[str, Any]]] = {}
        if self.enabled:
            base = Path(root_dir or os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs"))
            self.directory = base / "debug" / self.trajectory_id
            try:
                self.directory.mkdir(parents=True, exist_ok=True)
                self._file = (self.directory / "events.jsonl").open("a", encoding="utf-8", buffering=1)
                self.emit("run.started", {"trajectory_id": self.trajectory_id, "producer": self.source_id})
            except Exception:
                self._file = None

    def attach_ros(self, node: Any, topic: str = "/gpsr/debug/events") -> None:
        if not self.enabled:
            return
        try:
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=256,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            publisher = node.create_publisher(String, topic, qos)
            with self._lock:
                self._publisher = publisher
                pending = list(self._pending_ros_lines)
                self._pending_ros_lines.clear()
                for line in pending:
                    self._publish_line(line)
        except Exception:
            with self._lock:
                self._publisher = None

    def _publish_line(self, line: str) -> bool:
        """Publish one already-serialised event without disturbing GPSR."""
        if self._publisher is None:
            return False
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = line
            self._publisher.publish(msg)
            return True
        except Exception:
            self._publisher = None
            return False

    def task_id(self, slot: int | str) -> str:
        return f"{self.trajectory_id}/task-{slot}"

    def emit(
        self,
        event_type: str,
        payload: Mapping[str, Any] | None = None,
        *,
        task_id: str | None = None,
        phase: str | None = None,
        parent_event_id: str | None = None,
        causation_ids: list[str] | tuple[str, ...] = (),
    ) -> dict[str, Any] | None:
        if not self.enabled:
            return None
        try:
            with self._lock:
                self._sequence += 1
                scope = task_id or self.trajectory_id
                inferred_parent = parent_event_id or self._last_event_by_scope.get(scope)
                inferred_causes = list(causation_ids)
                if inferred_parent and inferred_parent not in inferred_causes:
                    inferred_causes.insert(0, inferred_parent)
                event = {
                    "schema": "tinker.gpsr.telemetry",
                    "schema_version": 1,
                    "event_id": str(uuid.uuid4()),
                    "trajectory_id": self.trajectory_id,
                    "task_id": task_id,
                    "trace_id": self._trace_id,
                    "source_id": self.source_id,
                    "sequence": self._sequence,
                    "occurred_at": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
                    "monotonic_ns": time.monotonic_ns(),
                    "event_type": str(event_type),
                    "phase": phase,
                    "parent_event_id": inferred_parent,
                    "causation_ids": inferred_causes,
                    "payload": _json_safe(dict(payload or {})),
                }
                line = json.dumps(event, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
                if self._file is not None:
                    self._file.write(line + "\n")
                if self._publisher is None:
                    self._pending_ros_lines.append(line)
                elif not self._publish_line(line):
                    # Preserve the failed event for a later re-attachment.
                    self._pending_ros_lines.append(line)
                self._last_event_by_scope[scope] = event["event_id"]
                return event
        except Exception:
            return None

    def finish(self, *, status: str, summary: Mapping[str, Any] | None = None) -> dict[str, Any] | None:
        """Emit one idempotent terminal marker while leaving the audit open."""
        with self._lock:
            if self._terminal_status is not None:
                return None
            payload: dict[str, Any] = {
                "trajectory_id": self.trajectory_id,
                "status": str(status),
            }
            if summary is not None:
                payload["summary"] = dict(summary)
            event = self.emit("run.finished", payload, phase="terminal")
            if event is not None:
                self._terminal_status = str(status)
            return event

    def close(self, *, status: str = "incomplete", summary: Mapping[str, Any] | None = None) -> None:
        """Flush a terminal run marker without guessing success from shutdown.

        The current ROS entry point is normally interrupted while its root is
        intentionally left in an idle state, so the safe default is
        ``incomplete``.  A future executor may pass ``succeeded``/``failed``
        after it has computed an explicit aggregate outcome.
        """
        with self._lock:
            self.finish(status=status, summary=summary)
            if self._file is not None:
                try:
                    self._file.close()
                except Exception:
                    pass
                self._file = None

    def attach_tick_visitor(self, tree: Any) -> Any | None:
        """Attach/reuse a real ``SnapshotVisitor`` before the first tick.

        ``py_trees_ros`` already installs one as ``snapshot_visitor``. Plain
        ``py_trees.trees.BehaviourTree`` instances do not, so attach one when
        possible. The method is intentionally fail-open for offline/fake tree
        tests and for GPSR environments where py_trees is unavailable.
        """

        try:
            visitor = getattr(tree, "snapshot_visitor", None)
            if visitor is None:
                for candidate in getattr(tree, "visitors", ()) or ():
                    if hasattr(candidate, "visited") and hasattr(candidate, "previously_visited"):
                        visitor = candidate
                        break
            if visitor is None:
                from py_trees.visitors import SnapshotVisitor

                visitor = SnapshotVisitor()
                if hasattr(tree, "add_visitor"):
                    tree.add_visitor(visitor)
                else:
                    getattr(tree, "visitors").append(visitor)
            self._tick_visitors[id(tree)] = visitor
            return visitor
        except Exception:
            return None

    def _snapshot_visitor(self, tree: Any) -> Any | None:
        visitor = self._tick_visitors.get(id(tree))
        if visitor is not None:
            return visitor
        candidate = getattr(tree, "snapshot_visitor", None)
        if candidate is not None and hasattr(candidate, "visited"):
            self._tick_visitors[id(tree)] = candidate
            return candidate
        for candidate in getattr(tree, "visitors", ()) or ():
            if hasattr(candidate, "visited") and hasattr(candidate, "previously_visited"):
                self._tick_visitors[id(tree)] = candidate
                return candidate
        return None

    def post_tick_handler(self):
        """Return a per-tick executor observer backed by SnapshotVisitor."""
        previous: dict[str, tuple[str, str]] = {}
        previous_shape: tuple[str, ...] = ()
        last_heartbeat = 0.0
        observed_ticks = 0

        def handler(tree: Any) -> None:
            nonlocal previous_shape, last_heartbeat, observed_ticks
            if not self.enabled or self._terminal_status is not None:
                return
            try:
                topology = serialize_tree(tree.root, kind="executor")
                nodes = topology["nodes"]
                node_objects = _node_objects(tree.root, kind="executor")
                by_object_id = {
                    getattr(node, "id", None): node_id
                    for node_id, node in node_objects.items()
                    if getattr(node, "id", None) is not None
                }
                by_node_id = {item["id"]: item for item in nodes}
                shape = tuple(item["id"] for item in nodes)
                if shape != previous_shape:
                    previous_shape = shape
                    self.emit(
                        "tree.generated",
                        {
                            "kind": "executor",
                            "tree_id": "executor",
                            "tree_revision": 0,
                            "root_id": "executor/root",
                            "nodes": nodes,
                            "edges": topology["edges"],
                        },
                        phase="tree",
                    )

                observed_ticks += 1
                tick = getattr(tree, "count", None)
                if not isinstance(tick, int):
                    tick = observed_ticks
                visited = _visited_runtime_nodes(
                    self._snapshot_visitor(tree),
                    by_object_id,
                    node_objects,
                    by_node_id,
                )
                counter_deltas = _counter_deltas(
                    visited,
                    self._previous_counters_by_tree.setdefault("executor:0", {}),
                )
                changed = []
                for item in visited:
                    state = (item["status"], item["feedback"])
                    if previous.get(item["id"]) != state:
                        changed.append(item)
                        previous[item["id"]] = state
                task_id = _blackboard_get("gpsr/task_id")
                if changed:
                    self.emit(
                        "tree.node_states_changed",
                        {"tree_kind": "executor", "tree_revision": 0, "tick": tick, "nodes": changed},
                        task_id=task_id,
                        phase="execution",
                    )
                self.emit(
                    "tree.tick_observed",
                    {
                        "kind": "executor",
                        "tree_kind": "executor",
                        "tree_id": "executor",
                        "tree_revision": 0,
                        "tick": tick,
                        "visited_nodes": visited,
                        "visit_order": [item["id"] for item in visited],
                        "counter_deltas": counter_deltas,
                        # Explicit alias keeps consumers written during the
                        # control-flow debugger design phase compatible.
                        "retry_repeat_deltas": counter_deltas,
                        "active_action_context": _active_action_context(),
                    },
                    task_id=task_id,
                    phase="execution",
                )
                # The orchestrator deliberately ends in a Running("idle")
                # node, so root.status never becomes terminal. The batch flow
                # is the actual mission boundary and gives the dashboard an
                # unambiguous completed/failed lifecycle.
                batch_status = None
                for child in getattr(tree.root, "children", []):
                    if getattr(child, "name", "") == "batch_command_flow":
                        batch_status = getattr(getattr(child, "status", None), "name", None)
                        break
                if batch_status in {"SUCCESS", "FAILURE"}:
                    self.finish(
                        status="succeeded" if batch_status == "SUCCESS" else "failed",
                        summary={"reason": "batch_command_flow", "tree_status": batch_status},
                    )
                now = time.monotonic()
                if self._terminal_status is None and now - last_heartbeat >= 2.0:
                    last_heartbeat = now
                    self.emit("run.heartbeat", {"tick": tick, "active_node_count": len(visited)}, task_id=task_id, phase="execution")
            except Exception:
                return

        return handler


def _node_objects(root: Any, *, kind: str) -> dict[str, Any]:
    result: dict[str, Any] = {}

    def walk(node: Any, path: str) -> None:
        result[stable_node_id(kind, path)] = node
        for index, child in enumerate(getattr(node, "children", ()) or ()):
            walk(child, f"{path}/{index}")

    walk(root, "root")
    return result


def _visited_runtime_nodes(
    visitor: Any | None,
    by_object_id: Mapping[Any, str],
    node_objects: Mapping[str, Any],
    topology: Mapping[str, Mapping[str, Any]],
) -> list[dict[str, Any]]:
    """Return visitor order/status, falling back for deliberately fake trees."""

    visited = getattr(visitor, "visited", None) if visitor is not None else None
    entries: list[tuple[str, Any]] = []
    if isinstance(visited, Mapping):
        for behaviour_id, status in visited.items():
            node_id = by_object_id.get(behaviour_id)
            if node_id is not None:
                entries.append((node_id, status))
    if not entries:
        entries = [(node_id, getattr(node, "status", None)) for node_id, node in node_objects.items()]

    runtime: list[dict[str, Any]] = []
    for order, (node_id, status) in enumerate(entries):
        node = node_objects[node_id]
        semantic = topology.get(node_id, {})
        record = {
            "id": node_id,
            "node_id": node_id,
            "visit_order": order,
            "topology_order": semantic.get("order"),
            "status": getattr(status, "name", str(status or "INVALID")),
            "feedback": str(getattr(node, "feedback_message", "") or "")[:1000],
            "node_class": semantic.get("node_class"),
            "semantics": semantic.get("semantics", {}),
            "blackboard_access": semantic.get("blackboard_access", {}),
            "action_context": semantic.get("action_context", {}),
        }
        counters = runtime_counters(node)
        if counters:
            record["counters"] = counters
        runtime.append(record)
    return runtime


def _counter_deltas(
    visited: list[Mapping[str, Any]],
    previous: dict[str, dict[str, Any]],
) -> list[dict[str, Any]]:
    deltas: list[dict[str, Any]] = []
    for item in visited:
        counters = item.get("counters")
        if not isinstance(counters, Mapping) or not isinstance(counters.get("value"), int):
            continue
        node_id = str(item["id"])
        current = dict(counters)
        before = previous.get(node_id)
        if before is not None and before.get("value") != current["value"]:
            deltas.append(
                {
                    "node_id": node_id,
                    "kind": current.get("kind"),
                    "counter": current.get("counter"),
                    "previous": before.get("value"),
                    "current": current["value"],
                    "delta": current["value"] - before.get("value", current["value"]),
                    "limit": current.get("limit"),
                }
            )
        previous[node_id] = current
    return deltas


def _blackboard_get(key: str) -> Any:
    """Read a py_trees key regardless of its client-facing spelling.

    ``Client.get("gpsr/task_id")`` resolves to ``/gpsr/task_id`` in the
    global storage.  Runtime telemetry reads that storage without owning a
    client, so it must normalise the key in the same way.
    """
    try:
        from py_trees.blackboard import Blackboard

        absolute = key if key.startswith("/") else f"/{key}"
        return Blackboard.storage.get(absolute)
    except Exception:
        return None


def _active_action_context() -> dict[str, Any]:
    try:
        context = {
            "action": _blackboard_get("gpsr/current_action"),
            "params": _blackboard_get("gpsr/current_params"),
            "plan_revision": _blackboard_get("gpsr/plan_revision"),
            "plan_index": _blackboard_get("gpsr/plan_index"),
        }
        return {key: _json_safe(value) for key, value in context.items() if value is not None}
    except Exception:
        return {}


_DEFAULT_TELEMETRY: GpsrTelemetry | None = None


def set_default_telemetry(telemetry: GpsrTelemetry | None) -> None:
    global _DEFAULT_TELEMETRY
    _DEFAULT_TELEMETRY = telemetry


def get_default_telemetry() -> GpsrTelemetry | None:
    return _DEFAULT_TELEMETRY
