"""Console entry point for the developer-only GPSR debugger."""
from __future__ import annotations

import argparse
import asyncio
import json
import os
from pathlib import Path
import secrets
import threading

from .ros_bridge import RosBridge
from .store import DebugStore
from .web import create_app


MAX_INGEST_EVENT_BYTES = 16 * 1024 * 1024


class UnixIngest:
    """NDJSON Unix socket collector for mixed-process trace clients."""

    def __init__(self, path: Path, callback):
        self.path = path
        self.callback = callback
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="gpsr-debug-ingest", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
        if self._thread:
            self._thread.join(timeout=2.0)

    def _run(self) -> None:  # pragma: no cover - exercised by integration tests
        asyncio.run(self._serve())

    async def _serve(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass
        # Executor tree snapshots routinely exceed asyncio's 64 KiB default
        # StreamReader limit (a full GPSR tree is currently about 1.4 MiB).
        # Keep the protocol newline-delimited, but accept one deliberately
        # bounded, large event without dropping the producer connection.
        server = await asyncio.start_unix_server(
            self._client,
            path=str(self.path),
            limit=MAX_INGEST_EVENT_BYTES,
        )
        try:
            os.chmod(self.path, 0o600)
            async with server:
                while not self._stop.is_set():
                    await asyncio.sleep(0.25)
        finally:
            server.close()
            await server.wait_closed()
            try:
                self.path.unlink()
            except FileNotFoundError:
                pass

    async def _client(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        try:
            while not self._stop.is_set():
                line = await reader.readline()
                if not line:
                    return
                try:
                    value = json.loads(line)
                    if isinstance(value, dict):
                        self.callback(value)
                except Exception:
                    continue
        finally:
            writer.close()
            try:
                await writer.wait_closed()
            except Exception:
                pass


def _state_dir() -> Path:
    configured = os.environ.get("GPSR_DEBUG_STATE_DIR")
    return Path(configured).expanduser() if configured else Path.home() / ".local" / "state" / "gpsr_debug_server"


def _webui_dir() -> Path:
    """Resolve source-tree and ament-installed static assets."""
    candidates = [
        Path(__file__).resolve().parents[1] / "webui",
        Path(__file__).resolve().parents[2] / "share" / "gpsr_debug_server" / "webui",
    ]
    for candidate in candidates:
        if (candidate / "index.html").exists():
            return candidate
    return candidates[0]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Developer GPSR causal debugger")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--state-dir", type=Path, default=_state_dir())
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument(
        "--no-ingest",
        action="store_true",
        help="serve an existing replay without opening the Unix ingest socket",
    )
    args = parser.parse_args(argv)
    if args.host not in {"127.0.0.1", "localhost", "::1"}:
        raise SystemExit("gpsr-debug-server only binds loopback; use an SSH tunnel")
    args.state_dir.mkdir(parents=True, exist_ok=True)
    # Keep ROS diagnostics with the debugger state. This also makes manual
    # launches work under service accounts whose home directory is read-only.
    ros_log_dir = args.state_dir / "ros_logs"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_LOG_DIR", str(ros_log_dir))
    token_path = args.state_dir / "session.token"
    if token_path.exists():
        token = token_path.read_text(encoding="utf-8").strip()
    else:
        token = secrets.token_urlsafe(32)
        token_path.write_text(token + "\n", encoding="utf-8")
        os.chmod(token_path, 0o600)

    store = DebugStore(args.state_dir / "events.sqlite3")
    artifact_dir = args.state_dir / "artifacts"
    artifact_dir.mkdir(parents=True, exist_ok=True)
    # Retention runs once at deliberate server startup, never in a ROS callback.
    # Active, named, and pinned trajectories are protected by the policy.
    try:
        store.retention()
    except Exception as exc:
        print(f"[gpsr-debug] retention warning (continuing): {exc}")
    app_holder: dict[str, object] = {}

    def ingest(event: dict) -> None:
        app = app_holder.get("app")
        if app is not None:
            app.state.ingest_event(event)
        else:
            store.append_event(event)

    ros = RosBridge(ingest, lambda message: print(f"[gpsr-debug] ROS warning: {message}"))
    app = create_app(
        store,
        webui_dir=_webui_dir(),
        artifact_dir=artifact_dir,
        secret=token,
        control_submitter=ros.send_command,
    )
    app_holder["app"] = app
    def ingest_from_app(event: dict) -> None:
        # ``append_event`` returns False for an idempotent replay.  Do not use
        # that return value as a boolean short-circuit: live viewers must still
        # receive the record when a producer retries delivery.
        app.state.store.append_event(event)
        app.state.broker.publish_threadsafe({"type": "gpsr.event", "event": event})

    app.state.ingest_event = ingest_from_app
    ingest_socket = UnixIngest(args.state_dir / "ingest.sock", ingest)
    if not args.no_ingest:
        ingest_socket.start()
    if not args.no_ros:
        ros.start()
    try:
        import uvicorn
        uvicorn.run(app, host=args.host, port=args.port, log_level="info", access_log=True)
    finally:
        if not args.no_ingest:
            ingest_socket.stop()
        ros.stop()
        store.close()


if __name__ == "__main__":
    main()
