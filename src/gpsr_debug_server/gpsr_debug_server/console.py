"""GPSR command console — a loopback web panel for driving one live GPSR run.

Shows, in real time (polled from ``/gpsr/debug/events``):

* the command(s) the console launched,
* the full materialized plan per command slot, and
* which step is currently executing (highlighted), which finished and how.

Lets the operator type a command (or draw one from the official command
generator) and start a fresh ``gpsr-orchestrator`` for it — the same
one-process-per-command shape the tier-2 battery uses. The orchestrator
process is a child of this server: Stop (or a new Start) SIGINTs its process
group, waits, then SIGKILLs survivors.

Loopback-only, like ``gpsr-debug-server``: reach it over an SSH tunnel.
"""
from __future__ import annotations

import argparse
import json
import os
import random
import signal
import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

_EVENTS_TOPIC = "/gpsr/debug/events"
_RECENT_EVENT_CAP = 120
_LOG_TAIL_LINES = 60

_LOOPBACK_HOSTS = {"127.0.0.1", "localhost", "::1"}


# ---------------------------------------------------------------------------
# State model (pure — no ROS, no FastAPI; unit-tested directly)
# ---------------------------------------------------------------------------


@dataclass
class PlanView:
    """One materialized plan (slot+target+revision) plus its step outcomes.

    A single GPSR command decomposes into several targets, each planned
    separately (revisions restart per target), so the console keeps plans as
    an ordered list and matches ``step.finished`` events to plans by
    execution order — ``step.finished`` carries no slot/target identity.
    """

    slot: int
    target_index: int
    revision: int
    steps: list[dict[str, Any]]
    outcomes: dict[int, str] = field(default_factory=dict)
    superseded: bool = False  # a later revision replaced this plan, pre-execution

    def complete(self) -> bool:
        return bool(self.steps) and all(i in self.outcomes for i in range(len(self.steps)))

    def view(self, running_step: Optional[int]) -> dict[str, Any]:
        rows = []
        for i, step in enumerate(self.steps):
            outcome = self.outcomes.get(i)
            if outcome is not None:
                status = outcome  # succeeded | failed | ...
            elif running_step == i:
                status = "running"
            else:
                status = "pending"
            rows.append(
                {
                    "index": i,
                    "action": step.get("action"),
                    "params": step.get("params", {}),
                    "status": status,
                }
            )
        return {
            "slot": self.slot,
            "target_index": self.target_index,
            "revision": self.revision,
            "superseded": self.superseded,
            "complete": self.complete(),
            "steps": rows,
        }


class ConsoleState:
    """Incrementally folds the telemetry event stream into a UI snapshot."""

    def __init__(self) -> None:
        self.trajectory_id: Optional[str] = None
        self.run_status: Optional[str] = None  # "running" | "finished"
        self.run_finished_status: Optional[str] = None
        self.last_event_at: Optional[str] = None
        self.last_heartbeat_at: Optional[str] = None
        self.plans: list[PlanView] = []
        self.active_index: Optional[int] = None
        self.recent: deque[dict[str, Any]] = deque(maxlen=_RECENT_EVENT_CAP)

    # -- ingestion ---------------------------------------------------------

    def feed(self, event: dict[str, Any]) -> None:
        try:
            self._feed(event)
        except Exception:
            # A malformed event must never take the console down.
            pass

    def _feed(self, event: dict[str, Any]) -> None:
        event_type = event.get("event_type")
        payload = event.get("payload") or {}
        occurred_at = event.get("occurred_at")
        if occurred_at:
            self.last_event_at = occurred_at
        self.recent.append(
            {
                "occurred_at": occurred_at,
                "event_type": event_type,
                "phase": event.get("phase"),
                "payload": payload,
            }
        )

        trajectory_id = event.get("trajectory_id")
        if trajectory_id and trajectory_id != self.trajectory_id:
            # A fresh orchestrator process started: drop the previous run.
            self.trajectory_id = trajectory_id
            self.run_status = "running"
            self.run_finished_status = None
            self.plans = []
            self.active_index = None

        if event_type == "run.started":
            self.run_status = "running"
        elif event_type == "run.heartbeat":
            self.last_heartbeat_at = occurred_at
        elif event_type == "run.finished":
            self.run_status = "finished"
            self.run_finished_status = payload.get("status")
        elif event_type == "plan.materialized":
            slot = int(payload.get("slot", 0))
            target_index = int(payload.get("target_index", 0))
            revision = int(payload.get("revision", 1))
            steps = list(payload.get("steps") or [])
            for plan in self.plans:
                if plan.slot == slot and plan.target_index == target_index:
                    plan.superseded = True
            self.plans.append(
                PlanView(slot=slot, target_index=target_index, revision=revision, steps=steps)
            )
        elif event_type == "step.finished":
            self._apply_step_finished(payload)
        elif event_type == "tree.tick_observed":
            context = payload.get("active_action_context")
            if isinstance(context, dict):
                self.active_index = context.get("plan_index")
            # A null context (tree between plans, or announcing) leaves the
            # last known one in place; the current-plan pointer below keeps
            # the highlight anchored to the right plan regardless.

    def _apply_step_finished(self, payload: dict[str, Any]) -> None:
        step_index = payload.get("step_index")
        revision = payload.get("plan_revision")
        action = payload.get("action")
        outcome = str(payload.get("outcome", "unknown"))
        if not isinstance(step_index, int):
            return

        def matches(plan: PlanView, use_revision: bool) -> bool:
            if plan.complete() or step_index >= len(plan.steps):
                return False
            if action is not None and plan.steps[step_index].get("action") != action:
                return False
            # Revisions restart per target, and replanned (superseded) plans
            # never execute — so revision is a preference, not a hard filter.
            if use_revision and revision is not None and plan.revision != revision:
                return False
            return True

        # Live plans first (superseded ones only ever match as a fallback),
        # and only drop the revision filter as a last resort.
        for use_revision in (True, False):
            for superseded in (False, True):
                for plan in self.plans:
                    if plan.superseded == superseded and matches(plan, use_revision):
                        plan.outcomes[step_index] = outcome
                        return

    # -- projection --------------------------------------------------------

    def _current_plan(self) -> Optional[PlanView]:
        """The plan the executor is on: first un-superseded, incomplete plan."""
        for plan in self.plans:
            if not plan.complete():
                return plan
        return None

    def snapshot(self) -> dict[str, Any]:
        current = self._current_plan()
        running_step = None
        if current is not None and self.run_status == "running":
            running_step = self.active_index if isinstance(self.active_index, int) else None
            if running_step is not None and running_step >= len(current.steps):
                running_step = None
        return {
            "trajectory_id": self.trajectory_id,
            "run_status": self.run_status,
            "run_finished_status": self.run_finished_status,
            "last_event_at": self.last_event_at,
            "last_heartbeat_at": self.last_heartbeat_at,
            "active_step": running_step,
            "plans": [p.view(running_step if p is current else None) for p in self.plans],
            "recent": list(self.recent),
        }


# ---------------------------------------------------------------------------
# Orchestrator process management
# ---------------------------------------------------------------------------


def parse_env_file(path: Path) -> dict[str, str]:
    """KEY=VALUE lines → dict. Skips blanks/comments; strips ``export``/quotes."""
    env: dict[str, str] = {}
    if not path.is_file():
        return env
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export "):].strip()
        if "=" not in line:
            continue
        key, _, value = line.partition("=")
        key = key.strip()
        value = value.strip().strip("'\"")
        if key:
            env[key] = value
    return env


class OrchestratorManager:
    """Owns at most one ``gpsr-orchestrator`` child, one command per run."""

    def __init__(self, tinker_ws: Path, log_dir: Path, domain_id: int = 42) -> None:
        self.tinker_ws = tinker_ws
        self.log_dir = log_dir
        self.domain_id = domain_id
        self.proc: Optional[subprocess.Popen] = None
        self.started_at: Optional[str] = None
        self.command: Optional[str] = None
        self.log_path: Optional[Path] = None
        self._lock = threading.Lock()

    # -- helpers -----------------------------------------------------------

    def _constants_path(self) -> Path:
        return (
            self.tinker_ws
            / "src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
        )

    def _mock_config_path(self) -> Path:
        return (
            self.tinker_ws
            / "src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.sim.json"
        )

    def _child_env(self, command: str, num_commands: int) -> dict[str, str]:
        env = dict(os.environ)
        env.update(parse_env_file(self.tinker_ws / ".env"))
        env.update(
            {
                "ROS_DOMAIN_ID": str(self.domain_id),
                "BT_GPSR_CMD": command,
                "BT_GPSR_NUM_COMMANDS": str(num_commands),
                "GPSR_CONSTANTS_PATH": str(self._constants_path()),
                "BT_MOCK_CONFIG": str(self._mock_config_path()),
            }
        )
        return env

    @staticmethod
    def _launch_script() -> str:
        return (
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            "exec ros2 run behavior_tree gpsr-orchestrator"
        )

    # -- lifecycle ---------------------------------------------------------

    def start(self, command: str) -> dict[str, Any]:
        command = command.strip()
        if not command:
            return {"ok": False, "error": "empty command"}
        num_commands = max(1, len([c for c in command.split("|") if c.strip()]))
        self.stop()
        self.log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.log_path = self.log_dir / f"orchestrator-{stamp}.log"
        with self._lock:
            log_file = self.log_path.open("a", encoding="utf-8")
            self.proc = subprocess.Popen(
                ["bash", "-c", self._launch_script()],
                cwd=str(self.tinker_ws),
                env=self._child_env(command, num_commands),
                stdout=log_file,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.started_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
            self.command = command
        return {"ok": True, "pid": self.proc.pid, "log": str(self.log_path)}

    def stop(self) -> dict[str, Any]:
        with self._lock:
            proc, self.proc = self.proc, None
        if proc is None or proc.poll() is not None:
            return {"ok": True, "stopped": False, "note": "not running"}
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            return {"ok": True, "stopped": True}
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait(timeout=10)
        return {"ok": True, "stopped": True}

    def status(self) -> dict[str, Any]:
        with self._lock:
            proc = self.proc
            running = proc is not None and proc.poll() is None
            return {
                "running": running,
                "pid": proc.pid if proc is not None else None,
                "returncode": proc.returncode if proc is not None else None,
                "started_at": self.started_at,
                "command": self.command,
                "log": str(self.log_path) if self.log_path else None,
                "log_tail": self._log_tail(),
            }

    def _log_tail(self) -> list[str]:
        if self.log_path is None or not self.log_path.is_file():
            return []
        try:
            lines = self.log_path.read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return []
        return lines[-_LOG_TAIL_LINES:]


# ---------------------------------------------------------------------------
# Random command from the official generator
# ---------------------------------------------------------------------------


def random_command(constants_path: Path, seed: Optional[int] = None) -> str:
    """One official-grammar command, expanded exactly as the bench corpus is."""
    from behavior_tree.GPSR.bench.corpus import build_knowledge, expand_template, make_generator

    kb = build_knowledge(constants_path)
    gen = make_generator(kb, seed=seed if seed is not None else random.SystemRandom().randint(0, 2**32 - 1))
    people = [name for name, _ in gen.person_cmd_list]
    objects = [name for name, _ in gen.object_cmd_list]
    template = random.choice(sorted(set(people) | set(objects)))
    category = (
        "objects" if template in objects and template not in people else "people"
    )
    text, _ = expand_template(gen, template, category)
    return text


# ---------------------------------------------------------------------------
# ROS listener
# ---------------------------------------------------------------------------


class RosEventListener:
    """Feeds ``/gpsr/debug/events`` (std_msgs/String JSON) into a ConsoleState.

    Also probes the core GPSR stack surfaces (one TTS service, three action
    servers) so the console can show whether Stages 1-5 are actually up —
    without them an orchestrator blocks in setup() at its first service node
    ("announce ready for gpsr") by design.
    """

    #: Service probes: label → (name, expected type string). Types are
    #: compared as graph strings — no message-package imports needed.
    PROBE_SERVICES = {
        "announce": ("announce", "tinker_audio_msgs/srv/TextToSpeech"),
    }
    #: Action servers, detected via the /_action/send_goal hidden services.
    PROBE_ACTIONS = ("listen_action", "navigate_to_pose", "joint_move_action")

    def __init__(self, state: ConsoleState, topic: str = _EVENTS_TOPIC) -> None:
        self.state = state
        self.topic = topic
        self._node = None
        self._thread = None

    def start(self) -> bool:
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import String
        except ImportError:
            return False
        rclpy.init()
        self._node = Node("gpsr_console_listener")
        self._node.create_subscription(
            String, self.topic, self._on_message, 10
        )
        self._thread = threading.Thread(target=self._spin, name="ros-listener", daemon=True)
        self._thread.start()
        return True

    def probe_readiness(self) -> Optional[dict[str, bool]]:
        """Which core GPSR services/actions exist right now (None = no ROS).

        Graph-based, mirroring tools/gpsr_interface_census.py: a typed client
        (`wait_for_service`) never matches when the probe's type differs from
        the server's, so presence is read from get_service_names_and_types().
        """
        if self._node is None:
            return None
        try:
            names_and_types = dict(self._node.get_service_names_and_types())
            actions = {
                name[: -len("/_action/send_goal")]
                for name in names_and_types
                if name.endswith("/_action/send_goal")
            }
            readiness: dict[str, bool] = {}
            for label, (name, expected_type) in self.PROBE_SERVICES.items():
                types = names_and_types.get(name) or names_and_types.get("/" + name)
                readiness[label] = bool(types) and expected_type in types
            for label in self.PROBE_ACTIONS:
                readiness[label] = ("/" + label) in actions
            return readiness
        except Exception:
            return None

    def _on_message(self, msg: "String") -> None:  # noqa: F821 — ROS import is lazy
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(event, dict):
            self.state.feed(event)

    def _spin(self) -> None:
        import rclpy

        try:
            rclpy.spin(self._node)
        except Exception:  # rclpy shutdown races at exit are expected
            pass

    def shutdown(self) -> None:
        try:
            if self._node is not None:
                self._node.destroy_node()
            import rclpy

            rclpy.shutdown()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Web app
# ---------------------------------------------------------------------------


def create_app(state: ConsoleState, manager: OrchestratorManager, webui: Path,
               constants_path: Path,
               readiness: Optional[Any] = None) -> "Any":
    from fastapi import FastAPI
    from fastapi.responses import FileResponse, JSONResponse

    app = FastAPI(title="GPSR Command Console", docs_url=None, redoc_url=None)

    @app.get("/")
    def index() -> Any:
        return FileResponse(webui / "console.html", headers={"Cache-Control": "no-store"})

    @app.get("/api/state")
    def api_state() -> Any:
        stack = readiness() if callable(readiness) else None
        return JSONResponse(
            {"telemetry": state.snapshot(), "orchestrator": manager.status(),
             "stack": stack},
            headers={"Cache-Control": "no-store"},
        )

    @app.post("/api/command")
    def api_command(text: str = "") -> Any:
        # Plain builtin-typed query param: FastAPI resolves route annotations
        # against module globals, so closure-local types (pydantic models,
        # Request) are invisible to it and get misread as required query
        # bodies. Keep every route annotation builtin-only.
        result = manager.start(text)
        return JSONResponse(result, status_code=200 if result.get("ok") else 400)

    @app.post("/api/stop")
    def api_stop() -> Any:
        return manager.stop()

    @app.get("/api/random")
    def api_random() -> Any:
        try:
            return {"ok": True, "text": random_command(constants_path)}
        except Exception as exc:  # generator failure must not 500 the panel
            return JSONResponse({"ok": False, "error": f"{type(exc).__name__}: {exc}"}, status_code=200)

    return app


def main(argv: Optional[list[str]] = None) -> None:
    parser = argparse.ArgumentParser(description="GPSR command console (loopback web panel)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8767)
    parser.add_argument("--tinker-ws", type=Path, default=Path("/home/tinker/tk25_ws"))
    parser.add_argument("--constants", type=Path, default=None,
                        help="constants JSON for the random generator (default: rcw2026)")
    parser.add_argument("--log-dir", type=Path, default=Path("/tmp/gpsr_console"))
    parser.add_argument("--domain-id", type=int, default=42)
    parser.add_argument("--no-ros", action="store_true", help="skip the ROS listener (UI testing)")
    args = parser.parse_args(argv)

    if args.host not in _LOOPBACK_HOSTS:
        raise SystemExit("gpsr-console only binds loopback; use an SSH tunnel")

    constants = args.constants or (
        args.tinker_ws
        / "src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.rcw2026.json"
    )
    webui = Path(__file__).resolve().parent.parent / "webui"

    state = ConsoleState()
    manager = OrchestratorManager(args.tinker_ws, args.log_dir, domain_id=args.domain_id)

    listener = None
    if not args.no_ros:
        listener = RosEventListener(state)
        if listener.start():
            print(f"listening on ROS topic {_EVENTS_TOPIC}", flush=True)
        else:
            print("rclpy unavailable — telemetry panel will stay empty", flush=True)

    import uvicorn

    try:
        uvicorn.run(
            create_app(
                state, manager, webui, constants,
                readiness=(listener.probe_readiness if listener is not None else None),
            ),
            host=args.host,
            port=args.port,
            log_level="warning",
        )
    finally:
        manager.stop()
        if listener is not None:
            listener.shutdown()


if __name__ == "__main__":
    main()
