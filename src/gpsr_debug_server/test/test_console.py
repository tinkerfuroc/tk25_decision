"""Unit tests for the GPSR command console's pure state model and env parsing."""
from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from gpsr_debug_server.console import ConsoleState, parse_env_file  # noqa: E402


def _event(event_type: str, payload: dict | None = None, *, seq: int = 0,
           trajectory: str = "t1", occurred_at: str = "2026-09-08T12:00:00Z") -> dict:
    return {
        "schema": "tinker.gpsr.telemetry",
        "event_type": event_type,
        "trajectory_id": trajectory,
        "sequence": seq,
        "occurred_at": occurred_at,
        "payload": payload or {},
    }


@pytest.fixture()
def state() -> ConsoleState:
    return ConsoleState()


def test_lifecycle_happy_path(state: ConsoleState) -> None:
    state.feed(_event("run.started", {"producer": "gpsr-orchestrator:1"}, seq=1))
    state.feed(_event("run.configured", {"expected_task_count": 1, "mode": "injected"}, seq=2))
    assert state.snapshot()["run_status"] == "running"

    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 0, "revision": 1,
        "steps": [
            {"action": "goto", "params": {"location": "living_room"}},
            {"action": "find_object", "params": {"object": "kitchen item", "location": "living_room"}},
        ],
    }, seq=3))

    state.feed(_event("tree.tick_observed",
                      {"active_action_context": {"plan_revision": 1, "plan_index": 0}}, seq=4))
    snap = state.snapshot()
    assert len(snap["plans"]) == 1
    assert snap["plans"][0]["steps"][0]["status"] == "running"
    assert snap["plans"][0]["steps"][1]["status"] == "pending"

    state.feed(_event("step.finished", {
        "step_index": 0, "plan_revision": 1, "action": "goto",
        "params": {"location": "living_room"}, "outcome": "succeeded", "feedback": "",
    }, seq=5))
    state.feed(_event("tree.tick_observed",
                      {"active_action_context": {"plan_revision": 1, "plan_index": 1}}, seq=6))
    snap = state.snapshot()
    assert snap["plans"][0]["steps"][0]["status"] == "succeeded"
    assert snap["plans"][0]["steps"][1]["status"] == "running"

    state.feed(_event("step.finished", {
        "step_index": 1, "plan_revision": 1, "action": "find_object",
        "params": {}, "outcome": "failed", "feedback": "",
    }, seq=7))
    state.feed(_event("run.finished", {"status": "failure"}, seq=8))
    snap = state.snapshot()
    assert snap["run_status"] == "finished"
    assert snap["run_finished_status"] == "failure"
    assert snap["plans"][0]["steps"][1]["status"] == "failed"


def test_new_trajectory_resets(state: ConsoleState) -> None:
    state.feed(_event("run.started", trajectory="t1", seq=1))
    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 0, "revision": 1,
        "steps": [{"action": "goto", "params": {}}],
    }, trajectory="t1", seq=2))
    state.feed(_event("run.finished", {"status": "success"}, trajectory="t1", seq=3))

    state.feed(_event("run.started", trajectory="t2", seq=1))
    snap = state.snapshot()
    assert snap["trajectory_id"] == "t2"
    assert snap["plans"] == []
    assert snap["run_status"] == "running"
    assert snap["run_finished_status"] is None


def test_replan_supersedes_and_steps_follow_execution_order(state: ConsoleState) -> None:
    """Mirrors the recorded t2 shape: revisions restart, steps carry no slot."""
    state.feed(_event("run.started", trajectory="t1", seq=1))
    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 0, "revision": 1,
        "steps": [{"action": "goto"}, {"action": "find_object"}],
    }, seq=2))
    state.feed(_event("step.finished", {
        "step_index": 0, "plan_revision": 1, "action": "goto", "outcome": "succeeded",
    }, seq=3))
    state.feed(_event("step.finished", {
        "step_index": 1, "plan_revision": None, "action": "find_object", "outcome": "failed",
    }, seq=4))
    # Replan of the same target, then the next targets.
    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 0, "revision": 2,
        "steps": [{"action": "goto"}, {"action": "find_object"}],
    }, seq=5))
    state.feed(_event("step.finished", {
        "step_index": 0, "plan_revision": 1, "action": "goto", "outcome": "succeeded",
    }, seq=6))
    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 1, "revision": 1,
        "steps": [{"action": "goto"}, {"action": "grasp"}],
    }, seq=7))
    state.feed(_event("step.finished", {
        "step_index": 1, "plan_revision": 1, "action": "find_object", "outcome": "succeeded",
    }, seq=8))
    state.feed(_event("step.finished", {
        "step_index": 0, "plan_revision": 1, "action": "goto", "outcome": "succeeded",
    }, seq=9))
    state.feed(_event("step.finished", {
        "step_index": 1, "plan_revision": 1, "action": "grasp", "outcome": "succeeded",
    }, seq=10))
    state.feed(_event("run.finished", {"status": "incomplete"}, seq=11))

    snap = state.snapshot()
    assert len(snap["plans"]) == 3
    first, second, third = snap["plans"]
    assert first["superseded"] is True
    assert [s["status"] for s in first["steps"]] == ["succeeded", "failed"]
    assert [s["status"] for s in second["steps"]] == ["succeeded", "succeeded"]
    assert [s["status"] for s in third["steps"]] == ["succeeded", "succeeded"]
    assert all(p["complete"] for p in snap["plans"])


def test_stale_step_does_not_touch_newer_revision(state: ConsoleState) -> None:
    """A superseded, never-executed plan must not absorb live outcomes."""
    state.feed(_event("run.started", trajectory="t1", seq=1))
    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 0, "revision": 1,
        "steps": [{"action": "goto", "params": {}}],
    }, seq=2))
    state.feed(_event("plan.materialized", {
        "slot": 0, "target_index": 0, "revision": 2,
        "steps": [
            {"action": "goto", "params": {"location": "kitchen"}},
            {"action": "grasp", "params": {"object": "coke"}},
            {"action": "place", "params": {}},
        ],
    }, seq=3))
    state.feed(_event("step.finished", {
        "step_index": 0, "plan_revision": 2, "action": "goto", "outcome": "succeeded",
    }, seq=4))
    snap = state.snapshot()
    assert snap["plans"][0]["superseded"] is True
    assert snap["plans"][0]["steps"][0]["status"] == "pending"
    assert snap["plans"][1]["steps"][0]["status"] == "succeeded"


def test_malformed_events_never_raise(state: ConsoleState) -> None:
    state.feed({})
    state.feed({"event_type": "plan.materialized", "payload": {"slot": "x", "revision": None}})
    state.feed({"event_type": "step.finished", "payload": {"step_index": [], "plan_revision": {}}})
    state.feed({"event_type": "tree.tick_observed", "payload": {"active_action_context": "nope"}})
    state.feed(None)  # type: ignore[arg-type]
    snap = state.snapshot()
    assert snap["trajectory_id"] is None


def test_parse_env_file(tmp_path: Path) -> None:
    envf = tmp_path / ".env"
    envf.write_text(
        "# comment\n"
        "\n"
        "OPENROUTER_API_KEY=sk-test-123\n"
        "export OTHER_KEY=\"quoted value\"\n"
        "BROKEN LINE WITHOUT EQUALS\n",
        encoding="utf-8",
    )
    parsed = parse_env_file(envf)
    assert parsed == {"OPENROUTER_API_KEY": "sk-test-123", "OTHER_KEY": "quoted value"}
    assert parse_env_file(tmp_path / "missing.env") == {}


_REAL_EVENTS = (
    Path(__file__).resolve().parents[2]
    / "behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-2026/runs"
    / "s2026-003-findObjInRoom/debug"
)


def _real_event_files() -> list[Path]:
    if not _REAL_EVENTS.is_dir():
        return []
    return sorted(_REAL_EVENTS.glob("*/events.jsonl"))


@pytest.mark.skipif(not _real_event_files(), reason="no recorded t2 telemetry on this checkout")
def test_replay_recorded_tier2_run() -> None:
    state = ConsoleState()
    for path in _real_event_files():
        state = ConsoleState()
        for line in path.read_text(encoding="utf-8").splitlines():
            state.feed(json.loads(line))
        snap = state.snapshot()
        assert snap["run_status"] == "finished"
        assert snap["plans"], "recorded run must have materialized a plan"
        final = snap["plans"][-1]
        statuses = [s["status"] for s in final["steps"]]
        # A finished run leaves no step visibly running/pending in its final plan.
        assert all(s in ("succeeded", "failed") or "error" in s for s in statuses), statuses
