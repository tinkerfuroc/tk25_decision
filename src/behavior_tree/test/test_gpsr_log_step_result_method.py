"""K3 (task-K, live-manipulation sim findings): BtNode_LogStepResult folds an
optional STEP_METHOD claim into step.finished's payload, then clears it.

Real GpsrTelemetry (writes to disk, same as test_gpsr_telemetry.py) so this
exercises the actual emit/serialise path, not a mock.
"""
from __future__ import annotations

import json

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR.orchestrator import BtNode_LogStepResult
from behavior_tree.GPSR.small_trees import bb_keys
from behavior_tree.GPSR.telemetry import GpsrTelemetry, set_default_telemetry


def _events(tmp_path, trajectory_id):
    path = tmp_path / "debug" / trajectory_id / "events.jsonl"
    return [json.loads(line) for line in path.read_text().splitlines()]


def _seed_and_tick(tmp_path, *, method=None, succeeded=True):
    py_trees.blackboard.Blackboard.clear()
    telemetry = GpsrTelemetry(tmp_path, trajectory_id="traj-method", enabled=True)
    set_default_telemetry(telemetry)
    try:
        writer = py_trees.blackboard.Client(name="seed")
        writer.register_key(bb_keys.CURRENT_ACTION, access=Access.WRITE)
        writer.register_key(bb_keys.CURRENT_PARAMS, access=Access.WRITE)
        writer.register_key(bb_keys.TASK_ID, access=Access.WRITE)
        writer.register_key(bb_keys.PLAN_REVISION, access=Access.WRITE)
        writer.register_key(bb_keys.PLAN_INDEX, access=Access.WRITE)
        writer.set(bb_keys.CURRENT_ACTION, "grasp", overwrite=True)
        writer.set(bb_keys.CURRENT_PARAMS, {}, overwrite=True)
        writer.set(bb_keys.TASK_ID, "traj-method/task-0", overwrite=True)
        writer.set(bb_keys.PLAN_REVISION, 1, overwrite=True)
        writer.set(bb_keys.PLAN_INDEX, 1, overwrite=True)
        if method is not None:
            writer.register_key(bb_keys.STEP_METHOD, access=Access.WRITE)
            writer.set(bb_keys.STEP_METHOD, method, overwrite=True)

        node = BtNode_LogStepResult("log result", succeeded=succeeded)
        py_trees.trees.BehaviourTree(node).setup()
        node.tick_once()

        events = _events(tmp_path, "traj-method")
    finally:
        telemetry.close()
        set_default_telemetry(None)
    return events


def _reader():
    reader = py_trees.blackboard.Client(name="reader")
    reader.register_key(bb_keys.STEP_METHOD, access=Access.READ)
    try:
        return reader.get(bb_keys.STEP_METHOD)
    except KeyError:
        return None


def test_method_claim_is_included_and_key_is_cleared(tmp_path):
    events = _seed_and_tick(tmp_path, method="referee_fallback")

    finished = [e for e in events if e["event_type"] == "step.finished"]
    assert len(finished) == 1
    assert finished[0]["payload"]["method"] == "referee_fallback"
    # Cleared after being read -- a later step must not inherit it.
    assert _reader() is None


def test_no_method_claim_means_no_method_field_at_all(tmp_path):
    # Never set the key -- assert its ABSENCE from the payload, not a null.
    events = _seed_and_tick(tmp_path, method=None)

    finished = [e for e in events if e["event_type"] == "step.finished"]
    assert len(finished) == 1
    assert "method" not in finished[0]["payload"]


def test_failed_step_can_still_carry_a_method_claim(tmp_path):
    # ex_machina sets referee_fallback as its FIRST action, before it is
    # known whether ex_machina itself will ultimately succeed -- a failed
    # attempt must still surface the method it was attempting.
    events = _seed_and_tick(tmp_path, method="referee_fallback", succeeded=False)

    finished = [e for e in events if e["event_type"] == "step.finished"]
    assert len(finished) == 1
    assert finished[0]["payload"]["outcome"] == "failed"
    assert finished[0]["payload"]["method"] == "referee_fallback"
