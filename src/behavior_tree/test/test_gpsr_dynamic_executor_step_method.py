"""V-1 fix verification (task-K review, HIGH): DynamicExecutor's OWN
``step.finished`` emitter (``_emit_failed_step``) must honor a STEP_METHOD
claim exactly like ``BtNode_LogStepResult`` does.

Review finding: ``BtNode_LogStepResult`` only runs after a SUCCESSFUL step; a
failed leaf short-circuits the sequence before it ever ticks. Under an active
supervisor (the production two-layer execution path), a failed step instead
surfaces through ``DynamicExecutor._emit_failed_step`` -- a second,
independent ``step.finished`` emitter that (before this fix) never read nor
cleared ``bb_keys.STEP_METHOD``. Concretely: grasp's ``ex_machina`` branch
sets ``STEP_METHOD="referee_fallback"`` as its first action, then can still
fail later in that same branch (e.g. the tuck-arm retry exhausts) -- under
supervision that failure goes through ``_emit_failed_step``, which used to
leave the stale claim on the blackboard for an unrelated LATER step's
successful ``step.finished`` (via ``BtNode_LogStepResult``) to wrongly
inherit.

Mirrors this codebase's own established ``DynamicExecutor`` telemetry-test
convention (``test_gpsr_two_layer_telemetry.py``): a real ``GpsrTelemetry``
(writes to disk), a minimal ``_Planner`` stub whose target subtree is a
single always-failing, non-gate leaf (so every tick's failure goes through
``_emit_failed_step``'s actual ``step.finished`` branch, not
``target.failed``), ticked a few times.
"""
from __future__ import annotations

import json
from pathlib import Path

import py_trees
import pytest
from py_trees.common import Access

from behavior_tree.GPSR import telemetry as telemetry_mod
from behavior_tree.GPSR.orchestrator import DynamicExecutor
from behavior_tree.GPSR.small_trees import bb_keys


@pytest.fixture(autouse=True)
def _clean_blackboard_and_telemetry():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()
    telemetry_mod.set_default_telemetry(None)


class _FailingLeaf(py_trees.behaviour.Behaviour):
    """A non-gate leaf (stands in for grasp's ex_machina failing after
    already having claimed a method) that fails with a stable message."""

    def update(self):
        self.feedback_message = "tuck arm before goto: exhausted"
        return py_trees.common.Status.FAILURE


def _leaf_target():
    seq = py_trees.composites.Sequence("target", memory=True)
    seq.add_child(_FailingLeaf("grasp action"))
    return seq


class _Planner:
    def __init__(self, plan):
        self._plan = plan

    def is_target_ready(self, slot, index):
        return True

    def _get_desc(self, slot, index):
        return "grasp the coke"

    def get_target_subtree(self, slot, index):
        return _leaf_target()

    def get_action_plan(self, slot, index):
        return list(self._plan)

    def get_facts(self, slot):
        return []

    def replan_target(self, slot, index, reason, completed_steps=None):
        pass


def _seed_blackboard(*, step_method=None):
    py_trees.blackboard.Blackboard.clear()
    w = py_trees.blackboard.Client(name="w")
    keys = [bb_keys.TASK_ID, bb_keys.CURRENT_ACTION, bb_keys.CURRENT_PARAMS, bb_keys.PLAN_INDEX,
            bb_keys.TARGET_INDEX, bb_keys.TARGET_REPLAN_COUNT, bb_keys.REPLAN_REQUEST,
            bb_keys.STATE_LOG, bb_keys.FACTS, bb_keys.DEFERRED_PRECONDITIONS,
            bb_keys.SUPERVISOR_STEP_DISPOSITION, bb_keys.CURRENT_TARGET,
            bb_keys.SAVED_TARGETS_PREFIX + "0", bb_keys.STEP_METHOD]
    for key in keys:
        w.register_key(key, access=Access.WRITE)
        w.register_key(key, access=Access.READ)
    w.set(bb_keys.TASK_ID, None, overwrite=True)
    w.set(bb_keys.SAVED_TARGETS_PREFIX + "0",
          [{"id": "t0", "desc": "grasp the coke", "depends_on": []}], overwrite=True)
    w.set(bb_keys.CURRENT_ACTION, "grasp", overwrite=True)
    w.set(bb_keys.CURRENT_PARAMS, {"object": "coke"}, overwrite=True)
    w.set(bb_keys.PLAN_INDEX, 1, overwrite=True)
    w.set(bb_keys.TARGET_INDEX, 0, overwrite=True)
    w.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
    w.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
    w.set(bb_keys.STATE_LOG, [], overwrite=True)
    w.set(bb_keys.FACTS, [], overwrite=True)
    if step_method is not None:
        w.set(bb_keys.STEP_METHOD, step_method, overwrite=True)
    return w


def _telemetry(tmp_path):
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    return tele, tele.directory / "events.jsonl"


def _executor(plan):
    ex = DynamicExecutor("executor task 1", 0, _Planner(plan), max_replans_per_target=3)
    tree = py_trees.trees.BehaviourTree(ex)
    ex.setup(gpsr_tree=tree, node=object())
    return ex, tree


def _lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def test_failed_step_via_dynamic_executor_carries_referee_fallback_method_then_clears_it(tmp_path):
    plan = [{"action": "grasp", "params": {"object": "coke"}}]
    tele, path = _telemetry(tmp_path)
    w = _seed_blackboard(step_method="referee_fallback")
    _, tree = _executor(plan)

    # Tick 1: the failing leaf's failure goes through
    # DynamicExecutor._emit_failed_step -- NOT BtNode_LogStepResult, which
    # never runs on a failed step. STEP_METHOD was claimed before this
    # failure (mirrors ex_machina setting it as its first action, then
    # failing later in the same branch).
    tree.tick()
    # Ticks 2-3: replanned attempts at the SAME step -- no claim of their
    # own, so their step.finished must carry no "method" at all (the key
    # was cleared after tick 1).
    tree.tick()
    tree.tick()
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)

    finished = [e for e in _lines(path) if e["event_type"] == "step.finished"]
    assert len(finished) == 3
    assert finished[0]["payload"]["outcome"] == "failed"
    assert finished[0]["payload"]["method"] == "referee_fallback"
    for later in finished[1:]:
        assert "method" not in later["payload"]

    # Cleared on the blackboard too -- nothing left to leak into whatever
    # runs next.
    assert w.get(bb_keys.STEP_METHOD) is None


def test_failed_step_via_dynamic_executor_with_no_claim_carries_no_method_field(tmp_path):
    plan = [{"action": "grasp", "params": {"object": "coke"}}]
    tele, path = _telemetry(tmp_path)
    _seed_blackboard(step_method=None)
    _, tree = _executor(plan)

    tree.tick()
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)

    finished = [e for e in _lines(path) if e["event_type"] == "step.finished"]
    assert len(finished) == 1
    assert "method" not in finished[0]["payload"]
