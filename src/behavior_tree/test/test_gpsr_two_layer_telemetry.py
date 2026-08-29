from __future__ import annotations

import json
from pathlib import Path

import py_trees
import pytest
from py_trees.common import Access

from behavior_tree.GPSR import telemetry as telemetry_mod
from behavior_tree.GPSR.bench.events import parse_events
from behavior_tree.GPSR.orchestrator import DynamicExecutor
from behavior_tree.GPSR.small_trees import bb_keys


@pytest.fixture(autouse=True)
def _clean_blackboard_and_telemetry():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()
    telemetry_mod.set_default_telemetry(None)


class _FailingGate(py_trees.behaviour.Behaviour):
    """A leaf that fails with a stable feedback message.

    ``py_trees.behaviours.Failure`` overwrites ``feedback_message`` with the
    literal "failure" on every ``update()``, so it cannot carry a message.
    """

    def update(self):
        self.feedback_message = "precondition unmet: at_robot(kitchen_table) (invalid)"
        return py_trees.common.Status.FAILURE


class _Planner:
    def __init__(self, plan):
        self._plan = plan

    def is_target_ready(self, slot, index):
        return True

    def _get_desc(self, slot, index):
        # DynamicExecutor.tick calls planner._get_desc unconditionally after a swap.
        return "place the coke"

    def get_target_subtree(self, slot, index):
        seq = py_trees.composites.Sequence("target", memory=True)
        seq.add_child(_FailingGate("gate"))
        return seq

    def get_action_plan(self, slot, index):
        return list(self._plan)

    def get_facts(self, slot):
        return []

    def replan_target(self, slot, index, reason):
        pass


def _events(tmp_path, plan):
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    path = tele.directory / "events.jsonl"
    py_trees.blackboard.Blackboard.clear()
    w = py_trees.blackboard.Client(name="w")
    for key in (bb_keys.TASK_ID, bb_keys.CURRENT_ACTION, bb_keys.CURRENT_PARAMS, bb_keys.PLAN_INDEX,
                bb_keys.TARGET_INDEX, bb_keys.TARGET_REPLAN_COUNT, bb_keys.REPLAN_REQUEST,
                bb_keys.STATE_LOG, bb_keys.FACTS, bb_keys.DEFERRED_PRECONDITIONS,
                bb_keys.SUPERVISOR_STEP_DISPOSITION, bb_keys.CURRENT_TARGET,
                bb_keys.SAVED_TARGETS_PREFIX + "0"):
        w.register_key(key, access=Access.WRITE)
        w.register_key(key, access=Access.READ)
    w.set(bb_keys.TASK_ID, None, overwrite=True)
    w.set(bb_keys.SAVED_TARGETS_PREFIX + "0",
          [{"id": "t0", "desc": "place the coke", "depends_on": []}], overwrite=True)
    w.set(bb_keys.CURRENT_ACTION, "place", overwrite=True)
    w.set(bb_keys.CURRENT_PARAMS, {"location": "kitchen_table"}, overwrite=True)
    w.set(bb_keys.PLAN_INDEX, 1, overwrite=True)
    w.set(bb_keys.TARGET_INDEX, 0, overwrite=True)
    w.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
    w.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
    w.set(bb_keys.STATE_LOG, [], overwrite=True)
    w.set(bb_keys.FACTS, [], overwrite=True)

    # Budget >= tick count: exhausting the replan budget triggers _announce(),
    # which needs a real rclpy node (create_client). Keep this test ROS-free.
    ex = DynamicExecutor("executor task 1", 0, _Planner(plan), max_replans_per_target=3)
    tree = py_trees.trees.BehaviourTree(ex)
    # DynamicExecutor reads SAVED_TARGETS_PREFIX+slot on its first tick.
    ex.setup(gpsr_tree=tree, node=object())
    for _ in range(3):
        tree.tick()
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)
    return path, w


def test_swap_in_emits_plan_materialized_and_sets_task_id(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    path, w = _events(tmp_path, plan)
    lines = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    mat = [e for e in lines if e["event_type"] == "plan.materialized"]
    assert mat and mat[0]["payload"]["steps"] == plan
    assert str(mat[0]["task_id"]).endswith("/task-1")
    assert w.get(bb_keys.TASK_ID) == mat[0]["task_id"]
    # Every tick swaps in a replan attempt: one materialization per attempt,
    # revisions counting up, and the task id set once and never overwritten.
    assert [e["payload"]["revision"] for e in mat] == [1, 2, 3]
    assert {e["task_id"] for e in mat} == {mat[0]["task_id"]}
    assert all(e["phase"] == "planning" for e in mat)


def test_failed_step_emits_step_finished_the_bench_can_parse(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    path, _ = _events(tmp_path, plan)
    results = parse_events(Path(path))
    # Three ticks = three failed attempts at the same step (replan each time).
    assert results[1].steps == [("place", "failed")] * 3
    lines = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    failed = [e for e in lines if e["event_type"] == "step.finished"]
    assert failed[0]["phase"] == "execution"
    assert failed[0]["payload"]["step_index"] == 0
    assert failed[0]["payload"]["params"] == {"location": "kitchen_table"}
    assert failed[0]["payload"]["feedback"] == (
        "precondition unmet: at_robot(kitchen_table) (invalid)")
