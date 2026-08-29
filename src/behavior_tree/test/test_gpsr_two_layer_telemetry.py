from __future__ import annotations

import json
from pathlib import Path

import py_trees
import pytest
from py_trees.common import Access

from behavior_tree.GPSR import planner as planner_mod
from behavior_tree.GPSR import telemetry as telemetry_mod
from behavior_tree.GPSR.bench.events import parse_events
from behavior_tree.GPSR.orchestrator import BtNode_TargetPreconditionCheck, DynamicExecutor
from behavior_tree.GPSR.small_trees import bb_keys


@pytest.fixture(autouse=True)
def _clean_blackboard_and_telemetry():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()
    telemetry_mod.set_default_telemetry(None)


class _FailingLeaf(py_trees.behaviour.Behaviour):
    """A non-gate leaf (stands in for an action) that fails with a stable message.

    ``py_trees.behaviours.Failure`` overwrites ``feedback_message`` with the
    literal "failure" on every ``update()``, so it cannot carry a message.
    """

    def update(self):
        self.feedback_message = "navigation failed: kitchen_table unreachable"
        return py_trees.common.Status.FAILURE


def _gate_target():
    """Target whose real precondition gate fails at entry (held(coke) unknown)."""
    seq = py_trees.composites.Sequence("target", memory=True)
    seq.add_child(BtNode_TargetPreconditionCheck(
        "precondition gate:0:0", ["held(coke)"], 0, action_plan=[]))
    return seq


def _leaf_target():
    seq = py_trees.composites.Sequence("target", memory=True)
    seq.add_child(_FailingLeaf("place action"))
    return seq


class _Planner:
    def __init__(self, plan, subtree_factory=_leaf_target):
        self._plan = plan
        self._factory = subtree_factory

    def is_target_ready(self, slot, index):
        return True

    def _get_desc(self, slot, index):
        # DynamicExecutor.tick calls planner._get_desc unconditionally after a swap.
        return "place the coke"

    def get_target_subtree(self, slot, index):
        return self._factory()

    def get_action_plan(self, slot, index):
        return list(self._plan)

    def get_facts(self, slot):
        return []

    def replan_target(self, slot, index, reason):
        pass


def _seed_blackboard(slots=(0,)):
    py_trees.blackboard.Blackboard.clear()
    w = py_trees.blackboard.Client(name="w")
    keys = [bb_keys.TASK_ID, bb_keys.CURRENT_ACTION, bb_keys.CURRENT_PARAMS, bb_keys.PLAN_INDEX,
            bb_keys.TARGET_INDEX, bb_keys.TARGET_REPLAN_COUNT, bb_keys.REPLAN_REQUEST,
            bb_keys.STATE_LOG, bb_keys.FACTS, bb_keys.DEFERRED_PRECONDITIONS,
            bb_keys.SUPERVISOR_STEP_DISPOSITION, bb_keys.CURRENT_TARGET]
    keys += [bb_keys.SAVED_TARGETS_PREFIX + str(s) for s in slots]
    for key in keys:
        w.register_key(key, access=Access.WRITE)
        w.register_key(key, access=Access.READ)
    w.set(bb_keys.TASK_ID, None, overwrite=True)
    for s in slots:
        w.set(bb_keys.SAVED_TARGETS_PREFIX + str(s),
              [{"id": "t0", "desc": "place the coke", "depends_on": []}], overwrite=True)
    w.set(bb_keys.CURRENT_ACTION, "place", overwrite=True)
    w.set(bb_keys.CURRENT_PARAMS, {"location": "kitchen_table"}, overwrite=True)
    w.set(bb_keys.PLAN_INDEX, 1, overwrite=True)
    w.set(bb_keys.TARGET_INDEX, 0, overwrite=True)
    w.set(bb_keys.TARGET_REPLAN_COUNT, 0, overwrite=True)
    w.set(bb_keys.REPLAN_REQUEST, {}, overwrite=True)
    w.set(bb_keys.STATE_LOG, [], overwrite=True)
    w.set(bb_keys.FACTS, [], overwrite=True)
    return w


def _telemetry(tmp_path):
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    return tele, tele.directory / "events.jsonl"


def _executor(slot, plan, subtree_factory=_leaf_target):
    # Budget >= tick count: exhausting the replan budget triggers _announce(),
    # which needs a real rclpy node (create_client). Keep this test ROS-free.
    ex = DynamicExecutor(f"executor task {slot + 1}", slot, _Planner(plan, subtree_factory),
                         max_replans_per_target=3)
    tree = py_trees.trees.BehaviourTree(ex)
    # DynamicExecutor reads SAVED_TARGETS_PREFIX+slot on its first tick.
    ex.setup(gpsr_tree=tree, node=object())
    return ex, tree


def _events(tmp_path, plan, subtree_factory=_leaf_target):
    tele, path = _telemetry(tmp_path)
    w = _seed_blackboard()
    _, tree = _executor(0, plan, subtree_factory)
    for _ in range(3):
        tree.tick()
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)
    return path, w


def _lines(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def test_swap_in_emits_plan_materialized_and_sets_task_id(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    path, w = _events(tmp_path, plan)
    mat = [e for e in _lines(path) if e["event_type"] == "plan.materialized"]
    assert mat and mat[0]["payload"]["steps"] == plan
    assert str(mat[0]["task_id"]).endswith("/task-1")
    assert w.get(bb_keys.TASK_ID) == mat[0]["task_id"]
    # Every tick swaps in a replan attempt: one materialization per attempt,
    # revisions counting up, and the task id stable for this executor.
    assert [e["payload"]["revision"] for e in mat] == [1, 2, 3]
    assert {e["task_id"] for e in mat} == {mat[0]["task_id"]}
    assert all(e["phase"] == "planning" for e in mat)


def test_failed_step_emits_step_finished_the_bench_can_parse(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    path, _ = _events(tmp_path, plan)
    results = parse_events(Path(path))
    # Three ticks = three failed attempts at the same step (replan each time).
    assert results[1].steps == [("place", "failed")] * 3
    failed = [e for e in _lines(path) if e["event_type"] == "step.finished"]
    assert failed[0]["phase"] == "execution"
    assert failed[0]["payload"]["step_index"] == 0
    assert failed[0]["payload"]["params"] == {"location": "kitchen_table"}
    assert failed[0]["payload"]["feedback"] == "navigation failed: kitchen_table unreachable"
    assert not [e for e in _lines(path) if e["event_type"] == "target.failed"]


# C1: every executor stamps its own task id, even when a previous slot's
# executor already wrote TASK_ID on the shared, process-global blackboard.
def test_each_slot_executor_stamps_its_own_task_id(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    tele, path = _telemetry(tmp_path)
    w = _seed_blackboard(slots=(0, 1))
    _, tree0 = _executor(0, plan)
    _, tree1 = _executor(1, plan)
    tree0.tick()
    assert str(w.get(bb_keys.TASK_ID)).endswith("/task-1")
    tree1.tick()
    assert str(w.get(bb_keys.TASK_ID)).endswith("/task-2")
    tree0.tick()
    assert str(w.get(bb_keys.TASK_ID)).endswith("/task-1")
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)

    events = [e for e in _lines(path)
              if e["event_type"] in ("plan.materialized", "step.finished")]
    slot_of = {}
    for e in events:
        if e["event_type"] == "plan.materialized":
            slot_of.setdefault(e["payload"]["slot"], set()).add(e["task_id"])
    assert all(t.endswith("/task-1") for t in slot_of[0])
    assert all(t.endswith("/task-2") for t in slot_of[1])
    results = parse_events(Path(path))
    assert results[1].steps == [("place", "failed")] * 2
    assert results[2].steps == [("place", "failed")]


# I2: a gate failure at target entry must not be attributed to the (stale)
# CURRENT_ACTION left over from the previous step/target.
def test_gate_failure_emits_target_failed_not_a_stale_step(tmp_path):
    plan = [{"action": "announce", "params": {"text": "done"}}]
    path, _ = _events(tmp_path, plan, subtree_factory=_gate_target)
    lines = _lines(path)
    assert not [e for e in lines if e["event_type"] == "step.finished"]
    failed = [e for e in lines if e["event_type"] == "target.failed"]
    assert len(failed) == 3
    assert failed[0]["phase"] == "execution"
    assert str(failed[0]["task_id"]).endswith("/task-1")
    assert failed[0]["payload"]["slot"] == 0
    assert failed[0]["payload"]["target_index"] == 0
    assert failed[0]["payload"]["reason"].startswith("precondition unmet: held(coke)")
    assert parse_events(Path(path))[1].steps == []


def test_non_gate_failure_without_current_action_emits_target_failed(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    tele, path = _telemetry(tmp_path)
    w = _seed_blackboard()
    w.set(bb_keys.CURRENT_ACTION, None, overwrite=True)
    _, tree = _executor(0, plan)
    tree.tick()
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)
    lines = _lines(path)
    assert not [e for e in lines if e["event_type"] == "step.finished"]
    failed = [e for e in lines if e["event_type"] == "target.failed"]
    assert len(failed) == 1
    assert failed[0]["payload"]["reason"] == "navigation failed: kitchen_table unreachable"


# T7: a supervisor same-target swap does not bump TARGET_REPLAN_COUNT, yet each
# materialization of the same target must carry a strictly increasing revision.
def test_same_target_swaps_get_strictly_increasing_revisions(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    tele, path = _telemetry(tmp_path)
    w = _seed_blackboard()
    ex, _ = _executor(0, plan)
    ex._swap_in(0)
    ex._swap_in(0)
    assert w.get(bb_keys.TARGET_REPLAN_COUNT) == 0
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)
    mat = [e for e in _lines(path) if e["event_type"] == "plan.materialized"]
    assert [e["payload"]["revision"] for e in mat] == [1, 2]


# Task 9: a replan that came back IDENTICAL to the failed plan is marked by the
# planner and must never be swapped in / executed again by the executor. Each
# skip burns one replan-budget slot and reports target.failed.
class _IdenticalAfterFirstPlanner(_Planner):
    def __init__(self, plan):
        super().__init__(plan)
        self._checks = 0

    def is_target_ready(self, slot, index):
        self._checks += 1
        return True

    def get_error(self, slot, index):
        if self._checks <= 1:
            return None
        return planner_mod.IDENTICAL_PLAN_ERROR_PREFIX + ": postcondition unmet: object_seen(x) (UNKNOWN)"


def test_identical_replan_is_skipped_not_executed(tmp_path):
    plan = [{"action": "place", "params": {"location": "kitchen_table"}}]
    tele, path = _telemetry(tmp_path)
    w = _seed_blackboard()
    ex = DynamicExecutor("executor task 1", 0, _IdenticalAfterFirstPlanner(plan),
                         max_replans_per_target=2)
    ex._announce = lambda text: None  # budget exhaustion announces via a real ROS node
    tree = py_trees.trees.BehaviourTree(ex)
    ex.setup(gpsr_tree=tree, node=object())
    replan_counts = []
    ticks = 0
    for _ in range(5):
        tree.tick()
        ticks += 1
        replan_counts.append(w.get(bb_keys.TARGET_REPLAN_COUNT))
        if ex.status != py_trees.common.Status.RUNNING:
            break  # a re-tick of a terminal executor re-activates it
    # execute(1) + skip(2) + skip-at-budget(3): the identical plan never ran again.
    assert ticks == 3
    tele.close(status="done")
    telemetry_mod.set_default_telemetry(None)

    lines = _lines(path)
    mat = [e for e in lines if e["event_type"] == "plan.materialized"]
    assert len(mat) == 1
    assert max(replan_counts) == 2
    log = w.get(bb_keys.STATE_LOG)
    assert sum("IDENTICAL_PLAN_SKIPPED" in entry for entry in log) == 2
    assert any("SKIPPED (replan budget exceeded)" in entry for entry in log)
    failed = [e for e in lines if e["event_type"] == "target.failed"]
    assert len(failed) >= 2
    assert all(e["payload"]["reason"].startswith(planner_mod.IDENTICAL_PLAN_ERROR_PREFIX) for e in failed)
    assert ex.status == py_trees.common.Status.FAILURE
