"""Regression tests for supervision on the two-layer dynamic executor."""

from datetime import datetime, timezone
from types import SimpleNamespace

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR.orchestrator import (
    BtNode_SupervisorBarrier,
    DynamicExecutor,
)
from behavior_tree.GPSR.planner import GPSRPlanner
from behavior_tree.GPSR.small_trees import bb_keys
from behavior_tree.GPSR.supervision.clients import ScriptedSupervisorClient
from behavior_tree.GPSR.supervision.context import StaticContextProvider
from behavior_tree.GPSR.supervision.controller import MissionSupervisor
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    SupervisionMode,
    SupervisorConfig,
)
from behavior_tree.GPSR.supervision.runtime import set_default_supervisor


def _provider() -> StaticContextProvider:
    captured_at = datetime.now(timezone.utc).isoformat()
    return StaticContextProvider(
        tuple(
            ArtifactRef.absent(role, captured_at, "unit test")
            for role in ("front_camera", "wrist_camera", "map", "arm")
        )
    )


def test_dynamic_target_subtree_uses_supervised_action_slots() -> None:
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.SHADOW),
        _provider(),
        ScriptedSupervisorClient(),
    )
    set_default_supervisor(supervisor)
    try:
        subtree = GPSRPlanner().build_target_subtree(
            0,
            0,
            [{"action": "announce", "params": {"text": "hello"}}],
        )
        assert any(
            type(node).__name__ == "SupervisedSubtaskSlot"
            for node in subtree.iterate()
        )
        assert any(
            isinstance(node, BtNode_SupervisorBarrier)
            for node in subtree.iterate()
        )
    finally:
        set_default_supervisor(None)
        supervisor.close()


def test_build_target_subtree_step_factory_builds_a_fresh_subtree_every_call() -> None:
    """M1 regression (task-M, round-4 battery fix): `build_target_subtree`
    used to build one small_tree object per step and hand
    `SupervisedSubtaskSlot` `lambda: small_tree` -- a closure returning that
    SAME cached instance on every call. `_materialize_subtask` calls the
    slot's factory a second time on every `local_recovery` intervention;
    with the old closure this returned the slot's own already-attached
    child, and building the replacement Sequence crashed inside py_trees
    `add_child` with "already has parent" (the verified crash, sim runs
    s2026-009-tellCatPropOnPlcmt etc.). Drive the REAL planner wiring (not a
    hand-written factory) and assert two fresh, parentless subtrees."""
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.SHADOW),
        _provider(),
        ScriptedSupervisorClient(),
    )
    set_default_supervisor(supervisor)
    try:
        subtree = GPSRPlanner().build_target_subtree(
            0,
            0,
            [{"action": "announce", "params": {"text": "hello"}}],
        )
        slot = next(
            node
            for node in subtree.iterate()
            if type(node).__name__ == "SupervisedSubtaskSlot"
        )
        first = slot._materialize_subtask()
        second = slot._materialize_subtask()
        assert first is not second
        assert first.parent is None
        assert second.parent is None
    finally:
        set_default_supervisor(None)
        supervisor.close()


class _RequestSupervisorReplan(py_trees.behaviour.Behaviour):
    def update(self) -> Status:
        py_trees.blackboard.Blackboard.set(
            bb_keys.REPLAN_REQUEST,
            {
                "level": "supervisor",
                "action": "replan_remaining",
                "reason": "postcondition was not achieved",
                "replacement_plan": [
                    {"action": "announce", "params": {"text": "recovered"}}
                ],
            },
        )
        return Status.SUCCESS


class _FakePlanner:
    def __init__(self) -> None:
        self.plan = [{"action": "goto", "params": {"location": "kitchen"}}]
        self.replacements = []

    def _get_desc(self, slot, index):
        return "go to the kitchen"

    def get_action_plan(self, slot, index):
        return list(self.plan)

    def get_target_subtree(self, slot, index):
        seq = py_trees.composites.Sequence("target", memory=True)
        if self.replacements:
            seq.add_child(py_trees.behaviours.Success("replacement succeeded"))
        else:
            seq.add_children(
                [
                    _RequestSupervisorReplan("request replan"),
                    BtNode_SupervisorBarrier(),
                ]
            )
        return seq

    def replace_target_plan(self, slot, index, plan, reason):
        self.plan = list(plan)
        self.replacements.append((slot, index, list(plan), reason))

    def replan_target(self, *args):
        raise AssertionError("the supervisor replacement must not call the LLM planner")


def test_supervisor_replacement_swaps_at_dynamic_executor_tick_boundary() -> None:
    planner = _FakePlanner()
    root = py_trees.composites.Sequence("root", memory=True)
    executor = DynamicExecutor("executor", 0, planner)
    root.add_child(executor)
    tree = py_trees.trees.BehaviourTree(root)
    node = SimpleNamespace(get_name=lambda: "stub")
    tree.setup(timeout=15, node=node, gpsr_tree=tree)

    bb = py_trees.blackboard.Client(name="two_layer_supervision_test")
    for key in (
        bb_keys.SAVED_TARGETS_PREFIX + "0",
        bb_keys.TARGETS,
    ):
        bb.register_key(key, access=Access.WRITE)
    bb.set(
        bb_keys.SAVED_TARGETS_PREFIX + "0",
        ["go to the kitchen"],
        overwrite=True,
    )
    bb.set(bb_keys.TARGETS, ["go to the kitchen"], overwrite=True)

    for _ in range(10):
        tree.tick()
        if tree.root.status is Status.SUCCESS:
            break

    assert tree.root.status is Status.SUCCESS
    assert len(planner.replacements) == 1
    assert planner.replacements[0][2][0]["action"] == "announce"
    assert py_trees.blackboard.Blackboard.get(bb_keys.REPLAN_REQUEST) == {}
