"""Regression tests for supervision on the two-layer dynamic executor."""

from datetime import datetime, timezone
from types import SimpleNamespace
import time

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
from behavior_tree.GPSR.supervision.contracts import NodeContractRegistry
from behavior_tree.GPSR.supervision.controller import MissionSupervisor
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    EffectRisk,
    NodeContract,
    RecoveryKind,
    SupervisionMode,
    SupervisorConfig,
)
from behavior_tree.GPSR.supervision.recovery import RecoveryMacroCompiler
from behavior_tree.GPSR.supervision.runtime import (
    SupervisedSubtaskSlot,
    set_default_supervisor,
)


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


class _ScriptedLeaf(py_trees.behaviour.Behaviour):
    """Minimal scripted effect leaf -- local stand-in for a real small tree,
    avoiding the ROS/action-client plumbing a real ``goto``/``announce``
    factory needs, since these multi-slot tests are about slot IDENTITY and
    GATING (M2), not about any particular action's real behaviour."""

    def __init__(self, name: str, result: Status, calls: list) -> None:
        super().__init__(name)
        self.result = result
        self.calls = calls

    def update(self) -> Status:
        self.calls.append(self.name)
        return self.result


def _seed_two_layer_blackboard() -> None:
    py_trees.blackboard.Blackboard.clear()
    for key, value in {
        "gpsr/task_id": "task",
        "gpsr/plan_revision": 1,
        "gpsr/plan_index": 0,
        "gpsr/command": "test command",
        "gpsr/current_params": {},
        "gpsr/state_log": [],
        "gpsr/arm_navigating": [0.0] * 7,
    }.items():
        py_trees.blackboard.Blackboard.set(key, value)


def test_multi_slot_dispatch_active_slot_consumes_idle_slot_never_ticks() -> None:
    """M2 (task-M, round-4 battery fix) multi-slot integration test: two
    ``SupervisedSubtaskSlot``s built with a FIXED (target_slot,
    target_index, step_index) identity -- exactly as
    ``GPSRPlanner.build_target_subtree`` builds one slot per plan step --
    sitting under a dispatcher-like memory ``Sequence``, driven the way the
    orchestrator drives one step at a time (``BtNode_MaterialiseStep`` bumps
    ``gpsr/plan_index`` before ticking each step's slot). A fixture
    intervention targeted at the ACTIVE (goto) slot is applied by it,
    cleanly (no crash, M1); the IDLE (announce, not yet reached) slot is
    never ticked at all under this architecture -- the Sequence's own
    memory gating structurally cannot reach it while goto is RUNNING, and
    its status (``INVALID``, unset) proves that directly."""
    _seed_two_layer_blackboard()
    calls: list[str] = []
    goto_checkpoint = (
        "task/target-0-0/plan-r1/step-0000:goto/tree-r1/goto/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                "checkpoint_id": goto_checkpoint,
                "verdict": "recoverable",
                "bt_assessment": "agree",
                "subtask_status": "not_achieved",
                "world_change": "none",
                "escalation": "local_recovery",
                "failure_category": "target_not_visible",
                "evidence": ["script"],
                "rationale": "runtime test",
                "confidence": 0.95,
            }
        ],
        recoveries=[
            {
                "checkpoint_id": goto_checkpoint,
                "issue_id": "filled-by-test",
                "strategy_id": "retry-goto",
                "kind": "scan_views",
                "arguments": {"angles": [[0, 0]], "perception_action": "goto"},
                "rationale": "retry",
                "expected_evidence": ["arrived"],
                "stop_conditions": ["arrived"],
            }
        ],
    )
    original = client.plan_local_recovery

    def local(snapshot, verification, issue_id):
        client._recoveries[0]["issue_id"] = issue_id
        return original(snapshot, verification, issue_id)

    client.plan_local_recovery = local
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _provider(),
        client,
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "_ScriptedLeaf", "navigation", EffectRisk.OBSERVATION, "arrived"
            )
        ]
    )
    compiler = RecoveryMacroCompiler(
        {
            RecoveryKind.SCAN_VIEWS: lambda proposal: py_trees.behaviours.Success(
                name=f"macro:{proposal.strategy_id}"
            )
        }
    )
    goto_calls = 0

    def goto_factory():
        nonlocal goto_calls
        goto_calls += 1
        status = Status.FAILURE if goto_calls == 1 else Status.SUCCESS
        return _ScriptedLeaf(f"goto-{goto_calls}", status, calls)

    goto_slot = SupervisedSubtaskSlot(
        action_name="goto",
        factory=goto_factory,
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
        target_slot=0,
        target_index=0,
        step_index=0,
    )
    announce_slot = SupervisedSubtaskSlot(
        action_name="announce",
        factory=lambda: _ScriptedLeaf("announce-idle", Status.SUCCESS, calls),
        supervisor=supervisor,
        registry=registry,
        target_slot=0,
        target_index=0,
        step_index=1,
    )
    dispatcher = py_trees.composites.Sequence("dispatcher-like", memory=True)
    dispatcher.add_child(goto_slot)
    dispatcher.add_child(announce_slot)
    try:
        # BtNode_MaterialiseStep's real effect for step 0: plan_index := 1.
        py_trees.blackboard.Blackboard.set("gpsr/plan_index", 1)
        for _ in range(300):
            dispatcher.tick_once()
            if goto_slot.tree_revision == 2:
                break
            time.sleep(0.005)
        assert goto_slot.tree_revision == 2, "goto slot never applied its recovery"
        assert announce_slot.status is Status.INVALID, (
            "the idle announce slot was ticked while goto was still RUNNING"
        )
        assert "announce-idle" not in calls
        # Drain to completion -- proves no crash across the whole sequence,
        # including the handoff into the (now genuinely active) announce
        # slot.
        for _ in range(300):
            dispatcher.tick_once()
            if dispatcher.status is Status.SUCCESS:
                break
            time.sleep(0.005)
        assert dispatcher.status is Status.SUCCESS
        assert calls[-1] == "announce-idle"
    finally:
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
