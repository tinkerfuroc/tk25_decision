"""Regression tests for supervision on the two-layer dynamic executor."""

from datetime import datetime, timezone
import json
from types import SimpleNamespace
import time
from unittest.mock import patch

import py_trees
from py_trees.common import Access, Status

from behavior_tree.GPSR import telemetry as telemetry_mod
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


class _DelayedSuccessLeaf(py_trees.behaviour.Behaviour):
    """Like ``_ScriptedLeaf``, but RUNNING for ``running_ticks`` updates
    before reporting SUCCESS -- a realistic stand-in for a retried action
    that takes more than zero ticks to complete (Q1, task-Q round-6: with
    recovery-retry checkpoints suppressed, a retry that succeeds in ZERO
    ticks now completes visibly in the SAME tick its recovery is applied,
    since there is no longer a redundant async verify of the retry's own
    outcome to wait on -- this leaf keeps this test's "still genuinely
    executing" window meaningful without relying on that removed latency)."""

    def __init__(self, name: str, running_ticks: int, calls: list) -> None:
        super().__init__(name)
        self._remaining = running_ticks
        self.calls = calls

    def update(self) -> Status:
        self.calls.append(self.name)
        if self._remaining > 0:
            self._remaining -= 1
            return Status.RUNNING
        return Status.SUCCESS


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
        if goto_calls == 1:
            return _ScriptedLeaf(f"goto-{goto_calls}", Status.FAILURE, calls)
        # Q1 (task-Q, round-6): RUNNING for one tick before SUCCEEDING, so
        # the retry is still genuinely in-flight for at least one tick
        # after `_apply_intervention` builds it -- see `_DelayedSuccessLeaf`.
        return _DelayedSuccessLeaf(f"goto-{goto_calls}", 1, calls)

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


# ---------------------------------------------------------------------------
# Task Q3 (round-6, supervision-economics fix): supervisor replans stop
# resetting the target's failure accounting after a cap. Events forensics on
# the supervised 019 run: RecoveryLedger's own cap fired correctly, triggering
# a global replan, but `_on_target_failure`'s supervisor branch unconditionally
# reset TARGET_REPLAN_COUNT and returned -- bypassing the `replans >
# max_replans` block, the ONLY emitter of target.failed. While the supervisor
# kept replanning the same failing target, target.failed was structurally
# unreachable and the bench timed out.


class _Q3Planner:
    """Always requests a supervisor replan for the SAME target -- drives
    ``DynamicExecutor._on_target_failure``'s supervisor branch repeatedly to
    exercise the Q3 replan cap. ``replace_target_plan``/``replan_target``
    match the REAL ``GPSRPlanner`` signatures (unlike the pre-existing
    ``_FakePlanner`` above, whose ``replace_target_plan`` is missing
    ``completed_steps`` -- a known baseline failure this test does not rely
    on or repeat)."""

    def __init__(self) -> None:
        self.plan = [{"action": "goto", "params": {"location": "kitchen"}}]
        self.replacements: list = []
        self.replan_calls = 0

    def _get_desc(self, slot, index):
        return "go to the kitchen"

    def is_target_ready(self, slot, index):
        return True

    def get_action_plan(self, slot, index):
        return list(self.plan)

    def get_target_subtree(self, slot, index):
        seq = py_trees.composites.Sequence("target", memory=True)
        seq.add_children(
            [_RequestSupervisorReplan("request replan"), BtNode_SupervisorBarrier()]
        )
        return seq

    def replace_target_plan(self, slot, index, plan, reason, completed_steps=None):
        self.plan = list(plan)
        self.replacements.append((slot, index, list(plan), reason))

    def replan_target(self, slot, index, reason, completed_steps=None):
        self.replan_calls += 1


def _seed_q3_targets() -> None:
    py_trees.blackboard.Blackboard.clear()
    bb = py_trees.blackboard.Client(name="q3_supervisor_replan_cap_test")
    for key in (bb_keys.SAVED_TARGETS_PREFIX + "0", bb_keys.TARGETS):
        bb.register_key(key, access=Access.WRITE)
    bb.set(bb_keys.SAVED_TARGETS_PREFIX + "0", ["go to the kitchen"], overwrite=True)
    bb.set(bb_keys.TARGETS, ["go to the kitchen"], overwrite=True)


def test_supervisor_replan_cap_lets_target_failed_become_reachable_again(
    tmp_path,
) -> None:
    # Up to GPSR_SUPERVISION_MAX_SUPERVISOR_REPLANS (2 here), a
    # supervisor-level replan keeps today's fresh-start semantics
    # (TARGET_REPLAN_COUNT reset to 0, planner.replace_target_plan called).
    # Beyond the cap, the supervisor branch no longer resets it and falls
    # through to the NORMAL replan machinery -- counting toward
    # max_replans_per_target, so target.failed becomes reachable again
    # instead of looping forever.
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    _seed_q3_targets()
    planner = _Q3Planner()
    root = py_trees.composites.Sequence("root", memory=True)
    with patch.dict("os.environ", {"GPSR_SUPERVISION_MAX_SUPERVISOR_REPLANS": "2"}):
        # A tiny normal-replan budget (1): once the cap trips and the
        # normal machinery takes over, it should not take many further
        # supervisor-replan cycles to exhaust it and reach target.failed.
        executor = DynamicExecutor("executor", 0, planner, max_replans_per_target=1)
    executor._announce = lambda text: None
    root.add_child(executor)
    tree = py_trees.trees.BehaviourTree(root)
    node = SimpleNamespace(get_name=lambda: "stub")
    tree.setup(timeout=15, node=node, gpsr_tree=tree)

    try:
        for _ in range(20):
            tree.tick()
            if executor.status != Status.RUNNING:
                break
        assert executor.status == Status.FAILURE, (
            "the perpetually-failing target must eventually reach "
            "target.failed, not loop forever"
        )
        # Two fresh starts under the cap.
        assert len(planner.replacements) == 2
        # Beyond the cap: the normal machinery replanned/skipped instead of
        # the supervisor's own replacement plan.
        assert planner.replan_calls >= 1
    finally:
        tele.close(status="done")
        telemetry_mod.set_default_telemetry(None)

    lines = [
        json.loads(line)
        for line in (tele.directory / "events.jsonl").read_text().splitlines()
        if line.strip()
    ]
    cap_events = [e for e in lines if e["event_type"] == "supervision.replan_cap"]
    assert len(cap_events) == 1, "telemetry must fire exactly once, on the trip"
    assert cap_events[0]["payload"]["supervisor_replans"] == 3
    assert cap_events[0]["payload"]["max_supervisor_replans"] == 2
    failed = [e for e in lines if e["event_type"] == "target.failed"]
    assert failed, "target.failed must be reachable once the cap trips"


def test_supervisor_replan_cap_is_isolated_per_target(tmp_path) -> None:
    # Per-target isolation: target A tripping the cap must not affect
    # target B's own (fresh) allowance -- SUPERVISOR_REPLAN_COUNT resets on
    # every genuine target advance (success, skip, or a new command).
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    py_trees.blackboard.Blackboard.clear()
    bb = py_trees.blackboard.Client(name="q3_per_target_isolation_test")
    for key in (bb_keys.SAVED_TARGETS_PREFIX + "0", bb_keys.TARGETS):
        bb.register_key(key, access=Access.WRITE)
    bb.set(
        bb_keys.SAVED_TARGETS_PREFIX + "0",
        ["go to the kitchen", "go to the lounge"],
        overwrite=True,
    )
    bb.set(
        bb_keys.TARGETS, ["go to the kitchen", "go to the lounge"], overwrite=True
    )
    planner = _Q3Planner()
    root = py_trees.composites.Sequence("root", memory=True)
    with patch.dict("os.environ", {"GPSR_SUPERVISION_MAX_SUPERVISOR_REPLANS": "2"}):
        executor = DynamicExecutor("executor", 0, planner, max_replans_per_target=1)
    executor._announce = lambda text: None
    root.add_child(executor)
    tree = py_trees.trees.BehaviourTree(root)
    node = SimpleNamespace(get_name=lambda: "stub")
    tree.setup(timeout=15, node=node, gpsr_tree=tree)

    try:
        for _ in range(30):
            tree.tick()
            if executor.status != Status.RUNNING:
                break
        assert executor.status == Status.FAILURE
    finally:
        tele.close(status="done")
        telemetry_mod.set_default_telemetry(None)

    lines = [
        json.loads(line)
        for line in (tele.directory / "events.jsonl").read_text().splitlines()
        if line.strip()
    ]
    cap_events = [e for e in lines if e["event_type"] == "supervision.replan_cap"]
    # Target A (index 0) trips the cap once; target B (index 1) gets its
    # OWN fresh allowance and, driven by the same always-fail planner, trips
    # its own cap once too -- never inheriting A's already-spent count.
    assert [e["payload"]["target_index"] for e in cap_events] == [0, 1]


# ---------------------------------------------------------------------------
# Task R-2 (round-6 review fix, MEDIUM): Q3's replan cap must exempt
# abort_and_report. An abort is terminal (always sets state DONE right
# there) and structurally cannot contribute to the "supervisor keeps
# replanning the same failing target forever" churn Q3 exists to cap -- so
# it must always be honored, complete with its operator_message, regardless
# of how many prior supervisor replans this target has already consumed.


class _AbortRequest(py_trees.behaviour.Behaviour):
    def update(self) -> Status:
        py_trees.blackboard.Blackboard.set(
            bb_keys.REPLAN_REQUEST,
            {
                "level": "supervisor",
                "action": "abort_and_report",
                "reason": "supervisor gave up",
                "operator_message": "I am sorry, I could not do this, giving up.",
            },
        )
        return Status.SUCCESS


class _Q3AbortPlanner:
    """Requests ordinary supervisor replans for the first ``cap`` attempts
    (exhausting the free "fresh start" budget), then requests
    ``abort_and_report`` -- proving the abort is still honored, with its
    operator_message intact, PAST the cap."""

    def __init__(self, cap: int) -> None:
        self.plan = [{"action": "goto", "params": {"location": "kitchen"}}]
        self.replacements: list = []
        self.replan_calls = 0
        self._cap = cap
        self._attempts = 0

    def _get_desc(self, slot, index):
        return "go to the kitchen"

    def is_target_ready(self, slot, index):
        return True

    def get_action_plan(self, slot, index):
        return list(self.plan)

    def get_target_subtree(self, slot, index):
        self._attempts += 1
        seq = py_trees.composites.Sequence("target", memory=True)
        request: py_trees.behaviour.Behaviour
        if self._attempts > self._cap:
            request = _AbortRequest("request abort")
        else:
            request = _RequestSupervisorReplan("request replan")
        seq.add_children([request, BtNode_SupervisorBarrier()])
        return seq

    def replace_target_plan(self, slot, index, plan, reason, completed_steps=None):
        self.plan = list(plan)
        self.replacements.append((slot, index, list(plan), reason))

    def replan_target(self, slot, index, reason, completed_steps=None):
        self.replan_calls += 1


def test_supervisor_replan_cap_exempts_abort_and_report(tmp_path) -> None:
    tele = telemetry_mod.GpsrTelemetry(tmp_path, enabled=True, trajectory_id="traj")
    telemetry_mod.set_default_telemetry(tele)
    _seed_q3_targets()
    planner = _Q3AbortPlanner(cap=2)
    root = py_trees.composites.Sequence("root", memory=True)
    with patch.dict("os.environ", {"GPSR_SUPERVISION_MAX_SUPERVISOR_REPLANS": "2"}):
        executor = DynamicExecutor("executor", 0, planner, max_replans_per_target=1)
    announced: list = []
    executor._announce = lambda text: announced.append(text)
    root.add_child(executor)
    tree = py_trees.trees.BehaviourTree(root)
    node = SimpleNamespace(get_name=lambda: "stub")
    tree.setup(timeout=15, node=node, gpsr_tree=tree)

    try:
        for _ in range(10):
            tree.tick()
            if executor.status != Status.RUNNING:
                break
        # A supervisor abort maps to task-level SUCCESS (a reported, told-
        # the-operator give-up), not a skip/FAILURE.
        assert executor.status == Status.SUCCESS
        # The operator_message reached the operator intact -- not silently
        # replaced by the generic "I could not complete X..." skip speech.
        assert announced == ["I am sorry, I could not do this, giving up."]
        # Two ordinary replans happened first (under the cap) -- proving
        # the abort really was requested PAST the cap, not merely because
        # the cap was never reached.
        assert len(planner.replacements) == 2
    finally:
        tele.close(status="done")
        telemetry_mod.set_default_telemetry(None)

    lines = [
        json.loads(line)
        for line in (tele.directory / "events.jsonl").read_text().splitlines()
        if line.strip()
    ]
    # The abort branch never touches SUPERVISOR_REPLAN_COUNT at all (it is
    # checked before that counter is even read) -- no cap-trip telemetry,
    # unlike test_supervisor_replan_cap_lets_target_failed_become_reachable_again.
    assert [e for e in lines if e["event_type"] == "supervision.replan_cap"] == []
