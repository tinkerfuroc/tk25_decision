from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import threading
import time

import py_trees
from geometry_msgs.msg import PoseStamped

from behavior_tree.GPSR.supervision.clients import ScriptedSupervisorClient
from behavior_tree.GPSR.supervision.context import StaticContextProvider
from behavior_tree.GPSR.supervision.contracts import NodeContractRegistry
from behavior_tree.GPSR.supervision.controller import MissionSupervisor
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    EffectRisk,
    NodeContract,
    SuccessMode,
    SupervisionMode,
    SupervisorConfig,
    BtAssessment,
    Escalation,
    SubtaskStatus,
    Verdict,
    VerificationDecision,
    WorldChange,
)
from behavior_tree.GPSR.supervision.runtime import SupervisedSubtaskSlot
from behavior_tree.GPSR.supervision.recovery import RecoveryMacroCompiler
from behavior_tree.GPSR.supervision.models import RecoveryKind
from behavior_tree.GPSR.small_trees import ACTION_FACTORIES, bb_keys


class FakeEffect(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, result: py_trees.common.Status, calls: list[str]):
        super().__init__(name)
        self.result = result
        self.calls = calls

    def update(self):
        self.calls.append(self.name)
        return self.result


def _provider():
    now = datetime.now(timezone.utc).isoformat()
    return StaticContextProvider(
        tuple(
            ArtifactRef.absent(role, now, "runtime unit test")
            for role in ("front_camera", "wrist_camera", "map", "arm")
        )
    )


def _decision(checkpoint_id: str, *, assessment="agree", verdict="all_clear"):
    return {
        "checkpoint_id": checkpoint_id,
        "verdict": verdict,
        "bt_assessment": assessment,
        "subtask_status": "achieved",
        "world_change": "none",
        "escalation": "none",
        "failure_category": "",
        "evidence": ["script"],
        "rationale": "runtime test",
        "confidence": 0.95,
    }


def _seed_blackboard():
    py_trees.blackboard.Blackboard.clear()
    values = {
        "gpsr/task_id": "task",
        "gpsr/plan_revision": 1,
        "gpsr/plan_index": 1,
        "gpsr/command": "test command",
        "gpsr/current_params": {},
        "gpsr/state_log": [],
        "gpsr/arm_navigating": [0.0] * 7,
    }
    for key, value in values.items():
        py_trees.blackboard.Blackboard.set(key, value)


def test_failure_waits_for_verifier_and_false_failure_becomes_success():
    _seed_blackboard()
    calls: list[str] = []
    checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[_decision(checkpoint, assessment="false_failure")]
    )
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _provider(),
        client,
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "FakeEffect",
                "perception",
                EffectRisk.OBSERVATION,
                "effect happened",
            )
        ]
    )
    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=lambda: FakeEffect("fail", py_trees.common.Status.FAILURE, calls),
        supervisor=supervisor,
        registry=registry,
    )
    try:
        for _ in range(100):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        assert calls == ["fail"]
    finally:
        supervisor.close()


def test_hybrid_blocks_irreversible_next_effect_until_prior_verdict():
    _seed_blackboard()
    calls: list[str] = []
    release = threading.Event()
    first_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/0/activation-0001"
    )
    second_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/1/activation-0001"
    )

    def delayed():
        release.wait(2)
        return _decision(first_checkpoint)

    client = ScriptedSupervisorClient(
        verifications=[delayed, _decision(second_checkpoint)]
    )
    supervisor = MissionSupervisor(
        SupervisorConfig(
            mode=SupervisionMode.ACTIVE,
            success_mode=SuccessMode.HYBRID,
        ),
        _provider(),
        client,
        executor=ThreadPoolExecutor(max_workers=1),
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "ObserveEffect", "perception", EffectRisk.OBSERVATION, "observed"
            ),
            NodeContract(
                "IrreversibleEffect",
                "manipulation",
                EffectRisk.IRREVERSIBLE,
                "placed",
                allow_local_recovery=False,
            ),
        ]
    )

    class ObserveEffect(FakeEffect):
        pass

    class IrreversibleEffect(FakeEffect):
        pass

    def factory():
        return py_trees.composites.Sequence(
            "subtask",
            memory=True,
            children=[
                ObserveEffect("observe", py_trees.common.Status.SUCCESS, calls),
                IrreversibleEffect("place", py_trees.common.Status.SUCCESS, calls),
            ],
        )

    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=factory,
        supervisor=supervisor,
        registry=registry,
    )
    try:
        slot.tick_once()
        assert calls == ["observe"]
        assert slot.status is py_trees.common.Status.RUNNING
        release.set()
        for _ in range(100):
            slot.tick_once()
            if "place" in calls:
                break
            time.sleep(0.005)
        assert calls == ["observe", "place"]
        for _ in range(100):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
    finally:
        release.set()
        supervisor.close()


def test_optimistic_mode_starts_next_effect_without_waiting():
    _seed_blackboard()
    calls: list[str] = []
    release = threading.Event()
    first_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/0/activation-0001"
    )
    second_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/1/activation-0001"
    )

    def delayed():
        release.wait(2)
        return _decision(first_checkpoint)

    client = ScriptedSupervisorClient(
        verifications=[delayed, _decision(second_checkpoint)]
    )
    supervisor = MissionSupervisor(
        SupervisorConfig(
            mode=SupervisionMode.ACTIVE,
            success_mode=SuccessMode.OPTIMISTIC,
        ),
        _provider(),
        client,
        executor=ThreadPoolExecutor(max_workers=2),
    )

    class ObserveEffect(FakeEffect):
        pass

    class IrreversibleEffect(FakeEffect):
        pass

    registry = NodeContractRegistry(
        [
            NodeContract(
                "ObserveEffect", "perception", EffectRisk.OBSERVATION, "observed"
            ),
            NodeContract(
                "IrreversibleEffect",
                "manipulation",
                EffectRisk.IRREVERSIBLE,
                "placed",
                allow_local_recovery=False,
            ),
        ]
    )
    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=lambda: py_trees.composites.Sequence(
            "subtask",
            memory=True,
            children=[
                ObserveEffect("observe", py_trees.common.Status.SUCCESS, calls),
                IrreversibleEffect("place", py_trees.common.Status.SUCCESS, calls),
            ],
        ),
        supervisor=supervisor,
        registry=registry,
    )
    try:
        slot.tick_once()
        assert calls == ["observe", "place"]
    finally:
        release.set()
        supervisor.close()


def test_all_registered_effect_leaves_are_wrapped_in_gpsr_action_factories():
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.SHADOW),
        _provider(),
        ScriptedSupervisorClient(),
    )
    registry = NodeContractRegistry(
        []  # populated from the production registry below
    )
    from behavior_tree.GPSR.supervision.contracts import default_node_contracts

    registry = default_node_contracts()
    raw_count = 0
    wrapped_count = 0
    uncontracted_internal = {
        "BtNode_BlackboardSet",
        "BtNode_BuildPersonPrompt",
        "BtNode_CheckBBContains",
        "BtNode_CheckBBKeySet",
        "BtNode_CheckBBTrue",
        "BtNode_CheckGraspAllowed",
        "BtNode_CountDetections",
        "BtNode_ExtractDetection",
        "BtNode_LLMQuery",
        "BtNode_RegisterLabeledPose",
        "BtNode_SetReportInfo",
        "BtNode_VLMQuery",
        "BtNode_WaitTicks",
        "Timer",
    }
    uncontracted = set()
    try:
        for action_name, factory in ACTION_FACTORIES.items():
            raw = factory()
            raw_count += sum(
                1 for node in raw.iterate() if registry.contract_for(node) is not None
            )
            uncontracted.update(
                type(node).__name__
                for node in raw.iterate()
                if not node.children
                and registry.contract_for(node) is None
                and type(node).__name__ not in uncontracted_internal
            )
            slot = SupervisedSubtaskSlot(
                action_name=action_name,
                factory=factory,
                supervisor=supervisor,
                registry=registry,
            )
            wrapped_count += sum(
                1
                for node in slot.iterate()
                if type(node).__name__ == "SupervisedEffect"
            )
        assert raw_count > 20
        assert wrapped_count == raw_count
        assert uncontracted == set()
    finally:
        supervisor.close()


def test_actual_goto_factory_runs_with_fixture_context_and_full_mock(
    monkeypatch, tmp_path: Path
):
    full_mock = Path(__file__).parents[1] / "config" / "full_mock.json"
    monkeypatch.setenv("BT_MOCK_CONFIG", str(full_mock))
    from behavior_tree.config import BehaviorTreeConfig
    from behavior_tree.GPSR.supervision.context import FixtureContextProvider

    BehaviorTreeConfig()._load_mock_config(force=True)
    _seed_blackboard()
    py_trees.blackboard.Blackboard.set(bb_keys.COMMAND, "go to the kitchen")
    py_trees.blackboard.Blackboard.set(
        bb_keys.CURRENT_PARAMS, {"location": "kitchen"}
    )
    py_trees.blackboard.Blackboard.set(bb_keys.TARGET_LOCATION, "kitchen")
    py_trees.blackboard.Blackboard.set(bb_keys.TARGET_POSE, PoseStamped())

    class AutoClearClient:
        def __init__(self):
            self.checkpoints = []

        def verify(self, snapshot):
            self.checkpoints.append(snapshot)
            return VerificationDecision(
                checkpoint_id=snapshot.request.checkpoint_id,
                verdict=Verdict.ALL_CLEAR,
                bt_assessment=BtAssessment.AGREE,
                subtask_status=SubtaskStatus.ACHIEVED,
                world_change=WorldChange.NONE,
                escalation=Escalation.NONE,
                failure_category="",
                evidence=("full-mock node status",),
                rationale="scripted full-mock verification",
                confidence=1.0,
            )

        def plan_local_recovery(self, *args):
            raise AssertionError("local recovery was not expected")

        def plan_global_replan(self, *args):
            raise AssertionError("global replan was not expected")

    class NullLogger:
        def __getattr__(self, name):
            return lambda *args, **kwargs: None

    class MockRosNode:
        def get_logger(self):
            return NullLogger()

    client = AutoClearClient()
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        FixtureContextProvider(output_dir=tmp_path),
        client,
    )
    slot = SupervisedSubtaskSlot(
        action_name="goto",
        factory=ACTION_FACTORIES["goto"],
        supervisor=supervisor,
    )
    try:
        for node in slot.iterate():
            node.setup(node=MockRosNode())
        for _ in range(200):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        assert len(client.checkpoints) >= 3
        assert all(
            {artifact.role for artifact in checkpoint.artifacts}
            == {"front_camera", "wrist_camera", "map", "arm"}
            for checkpoint in client.checkpoints
        )
    finally:
        supervisor.close()


def test_local_recovery_replaces_and_retries_the_current_subtask():
    _seed_blackboard()
    calls: list[str] = []
    factory_calls = 0
    first_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    retry_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r2/"
        "demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                **_decision(
                    first_checkpoint,
                    verdict="recoverable",
                ),
                "subtask_status": "not_achieved",
                "escalation": "local_recovery",
                "failure_category": "target_not_visible",
            },
            _decision(retry_checkpoint),
        ],
        recoveries=[
            {
                "checkpoint_id": first_checkpoint,
                "issue_id": "filled-by-test",
                "strategy_id": "look-left",
                "kind": "scan_views",
                "arguments": {
                    "angles": [[-30, 10]],
                    "perception_action": "find_object",
                },
                "rationale": "try a new view",
                "expected_evidence": ["target visible"],
                "stop_conditions": ["target absent"],
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
                "FakeEffect",
                "perception",
                EffectRisk.OBSERVATION,
                "target was observed",
            )
        ]
    )

    def factory():
        nonlocal factory_calls
        factory_calls += 1
        status = (
            py_trees.common.Status.FAILURE
            if factory_calls == 1
            else py_trees.common.Status.SUCCESS
        )
        return FakeEffect(f"scan-{factory_calls}", status, calls)

    compiler = RecoveryMacroCompiler(
        {
            RecoveryKind.SCAN_VIEWS: lambda proposal: py_trees.behaviours.Success(
                name=f"macro:{proposal.strategy_id}"
            )
        }
    )
    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=factory,
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    try:
        for _ in range(300):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        assert calls == ["scan-1", "scan-2"]
        attempts = supervisor.ledger.snapshot()
        assert len(attempts) == 1
        assert attempts[0]["succeeded"] is True
        assert slot.tree_revision == 2
    finally:
        supervisor.close()


def test_local_recovery_exhausts_after_three_identical_failures_and_escalates():
    """F1.1 regression: the 005 pattern (same issue, same failure_category,
    repeated) through the REAL SupervisedSubtaskSlot rebuild path (not just
    MissionSupervisor.submit calls, which mark each attempt's result
    synchronously and so cannot see the lag this test catches) -- the
    RecoveryLedger must hit ``exhausted()`` at the 3rd FAILED distinct
    attempt and escalate globally instead of ever proposing a 4th distinct
    strategy.

    Investigation finding (see task-F-report.md F1.1): this test FAILED
    against the pre-fix code -- not from issue_identity() hashing (stable
    across rebuild cycles: subtask_goal/effect/failure_category/target/
    location are all blackboard/contract-derived and constant for the same
    step) but from a timing lag: ``SupervisedSubtaskSlot`` only calls
    ``recovery_finished`` (marking an attempt failed) lazily, when it moves
    on to apply the NEXT intervention -- one whole rebuild cycle after the
    fact. ``MissionSupervisor._start_local`` checked ``exhausted()`` BEFORE
    that lag caught up, so a 4th (proposal "right" running as tree-r4, then
    a 4th distinct strategy) could slip past the 3-attempt cap. Fixed by
    ``RecoveryLedger.mark_unresolved_failed`` (recovery.py), called from
    ``_start_local`` before its exhaustion check: a fresh checkpoint for the
    SAME issue is itself the evidence the still-"executing" attempt failed.

    Separately (and unconditionally, per F1's requirement 2): run 005 itself
    exercised NONE of this -- supervisor.* telemetry never fired anywhere in
    that run's events.jsonl/orchestrator.log, meaning GPSR_SUPERVISION_MODE
    was not ACTIVE, so this ledger/controller/runtime path was never
    exercised in that specific run at all. See the wall-clock budget test
    below and task-F-report.md for the residual gap that leaves.
    """
    _seed_blackboard()
    calls: list[str] = []
    subtask_id = "task/plan-r1/step-0000:demo"
    # tree-r1 (initial) then 3 local_recovery rebuilds -> tree-r2, r3, r4;
    # the 4th checkpoint (tree-r4, running the 3rd/"right" proposal) is what
    # finally trips exhausted() -- so it needs its own verification too, even
    # though no 4th recovery proposal should ever be requested for it.
    checkpoints = [
        f"{subtask_id}/tree-r{revision}/demo/root/activation-0001"
        for revision in (1, 2, 3, 4)
    ]
    verifications = [
        {
            **_decision(checkpoint, verdict="recoverable"),
            "subtask_status": "not_achieved",
            "escalation": "local_recovery",
            "failure_category": "target_not_visible",
        }
        for checkpoint in checkpoints
    ]
    recoveries = [
        {
            "checkpoint_id": checkpoint,
            "issue_id": "filled-by-test",
            "strategy_id": name,
            "kind": "scan_views",
            "arguments": {"angles": [[pan, 10]], "perception_action": "find_object"},
            "rationale": "try a distinct viewpoint",
            "expected_evidence": ["target visible"],
            "stop_conditions": ["target absent"],
        }
        for checkpoint, (name, pan) in zip(
            checkpoints[:3], (("left", -35), ("center", 0), ("right", 35))
        )
    ]
    client = ScriptedSupervisorClient(
        verifications=verifications,
        recoveries=recoveries,
        global_plans=[
            {
                "checkpoint_id": checkpoints[3],
                "action": "abort_and_report",
                "replacement_plan": [],
                "preserved_completed_steps": 0,
                "relaxed_constraints": [],
                "rationale": "Three distinct scans failed.",
                "operator_message": "I could not find the target.",
            }
        ],
    )
    original = client.plan_local_recovery
    seen_issue_ids: list[str] = []

    def local(snapshot, verification, issue_id):
        client._recoveries[0]["issue_id"] = issue_id
        seen_issue_ids.append(issue_id)
        return original(snapshot, verification, issue_id)

    client.plan_local_recovery = local
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE, max_recoveries=3),
        _provider(),
        client,
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "FakeEffect",
                "perception",
                EffectRisk.OBSERVATION,
                "target was observed",
            )
        ]
    )

    activations = [0]

    def factory():
        activations[0] += 1
        return FakeEffect(f"scan-{activations[0]}", py_trees.common.Status.FAILURE, calls)

    compiler = RecoveryMacroCompiler(
        {
            RecoveryKind.SCAN_VIEWS: lambda proposal: py_trees.behaviours.Success(
                name=f"macro:{proposal.strategy_id}"
            )
        }
    )
    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=factory,
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    try:
        # 4 async round trips (verify+local x3, plus tree-r4's own verify) --
        # more headroom than the single-cycle tests above need.
        for _ in range(1500):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        # tree-r1 (initial) + 3 local_recovery rebuilds (tree-r2, r3, r4) --
        # never a 4th distinct strategy proposed once the 3rd ("right",
        # running as tree-r4) also failed.
        assert calls == ["scan-1", "scan-2", "scan-3", "scan-4"]
        assert slot.tree_revision == 4
        assert [role for role, _ in client.calls].count("local") == 3
        assert len(set(seen_issue_ids)) == 1  # same issue_id across all 3 proposals
        assert supervisor.ledger.failed_count(seen_issue_ids[0]) == 3
        outcome = py_trees.blackboard.Blackboard.get("gpsr/task_outcome")
        assert outcome["status"] == "aborted"
        assert "Three distinct scans failed" in outcome["reason"]
    finally:
        supervisor.close()


def test_recovery_budget_ceiling_forces_escalation_after_wall_clock_exhaustion(
    monkeypatch,
):
    """F1.2: the wall-clock budget is a ceiling independent of the ledger --
    once the deadline for this slot's activation has passed, a pending
    local_recovery is no longer honoured and the slot force-escalates with
    ``recovery budget exhausted after ...s``, emitting one
    ``supervision.budget_exhausted`` event."""
    import behavior_tree.GPSR.supervision.runtime as runtime_module

    _seed_blackboard()
    monkeypatch.setenv("GPSR_RECOVERY_BUDGET_S", "10")
    real_monotonic = time.monotonic
    offset = [0.0]
    monkeypatch.setattr(runtime_module.time, "monotonic", lambda: real_monotonic() + offset[0])

    first_checkpoint = "task/plan-r1/step-0000:demo/tree-r1/demo/root/activation-0001"
    second_checkpoint = "task/plan-r1/step-0000:demo/tree-r2/demo/root/activation-0001"
    verification = {
        **_decision(first_checkpoint, verdict="recoverable"),
        "subtask_status": "not_achieved",
        "escalation": "local_recovery",
        "failure_category": "target_not_visible",
    }

    def _slow_second_verification():
        # Only runtime_module.time.monotonic is patched below, not this
        # file's own time.sleep -- a real sleep, keeping this 2nd verify()
        # pending long enough for the deadline check to fire first. The
        # tree-r2 rebuild's own checkpoint still gets submitted
        # (SupervisedEffect doesn't know about the slot's budget), so a
        # scripted response must exist for it, but it must never resolve
        # into an applied local_recovery.
        time.sleep(0.05)
        return {
            **_decision(second_checkpoint, verdict="recoverable"),
            "subtask_status": "not_achieved",
            "escalation": "local_recovery",
            "failure_category": "target_not_visible",
        }

    client = ScriptedSupervisorClient(
        verifications=[verification, _slow_second_verification],
        recoveries=[
            {
                "checkpoint_id": first_checkpoint,
                "issue_id": "filled-by-test",
                "strategy_id": "look-left",
                "kind": "scan_views",
                "arguments": {"angles": [[-30, 10]], "perception_action": "find_object"},
                "rationale": "try a new view",
                "expected_evidence": ["target visible"],
                "stop_conditions": ["target absent"],
            }
        ],
    )
    original = client.plan_local_recovery

    def local(snapshot, verification_decision, issue_id):
        client._recoveries[0]["issue_id"] = issue_id
        return original(snapshot, verification_decision, issue_id)

    client.plan_local_recovery = local
    events: list[tuple[str, dict]] = []

    class _Recorder:
        def emit(self, event, payload, **kwargs):
            events.append((event, dict(payload)))

    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _provider(),
        client,
        telemetry=_Recorder(),
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "FakeEffect",
                "perception",
                EffectRisk.OBSERVATION,
                "target was observed",
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
    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=lambda: FakeEffect("scan", py_trees.common.Status.FAILURE, []),
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    try:
        jumped = False
        for _ in range(500):
            slot.tick_once()
            if not jumped and slot.tree_revision == 2:
                # The first local_recovery rebuild has happened; jump the
                # clock far past the 10s budget before it can resolve again.
                offset[0] = 10_000.0
                jumped = True
            if slot._hard_stop_reason is not None:
                break
            time.sleep(0.005)
        assert jumped
        assert slot._hard_stop_reason is not None
        assert slot._hard_stop_reason.startswith("recovery budget exhausted after")
        assert slot.tree_revision == 2  # never reached a 3rd rebuild
        outcome = py_trees.blackboard.Blackboard.get("gpsr/task_outcome")
        assert outcome["status"] == "stopped"
        assert outcome["reason"] == slot._hard_stop_reason
        budget_events = [e for e in events if e[0] == "supervision.budget_exhausted"]
        assert len(budget_events) == 1
        payload = budget_events[0][1]
        assert payload["action"] == "demo"
        assert payload["elapsed_s"] >= 10.0
    finally:
        supervisor.close()


def test_destructive_failure_aborts_and_records_operator_message():
    _seed_blackboard()
    py_trees.blackboard.Blackboard.set("gpsr/plan", [{"action": "grasp", "params": {}}])
    checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    verification = {
        **_decision(checkpoint, verdict="unrecoverable"),
        "subtask_status": "not_achieved",
        "world_change": "destructive",
        "escalation": "global_replan",
        "failure_category": "object_broken",
    }
    client = ScriptedSupervisorClient(
        verifications=[verification],
        global_plans=[
            {
                "checkpoint_id": checkpoint,
                "action": "abort_and_report",
                "replacement_plan": [],
                "preserved_completed_steps": 0,
                "relaxed_constraints": [],
                "rationale": "The object broke.",
                "operator_message": "I am sorry, the object broke, so I stopped.",
            }
        ],
    )
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _provider(),
        client,
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "FakeEffect",
                "manipulation",
                EffectRisk.IRREVERSIBLE,
                "object was safely grasped",
                allow_local_recovery=False,
            )
        ]
    )
    slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=lambda: FakeEffect(
            "grasp", py_trees.common.Status.FAILURE, []
        ),
        supervisor=supervisor,
        registry=registry,
    )
    try:
        for _ in range(200):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        outcome = py_trees.blackboard.Blackboard.get("gpsr/task_outcome")
        assert outcome["status"] == "aborted"
        assert outcome["source"] == "llm_supervisor"
        assert "object broke" in outcome["operator_message"]
        assert py_trees.blackboard.Blackboard.get("gpsr/plan") == []
    finally:
        supervisor.close()
