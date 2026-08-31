from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from pathlib import Path
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import py_trees
import pytest
from geometry_msgs.msg import PoseStamped

from behavior_tree.GPSR.supervision.clients import ScriptedSupervisorClient
from behavior_tree.GPSR.supervision.context import StaticContextProvider
from behavior_tree.GPSR.supervision.contracts import NodeContractRegistry
from behavior_tree.GPSR.supervision.controller import (
    MissionSupervisor,
    SupervisorIntervention,
)
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    CaptureRequest,
    EffectRisk,
    NodeContract,
    RecoveryProposal,
    ReportedStatus,
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
from behavior_tree.GPSR.supervision.runtime import (
    BtNode_RecoveryDirective,
    SupervisedEffect,
    SupervisedSubtaskSlot,
    default_recovery_compiler,
)
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


def test_uncertain_verdict_after_genuine_failure_resolves_to_failure():
    # O2 hang regression: SupervisedEffect.tick() masks a genuinely reported
    # FAILURE as outward RUNNING until self.supervisor.resolution(...) ==
    # "success" (a false_failure override) -- every OTHER pre-O2 terminal
    # resolution ("recovery", "global", "stop") was always paired with a
    # queued intervention that the slot consumes to force it out of that
    # masked RUNNING state. O2's new "unverified" resolution creates NO
    # intervention, so without also being treated as terminal here, a
    # genuinely-failed checkpoint whose verifier came back uncertain would
    # mask as RUNNING forever. It must resolve to the real FAILURE instead
    # (uncertainty must not fabricate a false success either).
    _seed_blackboard()
    calls: list[str] = []
    checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                "checkpoint_id": checkpoint,
                "verdict": "uncertain",
                "bt_assessment": "agree",
                "subtask_status": "unknown",
                "world_change": "unknown",
                "escalation": "none",
                "failure_category": "sensor_context_mismatch",
                "evidence": ["script"],
                "rationale": "cannot verify this checkpoint",
                "confidence": 0.4,
            }
        ]
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
            if slot.status is not py_trees.common.Status.RUNNING:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.FAILURE
        assert supervisor.resolution(checkpoint) == "unverified"
        assert supervisor.consume_intervention() is None
    finally:
        supervisor.close()


def test_query_error_after_genuine_failure_resolves_to_failure():
    # O4 hang regression, same shape as the O2 test above: a query error
    # now resolves as "unavailable" with no intervention, so a genuinely
    # failed checkpoint whose verifier errored out must also resolve to
    # the real FAILURE rather than mask as RUNNING forever.
    _seed_blackboard()
    calls: list[str] = []
    checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(verifications=[RuntimeError("offline")])
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
            if slot.status is not py_trees.common.Status.RUNNING:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.FAILURE
        assert supervisor.resolution(checkpoint) == "unavailable"
        assert supervisor.consume_intervention() is None
    finally:
        supervisor.close()


def test_degraded_supervisor_then_genuine_failure_does_not_hang():
    # Z-1 (task-O review, CRITICAL) -- the reviewer's own direct
    # reproduction. Once supervision degrades (N consecutive query
    # errors), MissionSupervisor.submit()'s O4 skip_query path resolves
    # straight from reported_status -- for a genuinely FAILED effect that
    # is the bare string "failure", which the previous fix's
    # _UNVERIFIED_RESOLUTIONS allowlist (only {"unverified", "unavailable"})
    # did not recognize, silently re-masking it as RUNNING forever. This
    # is production-realistic, not a synthetic edge case: a verifier
    # outage degrades supervision (as O4 intends, at zero mission cost so
    # far -- the three warmup checkpoints below all still succeed), and
    # the NEXT genuine robot failure (a missed grasp, a failed goto) must
    # still surface, not hang the mission forever.
    _seed_blackboard()
    calls: list[str] = []
    client = ScriptedSupervisorClient(verifications=[RuntimeError("offline")] * 3)
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE, max_consecutive_errors=3),
        _provider(),
        client,
    )
    registry = NodeContractRegistry(
        [
            NodeContract(
                "FakeEffect", "perception", EffectRisk.OBSERVATION, "observed"
            )
        ]
    )
    try:
        for index in range(3):
            warmup_slot = SupervisedSubtaskSlot(
                action_name=f"warmup-{index}",
                factory=lambda: FakeEffect(
                    "warmup", py_trees.common.Status.SUCCESS, calls
                ),
                supervisor=supervisor,
                registry=registry,
            )
            for _ in range(100):
                warmup_slot.tick_once()
                if warmup_slot.status is not py_trees.common.Status.RUNNING:
                    break
                time.sleep(0.005)
            assert warmup_slot.status is py_trees.common.Status.SUCCESS
        # Confirm supervision actually degraded (the precondition for the
        # skip_query path that produces the bare "failure" resolution).
        assert supervisor._degraded is True

        failing_slot = SupervisedSubtaskSlot(
            action_name="real-robot-failure",
            factory=lambda: FakeEffect(
                "grasp", py_trees.common.Status.FAILURE, calls
            ),
            supervisor=supervisor,
            registry=registry,
        )
        for _ in range(100):
            failing_slot.tick_once()
            if failing_slot.status is not py_trees.common.Status.RUNNING:
                break
            time.sleep(0.005)
        assert failing_slot.status is py_trees.common.Status.FAILURE
    finally:
        supervisor.close()


def _build_masked_failed_effect():
    """Real MissionSupervisor + SupervisedSubtaskSlot wrapping a FAILURE
    effect, ticked exactly once so the underlying SupervisedEffect is
    parked in its masked-RUNNING state: `_terminal_status` is FAILURE,
    `_checkpoint_id` is captured, and the query is genuinely still
    pending (blocked on a threading.Event, never released here) so the
    masking branch has not made a decision yet.

    Returns (slot, effect, real_supervisor, release_event). The caller is
    responsible for `release_event.set()` and `real_supervisor.close()`.
    Swap `effect.supervisor` for a stub to control exactly what the
    masking branch's next tick sees, without needing the real async
    verifier/recovery pipeline to produce every resolution.
    """
    _seed_blackboard()
    release = threading.Event()

    def delayed():
        release.wait(5)
        raise RuntimeError("kept pending for the test")

    client = ScriptedSupervisorClient(verifications=[delayed])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client
    )
    registry = NodeContractRegistry(
        [NodeContract("FakeEffect", "perception", EffectRisk.OBSERVATION, "observed")]
    )
    slot = SupervisedSubtaskSlot(
        action_name="masking-probe",
        factory=lambda: FakeEffect("fail", py_trees.common.Status.FAILURE, []),
        supervisor=supervisor,
        registry=registry,
    )
    slot.tick_once()
    assert slot.status is py_trees.common.Status.RUNNING
    effect = next(
        node for node in slot.decorated.iterate() if isinstance(node, SupervisedEffect)
    )
    assert effect.status is py_trees.common.Status.RUNNING
    assert effect._terminal_status is py_trees.common.Status.FAILURE
    return slot, effect, supervisor, release


class _StubSupervisorForMasking:
    """Controls exactly what SupervisedEffect.tick()'s masking branch sees
    for one tick, decoupled from the real async verifier/recovery
    pipeline -- see _build_masked_failed_effect()."""

    def __init__(self, *, resolution, has_intervention, stage="complete"):
        self.config = SimpleNamespace(mode=SupervisionMode.ACTIVE)
        self._resolution = resolution
        self._has_intervention = has_intervention
        self._stage = stage

    def resolution(self, checkpoint_id):
        return self._resolution

    def record(self, checkpoint_id):
        return SimpleNamespace(stage=self._stage)

    def has_queued_intervention(self, checkpoint_id):
        return self._has_intervention


# Z-1 (task-O review, CRITICAL): every `record.resolution = ...` literal
# grepped from controller.py, so a future resolution value that isn't
# added here fails this test instead of silently being able to hang a
# mission. `has_intervention` reflects how each value actually arises in
# controller.py: the STOP branch and else-fallback in
# `_handle_verification` (both "stop") and `_start_local`/
# `_handle_local_recovery` ("recovery"), `_start_global`/
# `_handle_global_replan` ("global") are the ONLY three call sites that
# also queue a SupervisorIntervention for the same checkpoint (see
# `MissionSupervisor.has_queued_intervention`'s docstring) -- every other
# resolution-setting call site never queues one.
_KNOWN_RESOLUTIONS = (
    # (resolution, has_intervention, expected_status)
    ("success", False, py_trees.common.Status.SUCCESS),  # false_failure override
    ("failure", False, py_trees.common.Status.FAILURE),  # O4 skip_query once degraded
    ("unverified", False, py_trees.common.Status.FAILURE),  # O2 uncertain downgrade
    ("unavailable", False, py_trees.common.Status.FAILURE),  # O4 query error
    ("recovery_succeeded", False, py_trees.common.Status.FAILURE),
    ("recovery_failed", False, py_trees.common.Status.FAILURE),
    ("stop", True, py_trees.common.Status.RUNNING),  # intervention queued, slot owns it
    ("recovery", True, py_trees.common.Status.RUNNING),
    ("global", True, py_trees.common.Status.RUNNING),
)


@pytest.mark.parametrize("resolution,has_intervention,expected", _KNOWN_RESOLUTIONS)
def test_every_known_resolution_value_is_safe_by_construction(
    resolution, has_intervention, expected
):
    # Z-1: no resolution value the controller can produce may hang the
    # masking branch. Only "success" maps to Status.SUCCESS; a resolution
    # paired with a still-queued intervention (stop/recovery/global) stays
    # RUNNING, correctly deferring to the owning SupervisedSubtaskSlot's
    # intervention-consumption machinery; every other resolution unmasks
    # to the real, already-reported FAILURE -- including
    # "recovery_succeeded"/"recovery_failed", which must NOT be
    # misread as this checkpoint's own success just because the word
    # "succeeded" appears in the string.
    slot, effect, supervisor, release = _build_masked_failed_effect()
    try:
        effect.supervisor = _StubSupervisorForMasking(
            resolution=resolution, has_intervention=has_intervention
        )
        list(effect.tick())
        assert effect.status is expected
    finally:
        release.set()
        supervisor.close()


def test_stage_complete_with_unset_resolution_fails_loud_not_running():
    # Z-1: a genuinely unknown/unset state (the controller's own
    # complete/shadow_complete-always-sets-resolution invariant somehow
    # violated) must fail loud -- surface a real terminal FAILURE with a
    # feedback message naming the anomaly -- never hold RUNNING forever.
    slot, effect, supervisor, release = _build_masked_failed_effect()
    try:
        effect.supervisor = _StubSupervisorForMasking(
            resolution=None, has_intervention=False, stage="complete"
        )
        list(effect.tick())
        assert effect.status is py_trees.common.Status.FAILURE
        assert "invariant violated" in effect.feedback_message
        assert str(effect._checkpoint_id) in effect.feedback_message
    finally:
        release.set()
        supervisor.close()


def test_uncertain_stop_verdict_after_child_success_still_returns_success():
    # O2 retro-fail regression test: hybrid success mode holds an
    # already-SUCCESS child RUNNING until the pending verification
    # resolves (see can_finish_subtask below). Before O2, an uncertain
    # verdict that also requested escalation=stop (e.g. a sensor-context
    # mismatch under the old prompt) created a stop intervention that
    # converted the already-successful step to FAILURE -- an LLM admitting
    # it could not verify silently retro-failed a nominal step. After O2
    # the uncertain verdict downgrades to "unverified" with no
    # intervention, so the slot must surface the child's real SUCCESS.
    _seed_blackboard()
    calls: list[str] = []
    checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                "checkpoint_id": checkpoint,
                "verdict": "uncertain",
                "bt_assessment": "agree",
                "subtask_status": "unknown",
                "world_change": "unknown",
                "escalation": "stop",
                "failure_category": "sensor_context_mismatch",
                "evidence": ["script"],
                "rationale": "cannot verify this checkpoint",
                "confidence": 0.4,
            }
        ]
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
        factory=lambda: FakeEffect(
            "succeed", py_trees.common.Status.SUCCESS, calls
        ),
        supervisor=supervisor,
        registry=registry,
    )
    try:
        for _ in range(100):
            slot.tick_once()
            if slot.status is not py_trees.common.Status.RUNNING:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        assert calls == ["succeed"]
        assert supervisor.resolution(checkpoint) == "unverified"
        assert supervisor.consume_intervention() is None
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
        # L1b (round-4 battery fix, run 016): one-shot blackboard-only guard
        # for create_find_object's reduced-query retry branch -- same kind
        # of internal bookkeeping leaf as BtNode_ExtractDetection above, no
        # real-world effect to supervise.
        "BtNode_ReduceObjectQuery",
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


# ---------------------------------------------------------------------------
# Task Q5 (round-6, supervision-economics fix): the "/goal missing" verify
# evidence. `gpsr/target_pose` -- the ACTUAL navigation goal the goto
# dispatch drives to -- must reach the checkpoint's captured blackboard
# snapshot (build_capture_request's explicit key set) so
# FixtureContextProvider.capture() can read it directly instead of
# re-deriving a goal marker from `gpsr/target_location`'s NAME through a
# separate, fallible lookup (see supervision/context.py's
# _goal_pose_from_blackboard for the read-side half of this fix).


def test_build_capture_request_includes_the_real_nav_goal_key():
    _seed_blackboard()
    py_trees.blackboard.Blackboard.set(
        "gpsr/target_pose",
        {
            "pose": {
                "position": {"x": 4.0, "y": 2.0, "z": 0.0},
                "orientation": {"w": 1.0},
            }
        },
    )
    client = ScriptedSupervisorClient(verifications=[_decision("irrelevant")])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.SHADOW), _provider(), client
    )
    registry = NodeContractRegistry(
        [NodeContract("FakeEffect", "navigation", EffectRisk.OBSERVATION, "arrived")]
    )
    slot = SupervisedSubtaskSlot(
        action_name="goto",
        factory=lambda: FakeEffect("goto-leaf", py_trees.common.Status.SUCCESS, []),
        supervisor=supervisor,
        registry=registry,
    )
    try:
        slot.tick_once()
        assert slot.status is py_trees.common.Status.SUCCESS
        records = supervisor.records()
        assert len(records) == 1
        blackboard = records[0].snapshot.request.blackboard
        assert blackboard.get("gpsr/target_pose") == {
            "pose": {
                "position": {"x": 4.0, "y": 2.0, "z": 0.0},
                "orientation": {"w": 1.0},
            }
        }
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


# ---------------------------------------------------------------------------
# Task Q1 (round-6, supervision-economics fix): a recovery retry subtree's
# effects must never create their own checkpoints -- exactly one verify/
# recovery pipeline per ORIGINAL attempt, none per retry. This is the
# measured root cause of the 10x120s recovery-churn stall: every retry used
# to re-supervise itself, multiplying supervisor.query queries and re-arming
# the full stall window against the fresh subtree.


def test_recovery_retry_internals_create_zero_checkpoints():
    _seed_blackboard()
    calls: list[str] = []
    factory_calls = 0
    events: list[tuple[str, dict]] = []
    first_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    # Only ONE verification is scripted -- if Q1 regressed and the retry's
    # own SUCCESS created a second checkpoint, its background verify() call
    # would starve for a scripted response (ScriptedSupervisorClient raises
    # "no scripted verify response remains"), which resolves as
    # "unavailable" rather than hanging -- so this test additionally counts
    # checkpoints/telemetry directly rather than relying on that alone.
    client = ScriptedSupervisorClient(
        verifications=[
            {
                **_decision(first_checkpoint, verdict="recoverable"),
                "subtask_status": "not_achieved",
                "escalation": "local_recovery",
                "failure_category": "target_not_visible",
            },
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
        telemetry=lambda event, payload: events.append((event, dict(payload))),
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
        # Exactly one checkpoint was ever created (the ORIGINAL failure) --
        # the retry's own terminal SUCCESS never reached `supervisor.submit`.
        assert len(supervisor._records) == 1
        created = [e for e in events if e[0] == "supervisor.checkpoint.created"]
        assert len(created) == 1
        assert created[0][1]["checkpoint_id"] == first_checkpoint
        # The retry's OUTCOME still reaches the ledger via the composite
        # result -- recovery_finished semantics unchanged.
        attempts = supervisor.ledger.snapshot()
        assert len(attempts) == 1
        assert attempts[0]["succeeded"] is True
        assert slot.tree_revision == 2
    finally:
        supervisor.close()


def test_recovery_retry_failure_still_records_ledger_outcome_via_composite_result():
    # With retry internals unsupervised, a FAILED retry no longer creates
    # its own checkpoint/intervention to notice the failure through --
    # `recovery_finished(succeeded=False)` must now come from the slot's
    # own hard-stop path, driven directly by the `recover+retry` Sequence's
    # own terminal FAILURE (see the FAILURE branch of `tick()`).
    _seed_blackboard()
    calls: list[str] = []
    first_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                **_decision(first_checkpoint, verdict="recoverable"),
                "subtask_status": "not_achieved",
                "escalation": "local_recovery",
                "failure_category": "target_not_visible",
            },
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
        SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client
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
        # Always fails -- including the retry.
        factory=lambda: FakeEffect("scan", py_trees.common.Status.FAILURE, calls),
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    try:
        for _ in range(300):
            slot.tick_once()
            if slot.status is py_trees.common.Status.FAILURE:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.FAILURE
        # Only the ORIGINAL checkpoint was ever created.
        assert len(supervisor._records) == 1
        attempts = supervisor.ledger.snapshot()
        assert len(attempts) == 1
        assert attempts[0]["succeeded"] is False
    finally:
        supervisor.close()


def test_a_new_subtask_after_a_completed_recovery_still_gets_its_own_checkpoint():
    # Q1's "not the subtask forever" guarantee: `in_recovery_retry` is
    # scoped to the ONE subtree built by `_apply_intervention` -- it must
    # never leak into a later, genuinely new subtask's own first attempt.
    # Drive slot 1 through a full fail -> recover -> retry -> SUCCESS cycle
    # (checkpoints suppressed for its retry, per the test above), then build
    # a brand new slot (standing in for the next plan step) whose own first
    # attempt fails: it must create its own checkpoint exactly like any
    # first attempt always has.
    _seed_blackboard()
    calls: list[str] = []
    first_checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                **_decision(first_checkpoint, verdict="recoverable"),
                "subtask_status": "not_achieved",
                "escalation": "local_recovery",
                "failure_category": "target_not_visible",
            },
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
        SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client
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
    factory_calls = 0

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
    slot1 = SupervisedSubtaskSlot(
        action_name="demo",
        factory=factory,
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    try:
        for _ in range(300):
            slot1.tick_once()
            if slot1.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot1.status is py_trees.common.Status.SUCCESS
        assert len(supervisor._records) == 1

        py_trees.blackboard.Blackboard.set("gpsr/plan_index", 2)
        second_checkpoint = (
            "task/plan-r1/step-0001:demo2/tree-r1/demo2/root/activation-0001"
        )
        client._verifications.append(_decision(second_checkpoint))
        slot2 = SupervisedSubtaskSlot(
            action_name="demo2",
            factory=lambda: FakeEffect(
                "scan2-1", py_trees.common.Status.FAILURE, calls
            ),
            supervisor=supervisor,
            registry=registry,
            recovery_compiler=compiler,
        )
        for _ in range(300):
            slot2.tick_once()
            if len(supervisor._records) == 2:
                break
            time.sleep(0.005)
        assert len(supervisor._records) == 2, (
            "a genuinely new subtask's first failure must still create its "
            "own checkpoint -- Q1 must not disable checkpoint creation "
            "beyond the specific retry subtree"
        )
    finally:
        supervisor.close()


# ---------------------------------------------------------------------------
# Task Q2 (round-6, supervision-economics fix): a retry attempt does not get
# a fresh full stall window. `_may_apply_intervention_now`'s full
# GPSR_SUPERVISION_STALL_S protection applies only to a subtask's FIRST
# attempt; once a subtask has been through >=1 applied intervention, a
# FURTHER intervention against a still-RUNNING retry only has to wait the
# much shorter GPSR_SUPERVISION_RETRY_STALL_S.


def test_post_retry_intervention_applies_at_the_shorter_retry_stall_not_the_full_stall():
    _seed_blackboard()
    checkpoint_id = "post-retry-checkpoint"
    client = ScriptedSupervisorClient(verifications=[_decision(checkpoint_id)])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client
    )
    registry = NodeContractRegistry([])
    clock = {"t": 0.0}
    with patch.dict(
        "os.environ",
        {"GPSR_SUPERVISION_STALL_S": "120", "GPSR_SUPERVISION_RETRY_STALL_S": "20"},
    ):
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=lambda: FakeEffect("goto", py_trees.common.Status.RUNNING, []),
            supervisor=supervisor,
            registry=registry,
            time_source=lambda: clock["t"],
        )
    # Normally set by `_apply_intervention`'s local_recovery branch; set
    # directly here to isolate the stall-threshold decision (Q2) from the
    # rest of the recovery pipeline (covered by the Q1 tests above).
    slot._has_retried = True
    try:
        subtask_id = slot.current_subtask_id()
        _register_checkpoint(supervisor, checkpoint_id=checkpoint_id, subtask_id=subtask_id)
        proposal = _proposal(checkpoint_id=checkpoint_id)
        supervisor.ledger.register(proposal)
        supervisor._interventions.append(
            SupervisorIntervention(
                kind="local_recovery",
                checkpoint_id=checkpoint_id,
                reason="hallucinated",
                payload=proposal,
            )
        )

        slot.tick_once()
        assert slot.status is py_trees.common.Status.RUNNING
        assert len(supervisor._interventions) == 1

        # Well under the 20s retry stall (and WAY under the 120s full
        # stall) -> still deferred.
        clock["t"] = 10.0
        slot.tick_once()
        assert len(supervisor._interventions) == 1

        # Past the 20s retry stall (but nowhere near the 120s full stall)
        # -> now applies, proving the SHORTER window governed.
        clock["t"] = 21.0
        slot.tick_once()
        assert len(supervisor._interventions) == 0
        assert slot.tree_revision == 2
    finally:
        supervisor.close()


def test_first_attempt_intervention_still_waits_the_full_stall_not_the_retry_stall():
    # Q2 counterpart: `_has_retried` starts False -- a subtask's FIRST
    # attempt keeps the existing full-stall protection even when the (much
    # shorter) retry-stall env var is configured.
    _seed_blackboard()
    checkpoint_id = "first-attempt-checkpoint"
    client = ScriptedSupervisorClient(verifications=[_decision(checkpoint_id)])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client
    )
    registry = NodeContractRegistry([])
    clock = {"t": 0.0}
    with patch.dict(
        "os.environ",
        {"GPSR_SUPERVISION_STALL_S": "120", "GPSR_SUPERVISION_RETRY_STALL_S": "20"},
    ):
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=lambda: FakeEffect("goto", py_trees.common.Status.RUNNING, []),
            supervisor=supervisor,
            registry=registry,
            time_source=lambda: clock["t"],
        )
    try:
        subtask_id = slot.current_subtask_id()
        _register_checkpoint(supervisor, checkpoint_id=checkpoint_id, subtask_id=subtask_id)
        proposal = _proposal(checkpoint_id=checkpoint_id)
        supervisor.ledger.register(proposal)
        supervisor._interventions.append(
            SupervisorIntervention(
                kind="local_recovery",
                checkpoint_id=checkpoint_id,
                reason="hallucinated",
                payload=proposal,
            )
        )
        slot.tick_once()
        # Past the 20s retry-stall window but well under the 120s full
        # stall -> STILL deferred, because this is the first attempt.
        clock["t"] = 21.0
        slot.tick_once()
        assert len(supervisor._interventions) == 1
        assert slot.tree_revision == 1
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


def _global_replan_slot(preserved_completed_steps, *, plan_index):
    """Build a SupervisedSubtaskSlot mid-global-replan, matching
    test_destructive_failure_aborts_and_records_operator_message's harness --
    J7 (round-3 adversarial review, H3, ACTIVE mode): the two-layer executor
    tracks completed steps for the CURRENT target's own plan via
    gpsr/plan_index, not gpsr/state_log (which is a whole-task log, not a
    per-target step count).
    """
    _seed_blackboard()
    py_trees.blackboard.Blackboard.set("gpsr/plan_index", plan_index)
    py_trees.blackboard.Blackboard.set("gpsr/plan", [{"action": "grasp", "params": {}}])
    step = max(0, plan_index - 1)
    checkpoint = (
        f"task/plan-r1/step-{step:04d}:demo/tree-r1/"
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
                "action": "replan_remaining",
                "replacement_plan": [{"action": "announce", "params": {"text": "resuming"}}],
                "preserved_completed_steps": preserved_completed_steps,
                "relaxed_constraints": [],
                "rationale": "target 0 already finished, resume target 1",
                "operator_message": "resuming after global replan",
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
        factory=lambda: FakeEffect("grasp", py_trees.common.Status.FAILURE, []),
        supervisor=supervisor,
        registry=registry,
    )
    return slot, supervisor


def test_global_decision_completed_count_derives_from_plan_index_not_state_log():
    # "target 1 after target 0 completed": plan_index=3 means 2 steps of
    # THIS target's own plan already materialised (step 0, step 1) before
    # the 3rd (index 2) is now in flight -- gpsr/state_log stays [] the
    # whole time (per _seed_blackboard), so the OLD len(state_log)-based
    # count (0) would have rejected a correctly-formed decision claiming 2.
    slot, supervisor = _global_replan_slot(preserved_completed_steps=2, plan_index=3)
    try:
        for _ in range(200):
            slot.tick_once()
            if slot.status is py_trees.common.Status.SUCCESS:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.SUCCESS
        request = py_trees.blackboard.Blackboard.get("gpsr/replan_request")
        assert request["preserved_completed_steps"] == 2
        assert request["level"] == "supervisor"
        # No stop/abort outcome was ever recorded -- the decision applied
        # cleanly (no SchemaError from a completed-count mismatch).
        assert not py_trees.blackboard.Blackboard.exists("gpsr/task_outcome")
    finally:
        supervisor.close()


def test_malformed_completed_count_becomes_stop_intervention_not_exception():
    # A decision whose preserved_completed_steps does not match the
    # derived count is malformed -- must be handled as a stop intervention
    # (no SchemaError escaping tick()).
    #
    # N1a (round-5 rerun fix): REWRITTEN from the old RUNNING-latch contract.
    # A hard-stop used to leave the slot RUNNING forever -- unreachable by
    # the only `gpsr/task_outcome` consumer (`BtNode_FinalizeTask`) and the
    # exact "hard-stop into an unconsumed state" defect documented in
    # docs/superpowers/notes/2026-08-29-supervision-ledger-followup.md
    # (F1/H3). The new contract releases the slot via a terminal FAILURE
    # instead, so the tree's own failure machinery can take over.
    slot, supervisor = _global_replan_slot(preserved_completed_steps=99, plan_index=3)
    try:
        for _ in range(200):
            slot.tick_once()
            if slot.status is py_trees.common.Status.FAILURE:
                break
            time.sleep(0.005)
        assert slot.status is py_trees.common.Status.FAILURE
        outcome = py_trees.blackboard.Blackboard.get("gpsr/task_outcome")
        assert outcome["status"] == "stopped"
        assert outcome["source"] == "llm_supervisor"
        assert "completed step count" in outcome["reason"]
        # Ticking a FAILURE slot again must not re-tick its child or change
        # the outcome -- the hard-stop transition happens exactly once.
        calls_before = slot.tree_revision
        slot.tick_once()
        assert slot.status is py_trees.common.Status.FAILURE
        assert slot.tree_revision == calls_before
    finally:
        supervisor.close()


# ---------------------------------------------------------------------------
# Task M (round-4 battery fix): active-supervision intervention crash + slot
# identity. The crash: `runtime.py:_apply_intervention`'s local_recovery
# branch built `Sequence(children=[macro, fresh_subtask])` -- add_child
# raised "already has parent" when `fresh_subtask` (from
# `_materialize_subtask`) was, due to a factory-wiring bug elsewhere
# (`GPSRPlanner.build_target_subtree` handing the slot `lambda: small_tree`,
# a closure over one already-built/already-attached instance), the SAME
# object as the slot's own currently-attached child. Every existing test
# above used a genuinely fresh-per-call factory, so this was structurally
# unreachable -- these tests close that coverage hole.
# ---------------------------------------------------------------------------


def test_materialize_subtask_builds_a_fresh_parentless_object_every_call():
    """Pins M1a: `_materialize_subtask` must invoke `self.factory` fresh
    every call and never hand back a subtree that already has a parent."""
    _seed_blackboard()
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.SHADOW),
        _provider(),
        ScriptedSupervisorClient(),
    )
    build_calls = 0

    def factory():
        nonlocal build_calls
        build_calls += 1
        return py_trees.behaviours.Success(name=f"leaf-{build_calls}")

    try:
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=factory,
            supervisor=supervisor,
        )
        # __init__ already called _materialize_subtask once (build_calls == 1).
        first = slot._materialize_subtask()
        second = slot._materialize_subtask()
        assert first is not second
        assert first.parent is None
        assert second.parent is None
        assert build_calls == 3
    finally:
        supervisor.close()


def test_materialize_subtask_asserts_loudly_on_a_cached_factory():
    """Pins M1a's defensive assertion: a factory that returns a cached/
    closure-captured instance (the exact shape of the historical
    `GPSRPlanner.build_target_subtree` bug) must fail loudly AT
    `_materialize_subtask`, not several layers away inside py_trees
    `add_child`."""
    _seed_blackboard()
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.SHADOW),
        _provider(),
        ScriptedSupervisorClient(),
    )
    cached = py_trees.behaviours.Success(name="cached-leaf")
    try:
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            # The FIRST call (inside __init__) is fine -- `cached` starts
            # parentless. It is the SECOND call, once `cached` is already
            # this slot's attached child, that must trip the assertion.
            factory=lambda: cached,
            supervisor=supervisor,
        )
        with pytest.raises(AssertionError, match="already has a parent"):
            slot._materialize_subtask()
    finally:
        supervisor.close()


def test_local_recovery_detaches_old_child_before_attaching_replacement():
    """Pins M1b: the exact crash regression. An intervention arriving while
    the slot's child is attached must replace it via detach-first -- the new
    `recover+retry:` Sequence becomes the slot's child, and the displaced
    child comes out with `parent is None` (never re-added anywhere)."""
    _seed_blackboard()
    calls: list[str] = []
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
                **_decision(first_checkpoint, verdict="recoverable"),
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
        return FakeEffect("scan", py_trees.common.Status.FAILURE, calls)

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
    old_child = slot.decorated
    try:
        for _ in range(300):
            slot.tick_once()
            if slot.tree_revision == 2:
                break
            time.sleep(0.005)
        assert slot.tree_revision == 2
        assert old_child.parent is None
        assert slot.decorated is not old_child
        assert slot.decorated.name == "recover+retry:look-left"
        assert slot.decorated.parent is slot
        assert [child.parent for child in slot.decorated.children] == (
            [slot.decorated] * len(slot.decorated.children)
        )
    finally:
        supervisor.close()


def test_idle_slot_never_consumes_even_with_a_colliding_subtask_id():
    """Pins M2b independent of id derivation (M2a): two slots built WITHOUT
    a fixed (target_slot, target_index, step_index) identity fall back to
    the old global-plan_index-derived id, so with the same action_name and
    blackboard state they compute the IDENTICAL subtask_id -- a genuine
    string collision. The active slot queues a real local_recovery
    intervention against that shared id; the idle slot (built identically,
    but never yet ticked) must not consume it on its first tick even though
    the ids match exactly."""
    _seed_blackboard()
    calls: list[str] = []
    checkpoint = (
        "task/plan-r1/step-0000:demo/tree-r1/"
        "demo/root/activation-0001"
    )
    client = ScriptedSupervisorClient(
        verifications=[
            {
                **_decision(checkpoint, verdict="recoverable"),
                "subtask_status": "not_achieved",
                "escalation": "local_recovery",
                "failure_category": "target_not_visible",
            }
        ],
        recoveries=[
            {
                "checkpoint_id": checkpoint,
                "issue_id": "filled-by-test",
                "strategy_id": "retry",
                "kind": "scan_views",
                "arguments": {"angles": [[0, 0]], "perception_action": "demo"},
                "rationale": "retry",
                "expected_evidence": ["ok"],
                "stop_conditions": ["ok"],
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
                "FakeEffect", "perception", EffectRisk.OBSERVATION, "observed"
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
    active_slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=lambda: FakeEffect(
            "active-fail", py_trees.common.Status.FAILURE, calls
        ),
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    idle_slot = SupervisedSubtaskSlot(
        action_name="demo",
        factory=lambda: FakeEffect(
            "idle-leaf", py_trees.common.Status.SUCCESS, calls
        ),
        supervisor=supervisor,
        registry=registry,
        recovery_compiler=compiler,
    )
    try:
        assert active_slot.current_subtask_id() == idle_slot.current_subtask_id()
        # Tick #1 submits the checkpoint (the child fails synchronously) but
        # -- being this slot's very first tick -- never attempts to consume
        # anything (M2b); nothing is queued yet regardless, since
        # verification is asynchronous.
        active_slot.tick_once()
        assert active_slot.status is py_trees.common.Status.RUNNING
        # Advance the async verification -> local_recovery pipeline via
        # `poll()` directly, NOT through any slot's `tick()` -- ticking
        # `active_slot` again would immediately consume its own
        # intervention in that same call, leaving no window to observe it
        # queued-but-unconsumed.
        for _ in range(300):
            supervisor.poll()
            if supervisor._interventions:
                break
            time.sleep(0.005)
        assert supervisor._interventions, (
            "expected a queued local_recovery intervention"
        )
        pending = len(supervisor._interventions)
        assert idle_slot.status is not py_trees.common.Status.RUNNING
        idle_slot.tick_once()
        assert len(supervisor._interventions) == pending, (
            "an idle slot with a colliding subtask_id consumed an "
            "intervention that was never meant for it"
        )
        assert "idle-leaf" not in calls
        for _ in range(300):
            active_slot.tick_once()
            if active_slot.tree_revision == 2:
                break
            time.sleep(0.005)
        assert active_slot.tree_revision == 2
    finally:
        supervisor.close()


# ---------------------------------------------------------------------------
# Task N1 (round-5 rerun fix): active-supervision first-recovery hang.
# N1b -- a missing production RecoveryKind handler must not doom the
# recovery: BtNode_RecoveryDirective returns SUCCESS (with telemetry)
# instead of FAILURE so the recover+retry Sequence proceeds to the retry.


def _proposal(kind=RecoveryKind.RETRY_NAVIGATION, **overrides) -> RecoveryProposal:
    defaults = dict(
        checkpoint_id="cp-1",
        issue_id="issue-1",
        strategy_id="retry",
        kind=kind,
        arguments={"target_location": "sofa", "approach_offset_m": 0.3, "attempts": 1},
        rationale="test",
        expected_evidence=("ok",),
        stop_conditions=("ok",),
    )
    defaults.update(overrides)
    return RecoveryProposal(**defaults)


def test_recovery_directive_without_handler_succeeds_and_emits_telemetry():
    proposal = _proposal()
    events: list[tuple[str, dict]] = []
    node = BtNode_RecoveryDirective(
        proposal,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    with patch("behavior_tree.config.is_full_mock_mode", return_value=False):
        node.tick_once()
    assert node.status is py_trees.common.Status.SUCCESS
    assert events == [("recovery.handler_missing", {"kind": "retry_navigation"})]


def test_recovery_directive_with_handler_is_consulted_and_skips_telemetry():
    proposal = _proposal()
    handled: list[RecoveryProposal] = []
    events: list[tuple[str, dict]] = []

    def handler(p: RecoveryProposal) -> bool:
        handled.append(p)
        return True

    node = BtNode_RecoveryDirective(
        proposal,
        {RecoveryKind.RETRY_NAVIGATION: handler},
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    with patch("behavior_tree.config.is_full_mock_mode", return_value=False):
        node.tick_once()
    assert node.status is py_trees.common.Status.SUCCESS
    assert handled == [proposal]
    assert events == []


def test_local_recovery_with_no_registered_handler_still_reaches_the_retry():
    # N1b end-to-end: `default_recovery_compiler()` (no handlers) used to
    # make the WHOLE recover+retry Sequence fail on the very first
    # intervention -- there are zero production RecoveryKind handlers
    # registered repo-wide. The directive must proceed to the fresh-subtask
    # retry instead.
    _seed_blackboard()
    calls: list[str] = []
    factory_calls = 0
    first_checkpoint = "task/plan-r1/step-0000:demo/tree-r1/demo/root/activation-0001"
    retry_checkpoint = "task/plan-r1/step-0000:demo/tree-r2/demo/root/activation-0001"
    client = ScriptedSupervisorClient(
        verifications=[
            {
                **_decision(first_checkpoint, verdict="recoverable"),
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
                "arguments": {"angles": [[-30, 10]], "perception_action": "find_object"},
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
    supervisor = MissionSupervisor(SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client)
    registry = NodeContractRegistry(
        [NodeContract("FakeEffect", "perception", EffectRisk.OBSERVATION, "target was observed")]
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

    with patch("behavior_tree.config.is_full_mock_mode", return_value=False):
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=factory,
            supervisor=supervisor,
            registry=registry,
            # No recovery_compiler override: the DEFAULT compiler, with no
            # handlers registered for scan_views -- exactly the production
            # gap this fix targets.
        )
        try:
            for _ in range(300):
                slot.tick_once()
                if slot.status is py_trees.common.Status.SUCCESS:
                    break
                time.sleep(0.005)
            assert slot.status is py_trees.common.Status.SUCCESS
            assert calls == ["scan-1", "scan-2"]
            assert slot.tree_revision == 2
        finally:
            supervisor.close()


# ---------------------------------------------------------------------------
# Task N1 (round-5 rerun fix): N1c -- premature intervention gating. An
# ACTIVE-mode intervention against a child that is genuinely still RUNNING
# (never terminated) is deferred until it has run longer than
# GPSR_SUPERVISION_STALL_S; a child that already reported FAILURE (masked
# as RUNNING behind the async verifier/recovery pipeline) applies
# immediately -- that is the real recovery case.


def _register_checkpoint(
    supervisor: MissionSupervisor, *, checkpoint_id: str, subtask_id: str
) -> None:
    request = CaptureRequest(
        checkpoint_id=checkpoint_id,
        task_id="task",
        subtask_id=subtask_id,
        tree_revision="1",
        plan_revision=1,
        original_instruction="test",
        subtask_goal="demo()",
        terminal_node={},
        next_node=None,
        subtask_tree={"nodes": []},
        blackboard={},
        execution_history=(),
        recovery_ledger=(),
    )
    contract = NodeContract("FakeEffect", "navigation", EffectRisk.REVERSIBLE_MOTION, "reached")
    supervisor.submit(request, contract, ReportedStatus.SUCCESS)


def test_premature_intervention_against_a_running_child_is_deferred_then_applied():
    _seed_blackboard()
    events: list[tuple[str, dict]] = []
    checkpoint_id = "premature-checkpoint"
    # The checkpoint's own async verify pipeline is scripted to resolve
    # cleanly (all_clear/agree against a SUCCESS-reported checkpoint) so it
    # can never itself append a competing intervention -- this test cares
    # only about the ONE intervention we queue by hand below.
    client = ScriptedSupervisorClient(verifications=[_decision(checkpoint_id)])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    # No contract for "FakeEffect": the child is a raw, unwrapped leaf, so
    # it can never be mistaken for an already-failed SupervisedEffect --
    # this is the "genuinely still executing" case.
    registry = NodeContractRegistry([])
    clock = {"t": 0.0}
    with patch.dict("os.environ", {"GPSR_SUPERVISION_STALL_S": "10"}):
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=lambda: FakeEffect("goto", py_trees.common.Status.RUNNING, []),
            supervisor=supervisor,
            registry=registry,
            time_source=lambda: clock["t"],
        )
    try:
        subtask_id = slot.current_subtask_id()
        _register_checkpoint(supervisor, checkpoint_id=checkpoint_id, subtask_id=subtask_id)
        proposal = _proposal(checkpoint_id=checkpoint_id)
        supervisor.ledger.register(proposal)
        supervisor._interventions.append(
            SupervisorIntervention(
                kind="local_recovery",
                checkpoint_id=checkpoint_id,
                reason="hallucinated",
                payload=proposal,
            )
        )

        # Tick #1: the slot's very first activation -- starts the child,
        # never even looks at interventions (M2b).
        slot.tick_once()
        assert slot.status is py_trees.common.Status.RUNNING
        assert len(supervisor._interventions) == 1

        # Tick #2, 1s into a still-genuinely-running attempt, well under the
        # 10s stall threshold -> deferred, not consumed, not dropped.
        clock["t"] = 1.0
        slot.tick_once()
        assert len(supervisor._interventions) == 1
        assert slot.tree_revision == 1
        deferrals = [event for event in events if event[0] == "intervention.deferred"]
        assert len(deferrals) == 1
        assert deferrals[0][1]["checkpoint_id"] == checkpoint_id

        # Tick #3, still under threshold -> deferred again, but telemetry
        # fires only once per intervention.
        clock["t"] = 2.0
        slot.tick_once()
        deferrals = [event for event in events if event[0] == "intervention.deferred"]
        assert len(deferrals) == 1

        # Tick #4, past the 10s stall threshold -> now applies.
        clock["t"] = 11.0
        slot.tick_once()
        assert len(supervisor._interventions) == 0
        assert slot.tree_revision == 2
    finally:
        supervisor.close()


def test_intervention_against_an_already_failed_child_bypasses_the_stall_threshold():
    _seed_blackboard()
    # O2/O4 fix note: this used to script an empty verifications deque so
    # the async verify() call would raise "no scripted response remains"
    # in the background almost immediately. That relied on the OLD
    # behavior where ANY non-"success" resolution (pending, or -- after a
    # race with that near-instant background error -- "stop") mapped to
    # outward RUNNING. Now "unavailable" is itself a terminal resolution
    # (see SupervisedEffect.tick()'s _UNVERIFIED_RESOLUTIONS branch), so
    # that race made the effect resolve to its real FAILURE before this
    # assertion, flaking this test on the runtime fix rather than testing
    # what it means to: hold the query genuinely pending so the RUNNING
    # assertion below is about the query, not a race with it.
    release = threading.Event()

    def delayed():
        release.wait(2)
        raise RuntimeError("still verifying")

    client = ScriptedSupervisorClient(verifications=[delayed])
    supervisor = MissionSupervisor(SupervisorConfig(mode=SupervisionMode.ACTIVE), _provider(), client)
    registry = NodeContractRegistry(
        [NodeContract("FakeEffect", "perception", EffectRisk.OBSERVATION, "observed")]
    )
    with patch.dict("os.environ", {"GPSR_SUPERVISION_STALL_S": "99999"}):
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=lambda: FakeEffect("scan", py_trees.common.Status.FAILURE, []),
            supervisor=supervisor,
            registry=registry,
        )
    try:
        # The child fails synchronously on the very first tick -- masked as
        # RUNNING (SupervisedEffect.terminal_status is FAILURE underneath)
        # while it waits on the async verdict pipeline.
        slot.tick_once()
        assert slot.status is py_trees.common.Status.RUNNING
        intervention = SupervisorIntervention(
            kind="local_recovery", checkpoint_id="irrelevant", reason="", payload=None
        )
        # Zero elapsed time, an enormous stall threshold -- still applies
        # immediately, because the child already failed.
        assert slot._may_apply_intervention_now(intervention) is True
    finally:
        release.set()
        supervisor.close()


# ---------------------------------------------------------------------------
# Y-1 (round-5 review fix, HIGH): N1c's peek-then-defer window reopens the
# X-2 orphaned-intervention hang. A "premature" intervention is peeked (not
# consumed) and left queued while its slot's child is still genuinely
# RUNNING. If, on a LATER tick, that same child independently produces a
# raw, un-instrumented FAILURE (no SupervisedEffect involved -- e.g. an
# un-wrapped guard/precondition leaf), `_child_terminal` becomes FAILURE
# AFTER that tick's own intervention-processing block already ran and
# deferred, so the slot goes straight to `_finish_hard_stop()` without ever
# re-checking the still-queued intervention. Pre-fix that intervention was
# never drained: `is_active` is permanently False from a terminal slot
# onward, so the block that could have consumed it never runs again, and
# `can_start_effect`/`can_finish_subtask`'s unscoped
# `if self._interventions: return False` gate then blocks EVERY later
# subtask for the rest of the mission.
# ---------------------------------------------------------------------------

class _RunningThenRawFailure(py_trees.behaviour.Behaviour):
    """RUNNING for the first ``running_ticks`` ticks, then a raw FAILURE
    forever after -- deliberately NOT wrapped by any NodeContract (an
    uncontracted, un-instrumented leaf), so it never becomes a
    ``SupervisedEffect`` and can never be "already failed, masked as
    RUNNING" -- exactly the un-instrumented-leaf shape Y-1 identifies as the
    newly-reachable trigger."""

    def __init__(self, name: str, running_ticks: int):
        super().__init__(name)
        self._remaining = running_ticks
        self.ticks = 0

    def update(self):
        self.ticks += 1
        if self._remaining > 0:
            self._remaining -= 1
            return py_trees.common.Status.RUNNING
        return py_trees.common.Status.FAILURE


def test_deferred_intervention_is_drained_when_an_unrelated_raw_failure_hard_stops_the_slot():
    _seed_blackboard()
    events: list[tuple[str, dict]] = []
    checkpoint_id = "stale-checkpoint"
    client = ScriptedSupervisorClient(verifications=[_decision(checkpoint_id)])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    # No contract for "FakeEffect"/"_RunningThenRawFailure": the child is a
    # raw, un-instrumented leaf -- it can never be mistaken for an
    # already-failed SupervisedEffect (see _has_failed_pending_effect).
    registry = NodeContractRegistry([])
    clock = {"t": 0.0}
    with patch.dict("os.environ", {"GPSR_SUPERVISION_STALL_S": "10"}):
        slot = SupervisedSubtaskSlot(
            action_name="demo",
            factory=lambda: _RunningThenRawFailure("goto", running_ticks=2),
            supervisor=supervisor,
            registry=registry,
            time_source=lambda: clock["t"],
        )
    try:
        subtask_id = slot.current_subtask_id()
        _register_checkpoint(supervisor, checkpoint_id=checkpoint_id, subtask_id=subtask_id)
        proposal = _proposal(checkpoint_id=checkpoint_id)
        supervisor.ledger.register(proposal)
        supervisor._interventions.append(
            SupervisorIntervention(
                kind="local_recovery",
                checkpoint_id=checkpoint_id,
                reason="hallucinated",
                payload=proposal,
            )
        )

        # Tick #1: first activation -- starts the child, never even looks at
        # interventions (M2b).
        slot.tick_once()
        assert slot.status is py_trees.common.Status.RUNNING
        assert len(supervisor._interventions) == 1

        # Tick #2: well under the 10s stall threshold, child still
        # genuinely RUNNING -> deferred (not consumed, not dropped).
        clock["t"] = 1.0
        slot.tick_once()
        assert slot.status is py_trees.common.Status.RUNNING
        assert len(supervisor._interventions) == 1
        assert [e for e in events if e[0] == "intervention.deferred"]

        # Tick #3: STILL under the stall threshold (age well below 10s), so
        # this tick's intervention block defers again -- but THIS tick the
        # child independently returns a raw FAILURE (an unrelated,
        # un-instrumented leaf; running_ticks=2 is exhausted), hard-stopping
        # the slot in the same tick, after the deferral decision was made.
        clock["t"] = 2.0
        slot.tick_once()

        assert slot.status is py_trees.common.Status.FAILURE
        outcome = py_trees.blackboard.Blackboard.get("gpsr/task_outcome")
        assert outcome["status"] == "failed"
        assert outcome["reason"] == "uncontracted_subtask_failure"
        # The core Y-1 assertion: the deferred intervention must not be
        # left behind for a dead slot -- drained, not orphaned.
        assert supervisor._interventions == []

        # And therefore a completely different, later subtask is never
        # blocked by the orphan (the actual mission-hanging symptom).
        assert supervisor.can_start_effect(
            EffectRisk.OBSERVATION, subtask_id="task/plan-r1/step-0001:next"
        ) is True
    finally:
        supervisor.close()
