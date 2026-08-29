from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path

import pytest

from behavior_tree.GPSR.supervision.clients import ScriptedSupervisorClient
from behavior_tree.GPSR.supervision.context import (
    FixtureContextProvider,
    StaticContextProvider,
    gpsr_arm_pose_navigating,
    gpsr_arm_pose_orbbec_look,
)
from behavior_tree.GPSR.supervision.controller import MissionSupervisor
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    CaptureRequest,
    EffectRisk,
    NodeContract,
    RecoveryProposal,
    ReportedStatus,
    SchemaError,
    SuccessMode,
    SupervisionMode,
    SupervisorConfig,
    VerificationDecision,
)
from behavior_tree.GPSR.supervision.recovery import (
    RECOVERY_BUDGET_S,
    RecoveryLedger,
    recovery_budget_seconds,
)


def _request(checkpoint_id: str = "checkpoint-1") -> CaptureRequest:
    return CaptureRequest(
        checkpoint_id=checkpoint_id,
        task_id="task-1",
        subtask_id="task-1/step-0:find_object",
        tree_revision="1",
        plan_revision=1,
        original_instruction="find the coke on the table",
        subtask_goal="find_object(coke)",
        terminal_node={
            "node_id": "find/root/0",
            "class_name": "FakeScan",
            "reported_status": "SUCCESS",
        },
        next_node=None,
        subtask_tree={"nodes": [], "edges": []},
        blackboard={
            "gpsr/target_object_name": "coke",
            "gpsr/target_location": "dining_table",
        },
        execution_history=(),
        recovery_ledger=(),
        robot_pose=(1.0, 2.0, 0.3),
        arm_joints=gpsr_arm_pose_orbbec_look(),
    )


def _contract(risk: EffectRisk = EffectRisk.OBSERVATION) -> NodeContract:
    return NodeContract(
        class_name="FakeScan",
        effect="perception",
        risk=risk,
        expected_postcondition="target observation completed",
        evidence_modalities=("front_camera", "blackboard"),
    )


def _verification(
    checkpoint_id: str,
    *,
    verdict: str = "all_clear",
    assessment: str = "agree",
    escalation: str = "none",
    world_change: str = "none",
    failure_category: str = "",
):
    return {
        "checkpoint_id": checkpoint_id,
        "verdict": verdict,
        "bt_assessment": assessment,
        "subtask_status": "achieved" if verdict == "all_clear" else "not_achieved",
        "world_change": world_change,
        "escalation": escalation,
        "failure_category": failure_category,
        "evidence": ["fixture evidence"],
        "rationale": "scripted test decision",
        "confidence": 0.9,
    }


def _provider() -> StaticContextProvider:
    timestamp = datetime.now(timezone.utc).isoformat()
    return StaticContextProvider(
        tuple(ArtifactRef.absent(role, timestamp, "unit test") for role in (
            "front_camera", "wrist_camera", "map", "arm"
        ))
    )


def test_config_defaults_and_mode_toggle() -> None:
    default = SupervisorConfig.from_env({})
    assert default.mode is SupervisionMode.OFF
    assert default.success_mode is SuccessMode.HYBRID
    assert default.model == "openai/gpt-5.6-luna"
    assert default.verify_effort == "medium"
    assert default.plan_effort == "high"

    optimistic = SupervisorConfig.from_env(
        {
            "GPSR_SUPERVISION_MODE": "active",
            "GPSR_SUPERVISION_SUCCESS_MODE": "optimistic",
        }
    )
    assert optimistic.mode is SupervisionMode.ACTIVE
    assert optimistic.success_mode is SuccessMode.OPTIMISTIC


def test_fixture_provider_renders_complete_context(tmp_path: Path) -> None:
    provider = FixtureContextProvider(output_dir=tmp_path)
    snapshot = provider.capture(_request())
    assert [artifact.role for artifact in snapshot.artifacts] == [
        "front_camera", "wrist_camera", "map", "arm"
    ]
    assert all(not artifact.missing and artifact.sha256 for artifact in snapshot.artifacts)
    assert (tmp_path / "checkpoint-1-map.png").exists()
    assert (tmp_path / "checkpoint-1-arm.png").exists()
    arm = next(artifact for artifact in snapshot.artifacts if artifact.role == "arm")
    assert arm.metadata["renderer"] in {
        "xarm_urdf_headless",
        "kinematic_fallback",
    }
    if arm.metadata["renderer"] == "xarm_urdf_headless":
        assert arm.metadata["geometry"] == (
            "Tinker base + xarm_description visual STL"
        )
        assert float(arm.metadata["camera_elevation_deg"]) > 30.0
    wrist = next(
        artifact for artifact in snapshot.artifacts
        if artifact.role == "wrist_camera"
    )
    assert wrist.metadata["view_direction"] == "upward"
    assert wrist.metadata["provenance"] == "synthetic_hardware_free"


def test_navigating_pose_comes_from_gpsr_runtime_constants() -> None:
    from behavior_tree.GPSR.gpsr_full import _load_arm_constants

    expected, _ = _load_arm_constants()
    assert gpsr_arm_pose_navigating() == pytest.approx(tuple(expected))


def test_orbbec_look_pose_comes_from_gpsr_runtime_constants() -> None:
    expected_degrees = (-86.0, -52.0, 18.0, 2.0, 34.0, -73.0, -9.0)
    actual_degrees = tuple(
        round(value * 180.0 / 3.141592653589793, 6)
        for value in gpsr_arm_pose_orbbec_look()
    )
    assert actual_degrees == expected_degrees


def test_false_failure_overrides_reported_failure() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    client = ScriptedSupervisorClient(
        verifications=[
            _verification(
                "checkpoint-1",
                assessment="false_failure",
            )
        ]
    )
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        supervisor.submit(_request(), _contract(), ReportedStatus.FAILURE)
        assert supervisor.wait_for_idle()
        assert supervisor.resolution("checkpoint-1") == "success"
        assert supervisor.consume_intervention() is None
    finally:
        supervisor.close()


def test_false_success_produces_typed_local_recovery() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    client = ScriptedSupervisorClient(
        verifications=[
            _verification(
                "checkpoint-1",
                verdict="recoverable",
                assessment="false_success",
                escalation="local_recovery",
                failure_category="target_not_visible",
            )
        ],
        recoveries=[
            {
                "checkpoint_id": "checkpoint-1",
                "issue_id": "placeholder",
                "strategy_id": "scan-three-head-angles",
                "kind": "scan_views",
                "arguments": {
                    "angles": [[-35, 10], [0, 10], [35, 10]],
                    "perception_action": "find_object",
                },
                "rationale": "cover the table surface",
                "expected_evidence": ["target appears in a camera frame"],
                "stop_conditions": ["target remains absent"],
            }
        ],
    )

    # The scripted response needs the controller-generated issue id.
    original = client.plan_local_recovery

    def local(snapshot, verification, issue_id):
        raw = client._recoveries[0]
        raw["issue_id"] = issue_id
        return original(snapshot, verification, issue_id)

    client.plan_local_recovery = local
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        supervisor.submit(_request(), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        intervention = supervisor.consume_intervention()
        assert intervention is not None
        assert intervention.kind == "local_recovery"
        assert intervention.payload.kind.value == "scan_views"
        assert supervisor.ledger.failed_count(intervention.payload.issue_id) == 0
    finally:
        supervisor.close()


def test_outage_policy_continues_observation_success_but_stops_failure() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    client = ScriptedSupervisorClient(
        verifications=[RuntimeError("offline"), RuntimeError("offline")]
    )
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        supervisor.submit(_request("success"), _contract(), ReportedStatus.SUCCESS)
        supervisor.submit(_request("failure"), _contract(), ReportedStatus.FAILURE)
        assert supervisor.wait_for_idle()
        assert supervisor.record("success").unverified is True
        assert supervisor.resolution("success") == "success"
        stop = supervisor.consume_intervention()
        assert stop is not None
        assert stop.kind == "stop"
        assert stop.reason == "supervisor_unavailable"
    finally:
        supervisor.close()


def test_recovery_ledger_counts_only_distinct_executed_failures() -> None:
    ledger = RecoveryLedger(max_distinct_failures=3)

    def proposal(strategy: str, pan: int) -> RecoveryProposal:
        return RecoveryProposal.from_dict(
            {
                "checkpoint_id": "cp",
                "issue_id": "issue",
                "strategy_id": strategy,
                "kind": "scan_views",
                "arguments": {
                    "angles": [[pan, 10]],
                    "perception_action": "find_object",
                },
                "rationale": "test",
                "expected_evidence": [],
                "stop_conditions": [],
            }
        )

    first = proposal("left", -30)
    ledger.register(first)
    with pytest.raises(SchemaError, match="duplicate"):
        ledger.register(proposal("renamed-left", -30))
    assert ledger.failed_count("issue") == 0

    for item in (first, proposal("center", 0), proposal("right", 30)):
        if item is not first:
            ledger.register(item)
        ledger.mark_executed("issue", item.strategy_id)
        ledger.mark_result("issue", item.strategy_id, succeeded=False)
    assert ledger.failed_count("issue") == 3
    assert ledger.exhausted("issue")


def test_recovery_budget_seconds_table_and_env_override(monkeypatch) -> None:
    monkeypatch.delenv("GPSR_RECOVERY_BUDGET_S", raising=False)
    assert recovery_budget_seconds("find_person") == RECOVERY_BUDGET_S["find_person"]
    assert recovery_budget_seconds("find_object") == RECOVERY_BUDGET_S["find_object"]
    assert recovery_budget_seconds("search_object") == RECOVERY_BUDGET_S["search_object"]
    assert recovery_budget_seconds("some_unlisted_action") == RECOVERY_BUDGET_S["*"]

    monkeypatch.setenv("GPSR_RECOVERY_BUDGET_S", "5")
    assert recovery_budget_seconds("find_person") == 5.0
    assert recovery_budget_seconds("some_unlisted_action") == 5.0

    # Malformed/non-positive overrides fall back to the table instead of
    # silently producing a zero or negative deadline.
    monkeypatch.setenv("GPSR_RECOVERY_BUDGET_S", "not-a-number")
    assert recovery_budget_seconds("find_person") == RECOVERY_BUDGET_S["find_person"]
    monkeypatch.setenv("GPSR_RECOVERY_BUDGET_S", "-1")
    assert recovery_budget_seconds("find_person") == RECOVERY_BUDGET_S["find_person"]


def test_verification_schema_rejects_stale_shapes() -> None:
    raw = _verification("cp")
    raw["confidence"] = 1.5
    with pytest.raises(SchemaError, match="confidence"):
        VerificationDecision.from_dict(raw)


def test_destructive_change_bypasses_local_recovery() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    client = ScriptedSupervisorClient(
        verifications=[
            _verification(
                "checkpoint-1",
                verdict="unrecoverable",
                escalation="global_replan",
                world_change="destructive",
                failure_category="object_broken",
            )
        ],
        global_plans=[
            {
                "checkpoint_id": "checkpoint-1",
                "action": "abort_and_report",
                "replacement_plan": [],
                "preserved_completed_steps": 0,
                "relaxed_constraints": [],
                "rationale": "The requested bottle is broken.",
                "operator_message": "I am sorry, the bottle broke and I stopped safely.",
            }
        ],
    )
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        supervisor.submit(_request(), _contract(), ReportedStatus.FAILURE)
        assert supervisor.wait_for_idle()
        intervention = supervisor.consume_intervention()
        assert intervention is not None
        assert intervention.kind == "global_replan"
        assert intervention.payload.action.value == "abort_and_report"
        assert [role for role, _ in client.calls] == ["verify", "global"]
    finally:
        supervisor.close()


def test_three_distinct_failed_recoveries_trigger_global_replan() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE, max_recoveries=3)
    verifications = [
        _verification(
            f"checkpoint-{index}",
            verdict="recoverable",
            escalation="local_recovery",
            failure_category="target_not_visible",
        )
        for index in range(1, 4)
    ]
    recoveries = [
        {
            "checkpoint_id": f"checkpoint-{index}",
            "issue_id": "filled-by-test",
            "strategy_id": name,
            "kind": "scan_views",
            "arguments": {
                "angles": [[pan, 10]],
                "perception_action": "find_object",
            },
            "rationale": "try a distinct viewpoint",
            "expected_evidence": ["target visible"],
            "stop_conditions": ["target absent"],
        }
        for index, (name, pan) in enumerate(
            (("left", -35), ("center", 0), ("right", 35)), start=1
        )
    ]
    client = ScriptedSupervisorClient(
        verifications=verifications,
        recoveries=recoveries,
        global_plans=[
            {
                "checkpoint_id": "checkpoint-3",
                "action": "abort_and_report",
                "replacement_plan": [],
                "preserved_completed_steps": 0,
                "relaxed_constraints": [],
                "rationale": "Three distinct scans failed.",
                "operator_message": "I could not find the requested object.",
            }
        ],
    )
    original = client.plan_local_recovery

    def local(snapshot, verification, issue_id):
        client._recoveries[0]["issue_id"] = issue_id
        return original(snapshot, verification, issue_id)

    client.plan_local_recovery = local
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        issue_id = None
        for index in range(1, 4):
            supervisor.submit(
                _request(f"checkpoint-{index}"),
                _contract(),
                ReportedStatus.FAILURE,
            )
            assert supervisor.wait_for_idle()
            intervention = supervisor.consume_intervention()
            assert intervention is not None
            assert intervention.kind == "local_recovery"
            proposal = intervention.payload
            issue_id = proposal.issue_id
            supervisor.recovery_started(proposal)
            supervisor.recovery_finished(proposal, succeeded=False)
        assert supervisor.wait_for_idle()
        global_intervention = supervisor.consume_intervention()
        assert global_intervention is not None
        assert global_intervention.kind == "global_replan"
        assert supervisor.ledger.failed_count(issue_id) == 3
    finally:
        supervisor.close()
