from __future__ import annotations

from dataclasses import replace
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
    RecoveryKind,
    RecoveryProposal,
    ReportedStatus,
    SchemaError,
    SuccessMode,
    SupervisionMode,
    SupervisorConfig,
    VerificationDecision,
)
from behavior_tree.GPSR.supervision.prompts import VERIFIER_SYSTEM_PROMPT
from behavior_tree.GPSR.supervision.recovery import RecoveryLedger
from unittest.mock import patch


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
    # O1 (honest fixture artifacts): the hardware-free default (no
    # scenario_id) has no live camera feed, so front/wrist camera artifacts
    # must be honestly reported as missing rather than sent to the verifier
    # as if they were real frames. The map/arm renders remain live-grounded
    # from the request's actual robot_pose/arm_joints, so they still render.
    provider = FixtureContextProvider(output_dir=tmp_path)
    snapshot = provider.capture(_request())
    assert [artifact.role for artifact in snapshot.artifacts] == [
        "front_camera", "wrist_camera", "map", "arm"
    ]
    by_role = {artifact.role: artifact for artifact in snapshot.artifacts}
    assert by_role["front_camera"].missing is True
    assert by_role["front_camera"].path is None
    assert "hardware-free" in by_role["front_camera"].metadata["reason"]
    assert by_role["wrist_camera"].missing is True
    assert by_role["wrist_camera"].path is None
    assert "hardware-free" in by_role["wrist_camera"].metadata["reason"]
    assert not by_role["map"].missing and by_role["map"].sha256
    assert not by_role["arm"].missing and by_role["arm"].sha256
    assert (tmp_path / "checkpoint-1-map.png").exists()
    assert (tmp_path / "checkpoint-1-arm.png").exists()
    arm = by_role["arm"]
    assert arm.metadata["renderer"] in {
        "xarm_urdf_headless",
        "kinematic_fallback",
    }
    if arm.metadata["renderer"] == "xarm_urdf_headless":
        assert arm.metadata["geometry"] == (
            "Tinker base + xarm_description visual STL"
        )
        assert float(arm.metadata["camera_elevation_deg"]) > 30.0


def test_fixture_provider_keeps_live_camera_frames_with_a_scenario_id(
    tmp_path: Path,
) -> None:
    # O1: a scenario_id names fixture images that actually depict the
    # scenario, so today's behavior (real, present artifacts) is preserved.
    provider = FixtureContextProvider(
        output_dir=tmp_path, scenario_id="case01-arrival-clear"
    )
    snapshot = provider.capture(_request())
    by_role = {artifact.role: artifact for artifact in snapshot.artifacts}
    assert by_role["front_camera"].missing is False
    assert by_role["front_camera"].sha256
    assert by_role["wrist_camera"].missing is False
    assert by_role["wrist_camera"].sha256
    assert by_role["wrist_camera"].metadata["view_direction"] == (
        "near_vertical_upward"
    )


def test_fixture_provider_never_labels_a_static_artifact_as_vision_log(
    tmp_path: Path,
) -> None:
    # O1: provenance strings must state what the content IS, never imply a
    # live capture. "vision_log" falsely implied a real logged observation
    # for a static, committed fixture image.
    provider = FixtureContextProvider(
        output_dir=tmp_path, scenario_id="case01-arrival-clear"
    )
    snapshot = provider.capture(_request())
    front = next(a for a in snapshot.artifacts if a.role == "front_camera")
    assert front.metadata["provenance"] == "static_fixture"
    for artifact in snapshot.artifacts:
        assert artifact.metadata.get("provenance") != "vision_log"


# N1d (round-5 rerun fix): without a scenario_id -- the actual production
# path, since `configure_default_supervisor` builds a bare
# `FixtureContextProvider()` -- `capture()` used to send a map fixture with
# no goal/last-known marker at all, no matter what was actually on the
# blackboard, leaving the verifier with nothing to ground a nominal
# in-flight action against ("cannot verify -> recoverable" for everything).
def test_fixture_provider_uses_live_blackboard_goal_without_scenario(
    tmp_path: Path,
) -> None:
    provider = FixtureContextProvider(output_dir=tmp_path)
    request = replace(
        _request(),
        blackboard={
            "gpsr/target_object_name": "coke",
            "gpsr/target_location": "dinner_table",
        },
    )
    snapshot = provider.capture(request)
    map_artifact = next(a for a in snapshot.artifacts if a.role == "map")
    assert map_artifact.metadata["goal_pose_name"] == "dinner_table"


def test_fixture_provider_uses_live_last_capture_without_scenario(
    tmp_path: Path,
) -> None:
    provider = FixtureContextProvider(output_dir=tmp_path)
    request = replace(
        _request(),
        blackboard={
            "gpsr/last_capture": {
                "pose": {
                    "position": {"x": 1.5, "y": -0.5, "z": 0.0},
                    "orientation": {"w": 1.0},
                }
            }
        },
    )
    snapshot = provider.capture(request)
    map_artifact = next(a for a in snapshot.artifacts if a.role == "map")
    assert map_artifact.metadata["last_known_pose_name"] == "gpsr/last_capture"


def test_fixture_provider_omits_live_evidence_without_a_populated_blackboard(
    tmp_path: Path,
) -> None:
    provider = FixtureContextProvider(output_dir=tmp_path)
    request = replace(_request(), blackboard={})
    snapshot = provider.capture(request)  # must not raise
    map_artifact = next(a for a in snapshot.artifacts if a.role == "map")
    assert map_artifact.metadata["goal_pose_name"] is None
    assert map_artifact.metadata["last_known_pose_name"] is None


def test_fixture_provider_ignores_an_unknown_target_location_name(
    tmp_path: Path,
) -> None:
    # `_request()` itself sets gpsr/target_location to "dining_table", which
    # is not a real named pose (the real key is "dinner_table") -- capture()
    # must still surface the raw location text as evidence (useful even
    # un-rendered) but never crash trying to resolve/draw an unrecognised
    # name.
    provider = FixtureContextProvider(output_dir=tmp_path)
    snapshot = provider.capture(_request())  # must not raise
    map_artifact = next(a for a in snapshot.artifacts if a.role == "map")
    assert map_artifact.metadata["goal_pose_name"] == "dining_table"


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


def test_uncertain_verdict_never_produces_an_intervention() -> None:
    # O2: uncertainty is not failure. A verifier that cannot confirm the
    # world state (verdict=uncertain) admits it has nothing actionable to
    # say -- even when it also asks for escalation=stop, as the old prompt
    # instructed for a sensor-context mismatch. The controller must never
    # turn that into an intervention; it marks the checkpoint "unverified"
    # and lets the mission proceed.
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    events: list[tuple[str, dict]] = []
    client = ScriptedSupervisorClient(
        verifications=[
            _verification(
                "checkpoint-1",
                verdict="uncertain",
                escalation="stop",
                world_change="unknown",
                failure_category="sensor_context_mismatch",
            )
        ]
    )
    supervisor = MissionSupervisor(
        config,
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    try:
        supervisor.submit(_request(), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        assert supervisor.consume_intervention() is None
        assert supervisor.resolution("checkpoint-1") == "unverified"
        assert supervisor.can_finish_subtask(_request().subtask_id) is True
        downgrades = [payload for event, payload in events if event == "supervisor.verdict.downgraded"]
        assert len(downgrades) == 1
        assert downgrades[0]["checkpoint_id"] == "checkpoint-1"
        assert downgrades[0]["failure_category"] == "sensor_context_mismatch"
        assert downgrades[0]["escalation_requested"] == "stop"
    finally:
        supervisor.close()


def test_unknown_subtask_status_never_produces_an_intervention() -> None:
    # O2 also covers subtask_status=unknown -- the verifier admitting it
    # cannot tell whether the subtask itself was achieved, independent of
    # the top-level verdict label.
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    client = ScriptedSupervisorClient(
        verifications=[
            {
                "checkpoint_id": "checkpoint-1",
                "verdict": "recoverable",
                "bt_assessment": "agree",
                "subtask_status": "unknown",
                "world_change": "non_destructive",
                "escalation": "stop",
                "failure_category": "context_incomplete",
                "evidence": ["fixture evidence"],
                "rationale": "scripted test decision",
                "confidence": 0.9,
            }
        ]
    )
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        supervisor.submit(_request(), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        assert supervisor.consume_intervention() is None
        assert supervisor.resolution("checkpoint-1") == "unverified"
    finally:
        supervisor.close()


def test_positive_failure_verdict_with_stop_escalation_still_hard_stops() -> None:
    # O2 must not over-broaden: a POSITIVE failure finding (recoverable or
    # unrecoverable -- the verifier actually identifying a problem, not
    # merely admitting uncertainty) that requests escalation=stop still
    # hard-stops the mission exactly as before.
    for verdict in ("recoverable", "unrecoverable"):
        config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
        client = ScriptedSupervisorClient(
            verifications=[
                _verification(
                    "checkpoint-1",
                    verdict=verdict,
                    escalation="stop",
                    world_change="non_destructive",
                    failure_category="object_broken",
                )
            ]
        )
        supervisor = MissionSupervisor(config, _provider(), client)
        try:
            supervisor.submit(_request(), _contract(), ReportedStatus.FAILURE)
            assert supervisor.wait_for_idle()
            intervention = supervisor.consume_intervention()
            assert intervention is not None
            assert intervention.kind == "stop"
            assert supervisor.resolution("checkpoint-1") == "stop"
        finally:
            supervisor.close()


def test_verifier_prompt_instructs_the_downgraded_mismatch_escalation() -> None:
    # O3 (prompt alignment): the instructed response for a material
    # sensor-context mismatch must match the O2 enforcement policy -- a
    # mismatch is an inability to verify, not a positive failure finding, so
    # the least-aggressive escalation member (none) is instructed, not stop.
    assert "escalation=none, failure_category=sensor_context_mismatch" in (
        VERIFIER_SYSTEM_PROMPT
    )
    assert "escalation=stop" not in VERIFIER_SYSTEM_PROMPT


def test_verifier_prompt_tells_the_model_missing_artifacts_are_not_failure() -> None:
    prompt = VERIFIER_SYSTEM_PROMPT.lower()
    assert "missing" in prompt and "not evidence" in prompt


def test_outage_policy_never_stops_the_mission() -> None:
    # Was "test_outage_policy_continues_observation_success_but_stops_failure":
    # O4 removes the stop-on-unavailable branch entirely. A broken/offline
    # verifier is an LLM formatting hiccup, not a mission failure -- it
    # must cost nothing, for an observation-risk SUCCESS checkpoint or any
    # other kind (including a reported FAILURE, which used to hard-stop).
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    events: list[tuple[str, dict]] = []
    client = ScriptedSupervisorClient(
        verifications=[RuntimeError("offline"), RuntimeError("offline")]
    )
    supervisor = MissionSupervisor(
        config,
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    try:
        supervisor.submit(_request("success"), _contract(), ReportedStatus.SUCCESS)
        supervisor.submit(_request("failure"), _contract(), ReportedStatus.FAILURE)
        assert supervisor.wait_for_idle()
        assert supervisor.record("success").unverified is True
        assert supervisor.resolution("success") == "unavailable"
        assert supervisor.record("failure").unverified is True
        assert supervisor.resolution("failure") == "unavailable"
        assert supervisor.consume_intervention() is None
        unavailable = [p for e, p in events if e == "supervisor.unavailable"]
        assert len(unavailable) == 2
        assert all(p["continued_unverified"] is True for p in unavailable)
    finally:
        supervisor.close()


def test_consecutive_query_errors_degrade_supervision_without_stopping() -> None:
    # O4: after N consecutive unavailable results (here N=3), supervision
    # degrades to off for the rest of the run -- a broken verifier must
    # cost the mission nothing, not even ongoing LLM calls that will keep
    # failing. supervisor.degraded fires exactly once; existing/new
    # checkpoints still complete (undegraded, i.e. resolved from
    # reported_status) without ever creating another query.
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE, max_consecutive_errors=3)
    events: list[tuple[str, dict]] = []
    client = ScriptedSupervisorClient(
        verifications=[RuntimeError("offline")] * 3
    )
    supervisor = MissionSupervisor(
        config,
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    try:
        for index in range(3):
            supervisor.submit(
                _request(f"cp-{index}"), _contract(), ReportedStatus.SUCCESS
            )
        assert supervisor.wait_for_idle()
        assert supervisor.consume_intervention() is None
        degraded = [p for e, p in events if e == "supervisor.degraded"]
        assert len(degraded) == 1

        supervisor.submit(_request("cp-after"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.resolution("cp-after") == "success"
        assert supervisor.record("cp-after").unverified is False
        # no fourth verify call was made for cp-after
        assert [role for role, _ in client.calls] == ["verify", "verify", "verify"]
    finally:
        supervisor.close()


def test_consecutive_error_counter_resets_on_a_successful_verdict() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE, max_consecutive_errors=2)
    events: list[tuple[str, dict]] = []
    client = ScriptedSupervisorClient(
        verifications=[
            RuntimeError("offline"),
            _verification("cp-1"),
            RuntimeError("offline"),
            RuntimeError("offline"),
        ]
    )
    supervisor = MissionSupervisor(
        config,
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    try:
        supervisor.submit(_request("cp-error-a"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        assert not [p for e, p in events if e == "supervisor.degraded"]

        supervisor.submit(_request("cp-1"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        assert supervisor.resolution("cp-1") == "success"
        assert not [p for e, p in events if e == "supervisor.degraded"]

        supervisor.submit(_request("cp-error-b"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        assert not [p for e, p in events if e == "supervisor.degraded"]

        supervisor.submit(_request("cp-error-c"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        degraded = [p for e, p in events if e == "supervisor.degraded"]
        assert len(degraded) == 1
    finally:
        supervisor.close()


def test_config_default_and_env_max_consecutive_errors() -> None:
    default = SupervisorConfig.from_env({})
    assert default.max_consecutive_errors == 5
    custom = SupervisorConfig.from_env(
        {"GPSR_SUPERVISION_MAX_CONSECUTIVE_ERRORS": "2"}
    )
    assert custom.max_consecutive_errors == 2


# ---------------------------------------------------------------------------
# Task Q4 (round-6, supervision-economics fix): cumulative error/overhead
# budgets degrade supervision. The consecutive-only degrade policy
# (max_consecutive_errors, reset on any success) let 10 total errors across
# the measured 019 run never degrade anything -- its longest streak was only
# 3. Alongside the existing consecutive counter: a per-run TOTAL error budget
# (never reset within the run) and a cumulative overhead budget (deferral
# ages + recovery-cycle spans).


def test_config_default_and_env_total_errors_and_overhead_budget() -> None:
    default = SupervisorConfig.from_env({})
    assert default.max_total_errors == 12
    assert default.overhead_budget_s == 300.0
    custom = SupervisorConfig.from_env(
        {
            "GPSR_SUPERVISION_MAX_TOTAL_ERRORS": "3",
            "GPSR_SUPERVISION_OVERHEAD_BUDGET_S": "45",
        }
    )
    assert custom.max_total_errors == 3
    assert custom.overhead_budget_s == 45.0


def test_total_error_and_overhead_counters_start_at_zero_on_a_fresh_controller() -> None:
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE)
    supervisor = MissionSupervisor(config, _provider(), ScriptedSupervisorClient())
    try:
        assert supervisor._total_errors == 0
        assert supervisor._overhead_s == 0.0
        assert supervisor._degraded is False
    finally:
        supervisor.close()


def test_twelve_non_consecutive_errors_degrade_via_the_total_error_budget() -> None:
    # 12 errors, each immediately followed by a successful verdict that
    # resets the CONSECUTIVE counter -- it never gets anywhere near
    # max_consecutive_errors (5, well above the longest streak of 1 here).
    # The cumulative TOTAL budget (12) still degrades supervision.
    config = SupervisorConfig(
        mode=SupervisionMode.ACTIVE, max_consecutive_errors=5, max_total_errors=12
    )
    events: list[tuple[str, dict]] = []
    verifications: list = []
    for index in range(12):
        verifications.append(RuntimeError("offline"))
        verifications.append(_verification(f"cp-ok-{index}"))
    client = ScriptedSupervisorClient(verifications=verifications)
    supervisor = MissionSupervisor(
        config,
        _provider(),
        client,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    try:
        for index in range(12):
            supervisor.submit(
                _request(f"cp-err-{index}"), _contract(), ReportedStatus.SUCCESS
            )
            assert supervisor.wait_for_idle()
            supervisor.submit(
                _request(f"cp-ok-{index}"), _contract(), ReportedStatus.SUCCESS
            )
            assert supervisor.wait_for_idle()
        assert supervisor._total_errors == 12
        # The consecutive counter was reset to 0 by every "-ok-" success
        # BEFORE the final error -- it never got anywhere near
        # max_consecutive_errors (5); at most 1 by the time the 12th error
        # itself trips the total budget (its own paired success is skipped
        # once degraded, per O4's skip_query gate, so it never gets a
        # chance to reset the counter one last time -- that is the
        # existing O4 behavior, not part of what Q4 changes).
        assert supervisor._consecutive_errors <= 1
        degraded = [p for e, p in events if e == "supervisor.degraded"]
        assert len(degraded) == 1
        assert degraded[0]["reason"] == "total_errors"
        assert supervisor._degraded is True
    finally:
        supervisor.close()


def test_overhead_budget_crossing_300s_degrades_supervision() -> None:
    # (i) each applied intervention's deferral age -- reported directly via
    # `record_deferral_overhead`, the same seam `SupervisedSubtaskSlot.
    # _note_deferral` reports through -- and (ii) each recovery cycle's
    # span (`recovery_started` -> `recovery_finished`) both accumulate.
    config = SupervisorConfig(mode=SupervisionMode.ACTIVE, overhead_budget_s=100.0)
    events: list[tuple[str, dict]] = []
    supervisor = MissionSupervisor(
        config,
        _provider(),
        ScriptedSupervisorClient(),
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    try:
        supervisor.record_deferral_overhead(40.0)
        assert supervisor._overhead_s == 40.0
        assert supervisor._degraded is False
        assert not [p for e, p in events if e == "supervisor.degraded"]

        proposal = RecoveryProposal(
            checkpoint_id="cp-1",
            issue_id="issue-1",
            strategy_id="s1",
            kind=RecoveryKind.SCAN_VIEWS,
            arguments={"angles": [[0, 0]], "perception_action": "x"},
            rationale="r",
            expected_evidence=("e",),
            stop_conditions=("s",),
        )
        supervisor.ledger.register(proposal)
        # Patch the controller module's clock (not the slot's injectable
        # one -- `recovery_started`/`recovery_finished` read `time.
        # monotonic` directly) so the test does not need to sleep 70s.
        with patch(
            "behavior_tree.GPSR.supervision.controller.time.monotonic",
            side_effect=[1000.0, 1070.0],
        ):
            supervisor.recovery_started(proposal)
            supervisor.recovery_finished(proposal, succeeded=True)

        # 40.0 + 70.0 = 110.0 >= 100.0 -> degraded.
        assert supervisor._overhead_s == 110.0
        degraded = [p for e, p in events if e == "supervisor.degraded"]
        assert len(degraded) == 1
        assert degraded[0]["reason"] == "overhead_budget"
        assert supervisor._degraded is True
    finally:
        supervisor.close()


def test_total_error_and_overhead_budgets_degrade_in_shadow_without_skipping_queries() -> None:
    # Shadow: the accumulators/degrade flag may still tick (telemetry
    # only) but must change no OBSERVABLE behavior -- SHADOW always
    # queries regardless of `_degraded` (MissionSupervisor.submit's
    # skip_query gate is `mode is ACTIVE and self._degraded`).
    config = SupervisorConfig(mode=SupervisionMode.SHADOW, max_total_errors=2)
    client = ScriptedSupervisorClient(
        verifications=[
            RuntimeError("offline"),
            RuntimeError("offline"),
            _verification("cp-after"),
        ]
    )
    supervisor = MissionSupervisor(config, _provider(), client)
    try:
        supervisor.submit(_request("cp-1"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        supervisor.submit(_request("cp-2"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        assert supervisor._total_errors == 2
        assert supervisor._degraded is True

        supervisor.submit(_request("cp-after"), _contract(), ReportedStatus.SUCCESS)
        assert supervisor.wait_for_idle()
        # Every submission still queried -- degraded had no effect in SHADOW.
        assert [role for role, _ in client.calls] == ["verify", "verify", "verify"]
        assert supervisor.record("cp-after").stage == "shadow_complete"
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
