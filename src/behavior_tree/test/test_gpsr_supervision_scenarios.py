from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path

from PIL import Image

from behavior_tree.GPSR.supervision.clients import ScriptedSupervisorClient
from behavior_tree.GPSR.supervision.context import (
    FixtureContextProvider,
    StaticContextProvider,
    gpsr_named_pose,
)
from behavior_tree.GPSR.supervision.controller import MissionSupervisor
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    Escalation,
    GlobalAction,
    GlobalPlanDecision,
    RecoveryKind,
    RecoveryProposal,
    ReportedStatus,
    SupervisionMode,
    SupervisorConfig,
    Verdict,
    VerificationDecision,
    WorldChange,
)
from behavior_tree.GPSR.supervision.recovery import (
    validate_global_decision,
    validate_recovery_macro,
)
from behavior_tree.GPSR.supervision.scenarios import (
    SCENARIO_CASES,
    build_capture_request,
    iter_stages,
    node_contract,
)


def _decision(stage) -> dict:
    expected = stage.verification
    return {
        "checkpoint_id": stage.stage_id,
        "verdict": expected.verdict,
        "bt_assessment": expected.bt_assessment,
        "subtask_status": expected.subtask_status,
        "world_change": expected.world_change,
        "escalation": expected.escalation,
        "failure_category": expected.failure_category,
        "evidence": ["fixture evidence"],
        "rationale": "scripted scenario expectation",
        "confidence": 0.95,
    }


def _empty_provider() -> StaticContextProvider:
    timestamp = datetime.now(timezone.utc).isoformat()
    return StaticContextProvider(
        tuple(
            ArtifactRef.absent(role, timestamp, "unit test")
            for role in ("front_camera", "wrist_camera", "map", "arm")
        )
    )


def test_catalog_contains_ten_cases_and_eleven_checkpoints() -> None:
    assert [case.number for case in SCENARIO_CASES] == list(range(1, 11))
    assert len({case.scenario_id for case in SCENARIO_CASES}) == 10
    assert len(iter_stages()) == 11
    assert len({stage.stage_id for _, stage in iter_stages()}) == 11


def test_catalog_covers_every_typed_supervisor_response() -> None:
    expectations = [stage.verification for _, stage in iter_stages()]
    assert {item.verdict for item in expectations} == {
        "all_clear",
        "recoverable",
        "unrecoverable",
        "uncertain",
    }
    assert {item.bt_assessment for item in expectations} == {
        "agree",
        "false_success",
        "false_failure",
    }
    assert {item.subtask_status for item in expectations} == {
        "achieved",
        "not_achieved",
        "unknown",
    }
    assert {item.world_change for item in expectations} == {
        "none",
        "non_destructive",
        "destructive",
        "unknown",
    }
    # O3 (prompt alignment): the sensor-mismatch scenario (case08) now
    # expects escalation=none, the least-aggressive member, instead of
    # stop -- see the O2 ruling that a bare uncertain verdict is never a
    # positive failure finding, so the verifier is no longer instructed to
    # request stop for it. No scenario is expected to request "stop"
    # anymore; the controller's own STOP branch still exists in code as a
    # backstop for a genuine positive failure finding requesting it.
    assert {item.escalation for item in expectations} == {
        "none",
        "local_recovery",
        "global_replan",
    }
    planners = [
        stage.planner
        for _, stage in iter_stages()
        if stage.planner is not None
    ]
    assert {
        item.action for item in planners if item.role == "local_recovery"
    } == {item.value for item in RecoveryKind}
    assert {
        item.action for item in planners if item.role == "global_replan"
    } == {item.value for item in GlobalAction}


def test_every_expected_verification_is_schema_valid() -> None:
    for _, stage in iter_stages():
        parsed = VerificationDecision.from_dict(_decision(stage))
        assert stage.verification.matches(parsed)


def test_terminal_subtask_state_does_not_contradict_completed_node() -> None:
    for case, stage in iter_stages():
        request = build_capture_request(case, stage)
        root = next(
            node
            for node in request.subtask_tree["nodes"]
            if node["id"] == request.subtask_tree["root"]
        )
        if request.next_node is None:
            assert root["status"] == stage.reported_status.value
        else:
            assert root["status"] == "RUNNING"


def test_all_eight_planner_actions_have_valid_canonical_payloads() -> None:
    local_arguments = {
        "scan_views": {
            "angles": [[-35, 10], [0, 10], [35, 10]],
            "perception_action": "find_object",
        },
        "reacquire_object": {
            "target": "cola",
            "viewpoints": [[-15, -10], [15, -10]],
        },
        "retry_navigation": {
            "target_location": "counter",
            "approach_offset_m": 0.25,
            "attempts": 1,
        },
        "relocalize": {"method": "spin_in_place"},
        "ask_human": {
            "prompt": "Which of the three people is Alex?",
            "confirmation_required": True,
        },
    }
    for kind, arguments in local_arguments.items():
        proposal = RecoveryProposal.from_dict(
            {
                "checkpoint_id": "cp",
                "issue_id": "issue",
                "strategy_id": f"canonical-{kind}",
                "kind": kind,
                "arguments": arguments,
                "rationale": "canonical scenario payload",
                "expected_evidence": ["expected evidence"],
                "stop_conditions": ["bounded stop"],
            }
        )
        validate_recovery_macro(proposal)

    locations = {
        case.map_pose_name for case in SCENARIO_CASES
    } | {
        name
        for case in SCENARIO_CASES
        for name in (
            case.map_goal_name,
            case.map_last_known_name,
        )
        if name
    } | {"start_position"}
    replacement = [
        {"action": "goto", "params": {"location": "refrigerator"}},
        {"action": "grasp", "params": {"object": "cola"}},
        {
            "action": "deliver",
            "params": {
                "object": "cola",
                "recipient": "me",
                "recipient_location": "start_position",
            },
        },
    ]
    globals_and_commands = (
        (
            "replan_remaining",
            "Bring me a cola.",
            replacement,
            [],
        ),
        (
            "relax_goal",
            "Bring me a zero-sugar cola from the refrigerator.",
            replacement,
            ["product attribute: zero-sugar -> regular"],
        ),
        (
            "abort_and_report",
            "Pick up the broken cola.",
            [],
            [],
        ),
    )
    for action, command, plan, relaxed in globals_and_commands:
        decision = GlobalPlanDecision.from_dict(
            {
                "checkpoint_id": "cp",
                "action": action,
                "replacement_plan": plan,
                "preserved_completed_steps": 1,
                "relaxed_constraints": relaxed,
                "rationale": "canonical scenario payload",
                "operator_message": "I will proceed safely and explain any change.",
            }
        )
        validate_global_decision(
            decision,
            completed_steps=1,
            original_instruction=command,
            known_actions={"goto", "grasp", "deliver"},
            known_locations=locations,
        )


def test_named_scenario_map_positions_are_free_space() -> None:
    fixture_dir = (
        Path(__file__).resolve().parents[1]
        / "behavior_tree/GPSR/supervision/fixtures"
    )
    image = Image.open(fixture_dir / "arena_map.pgm").convert("L")
    metadata = (fixture_dir / "arena_map.yaml").read_text(encoding="utf-8")
    assert "resolution: 0.05" in metadata
    origin_x, origin_y = -3.7, -3.48
    for case in SCENARIO_CASES:
        names = {
            case.map_pose_name,
            case.map_goal_name,
            case.map_last_known_name,
        } - {None}
        for name in names:
            x, y, _ = gpsr_named_pose(str(name))
            px = round((x - origin_x) / 0.05)
            py = round(image.height - (y - origin_y) / 0.05)
            assert image.getpixel((px, py)) >= 250, (case.scenario_id, name)


def test_fixture_manifest_covers_and_hashes_all_scenario_images(
    tmp_path: Path,
) -> None:
    provider = FixtureContextProvider(output_dir=tmp_path)
    manifest = json.loads(
        (provider.fixture_dir / "manifest.json").read_text(encoding="utf-8")
    )
    assets = manifest["assets"]
    for case in SCENARIO_CASES:
        for name in (case.front_image, case.wrist_image):
            assert name in assets
            assert (provider.fixture_dir / name).is_file()


def test_scene_only_urdf_arm_renders_all_four_runtime_poses(
    tmp_path: Path,
) -> None:
    expected_directions = {
        "arm_pos_navigating": ("upward", 30.0),
        "arm_pos_orbbec_look": ("upward", 30.0),
        "arm_pos_scan": ("downward", -20.0),
        "arm_pos_scan_original": ("downward", -40.0),
    }
    rendered = {}
    for case in SCENARIO_CASES:
        if case.arm_pose_name in rendered:
            continue
        stage = case.stages[0]
        snapshot = FixtureContextProvider(
            output_dir=tmp_path,
            scenario_id=case.scenario_id,
            require_urdf_renderer=True,
        ).capture(build_capture_request(case, stage))
        arm = next(item for item in snapshot.artifacts if item.role == "arm")
        rendered[case.arm_pose_name] = arm
        assert arm.metadata["renderer"] == "xarm_urdf_headless"
        assert arm.metadata["geometry"] == (
            "Tinker base + xarm_description visual STL"
        )
        assert arm.metadata["layout"] == "scene_only"
        assert arm.metadata["image_size"] == "768x768"
        assert Image.open(arm.path).size == (768, 768)
        direction, threshold = expected_directions[case.arm_pose_name]
        assert arm.metadata["camera_direction"].startswith(direction)
        elevation = float(arm.metadata["camera_elevation_deg"])
        if threshold > 0:
            assert elevation > threshold
        else:
            assert elevation < threshold
    assert set(rendered) == set(expected_directions)


def test_uncertain_sensor_mismatch_scenario_never_intervenes() -> None:
    # Was "test_stop_escalation_preempts_uncertain_local_recovery": under
    # the old policy a bare uncertain verdict requesting escalation=stop
    # (the sensor-context-mismatch scenario) hard-stopped the mission. O2
    # (uncertainty is not failure) removes that: the checkpoint completes
    # as "unverified" and the mission proceeds -- see the case08 scenario
    # update (scenarios.py) and the O2 ruling in the task-O brief.
    case = SCENARIO_CASES[7]
    stage = case.stages[0]
    client = ScriptedSupervisorClient(verifications=[_decision(stage)])
    supervisor = MissionSupervisor(
        SupervisorConfig(mode=SupervisionMode.ACTIVE),
        _empty_provider(),
        client,
    )
    try:
        supervisor.submit(
            build_capture_request(case, stage),
            node_contract(stage),
            ReportedStatus.FAILURE,
        )
        assert supervisor.wait_for_idle()
        assert supervisor.consume_intervention() is None
        assert supervisor.resolution(stage.stage_id) == "unverified"
        assert [role for role, _ in client.calls] == ["verify"]
        decision = supervisor.record(stage.stage_id).verification
        assert decision is not None
        assert decision.verdict is Verdict.UNCERTAIN
        assert decision.escalation is Escalation.NONE
        assert decision.world_change is WorldChange.UNKNOWN
    finally:
        supervisor.close()
