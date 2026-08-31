"""Shared hardware-free VLM validation scenarios.

The catalogue is intentionally data-only at the model boundary: expected
answers are used by tests and the debugger, but are never included in a model
prompt.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping

from .context import gpsr_arm_pose, gpsr_named_pose
from .models import (
    CaptureRequest,
    EffectRisk,
    NodeContract,
    ReportedStatus,
    VerificationDecision,
)


@dataclass(frozen=True)
class VerificationExpectation:
    verdict: str
    bt_assessment: str
    subtask_status: str
    world_change: str
    escalation: str
    failure_category: str = ""
    evidence_terms: tuple[str, ...] = ()

    def matches(self, decision: VerificationDecision) -> bool:
        return (
            decision.verdict.value == self.verdict
            and decision.bt_assessment.value == self.bt_assessment
            and decision.subtask_status.value == self.subtask_status
            and decision.world_change.value == self.world_change
            and decision.escalation.value == self.escalation
            and (
                not self.failure_category
                or decision.failure_category == self.failure_category
            )
        )


@dataclass(frozen=True)
class PlannerExpectation:
    role: str
    action: str
    reason: str


@dataclass(frozen=True)
class ScenarioStage:
    stage_id: str
    title: str
    original_instruction: str
    subtask_goal: str
    reported_status: ReportedStatus
    class_name: str
    node_name: str
    effect: str
    risk: EffectRisk
    expected_postcondition: str
    blackboard: Mapping[str, Any]
    verification: VerificationExpectation
    planner: PlannerExpectation | None = None
    recovery_ledger: tuple[Mapping[str, Any], ...] = ()
    completed_steps: int = 0


@dataclass(frozen=True)
class ScenarioCase:
    number: int
    scenario_id: str
    title: str
    front_image: str
    wrist_image: str
    wrist_view_direction: str
    arm_pose_name: str
    map_pose_name: str
    map_goal_name: str | None = None
    map_last_known_name: str | None = None
    map_uncertainty_radius_m: float | None = None
    generated: bool = False
    stages: tuple[ScenarioStage, ...] = field(default_factory=tuple)


def _verify(
    verdict: str,
    assessment: str,
    subtask: str,
    world: str,
    escalation: str,
    *,
    category: str = "",
    evidence: tuple[str, ...] = (),
) -> VerificationExpectation:
    return VerificationExpectation(
        verdict=verdict,
        bt_assessment=assessment,
        subtask_status=subtask,
        world_change=world,
        escalation=escalation,
        failure_category=category,
        evidence_terms=evidence,
    )


def _stage(
    stage_id: str,
    title: str,
    instruction: str,
    goal: str,
    status: str,
    class_name: str,
    node_name: str,
    effect: str,
    risk: EffectRisk,
    postcondition: str,
    blackboard: Mapping[str, Any],
    verification: VerificationExpectation,
    *,
    planner: PlannerExpectation | None = None,
    recovery_ledger: tuple[Mapping[str, Any], ...] = (),
    completed_steps: int = 0,
) -> ScenarioStage:
    return ScenarioStage(
        stage_id=stage_id,
        title=title,
        original_instruction=instruction,
        subtask_goal=goal,
        reported_status=ReportedStatus(status),
        class_name=class_name,
        node_name=node_name,
        effect=effect,
        risk=risk,
        expected_postcondition=postcondition,
        blackboard=dict(blackboard),
        verification=verification,
        planner=planner,
        recovery_ledger=recovery_ledger,
        completed_steps=completed_steps,
    )


_THREE_FAILED_SCANS: tuple[Mapping[str, Any], ...] = (
    {
        "kind": "scan_views",
        "strategy_id": "scan-left",
        "arguments": {"angles": [[-40, 10]], "perception_action": "find_object"},
        "state": "failed",
        "succeeded": False,
    },
    {
        "kind": "scan_views",
        "strategy_id": "scan-center-high",
        "arguments": {"angles": [[0, 28]], "perception_action": "find_object"},
        "state": "failed",
        "succeeded": False,
    },
    {
        "kind": "scan_views",
        "strategy_id": "scan-right-low",
        "arguments": {"angles": [[40, -5]], "perception_action": "find_object"},
        "state": "failed",
        "succeeded": False,
    },
)


SCENARIO_CASES: tuple[ScenarioCase, ...] = (
    ScenarioCase(
        number=1,
        scenario_id="case01-arrival-clear",
        title="Arrival confirmed",
        front_image="front_camera.jpg",
        wrist_image="wrist_camera.jpg",
        wrist_view_direction="near_vertical_upward",
        arm_pose_name="arm_pos_navigating",
        map_pose_name="dinner_table",
        map_goal_name="dinner_table",
        stages=(
            _stage(
                "case01-arrival-clear",
                "Navigation success is physically supported",
                "Go to the dinner table.",
                "arrive at dinner_table and face the table",
                "SUCCESS",
                "BtNode_GotoAction",
                "go to dinner table",
                "navigation",
                EffectRisk.REVERSIBLE_MOTION,
                "robot pose is at the dinner_table goal",
                {
                    "gpsr/target_location": "dinner_table",
                    "gpsr/navigation_result": "goal_reached",
                    "gpsr/localization_confidence": 0.96,
                },
                _verify(
                    "all_clear",
                    "agree",
                    "achieved",
                    "none",
                    "none",
                    evidence=("goal", "map", "table"),
                ),
                completed_steps=1,
            ),
        ),
    ),
    ScenarioCase(
        number=2,
        scenario_id="case02-false-failure-wave",
        title="Visible wave overrides BT failure",
        front_image="scenarios/case02-front-waving.png",
        wrist_image="scenarios/sofa-wrist-upper-wall.png",
        wrist_view_direction="upward",
        arm_pose_name="arm_pos_orbbec_look",
        map_pose_name="sofa",
        map_goal_name="sofa",
        generated=True,
        stages=(
            _stage(
                "case02-false-failure-wave",
                "False failure: waving person is visible",
                "Find the person who is waving.",
                "identify a visibly waving person near the sofa",
                "FAILURE",
                "BtNode_DetectWavingPersons",
                "detect waving person",
                "perception",
                EffectRisk.OBSERVATION,
                "at least one person is visibly waving",
                {
                    "gpsr/target_person_descriptor": "waving",
                    "gpsr/waving_detection_count": 0,
                    "gpsr/target_location": "sofa",
                },
                _verify(
                    "all_clear",
                    "false_failure",
                    "achieved",
                    "none",
                    "none",
                    evidence=("wave", "raised hand", "person"),
                ),
            ),
        ),
    ),
    ScenarioCase(
        number=3,
        scenario_id="case03-false-success-budget",
        title="False success, bounded scan, then replan",
        front_image="front_camera.jpg",
        wrist_image="wrist_camera.jpg",
        wrist_view_direction="upward",
        arm_pose_name="arm_pos_orbbec_look",
        map_pose_name="dinner_table",
        map_goal_name="dinner_table",
        stages=(
            _stage(
                "case03a-false-success-scan",
                "False success requests a coverage scan",
                "Bring me a cola.",
                "find a cola at the dinner_table before grasping",
                "SUCCESS",
                "BtNode_ScanForGeneralist",
                "find cola",
                "perception",
                EffectRisk.OBSERVATION,
                "a cola is visibly detected and stored on the blackboard",
                {
                    "gpsr/target_object_name": "cola",
                    "gpsr/target_location": "dinner_table",
                    "gpsr/detected_objects": [],
                    "gpsr/candidate_locations": ["refrigerator"],
                },
                _verify(
                    "recoverable",
                    "false_success",
                    "not_achieved",
                    "non_destructive",
                    "local_recovery",
                    category="target_not_visible",
                    evidence=("cola", "not visible", "table"),
                ),
                planner=PlannerExpectation(
                    "local_recovery", "scan_views", "target_not_visible"
                ),
                completed_steps=1,
            ),
            _stage(
                "case03b-budget-exhausted-replan",
                "Three failed local strategies trigger a minimal replan",
                "Bring me a cola.",
                "find a cola at the dinner_table before grasping",
                "FAILURE",
                "BtNode_ScanForGeneralist",
                "find cola after bounded recovery",
                "perception",
                EffectRisk.OBSERVATION,
                "a cola is visibly detected and stored on the blackboard",
                {
                    "gpsr/target_object_name": "cola",
                    "gpsr/target_location": "dinner_table",
                    "gpsr/detected_objects": [],
                    "gpsr/candidate_locations": ["refrigerator"],
                    "gpsr/recovery_budget": 3,
                    "gpsr/recovery_failures": 3,
                },
                _verify(
                    "unrecoverable",
                    "agree",
                    "not_achieved",
                    "non_destructive",
                    "global_replan",
                    category="recovery_budget_exhausted",
                    evidence=("three", "recovery", "cola"),
                ),
                planner=PlannerExpectation(
                    "global_replan",
                    "replan_remaining",
                    "recovery_budget_exhausted",
                ),
                recovery_ledger=_THREE_FAILED_SCANS,
                completed_steps=1,
            ),
        ),
    ),
    ScenarioCase(
        number=4,
        scenario_id="case04-reacquire-occluded",
        title="Reacquire a previously tracked object",
        front_image="scenarios/case04-front-occluded-cola.png",
        wrist_image="scenarios/case04-wrist-occluded-cola.png",
        wrist_view_direction="downward",
        arm_pose_name="arm_pos_scan",
        map_pose_name="dinner_table",
        map_goal_name="dinner_table",
        generated=True,
        stages=(
            _stage(
                "case04-reacquire-occluded",
                "A prior target lock was lost behind other bottles",
                "Pick up the cola from the dinner table.",
                "restore the existing cola track before grasping",
                "FAILURE",
                "BtNode_TrackObject",
                "track previously detected cola",
                "perception",
                EffectRisk.OBSERVATION,
                "the previously detected cola has a stable current track",
                {
                    "gpsr/target_object_name": "cola",
                    "gpsr/target_location": "dinner_table",
                    "gpsr/previous_target_lock": True,
                    "gpsr/last_seen_seconds_ago": 1.7,
                    "gpsr/track_confidence": 0.18,
                    "gpsr/occlusion_hint": "clear bottle and carton",
                },
                _verify(
                    "recoverable",
                    "agree",
                    "not_achieved",
                    "non_destructive",
                    "local_recovery",
                    category="target_track_lost",
                    evidence=("occlud", "partially", "bottle"),
                ),
                planner=PlannerExpectation(
                    "local_recovery", "reacquire_object", "target_track_lost"
                ),
            ),
        ),
    ),
    ScenarioCase(
        number=5,
        scenario_id="case05-retry-navigation",
        title="Retry navigation from a trustworthy pose",
        front_image="front_camera.jpg",
        wrist_image="wrist_camera.jpg",
        wrist_view_direction="near_vertical_upward",
        arm_pose_name="arm_pos_navigating",
        map_pose_name="coffee_table",
        map_goal_name="counter",
        stages=(
            _stage(
                "case05-retry-navigation",
                "Robot stopped at the wrong table but remains localized",
                "Go to the kitchen counter.",
                "reach the counter approach pose",
                "FAILURE",
                "BtNode_GotoAction",
                "go to counter",
                "navigation",
                EffectRisk.REVERSIBLE_MOTION,
                "robot pose is within the counter goal tolerance",
                {
                    "gpsr/target_location": "counter",
                    "gpsr/current_location": "coffee_table",
                    "gpsr/navigation_result": "controller_patience_exceeded",
                    "gpsr/localization_confidence": 0.94,
                    "gpsr/costmap_state": "healthy",
                },
                _verify(
                    "recoverable",
                    "agree",
                    "not_achieved",
                    "non_destructive",
                    "local_recovery",
                    category="navigation_goal_not_reached",
                    evidence=("goal", "counter", "map"),
                ),
                planner=PlannerExpectation(
                    "local_recovery",
                    "retry_navigation",
                    "navigation_goal_not_reached",
                ),
            ),
        ),
    ),
    ScenarioCase(
        number=6,
        scenario_id="case06-relocalize",
        title="Relocalize before motion",
        front_image="scenarios/case06-front-featureless-corridor.png",
        wrist_image="scenarios/case06-wrist-corridor-ceiling.png",
        wrist_view_direction="near_vertical_upward",
        arm_pose_name="arm_pos_navigating",
        map_pose_name="refrigerator",
        map_goal_name="counter",
        map_last_known_name="coffee_table",
        map_uncertainty_radius_m=2.0,
        generated=True,
        stages=(
            _stage(
                "case06-relocalize",
                "Pose confidence collapsed in a feature-poor corridor",
                "Go to the kitchen counter.",
                "restore a trustworthy map pose before continuing navigation",
                "FAILURE",
                "BtNode_GotoAction",
                "navigate with degraded localization",
                "navigation",
                EffectRisk.REVERSIBLE_MOTION,
                "localization is trustworthy enough to resume navigation",
                {
                    "gpsr/target_location": "counter",
                    "gpsr/last_known_location": "coffee_table",
                    "gpsr/localization_confidence": 0.14,
                    "gpsr/localization_uncertainty_radius_m": 2.0,
                    "gpsr/pose_jump_detected": True,
                    "gpsr/navigation_result": "localization_lost",
                },
                _verify(
                    "recoverable",
                    "agree",
                    "not_achieved",
                    "non_destructive",
                    "local_recovery",
                    category="localization_unreliable",
                    evidence=("uncertainty", "localization", "pose"),
                ),
                planner=PlannerExpectation(
                    "local_recovery", "relocalize", "localization_unreliable"
                ),
            ),
        ),
    ),
    ScenarioCase(
        number=7,
        scenario_id="case07-ask-human",
        title="Ask for missing identity information",
        front_image="scenarios/sofa-front-three-people.jpg",
        wrist_image="scenarios/sofa-wrist-upper-wall.png",
        wrist_view_direction="upward",
        arm_pose_name="arm_pos_orbbec_look",
        map_pose_name="sofa",
        map_goal_name="sofa",
        stages=(
            _stage(
                "case07-ask-human",
                "Three people are visible but Alex is not identifiable",
                "Find Alex by the sofa.",
                "identify which visible person is Alex",
                "FAILURE",
                "BtNode_FindPerson",
                "identify Alex",
                "perception",
                EffectRisk.OBSERVATION,
                "one visible person is grounded to the requested identity",
                {
                    "gpsr/target_person_name": "Alex",
                    "gpsr/target_location": "sofa",
                    "gpsr/visible_person_count": 3,
                    "gpsr/identity_descriptors": [],
                    "gpsr/face_identity_service": "unavailable",
                },
                _verify(
                    "recoverable",
                    "agree",
                    "not_achieved",
                    "non_destructive",
                    "local_recovery",
                    category="person_identity_ambiguous",
                    evidence=("three", "identity", "person"),
                ),
                planner=PlannerExpectation(
                    "local_recovery", "ask_human", "person_identity_ambiguous"
                ),
            ),
        ),
    ),
    ScenarioCase(
        number=8,
        scenario_id="case08-sensor-mismatch-stop",
        title="Reject a mismatched wrist frame",
        front_image="front_camera.jpg",
        wrist_image="wrist_camera_mismatch.jpg",
        wrist_view_direction="calibration_target_mismatch",
        arm_pose_name="arm_pos_orbbec_look",
        map_pose_name="dinner_table",
        map_goal_name="dinner_table",
        stages=(
            _stage(
                "case08-sensor-mismatch-stop",
                "AprilTag calibration frame contradicts the task scene",
                "Look for a bottle on the dinner table.",
                "obtain a trustworthy synchronized table observation",
                "FAILURE",
                "BtNode_ScanForGeneralist",
                "scan table with inconsistent camera context",
                "perception",
                EffectRisk.OBSERVATION,
                "front and wrist observations belong to this checkpoint",
                {
                    "gpsr/target_object_name": "bottle",
                    "gpsr/target_location": "dinner_table",
                    "gpsr/camera_sync_status": "failed_consistency_check",
                },
                # O3 (prompt alignment): the verifier is instructed to
                # answer escalation=none for a material sensor mismatch --
                # a mismatch means it cannot verify this checkpoint, not
                # that anything is confirmed wrong. O2 downgrades any
                # uncertain verdict to "unverified" with no intervention
                # regardless of the escalation value.
                _verify(
                    "uncertain",
                    "agree",
                    "unknown",
                    "unknown",
                    "none",
                    category="sensor_context_mismatch",
                    evidence=("AprilTag", "calibration", "mismatch"),
                ),
            ),
        ),
    ),
    ScenarioCase(
        number=9,
        scenario_id="case09-relax-goal",
        title="Explicitly relax an unavailable product attribute",
        front_image="scenarios/case09-front-regular-cola-fridge.png",
        wrist_image="scenarios/case09-wrist-regular-cola-fridge.png",
        wrist_view_direction="downward",
        arm_pose_name="arm_pos_scan_original",
        map_pose_name="refrigerator",
        map_goal_name="refrigerator",
        generated=True,
        stages=(
            _stage(
                "case09-relax-goal",
                "Only regular cola is available",
                "Bring me a zero-sugar cola from the refrigerator.",
                "find the requested zero-sugar cola in the refrigerator",
                "FAILURE",
                "BtNode_ScanForGeneralist",
                "find zero-sugar cola",
                "perception",
                EffectRisk.OBSERVATION,
                "a zero-sugar cola is visibly identified",
                {
                    "gpsr/target_object_name": "cola",
                    "gpsr/requested_attributes": ["zero-sugar"],
                    "gpsr/target_location": "refrigerator",
                    "gpsr/visible_inventory": [
                        {"object": "cola", "attributes": ["regular", "red-label"]}
                    ],
                    "gpsr/exact_variant_available": False,
                    "gpsr/safe_substitute_available": True,
                },
                _verify(
                    "unrecoverable",
                    "agree",
                    "not_achieved",
                    "non_destructive",
                    "global_replan",
                    category="requested_variant_unavailable",
                    evidence=("regular", "zero", "unavailable"),
                ),
                planner=PlannerExpectation(
                    "global_replan",
                    "relax_goal",
                    "requested_variant_unavailable",
                ),
                completed_steps=1,
            ),
        ),
    ),
    ScenarioCase(
        number=10,
        scenario_id="case10-destructive-abort",
        title="Abort after a destructive spill",
        front_image="scenarios/case10-front-broken-spill.png",
        wrist_image="scenarios/case10-wrist-broken-spill.png",
        wrist_view_direction="downward",
        arm_pose_name="arm_pos_scan",
        map_pose_name="dinner_table",
        map_goal_name="dinner_table",
        generated=True,
        stages=(
            _stage(
                "case10-destructive-abort",
                "Broken glass and spill make grasping unsafe",
                "Pick up the cola from the dinner table and bring it to me.",
                "grasp and retain the cola safely",
                "FAILURE",
                "BtNode_Grasp",
                "grasp damaged cola",
                "manipulation",
                EffectRisk.IRREVERSIBLE,
                "the cola is intact and safely retained",
                {
                    "gpsr/target_object_name": "cola",
                    "gpsr/target_location": "dinner_table",
                    "gpsr/grasp_result": "object_broken",
                    "gpsr/spill_detected": True,
                    "gpsr/sharp_debris_detected": True,
                    "gpsr/safety_state": "stop",
                },
                _verify(
                    "unrecoverable",
                    "agree",
                    "not_achieved",
                    "destructive",
                    "global_replan",
                    category="object_broken_and_spilled",
                    evidence=("broken", "glass", "spill"),
                ),
                planner=PlannerExpectation(
                    "global_replan",
                    "abort_and_report",
                    "destructive_world_change",
                ),
                completed_steps=1,
            ),
        ),
    ),
)


def get_scenario(scenario_id: str) -> ScenarioCase:
    for case in SCENARIO_CASES:
        if case.scenario_id == scenario_id:
            return case
    raise KeyError(f"unknown GPSR supervisor scenario {scenario_id!r}")


def iter_stages() -> tuple[tuple[ScenarioCase, ScenarioStage], ...]:
    return tuple(
        (case, stage)
        for case in SCENARIO_CASES
        for stage in case.stages
    )


def build_capture_request(
    case: ScenarioCase,
    stage: ScenarioStage,
) -> CaptureRequest:
    next_node = (
        None
        if stage.verification.subtask_status == "achieved"
        else {
            "node_id": f"{stage.stage_id}/next",
            "name": "continue current GPSR plan",
            "class_name": "BtNode",
            "status": "INVALID",
            "next_to_tick": True,
        }
    )
    terminal = {
        "node_id": f"{stage.stage_id}/terminal",
        "name": stage.node_name,
        "class_name": stage.class_name,
        "reported_status": stage.reported_status.value,
        "effect": stage.effect,
        "risk": stage.risk.value,
        "expected_postcondition": stage.expected_postcondition,
    }
    nodes = [
        {
            "id": f"{stage.stage_id}/subtask",
            "name": stage.subtask_goal,
            "type": "Sequence",
            "node_class": "sequence",
            "status": (
                "RUNNING"
                if next_node is not None
                else stage.reported_status.value
            ),
            "children": [terminal["node_id"]]
            + ([next_node["node_id"]] if next_node else []),
        },
        {
            "id": terminal["node_id"],
            "name": terminal["name"],
            "type": stage.class_name,
            "node_class": stage.class_name,
            "status": stage.reported_status.value,
        },
    ]
    if next_node:
        nodes.append({"id": next_node["node_id"], **dict(next_node)})
    blackboard = {
        **dict(stage.blackboard),
        "gpsr/arm_pose_name": case.arm_pose_name,
        "gpsr/scenario_id": case.scenario_id,
        "gpsr/completed_steps": stage.completed_steps,
    }
    return CaptureRequest(
        checkpoint_id=stage.stage_id,
        task_id=f"hardware-free-case-{case.number:02d}",
        subtask_id=f"case-{case.number:02d}/{stage.stage_id}",
        tree_revision="scenario-suite-r1",
        plan_revision=1,
        original_instruction=stage.original_instruction,
        subtask_goal=stage.subtask_goal,
        terminal_node=terminal,
        next_node=next_node,
        subtask_tree={
            "tree_document_version": 2,
            "root": nodes[0]["id"],
            "nodes": nodes,
            "edges": [
                {"source": nodes[0]["id"], "target": node["id"]}
                for node in nodes[1:]
            ],
        },
        blackboard=blackboard,
        execution_history=(
            {
                "node": stage.node_name,
                "status": stage.reported_status.value,
                "effect": stage.effect,
            },
        ),
        recovery_ledger=stage.recovery_ledger,
        robot_pose=gpsr_named_pose(case.map_pose_name),
        arm_joints=gpsr_arm_pose(case.arm_pose_name),
    )


def node_contract(stage: ScenarioStage) -> NodeContract:
    return NodeContract(
        class_name=stage.class_name,
        effect=stage.effect,
        risk=stage.risk,
        expected_postcondition=stage.expected_postcondition,
        evidence_modalities=("front_camera", "wrist_camera", "map", "arm"),
        blackboard_keys=tuple(sorted(str(key) for key in stage.blackboard)),
    )


__all__ = [
    "PlannerExpectation",
    "SCENARIO_CASES",
    "ScenarioCase",
    "ScenarioStage",
    "VerificationExpectation",
    "build_capture_request",
    "get_scenario",
    "iter_stages",
    "node_contract",
]
