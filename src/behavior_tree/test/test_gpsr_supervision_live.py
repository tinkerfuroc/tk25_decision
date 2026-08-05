"""Opt-in OpenRouter smoke tests; no robot or ROS graph is required."""
from __future__ import annotations

import os

import pytest

from behavior_tree.GPSR.supervision.clients import OpenRouterSupervisorClient
from behavior_tree.GPSR.supervision.context import FixtureContextProvider
from behavior_tree.GPSR.supervision.models import (
    BtAssessment,
    CaptureRequest,
    Escalation,
    SubtaskStatus,
    SupervisorConfig,
    Verdict,
    VerificationDecision,
    WorldChange,
)
from behavior_tree.GPSR.config import OPENAI_API_KEY


pytestmark = pytest.mark.live_openrouter


def _enabled() -> bool:
    return os.environ.get("GPSR_RUN_LIVE_LLM_TESTS", "0").lower() in {
        "1", "true", "yes", "on"
    }


@pytest.mark.skipif(not _enabled(), reason="set GPSR_RUN_LIVE_LLM_TESTS=1")
def test_luna_multimodal_verifier_medium(tmp_path):
    key = os.environ.get("OPENROUTER_API_KEY") or os.environ.get("OPENAI_API_KEY") or OPENAI_API_KEY
    if not key:
        pytest.skip("OpenRouter credential is not configured")
    request = CaptureRequest(
        checkpoint_id="live-fixture-checkpoint",
        task_id="live-task",
        subtask_id="live-task/find-table-object",
        tree_revision="1",
        plan_revision=1,
        original_instruction="Look for a bottle on the dining table.",
        subtask_goal="determine whether a bottle is visible on the table",
        terminal_node={
            "node_id": "scan",
            "class_name": "BtNode_ScanForGeneralist",
            "reported_status": "SUCCESS",
            "effect": "perception",
            "expected_postcondition": "A table observation was completed.",
        },
        next_node=None,
        subtask_tree={"nodes": [], "edges": []},
        blackboard={"gpsr/target_object_name": "bottle"},
        execution_history=(),
        recovery_ledger=(),
        robot_pose=(1.0, 1.0, 0.0),
        arm_joints=(0.0, 0.2, -0.4, 0.1, 0.0, 0.3, 0.0),
    )
    snapshot = FixtureContextProvider(output_dir=tmp_path).capture(request)
    client = OpenRouterSupervisorClient(
        SupervisorConfig.from_env(os.environ),
        api_key=key,
    )
    decision = client.verify(snapshot)
    assert decision.checkpoint_id == request.checkpoint_id
    assert 0.0 <= decision.confidence <= 1.0


@pytest.mark.skipif(not _enabled(), reason="set GPSR_RUN_LIVE_LLM_TESTS=1")
def test_luna_high_local_and_global_planners(tmp_path):
    key = os.environ.get("OPENROUTER_API_KEY") or os.environ.get("OPENAI_API_KEY") or OPENAI_API_KEY
    if not key:
        pytest.skip("OpenRouter credential is not configured")
    request = CaptureRequest(
        checkpoint_id="live-planner-checkpoint",
        task_id="live-task",
        subtask_id="live-task/find-coke",
        tree_revision="1",
        plan_revision=1,
        original_instruction="Pick up the coke from the dining table.",
        subtask_goal="find the coke before grasping it",
        terminal_node={
            "node_id": "scan",
            "class_name": "BtNode_ScanForGeneralist",
            "reported_status": "FAILURE",
            "effect": "perception",
        },
        next_node={"name": "grasp coke"},
        subtask_tree={"nodes": [], "edges": []},
        blackboard={
            "gpsr/target_object_name": "coke",
            "gpsr/target_location": "dining_table",
        },
        execution_history=(),
        recovery_ledger=(),
    )
    snapshot = FixtureContextProvider(output_dir=tmp_path).capture(request)
    client = OpenRouterSupervisorClient(
        SupervisorConfig.from_env(os.environ),
        api_key=key,
    )
    verification = VerificationDecision(
        checkpoint_id=request.checkpoint_id,
        verdict=Verdict.RECOVERABLE,
        bt_assessment=BtAssessment.AGREE,
        subtask_status=SubtaskStatus.NOT_ACHIEVED,
        world_change=WorldChange.NON_DESTRUCTIVE,
        escalation=Escalation.LOCAL_RECOVERY,
        failure_category="target_not_detected",
        evidence=("The scan service reported no target.",),
        rationale="A different viewpoint may expose the target.",
        confidence=0.8,
    )
    recovery = client.plan_local_recovery(
        snapshot, verification, "live-issue-find-coke"
    )
    assert recovery.checkpoint_id == request.checkpoint_id
    assert recovery.issue_id == "live-issue-find-coke"

    destructive = VerificationDecision(
        checkpoint_id=request.checkpoint_id,
        verdict=Verdict.UNRECOVERABLE,
        bt_assessment=BtAssessment.AGREE,
        subtask_status=SubtaskStatus.NOT_ACHIEVED,
        world_change=WorldChange.DESTRUCTIVE,
        escalation=Escalation.GLOBAL_REPLAN,
        failure_category="object_broken",
        evidence=("Assume the requested object is visibly broken.",),
        rationale="Continuing to manipulate it would be unsafe.",
        confidence=0.95,
    )
    global_decision = client.plan_global_replan(
        snapshot, destructive, "destructive_world_change"
    )
    assert global_decision.checkpoint_id == request.checkpoint_id
