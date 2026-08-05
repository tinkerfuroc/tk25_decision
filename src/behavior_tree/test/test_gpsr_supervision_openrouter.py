from __future__ import annotations

from datetime import datetime, timezone
import json
from types import SimpleNamespace

from behavior_tree.GPSR.supervision.clients import OpenRouterSupervisorClient
from behavior_tree.GPSR.supervision.context import StaticContextProvider
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    BtAssessment,
    CaptureRequest,
    Escalation,
    SubtaskStatus,
    SupervisorConfig,
    Verdict,
    VerificationDecision,
    WorldChange,
)


def _snapshot():
    request = CaptureRequest(
        checkpoint_id="cp",
        task_id="task",
        subtask_id="subtask",
        tree_revision="1",
        plan_revision=1,
        original_instruction="inspect the table",
        subtask_goal="find the bottle",
        terminal_node={"reported_status": "SUCCESS"},
        next_node=None,
        subtask_tree={"nodes": []},
        blackboard={},
        execution_history=(),
        recovery_ledger=(),
    )
    now = datetime.now(timezone.utc).isoformat()
    provider = StaticContextProvider(
        tuple(
            ArtifactRef.absent(role, now, "unit test")
            for role in ("front_camera", "wrist_camera", "map", "arm")
        )
    )
    return provider.capture(request)


class _FakeCompletions:
    def __init__(self, contents=None):
        self.requests = []
        self.contents = list(contents or [])

    def create(self, **kwargs):
        self.requests.append(kwargs)
        content = self.contents.pop(0) if self.contents else json.dumps(
            {
                "checkpoint_id": "cp",
                "verdict": "all_clear",
                "bt_assessment": "agree",
                "subtask_status": "achieved",
                "world_change": "none",
                "escalation": "none",
                "failure_category": "",
                "evidence": ["status"],
                "rationale": "test",
                "confidence": 0.9,
            }
        )
        return SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(content=content))],
            usage=SimpleNamespace(
                prompt_tokens=10, completion_tokens=5, total_tokens=15
            ),
        )


def test_openrouter_verifier_uses_luna_medium_without_temperature():
    completions = _FakeCompletions()
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(),
        api_key="test-key",
        client=fake,
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"
    request = completions.requests[0]
    assert request["model"] == "openai/gpt-5.6-luna"
    assert request["extra_body"]["reasoning"]["effort"] == "medium"
    assert request["response_format"]["type"] == "json_schema"
    assert "temperature" not in request


def test_openrouter_planners_decode_strict_embedded_json_fields():
    local = json.dumps(
        {
            "checkpoint_id": "cp",
            "issue_id": "issue",
            "strategy_id": "look-left",
            "kind": "scan_views",
            "arguments": json.dumps(
                {
                    "angles": [[-30, 10]],
                    "perception_action": "find_object",
                }
            ),
            "rationale": "new view",
            "expected_evidence": ["target visible"],
            "stop_conditions": ["target absent"],
        }
    )
    global_abort = json.dumps(
        {
            "checkpoint_id": "cp",
            "action": "abort_and_report",
            "replacement_plan": [],
            "preserved_completed_steps": 0,
            "relaxed_constraints": [],
            "rationale": "unsafe",
            "operator_message": "I stopped safely.",
        }
    )
    completions = _FakeCompletions([local, global_abort])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(),
        api_key="test-key",
        client=fake,
    )
    verification = VerificationDecision(
        checkpoint_id="cp",
        verdict=Verdict.RECOVERABLE,
        bt_assessment=BtAssessment.AGREE,
        subtask_status=SubtaskStatus.NOT_ACHIEVED,
        world_change=WorldChange.NON_DESTRUCTIVE,
        escalation=Escalation.LOCAL_RECOVERY,
        failure_category="missing",
        evidence=(),
        rationale="test",
        confidence=0.8,
    )
    recovery = client.plan_local_recovery(_snapshot(), verification, "issue")
    assert recovery.arguments["angles"] == [[-30, 10]]
    global_decision = client.plan_global_replan(
        _snapshot(), verification, "unsafe"
    )
    assert global_decision.action.value == "abort_and_report"
    assert all(
        request["extra_body"]["reasoning"]["effort"] == "high"
        for request in completions.requests
    )
