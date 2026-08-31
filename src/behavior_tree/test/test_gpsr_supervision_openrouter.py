from __future__ import annotations

from datetime import datetime, timezone
import json
from types import SimpleNamespace

import pytest

from behavior_tree.GPSR.supervision.clients import OpenRouterSupervisorClient
from behavior_tree.GPSR.supervision.context import StaticContextProvider
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    BtAssessment,
    CaptureRequest,
    Escalation,
    SubtaskStatus,
    SupervisorConfig,
    SupervisorUnavailable,
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
            _DEFAULT_VERIFICATION
        )
        if isinstance(content, BaseException):
            raise content
        return SimpleNamespace(
            choices=[SimpleNamespace(message=SimpleNamespace(content=content))],
            usage=SimpleNamespace(
                prompt_tokens=10, completion_tokens=5, total_tokens=15
            ),
        )


_DEFAULT_VERIFICATION = {
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
    assert "sensor_context_mismatch" in request["messages"][0]["content"]
    assert "mutually consistent" in request["messages"][1]["content"][0]["text"]


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


def test_schema_error_gets_one_retry_like_a_transport_error():
    # O5: a malformed/empty structured response (SchemaError, or a raw
    # json.JSONDecodeError from _decode_json_content) used to re-raise
    # immediately with no retry, unlike a transient transport error one
    # request later. A single bad response from an otherwise healthy model
    # should not be treated any differently from one dropped connection.
    completions = _FakeCompletions(["not a json document"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"
    assert len(completions.requests) == 2


def test_schema_error_on_both_attempts_raises_and_emits_query_failed():
    events: list[tuple[str, dict]] = []
    completions = _FakeCompletions(["not a json document", "still not json"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(),
        api_key="test-key",
        client=fake,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    with pytest.raises(SupervisorUnavailable):
        client.verify(_snapshot())
    # keep total attempts <= 2 for every error class -- no third call
    assert len(completions.requests) == 2
    failed = [payload for event, payload in events if event == "supervisor.query.failed"]
    assert len(failed) == 1
    assert failed[0]["error_type"] == "JSONDecodeError"


def test_transport_error_retry_is_unchanged_by_the_schema_error_fix():
    completions = _FakeCompletions([RuntimeError("connection reset")])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"
    assert len(completions.requests) == 2
