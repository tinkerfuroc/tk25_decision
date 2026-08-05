from __future__ import annotations

from datetime import datetime, timezone
import json
from types import SimpleNamespace

from behavior_tree.GPSR.supervision.clients import OpenRouterSupervisorClient
from behavior_tree.GPSR.supervision.context import StaticContextProvider
from behavior_tree.GPSR.supervision.models import (
    ArtifactRef,
    CaptureRequest,
    SupervisorConfig,
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
    def __init__(self):
        self.requests = []

    def create(self, **kwargs):
        self.requests.append(kwargs)
        content = json.dumps(
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
