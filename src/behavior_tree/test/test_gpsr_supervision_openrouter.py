from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
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


@pytest.fixture(autouse=True)
def _pin_schema_error_fallback_dir(tmp_path, monkeypatch):
    # Task R: a SchemaError with no reachable artifact directory (every
    # snapshot below uses ArtifactRef.absent -- path=None) falls back to the
    # gpsr_runs/debug-style dir GpsrTelemetry uses. Pin it under tmp_path so
    # the existing SchemaError-retry tests in this file don't leave
    # gpsr_runs/debug/schema_errors litter in the repo working tree.
    monkeypatch.setenv("BT_GPSR_PLAN_DIR", str(tmp_path))


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


def test_schema_error_from_from_dict_validation_also_gets_one_retry():
    # Z-2 (task-O review, MEDIUM) -- the reviewer's own direct
    # reproduction. A response can be well-formed, schema-valid-looking
    # JSON that still raises SchemaError from VerificationDecision.from_dict
    # (here: confidence=1.5, out of [0,1]) -- not from _decode_json_content
    # at all. Before this fix that SchemaError was raised from OUTSIDE
    # _query()'s retry loop entirely (in verify(), after _query() already
    # returned successfully), getting zero retries and silently
    # contradicting the O5 ruling's "keep total attempts <= 2 for every
    # error class." Confirmed by direct reproduction: this used to raise
    # after exactly 1 HTTP request, not 2.
    bad_confidence = json.dumps({**_DEFAULT_VERIFICATION, "confidence": 1.5})
    completions = _FakeCompletions([bad_confidence])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"
    assert len(completions.requests) == 2


def test_schema_error_from_stale_checkpoint_id_also_gets_one_retry():
    # Z-2: _same_checkpoint() (a hallucinated/stale checkpoint_id) is
    # another SchemaError raise site that used to run after _query()
    # already returned -- same fix, different raise site.
    stale = json.dumps({**_DEFAULT_VERIFICATION, "checkpoint_id": "wrong-cp"})
    completions = _FakeCompletions([stale])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"
    assert len(completions.requests) == 2


def test_schema_error_from_from_dict_on_both_attempts_still_caps_at_two():
    events: list[tuple[str, dict]] = []
    bad_confidence = json.dumps({**_DEFAULT_VERIFICATION, "confidence": 1.5})
    completions = _FakeCompletions([bad_confidence, bad_confidence])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(),
        api_key="test-key",
        client=fake,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    with pytest.raises(SupervisorUnavailable):
        client.verify(_snapshot())
    assert len(completions.requests) == 2
    failed = [payload for event, payload in events if event == "supervisor.query.failed"]
    assert len(failed) == 1
    assert failed[0]["error_type"] == "SchemaError"


def test_transport_error_retry_is_unchanged_by_the_schema_error_fix():
    completions = _FakeCompletions([RuntimeError("connection reset")])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"
    assert len(completions.requests) == 2


# --- Task R: capture the raw malformed body a SchemaError discards -------


def _debug_dir(tmp_path):
    return tmp_path / "debug" / "schema_errors"


def test_schema_error_persists_raw_body_on_attempt_1(tmp_path):
    completions = _FakeCompletions(["not a json document"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    decision = client.verify(_snapshot())
    assert decision.verdict.value == "all_clear"  # attempt 2 succeeds, unaffected
    files = sorted(_debug_dir(tmp_path).glob("*schema_error_verify_1.txt"))
    assert len(files) == 1
    assert files[0].read_text(encoding="utf-8") == "not a json document"


def test_schema_error_on_both_attempts_persists_two_distinguishable_files(tmp_path):
    completions = _FakeCompletions(["not a json document", "still not json"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    with pytest.raises(SupervisorUnavailable):
        client.verify(_snapshot())
    attempt_1 = list(_debug_dir(tmp_path).glob("*schema_error_verify_1.txt"))
    attempt_2 = list(_debug_dir(tmp_path).glob("*schema_error_verify_2.txt"))
    assert len(attempt_1) == 1
    assert len(attempt_2) == 1
    assert attempt_1[0].read_text(encoding="utf-8") == "not a json document"
    assert attempt_2[0].read_text(encoding="utf-8") == "still not json"


def test_schema_error_body_truncated_at_32kb_with_marker(tmp_path):
    oversized = "A" * (40 * 1024)
    completions = _FakeCompletions([oversized])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    client.verify(_snapshot())  # attempt 2's default response succeeds
    files = list(_debug_dir(tmp_path).glob("*schema_error_verify_1.txt"))
    assert len(files) == 1
    body = files[0].read_text(encoding="utf-8")
    assert body.endswith("...[truncated]")
    assert len(body.encode("utf-8")) <= 32 * 1024


def test_schema_error_persists_next_to_checkpoint_artifacts_when_reachable(tmp_path, monkeypatch):
    # A different directory than the fallback pinned by the autouse fixture
    # -- confirms the client prefers the checkpoint's own artifact dir over
    # the telemetry-style fallback when a real artifact path is available.
    artifact_dir = tmp_path / "artifacts"
    artifact_dir.mkdir()
    map_path = artifact_dir / "cp-map.png"
    map_path.write_bytes(b"fake-png-bytes")
    now = datetime.now(timezone.utc).isoformat()
    provider = StaticContextProvider(
        (
            ArtifactRef.from_path(
                role="map", mime_type="image/png", path=map_path, captured_at=now
            ),
        )
    )
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
    snapshot = provider.capture(request)
    completions = _FakeCompletions(["not a json document"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )

    def decode(raw):
        return VerificationDecision.from_dict(raw)

    client._query(
        role="verify",
        system="s",
        text="t",
        snapshot=snapshot,
        schema={},
        effort="medium",
        max_completion_tokens=10,
        timeout_s=1.0,
        decode=decode,
    )
    artifact_dir_files = list(artifact_dir.glob("*schema_error_verify_1.txt"))
    assert len(artifact_dir_files) == 1
    assert not _debug_dir(tmp_path).exists()


_SUPERVISION_FIXTURES_DIR = Path(__file__).resolve().parents[1] / (
    "behavior_tree/GPSR/supervision/fixtures"
)


def _live_request():
    return CaptureRequest(
        checkpoint_id="cp-live",
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


def _run_query_expecting_schema_error(client, snapshot):
    def decode(raw):
        return VerificationDecision.from_dict(raw)

    client._query(
        role="verify",
        system="s",
        text="t",
        snapshot=snapshot,
        schema={},
        effort="medium",
        max_completion_tokens=10,
        timeout_s=1.0,
        decode=decode,
    )


def test_schema_error_prefers_temp_map_dir_over_fixture_tree_camera_dir(tmp_path):
    # S-1 regression (review of f0419ea): FixtureContextProvider(scenario_id=...)
    # -- the live-LLM validation path -- builds snapshot.artifacts as
    # (*camera_artifacts, map, arm), with the camera artifacts pointing into
    # the git-tracked, checksum-manifested supervision/fixtures/ tree. The
    # per-request map/arm renders (into a temp/output dir) must win over the
    # fixture-tree camera artifacts when deriving the debug dir -- a
    # SchemaError capture must never land inside the source tree.
    assert (_SUPERVISION_FIXTURES_DIR / "front_camera.jpg").exists()
    now = datetime.now(timezone.utc).isoformat()
    render_dir = tmp_path / "live-renders"
    render_dir.mkdir()
    map_path = render_dir / "cp-live-map.png"
    map_path.write_bytes(b"fake-map-png")
    provider = StaticContextProvider(
        (
            ArtifactRef.from_path(
                role="front_camera",
                mime_type="image/jpeg",
                path=_SUPERVISION_FIXTURES_DIR / "front_camera.jpg",
                captured_at=now,
            ),
            ArtifactRef.from_path(
                role="wrist_camera",
                mime_type="image/jpeg",
                path=_SUPERVISION_FIXTURES_DIR / "wrist_camera.jpg",
                captured_at=now,
            ),
            ArtifactRef.from_path(
                role="map", mime_type="image/png", path=map_path, captured_at=now
            ),
        )
    )
    snapshot = provider.capture(_live_request())
    completions = _FakeCompletions(["not a json document"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    _run_query_expecting_schema_error(client, snapshot)
    render_dir_files = list(render_dir.glob("*schema_error_verify_1.txt"))
    assert len(render_dir_files) == 1
    fixture_txt_files = list(_SUPERVISION_FIXTURES_DIR.glob("*schema_error*"))
    assert fixture_txt_files == []


def test_schema_error_falls_back_when_only_fixture_tree_paths_are_available(tmp_path):
    # S-1 regression: every real artifact path lives under the source tree
    # (as if a provider only ever had fixture-rooted images to offer) --
    # the guard must reject all of them and use the gpsr_runs-style
    # fallback instead of ever writing under supervision/.
    now = datetime.now(timezone.utc).isoformat()
    provider = StaticContextProvider(
        (
            ArtifactRef.from_path(
                role="map",
                mime_type="image/jpeg",
                path=_SUPERVISION_FIXTURES_DIR / "front_camera.jpg",
                captured_at=now,
            ),
            ArtifactRef.from_path(
                role="arm",
                mime_type="image/jpeg",
                path=_SUPERVISION_FIXTURES_DIR / "wrist_camera.jpg",
                captured_at=now,
            ),
            ArtifactRef.from_path(
                role="front_camera",
                mime_type="image/jpeg",
                path=_SUPERVISION_FIXTURES_DIR / "front_camera.jpg",
                captured_at=now,
            ),
            ArtifactRef.from_path(
                role="wrist_camera",
                mime_type="image/jpeg",
                path=_SUPERVISION_FIXTURES_DIR / "wrist_camera.jpg",
                captured_at=now,
            ),
        )
    )
    snapshot = provider.capture(_live_request())
    completions = _FakeCompletions(["not a json document"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    _run_query_expecting_schema_error(client, snapshot)
    fallback_files = list(_debug_dir(tmp_path).glob("*schema_error_verify_1.txt"))
    assert len(fallback_files) == 1
    assert list(_SUPERVISION_FIXTURES_DIR.glob("*schema_error*")) == []


def test_schema_error_failed_event_carries_snippet_message_and_attempt():
    events: list[tuple[str, dict]] = []
    completions = _FakeCompletions(["not a json document", "  still\n\tnot   json  "])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(),
        api_key="test-key",
        client=fake,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    with pytest.raises(SupervisorUnavailable):
        client.verify(_snapshot())
    failed = [payload for event, payload in events if event == "supervisor.query.failed"]
    assert len(failed) == 1
    payload = failed[0]
    assert payload["attempt"] == 2
    assert payload["raw_content_snippet"] == "still not json"
    assert "expecting value" in payload["error_message"].lower()


def test_transport_error_never_captures_anything(tmp_path):
    events: list[tuple[str, dict]] = []
    completions = _FakeCompletions(
        [RuntimeError("connection reset"), RuntimeError("connection reset")]
    )
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(),
        api_key="test-key",
        client=fake,
        telemetry=lambda event, payload: events.append((event, dict(payload))),
    )
    with pytest.raises(SupervisorUnavailable):
        client.verify(_snapshot())
    assert not _debug_dir(tmp_path).exists()
    failed = [payload for event, payload in events if event == "supervisor.query.failed"]
    assert len(failed) == 1
    assert "raw_content_snippet" not in failed[0]
    assert "error_message" not in failed[0]
    assert "attempt" not in failed[0]


def test_persistence_failure_does_not_mask_the_original_schema_error(monkeypatch):
    # An unwritable debug dir (or any other persistence failure) must never
    # crash the query path or swallow the real SchemaError.
    def _boom(self, *args, **kwargs):
        raise OSError("read-only filesystem")

    monkeypatch.setattr(Path, "mkdir", _boom)
    completions = _FakeCompletions(["not a json document", "still not json"])
    fake = SimpleNamespace(chat=SimpleNamespace(completions=completions))
    client = OpenRouterSupervisorClient(
        SupervisorConfig(), api_key="test-key", client=fake
    )
    with pytest.raises(SupervisorUnavailable) as excinfo:
        client.verify(_snapshot())
    assert isinstance(excinfo.value.__cause__, json.JSONDecodeError)
