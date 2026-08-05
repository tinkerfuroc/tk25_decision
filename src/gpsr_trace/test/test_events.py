from datetime import datetime
import json

import pytest

from gpsr_trace import ArtifactReference, ContentPolicy, RedactionPolicy, TraceEvent, ValidationError


def test_event_round_trip_preserves_versioned_causality() -> None:
    event = TraceEvent.create(
        source_id="planner",
        sequence=7,
        event_type="proposal.created",
        payload={"goal": "bring water"},
        trace_id="trace-1",
        parent_event_id="event-parent",
        causation_ids=("speech-19", "vision-2"),
    )

    restored = TraceEvent.from_json(event.to_json())

    assert restored == event
    assert json.loads(event.to_json())["timestamp"].endswith("Z")
    assert restored.sequence == 7
    assert restored.causation_ids == ("speech-19", "vision-2")


def test_event_validation_rejects_naive_timestamp_and_unknown_fields() -> None:
    with pytest.raises(ValidationError, match="timezone"):
        TraceEvent.create(
            source_id="planner",
            sequence=0,
            event_type="started",
            timestamp=datetime(2026, 1, 1),
        )

    event = TraceEvent.create(source_id="planner", sequence=0, event_type="started")
    data = event.to_dict()
    data["extra"] = True
    with pytest.raises(ValidationError, match="unknown"):
        TraceEvent.from_dict(data)


def test_redaction_and_content_policy_externalise_large_values() -> None:
    observed: list[tuple[bytes, str]] = []

    def artifact_hook(content: bytes, path: str) -> ArtifactReference:
        observed.append((content, path))
        return ArtifactReference(
            uri="file:///tmp/trace-artifact.json",
            sha256="a" * 64,
            size_bytes=len(content),
        )

    redacted = RedactionPolicy(frozenset({"token"})).redact(
        {"token": "secret", "diagnostic": "x" * 1024}
    )
    bounded = ContentPolicy(max_inline_bytes=256, artifact_hook=artifact_hook).apply(redacted, path="$.payload")

    assert bounded["token"] == "[REDACTED]"
    assert bounded["diagnostic"]["artifact_ref"]["uri"] == "file:///tmp/trace-artifact.json"
    assert observed and observed[0][1] == "$.payload.diagnostic"
