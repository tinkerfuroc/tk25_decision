"""Versioned, JSON-safe trace-event envelopes and payload policies.

The module deliberately uses only the Python standard library so it may be
used by ROS nodes, helper processes, and offline tools without introducing a
new runtime dependency.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime, timezone
import hashlib
import json
from typing import Any, Callable, Iterable, Mapping
from uuid import uuid4


TRACE_EVENT_VERSION = 1


class ValidationError(ValueError):
    """Raised when a public trace or tree value is invalid."""


JsonValue = None | bool | int | float | str | list["JsonValue"] | dict[str, "JsonValue"]
ArtifactHook = Callable[[bytes, str], "ArtifactReference | Mapping[str, Any] | str | None"]


def utc_now() -> datetime:
    """Return an aware UTC timestamp suitable for a trace envelope."""

    return datetime.now(timezone.utc)


def timestamp_to_string(value: datetime) -> str:
    """Serialize an aware datetime in a stable RFC 3339-compatible form."""

    if value.tzinfo is None or value.utcoffset() is None:
        raise ValidationError("timestamps must include a timezone")
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def timestamp_from_string(value: str) -> datetime:
    """Parse an RFC 3339 timestamp accepted by the event envelope."""

    if not isinstance(value, str) or not value:
        raise ValidationError("timestamp must be a non-empty string")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValidationError("timestamp must be RFC 3339 / ISO 8601") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValidationError("timestamp must include a timezone")
    return parsed.astimezone(timezone.utc)


def _require_identifier(name: str, value: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValidationError(f"{name} must be a non-empty string")
    if len(value) > 256:
        raise ValidationError(f"{name} must be at most 256 characters")


def ensure_json_value(value: Any, *, field_name: str = "value") -> None:
    """Verify that *value* can be represented as strict JSON.

    ``allow_nan=False`` rejects NaN and infinity, which makes events portable
    across JSON parsers and prevents silently non-standard NDJSON records.
    """

    try:
        json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
    except (TypeError, ValueError) as exc:
        raise ValidationError(f"{field_name} must contain only strict JSON values") from exc


def _copy_json(value: Any, *, field_name: str) -> JsonValue:
    ensure_json_value(value, field_name=field_name)
    # A JSON round-trip detaches caller-owned mutable input and normalises any
    # Mapping/list subclasses into the documented wire types.
    return json.loads(json.dumps(value, ensure_ascii=False, allow_nan=False))


@dataclass(frozen=True)
class ArtifactReference:
    """A compact reference to content stored outside the trace event."""

    uri: str
    sha256: str
    size_bytes: int
    media_type: str = "application/json"

    def __post_init__(self) -> None:
        _require_identifier("artifact uri", self.uri)
        if not isinstance(self.sha256, str) or len(self.sha256) != 64:
            raise ValidationError("artifact sha256 must be a 64-character hex digest")
        try:
            int(self.sha256, 16)
        except ValueError as exc:
            raise ValidationError("artifact sha256 must be hexadecimal") from exc
        if not isinstance(self.size_bytes, int) or self.size_bytes < 0:
            raise ValidationError("artifact size_bytes must be a non-negative integer")
        _require_identifier("artifact media_type", self.media_type)

    def to_dict(self) -> dict[str, JsonValue]:
        return {
            "uri": self.uri,
            "sha256": self.sha256,
            "size_bytes": self.size_bytes,
            "media_type": self.media_type,
        }


@dataclass(frozen=True)
class RedactionPolicy:
    """Redact values selected by case-insensitive mapping-key names."""

    keys: frozenset[str] = field(
        default_factory=lambda: frozenset(
            {"password", "secret", "token", "authorization", "api_key", "cookie"}
        )
    )
    replacement: str = "[REDACTED]"

    def __post_init__(self) -> None:
        if not isinstance(self.replacement, str):
            raise ValidationError("redaction replacement must be a string")
        normalised = frozenset(str(key).casefold() for key in self.keys)
        if any(not key for key in normalised):
            raise ValidationError("redaction keys must be non-empty")
        object.__setattr__(self, "keys", normalised)

    def redact(self, value: Any) -> JsonValue:
        """Return a detached JSON-safe value with sensitive fields replaced."""

        ensure_json_value(value, field_name="payload")

        def visit(item: Any) -> JsonValue:
            if isinstance(item, Mapping):
                result: dict[str, JsonValue] = {}
                for key, child in item.items():
                    key_text = str(key)
                    result[key_text] = (
                        self.replacement if key_text.casefold() in self.keys else visit(child)
                    )
                return result
            if isinstance(item, (list, tuple)):
                return [visit(child) for child in item]
            return item

        return visit(value)


@dataclass(frozen=True)
class ContentPolicy:
    """Keep inline payload values bounded and optionally externalise them.

    ``artifact_hook`` receives the canonical JSON bytes and a JSONPath-like
    location.  It can persist those bytes and return an :class:`ArtifactReference`,
    a mapping, or a URI string.  If it returns ``None`` (or is absent), a
    digest-only reference is emitted instead.
    """

    max_inline_bytes: int = 16 * 1024
    artifact_hook: ArtifactHook | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.max_inline_bytes, int) or self.max_inline_bytes < 1:
            raise ValidationError("max_inline_bytes must be a positive integer")
        if self.artifact_hook is not None and not callable(self.artifact_hook):
            raise ValidationError("artifact_hook must be callable")

    def apply(self, value: Any, *, path: str = "$") -> JsonValue:
        """Externalise individual oversized values without mutating *value*."""

        ensure_json_value(value, field_name="payload")

        def encoded(item: Any) -> bytes:
            return json.dumps(
                item, ensure_ascii=False, allow_nan=False, separators=(",", ":")
            ).encode("utf-8")

        def reference(item: Any, item_path: str) -> dict[str, JsonValue]:
            content = encoded(item)
            digest = hashlib.sha256(content).hexdigest()
            supplied = self.artifact_hook(content, item_path) if self.artifact_hook else None
            if isinstance(supplied, ArtifactReference):
                details: JsonValue = supplied.to_dict()
            elif isinstance(supplied, Mapping):
                details = _copy_json(supplied, field_name="artifact reference")
            elif isinstance(supplied, str):
                details = {"uri": supplied, "sha256": digest, "size_bytes": len(content)}
            elif supplied is None:
                details = {
                    "sha256": digest,
                    "size_bytes": len(content),
                    "media_type": "application/json",
                    "omitted": True,
                }
            else:
                raise ValidationError("artifact_hook must return a reference, mapping, URI, or None")
            return {"artifact_ref": details}

        def visit(item: Any, item_path: str) -> JsonValue:
            if isinstance(item, Mapping):
                candidate = {
                    str(key): visit(child, f"{item_path}.{key}") for key, child in item.items()
                }
            elif isinstance(item, (list, tuple)):
                candidate = [visit(child, f"{item_path}[{index}]") for index, child in enumerate(item)]
            else:
                candidate = item
            return reference(candidate, item_path) if len(encoded(candidate)) > self.max_inline_bytes else candidate

        return visit(value, path)


@dataclass(frozen=True)
class TraceEvent:
    """The versioned event envelope written to the GPSR observability stream."""

    event_id: str
    trace_id: str
    source_id: str
    sequence: int
    timestamp: datetime
    event_type: str
    payload: Mapping[str, JsonValue] = field(default_factory=dict)
    parent_event_id: str | None = None
    causation_ids: tuple[str, ...] = ()
    version: int = TRACE_EVENT_VERSION

    def __post_init__(self) -> None:
        if self.version != TRACE_EVENT_VERSION:
            raise ValidationError(f"unsupported trace event version: {self.version}")
        _require_identifier("event_id", self.event_id)
        _require_identifier("trace_id", self.trace_id)
        _require_identifier("source_id", self.source_id)
        _require_identifier("event_type", self.event_type)
        if not isinstance(self.sequence, int) or isinstance(self.sequence, bool) or self.sequence < 0:
            raise ValidationError("sequence must be a non-negative integer")
        if not isinstance(self.timestamp, datetime):
            raise ValidationError("timestamp must be a datetime")
        timestamp_to_string(self.timestamp)
        if self.parent_event_id is not None:
            _require_identifier("parent_event_id", self.parent_event_id)
        if not isinstance(self.causation_ids, tuple):
            object.__setattr__(self, "causation_ids", tuple(self.causation_ids))
        for causation_id in self.causation_ids:
            _require_identifier("causation_id", causation_id)
        if not isinstance(self.payload, Mapping):
            raise ValidationError("payload must be a mapping")
        copied = _copy_json(dict(self.payload), field_name="payload")
        if not isinstance(copied, dict):  # Defensive; dict input must remain an object.
            raise ValidationError("payload must be a JSON object")
        object.__setattr__(self, "payload", copied)

    @classmethod
    def create(
        cls,
        *,
        source_id: str,
        sequence: int,
        event_type: str,
        payload: Mapping[str, Any] | None = None,
        trace_id: str | None = None,
        parent_event_id: str | None = None,
        causation_ids: Iterable[str] = (),
        timestamp: datetime | None = None,
        event_id: str | None = None,
    ) -> "TraceEvent":
        """Create a new envelope with UUIDs and an aware UTC timestamp by default."""

        return cls(
            event_id=event_id or str(uuid4()),
            trace_id=trace_id or str(uuid4()),
            source_id=source_id,
            sequence=sequence,
            timestamp=timestamp or utc_now(),
            event_type=event_type,
            payload=dict(payload or {}),
            parent_event_id=parent_event_id,
            causation_ids=tuple(causation_ids),
        )

    def to_dict(self) -> dict[str, JsonValue]:
        """Return the public, strictly JSON-compatible representation."""

        return {
            "version": self.version,
            "event_id": self.event_id,
            "trace_id": self.trace_id,
            "source_id": self.source_id,
            "sequence": self.sequence,
            "timestamp": timestamp_to_string(self.timestamp),
            "event_type": self.event_type,
            "payload": _copy_json(self.payload, field_name="payload"),
            "parent_event_id": self.parent_event_id,
            "causation_ids": list(self.causation_ids),
        }

    def to_json(self) -> str:
        """Serialize the event as one canonical JSON document (without newline)."""

        return json.dumps(self.to_dict(), ensure_ascii=False, allow_nan=False, separators=(",", ":"), sort_keys=True)

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "TraceEvent":
        """Parse and validate a deserialized envelope."""

        if not isinstance(value, Mapping):
            raise ValidationError("trace event must be a mapping")
        expected = {
            "version",
            "event_id",
            "trace_id",
            "source_id",
            "sequence",
            "timestamp",
            "event_type",
            "payload",
            "parent_event_id",
            "causation_ids",
        }
        unknown = set(value) - expected
        missing = expected - set(value)
        if unknown or missing:
            message = []
            if missing:
                message.append("missing: " + ", ".join(sorted(missing)))
            if unknown:
                message.append("unknown: " + ", ".join(sorted(unknown)))
            raise ValidationError("invalid trace event fields (" + "; ".join(message) + ")")
        return cls(
            version=value["version"],
            event_id=value["event_id"],
            trace_id=value["trace_id"],
            source_id=value["source_id"],
            sequence=value["sequence"],
            timestamp=timestamp_from_string(value["timestamp"]),
            event_type=value["event_type"],
            payload=value["payload"],
            parent_event_id=value["parent_event_id"],
            causation_ids=tuple(value["causation_ids"]),
        )

    @classmethod
    def from_json(cls, value: str | bytes | bytearray) -> "TraceEvent":
        """Deserialize a single JSON event document."""

        try:
            decoded = json.loads(value)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValidationError("trace event must be valid JSON") from exc
        return cls.from_dict(decoded)
