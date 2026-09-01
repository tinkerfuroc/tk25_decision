"""Scripted and OpenRouter clients for GPSR supervision."""
from __future__ import annotations

import base64
from collections import deque
import json
import os
from pathlib import Path
import threading
import time
from typing import Any, Callable, Mapping, Protocol, TypeVar

from .models import (
    GLOBAL_PLAN_JSON_SCHEMA,
    RECOVERY_JSON_SCHEMA,
    VERIFICATION_JSON_SCHEMA,
    GlobalPlanDecision,
    RecoveryProposal,
    SchemaError,
    SnapshotBundle,
    SupervisorConfig,
    SupervisorUnavailable,
    VerificationDecision,
)
from .prompts import (
    GLOBAL_REPLAN_SYSTEM_PROMPT,
    LOCAL_RECOVERY_SYSTEM_PROMPT,
    PROMPT_VERSION,
    VERIFIER_SYSTEM_PROMPT,
    global_replan_text,
    local_recovery_text,
    verifier_text,
)


_Decision = TypeVar("_Decision")


class SupervisorClient(Protocol):
    def verify(self, snapshot: SnapshotBundle) -> VerificationDecision:
        ...

    def plan_local_recovery(
        self,
        snapshot: SnapshotBundle,
        verification: VerificationDecision,
        issue_id: str,
    ) -> RecoveryProposal:
        ...

    def plan_global_replan(
        self,
        snapshot: SnapshotBundle,
        verification: VerificationDecision,
        reason: str,
    ) -> GlobalPlanDecision:
        ...


class ScriptedSupervisorClient:
    """Thread-safe deterministic client used by unit and full-mock tests."""

    def __init__(
        self,
        *,
        verifications=(),
        recoveries=(),
        global_plans=(),
    ) -> None:
        self._verifications = deque(verifications)
        self._recoveries = deque(recoveries)
        self._global_plans = deque(global_plans)
        self._lock = threading.Lock()
        self.calls: list[tuple[str, str]] = []

    def verify(self, snapshot: SnapshotBundle) -> VerificationDecision:
        item = self._pop(self._verifications, "verify", snapshot.request.checkpoint_id)
        if isinstance(item, VerificationDecision):
            return item
        return VerificationDecision.from_dict(item)

    def plan_local_recovery(
        self,
        snapshot: SnapshotBundle,
        verification: VerificationDecision,
        issue_id: str,
    ) -> RecoveryProposal:
        item = self._pop(self._recoveries, "local", snapshot.request.checkpoint_id)
        if isinstance(item, RecoveryProposal):
            return item
        return RecoveryProposal.from_dict(item)

    def plan_global_replan(
        self,
        snapshot: SnapshotBundle,
        verification: VerificationDecision,
        reason: str,
    ) -> GlobalPlanDecision:
        item = self._pop(self._global_plans, "global", snapshot.request.checkpoint_id)
        if isinstance(item, GlobalPlanDecision):
            return item
        return GlobalPlanDecision.from_dict(item)

    def _pop(self, queue: deque, role: str, checkpoint_id: str):
        with self._lock:
            self.calls.append((role, checkpoint_id))
            if not queue:
                raise SupervisorUnavailable(f"no scripted {role} response remains")
            item = queue.popleft()
        if isinstance(item, BaseException):
            raise item
        if callable(item):
            item = item()
        return item


class OpenRouterSupervisorClient:
    """OpenAI-compatible OpenRouter client with strict structured responses."""

    def __init__(
        self,
        config: SupervisorConfig,
        *,
        api_key: str,
        client: Any = None,
        telemetry: Callable[[str, Mapping[str, Any]], None] | None = None,
    ) -> None:
        if not api_key:
            raise SupervisorUnavailable("OPENROUTER_API_KEY is not configured")
        self.config = config
        self._telemetry = telemetry
        if client is None:
            try:
                import openai
            except ImportError as exc:  # pragma: no cover - optional dependency
                raise SupervisorUnavailable("openai package is not installed") from exc
            client = openai.OpenAI(
                api_key=api_key,
                base_url="https://openrouter.ai/api/v1",
                timeout=max(config.verify_timeout_s, config.plan_timeout_s),
            )
        self._client = client

    def verify(self, snapshot: SnapshotBundle) -> VerificationDecision:
        def decode(raw: Mapping[str, Any]) -> VerificationDecision:
            decision = VerificationDecision.from_dict(raw)
            _same_checkpoint(snapshot, decision.checkpoint_id)
            return decision

        return self._query(
            role="verify",
            system=VERIFIER_SYSTEM_PROMPT,
            text=verifier_text(snapshot),
            snapshot=snapshot,
            schema=VERIFICATION_JSON_SCHEMA,
            effort=self.config.verify_effort,
            max_completion_tokens=4096,
            timeout_s=self.config.verify_timeout_s,
            decode=decode,
        )

    def plan_local_recovery(
        self,
        snapshot: SnapshotBundle,
        verification: VerificationDecision,
        issue_id: str,
    ) -> RecoveryProposal:
        def decode(raw: Mapping[str, Any]) -> RecoveryProposal:
            raw = _decode_embedded_object(raw, "arguments")
            decision = RecoveryProposal.from_dict(raw)
            _same_checkpoint(snapshot, decision.checkpoint_id)
            if decision.issue_id != issue_id:
                raise SchemaError(
                    f"stale issue id {decision.issue_id!r}; expected {issue_id!r}"
                )
            return decision

        return self._query(
            role="local_recovery",
            system=LOCAL_RECOVERY_SYSTEM_PROMPT,
            text=local_recovery_text(snapshot, verification, issue_id),
            snapshot=snapshot,
            schema=RECOVERY_JSON_SCHEMA,
            effort=self.config.plan_effort,
            max_completion_tokens=16384,
            timeout_s=self.config.plan_timeout_s,
            decode=decode,
        )

    def plan_global_replan(
        self,
        snapshot: SnapshotBundle,
        verification: VerificationDecision,
        reason: str,
    ) -> GlobalPlanDecision:
        def decode(raw: Mapping[str, Any]) -> GlobalPlanDecision:
            raw = _decode_embedded_plan(raw)
            decision = GlobalPlanDecision.from_dict(raw)
            _same_checkpoint(snapshot, decision.checkpoint_id)
            return decision

        return self._query(
            role="global_replan",
            system=GLOBAL_REPLAN_SYSTEM_PROMPT,
            text=global_replan_text(snapshot, verification, reason),
            snapshot=snapshot,
            schema=GLOBAL_PLAN_JSON_SCHEMA,
            effort=self.config.plan_effort,
            max_completion_tokens=16384,
            timeout_s=self.config.plan_timeout_s,
            decode=decode,
        )

    def _query(
        self,
        *,
        role: str,
        system: str,
        text: str,
        snapshot: SnapshotBundle,
        schema: Mapping[str, Any],
        effort: str,
        max_completion_tokens: int,
        timeout_s: float,
        decode: Callable[[Mapping[str, Any]], _Decision],
    ) -> _Decision:
        user_content: list[dict[str, Any]] = [{"type": "text", "text": text}]
        for artifact in snapshot.artifacts:
            if artifact.missing or not artifact.path or not artifact.mime_type.startswith("image/"):
                continue
            data = base64.b64encode(Path(artifact.path).read_bytes()).decode("ascii")
            user_content.append(
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:{artifact.mime_type};base64,{data}",
                    },
                }
            )
        request = {
            "model": self.config.model,
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": user_content},
            ],
            "response_format": {"type": "json_schema", "json_schema": dict(schema)},
            "max_completion_tokens": max_completion_tokens,
            "extra_body": {"reasoning": {"effort": effort}},
            "timeout": timeout_s,
        }
        started = time.monotonic()
        last_error: Exception | None = None
        for attempt in range(2):
            content: Any = None
            try:
                response = self._client.chat.completions.create(**request)
                content = response.choices[0].message.content
                raw = _decode_json_content(content)
                # Z-2 (task-O review, MEDIUM): the retry boundary must cover
                # every SchemaError raise site, not just _decode_json_content
                # (empty/non-JSON/non-object content). VerificationDecision
                # /RecoveryProposal/GlobalPlanDecision.from_dict() (e.g.
                # confidence out of [0,1]), _same_checkpoint() (a
                # hallucinated/stale checkpoint_id), and
                # _decode_embedded_object()/_decode_embedded_plan() (a
                # malformed embedded-JSON arguments/replacement_plan string)
                # used to run AFTER _query() already returned successfully,
                # in verify()/plan_local_recovery()/plan_global_replan(), so
                # a SchemaError from any of them got zero retries --
                # contradicting "keep total attempts <= 2 for every error
                # class". Decoding the full raw response into its typed
                # decision here, inside the same try/except as the HTTP call
                # and JSON parse, gives every one of those raise sites the
                # same one-fresh-request retry.
                decision = decode(raw)
                self._emit(
                    "supervisor.query.completed",
                    {
                        "role": role,
                        "checkpoint_id": snapshot.request.checkpoint_id,
                        "model": self.config.model,
                        "reasoning_effort": effort,
                        "prompt_version": PROMPT_VERSION,
                        "latency_ms": round((time.monotonic() - started) * 1000),
                        "attempt": attempt + 1,
                        "usage": _usage_dict(getattr(response, "usage", None)),
                    },
                )
                return decision
            except Exception as exc:
                # A malformed/empty structured response (SchemaError, or a
                # raw json.JSONDecodeError) gets the same single
                # fresh-request retry as a transient transport/provider
                # failure -- total attempts stay <= 2 for every error class.
                last_error = exc
                # Task R: a SchemaError body is currently discarded the
                # moment it propagates, so the 6-9 persistent verifier
                # failures per sim run are untypeable offline -- capture the
                # raw response for each failed attempt of the *retried
                # unit's own* schema-class errors only. Transport/timeout
                # errors keep today's behavior exactly (no capture).
                if isinstance(exc, (SchemaError, json.JSONDecodeError)):
                    _persist_schema_error_body(snapshot, role, attempt + 1, content)
                if attempt == 0:
                    continue
        failure_payload: dict[str, Any] = {
            "role": role,
            "checkpoint_id": snapshot.request.checkpoint_id,
            "model": self.config.model,
            "error_type": type(last_error).__name__ if last_error else "unknown",
        }
        if isinstance(last_error, (SchemaError, json.JSONDecodeError)):
            failure_payload["raw_content_snippet"] = _content_snippet(content)
            failure_payload["error_message"] = str(last_error)
            failure_payload["attempt"] = attempt + 1
        self._emit("supervisor.query.failed", failure_payload)
        raise SupervisorUnavailable(
            f"{role} query failed after two attempts: {type(last_error).__name__}"
        ) from last_error

    def _emit(self, event: str, payload: Mapping[str, Any]) -> None:
        if self._telemetry is not None:
            try:
                self._telemetry(event, payload)
            except Exception:
                pass


def _stringify_content(content: Any) -> str:
    """Best-effort flatten of a chat-completion message content field."""
    # Handles the multi-part list shape ([{"type": "text", "text": ...}])
    # the same way _decode_json_content always has; anything that isn't a
    # string or a list of such parts becomes "" (matching the previous
    # `not isinstance(content, str)` rejection in that function).
    if isinstance(content, list):
        return "".join(
            item.get("text", "") if isinstance(item, Mapping) else str(item)
            for item in content
        )
    if isinstance(content, str):
        return content
    return ""


def _decode_json_content(content: Any) -> Mapping[str, Any]:
    text = _stringify_content(content)
    if not text.strip():
        raise SchemaError("model returned empty structured content")
    text = text.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines)
    raw = json.loads(text)
    if not isinstance(raw, Mapping):
        raise SchemaError("model response must be a JSON object")
    return raw


def _decode_embedded_object(
    raw: Mapping[str, Any], key: str
) -> Mapping[str, Any]:
    value = raw.get(key)
    if not isinstance(value, str):
        raise SchemaError(f"{key} must be a JSON-encoded object string")
    try:
        decoded = json.loads(value)
    except json.JSONDecodeError as exc:
        raise SchemaError(f"{key} is not valid encoded JSON") from exc
    if not isinstance(decoded, Mapping):
        raise SchemaError(f"{key} must decode to an object")
    return {**raw, key: dict(decoded)}


def _decode_embedded_plan(raw: Mapping[str, Any]) -> Mapping[str, Any]:
    value = raw.get("replacement_plan")
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise SchemaError(
            "replacement_plan must be an array of JSON-encoded step strings"
        )
    decoded = []
    for index, item in enumerate(value):
        try:
            step = json.loads(item)
        except json.JSONDecodeError as exc:
            raise SchemaError(
                f"replacement_plan[{index}] is not valid encoded JSON"
            ) from exc
        if not isinstance(step, Mapping):
            raise SchemaError(
                f"replacement_plan[{index}] must decode to an object"
            )
        decoded.append(dict(step))
    return {**raw, "replacement_plan": decoded}


def _same_checkpoint(snapshot: SnapshotBundle, response_checkpoint_id: str) -> None:
    expected = snapshot.request.checkpoint_id
    if response_checkpoint_id != expected:
        raise SchemaError(
            f"stale checkpoint {response_checkpoint_id!r}; expected {expected!r}"
        )


_SCHEMA_ERROR_BODY_LIMIT_BYTES = 32 * 1024
_SCHEMA_ERROR_TRUNCATION_MARKER = "...[truncated]"


def _content_snippet(content: Any, limit: int = 500) -> str:
    """500-char, whitespace-collapsed preview for the failed-query event."""
    collapsed = " ".join(_stringify_content(content).split())
    return collapsed[:limit]


def _truncate_body(text: str) -> str:
    encoded = text.encode("utf-8", errors="replace")
    if len(encoded) <= _SCHEMA_ERROR_BODY_LIMIT_BYTES:
        return text
    marker = _SCHEMA_ERROR_TRUNCATION_MARKER.encode("utf-8")
    keep = max(_SCHEMA_ERROR_BODY_LIMIT_BYTES - len(marker), 0)
    return encoded[:keep].decode("utf-8", errors="ignore") + _SCHEMA_ERROR_TRUNCATION_MARKER


def _safe_checkpoint_name(checkpoint_id: str) -> str:
    # Same sanitisation FixtureContextProvider.capture() uses for its own
    # artifact filenames (context.py) -- keep the two conventions aligned so
    # a debug file sits next to the map/arm renders it names.
    return "".join(
        char if char.isalnum() or char in "-_" else "_" for char in checkpoint_id
    )[:120]


def _artifact_debug_dir(snapshot: SnapshotBundle) -> Path | None:
    """Return the directory a context provider already wrote artifacts into."""
    # Derived from the first artifact that actually has a path (camera
    # artifacts may be legitimately absent; map/arm renders are not).
    # Returns None when nothing in the snapshot carries a real path -- e.g.
    # a ContextProvider or test double that never persisted anything.
    for artifact in snapshot.artifacts:
        if artifact.missing or not artifact.path:
            continue
        return Path(artifact.path).parent
    return None


def _fallback_debug_dir() -> Path:
    # Same root + "debug" convention GpsrTelemetry uses for its run-scoped
    # NDJSON audit log (behavior_tree/GPSR/telemetry.py); the client has no
    # trajectory id to scope by, so these land in a shared "schema_errors"
    # leaf instead, distinguished by the checkpoint-id filename prefix.
    base = Path(os.environ.get("BT_GPSR_PLAN_DIR", "gpsr_runs"))
    return base / "debug" / "schema_errors"


def _persist_schema_error_body(
    snapshot: SnapshotBundle, role: str, attempt: int, content: Any
) -> None:
    """Best-effort dump of a failed attempt's raw response body to disk."""
    # So a persistent SchemaError/JSONDecodeError can be classified offline.
    # Must never raise: a failed write cannot mask the original SchemaError
    # or crash the query path (task R requirement 2).
    try:
        directory = _artifact_debug_dir(snapshot) or _fallback_debug_dir()
        directory.mkdir(parents=True, exist_ok=True)
        body = _truncate_body(_stringify_content(content))
        checkpoint = _safe_checkpoint_name(snapshot.request.checkpoint_id)
        path = directory / f"{checkpoint}_schema_error_{role}_{attempt}.txt"
        path.write_text(body, encoding="utf-8")
    except Exception:
        pass


def _usage_dict(usage: Any) -> Mapping[str, Any]:
    if usage is None:
        return {}
    if hasattr(usage, "model_dump"):
        return usage.model_dump()
    if isinstance(usage, Mapping):
        return dict(usage)
    return {
        key: getattr(usage, key)
        for key in ("prompt_tokens", "completion_tokens", "total_tokens")
        if hasattr(usage, key)
    }


__all__ = [
    "OpenRouterSupervisorClient",
    "ScriptedSupervisorClient",
    "SupervisorClient",
]
