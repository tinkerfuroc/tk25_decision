"""Plan-time checker for GPSR small-tree modifications.

The lower-layer planner authorizes a modification (a typed, template-constrained
directive against a target step's small tree). This module audits it: given the
target's plan, the serialized BEFORE/AFTER tree (via ``serialize_tree``), and the
``(template, node_id, reason)`` audit tuples, a checker LLM pinpoints exactly
which nodes changed and whether each is consistent with the command.

The checker reuses the supervision client pattern (strict ``json_schema``
responses, reasoning-effort knob, telemetry) with a ``ScriptedPlanChecker``
deterministic twin for offline tests. It is plan-time only and env-gated
(``GPSR_PLAN_CHECKER=on``), defaulting to OFF — a fully-offline pipeline is
unaffected.
"""
from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Protocol

from .modifiable_nodes import diff_trees


@dataclass(frozen=True)
class CheckedNode:
    """One modified node the checker pinpointed in a plan's tree."""

    node_id: str
    template: str
    reason: str
    consistent_with_command: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "node_id": self.node_id,
            "template": self.template,
            "reason": self.reason,
            "consistent_with_command": self.consistent_with_command,
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "CheckedNode":
        return cls(
            node_id=str(raw.get("node_id") or ""),
            template=str(raw.get("template") or ""),
            reason=str(raw.get("reason") or ""),
            consistent_with_command=bool(raw.get("consistent_with_command", False)),
        )


@dataclass(frozen=True)
class PlanCheck:
    """The checker's verdict for one target's modifications."""

    checkpoint: str
    approved: bool
    modified_nodes: tuple[CheckedNode, ...] = ()
    summary: str = ""

    @classmethod
    def approved(cls, checkpoint: str, *, summary: str = "") -> "PlanCheck":
        return cls(checkpoint=checkpoint, approved=True, summary=summary)

    @classmethod
    def denied(cls, checkpoint: str, *, summary: str = "") -> "PlanCheck":
        return cls(checkpoint=checkpoint, approved=False, summary=summary)

    def to_dict(self) -> dict[str, Any]:
        return {
            "checkpoint": self.checkpoint,
            "approved": self.approved,
            "modified_nodes": [node.to_dict() for node in self.modified_nodes],
            "summary": self.summary,
        }


class PlanCheckerError(ValueError):
    """Raised when a checker response violates its typed contract."""


#: Strict response schema for the checker LLM (OpenRouter json_schema mode).
CHECKER_JSON_SCHEMA = {
    "name": "plan_check",
    "schema": {
        "type": "object",
        "properties": {
            "approved": {"type": "boolean"},
            "summary": {"type": "string"},
            "modified_nodes": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "node_id": {"type": "string"},
                        "template": {"type": "string"},
                        "reason": {"type": "string"},
                        "consistent_with_command": {"type": "boolean"},
                    },
                    "required": ["node_id", "consistent_with_command"],
                },
            },
        },
        "required": ["approved"],
    },
}

CHECKER_SYSTEM_PROMPT = (
    "You audit a household-robot planning change. The lower-layer planner added "
    "a typed modification to a target's behavior tree. Given the target's "
    "command, the BEFORE and AFTER trees (stable structural node ids), and the "
    "change tuples, decide whether each modified node is consistent with the "
    "command. Approve ONLY changes that genuinely match the instruction and "
    "introduce no unrelated behaviour. Reply with JSON only."
)


def checker_text(
    *,
    command: str,
    desc: str,
    applied: list[tuple[str, str, str]],
    before_tree: Mapping[str, Any],
    after_tree: Mapping[str, Any],
) -> str:
    """Build the checker user prompt for one target's plan + modification."""
    lines = [
        f"Target command: {command}",
        f"Target goal: {desc}",
        "",
        "Change tuples (template, node_id, reason):",
    ]
    if applied:
        for template, node_id, reason in applied:
            lines.append(f"  - {template} @ {node_id}: {reason}")
    else:
        lines.append("  (none)")
    lines += ["", "BEFORE tree (serialized):", json.dumps(before_tree, default=str)[:4000]]
    lines += ["", "AFTER tree (serialized):", json.dumps(after_tree, default=str)[:4000]]
    lines += ["", "Return the JSON plan_check object now."]
    return "\n".join(lines)


@dataclass(frozen=True)
class CheckerConfig:
    enabled: bool = False
    model: str = "openai/gpt-5.6-luna"
    effort: str = "medium"
    timeout_s: float = 60.0

    @classmethod
    def from_env(cls, environ: Mapping[str, str] | None = None) -> "CheckerConfig":
        env = os.environ if environ is None else environ
        enabled = str(env.get("GPSR_PLAN_CHECKER", "off")).strip().lower() in {
            "1", "true", "yes", "on",
        }
        return cls(
            enabled=enabled,
            model=env.get("GPSR_PLAN_CHECKER_MODEL", "openai/gpt-5.6-luna").strip(),
            effort=env.get("GPSR_PLAN_CHECKER_EFFORT", "medium").strip().lower(),
            timeout_s=float(env.get("GPSR_PLAN_CHECKER_TIMEOUT_S", "60")),
        )


class PlanCheckerClient(Protocol):
    def check(self, *, command: str, desc: str, applied, before_tree, after_tree) -> PlanCheck:
        ...


class ScriptedPlanChecker:
    """Deterministic, network-free checker for offline tests and full-mock runs.

    ``checks`` is a list of either ``PlanCheck`` objects or a callable that
    returns one. Each ``check`` call pops the next. When exhausted, ``default``
    (a ``PlanCheck`` or callable returning one) is used — so a harness can pass
    an auto-approve default and never exhaust.
    """

    def __init__(
        self,
        checks: list[PlanCheck | Callable[[], PlanCheck]] | None = None,
        *,
        default: PlanCheck | Callable[[], PlanCheck] | None = None,
    ) -> None:
        from collections import deque
        self._checks = deque(checks or [])
        self._default = default
        self.calls: list[str] = []

    def check(self, *, command: str, desc: str, applied, before_tree, after_tree) -> PlanCheck:
        self.calls.append(command)
        if self._checks:
            item = self._checks.popleft()
            return item() if callable(item) else item
        if self._default is not None:
            return self._default() if callable(self._default) else self._default
        return PlanCheck.denied("scripted", summary="no scripted response remains")


class OpenRouterPlanChecker:
    """OpenAI-compatible OpenRouter checker with strict structured responses."""

    def __init__(
        self,
        config: CheckerConfig,
        *,
        api_key: str,
        client: Any = None,
        telemetry: Callable[[str, Mapping[str, Any]], None] | None = None,
    ) -> None:
        if not config.enabled:
            raise PlanCheckerError("checker is disabled (GPSR_PLAN_CHECKER=off)")
        self.config = config
        self._telemetry = telemetry
        if client is None:
            import openai
            client = openai.OpenAI(
                api_key=api_key,
                base_url="https://openrouter.ai/api/v1",
                timeout=config.timeout_s,
            )
        self._client = client

    def check(self, *, command: str, desc: str, applied, before_tree, after_tree) -> PlanCheck:
        text = checker_text(
            command=command, desc=desc, applied=list(applied),
            before_tree=before_tree, after_tree=after_tree,
        )
        started = time.monotonic()
        request = {
            "model": self.config.model,
            "messages": [
                {"role": "system", "content": CHECKER_SYSTEM_PROMPT},
                {"role": "user", "content": text},
            ],
            "response_format": {"type": "json_schema", "json_schema": dict(CHECKER_JSON_SCHEMA)},
            "max_completion_tokens": 4096,
            "extra_body": {"reasoning": {"effort": self.config.effort}},
            "timeout": self.config.timeout_s,
        }
        response = self._client.chat.completions.create(**request)
        content = response.choices[0].message.content
        raw = _decode_json_object(content)
        if "approved" not in raw or not isinstance(raw["approved"], bool):
            raise PlanCheckerError("checker response missing boolean 'approved'")
        nodes = tuple(
            CheckedNode.from_dict(item) for item in (raw.get("modified_nodes") or [])
        )
        check = PlanCheck(
            checkpoint="plan-check",
            approved=raw["approved"],
            modified_nodes=nodes,
            summary=str(raw.get("summary") or ""),
        )
        self._emit("plan_checker.completed", {
            "approved": check.approved,
            "modified_nodes": len(check.modified_nodes),
            "latency_ms": round((time.monotonic() - started) * 1000),
        })
        return check

    def _emit(self, event: str, payload: Mapping[str, Any]) -> None:
        if self._telemetry is not None:
            try:
                self._telemetry(event, dict(payload))
            except Exception:
                pass


def _decode_json_object(content: Any) -> Mapping[str, Any]:
    if isinstance(content, list):
        content = "".join(item.get("text", "") for item in content if isinstance(item, Mapping))
    if not isinstance(content, str) or not content.strip():
        raise PlanCheckerError("checker returned empty structured content")
    text = content.strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if lines and lines[0].startswith("```"):
            lines = lines[1:]
        if lines and lines[-1].strip() == "```":
            lines = lines[:-1]
        text = "\n".join(lines)
    raw = json.loads(text)
    if not isinstance(raw, Mapping):
        raise PlanCheckerError("checker response must be a JSON object")
    return raw


def make_checker(
    config: CheckerConfig | None = None,
    *,
    api_key: str = "",
    telemetry: Callable[[str, Mapping[str, Any]], None] | None = None,
) -> PlanCheckerClient | None:
    """Return a live OpenRouter checker, or None when the checker is OFF.

    Returns None (not a scripted stub) so callers treat it as "checking is
    disabled" — the ten-command harness swaps in a ``ScriptedPlanChecker`` when
    it wants offline auditing.
    """
    config = config or CheckerConfig.from_env()
    if not config.enabled:
        return None
    return OpenRouterPlanChecker(
        config,
        api_key=api_key,
        telemetry=telemetry,
    )


def build_check_context(
    before: Any,
    after: Any,
    action: str,
    applied: list[tuple[str, str, str]],
) -> dict[str, Any]:
    """Assemble the before/after serialized trees + the structural diff."""
    before_tree = _serialize(before, action, "before")
    after_tree = _serialize(after, action, "after")
    return {
        "before_tree": before_tree,
        "after_tree": after_tree,
        "diff": diff_trees(before, after, f"small/{action}"),
        "applied": list(applied),
    }


def _serialize(root: Any, action: str, label: str) -> dict[str, Any]:
    from .tree_serialization import serialize_tree
    return serialize_tree(root, kind=f"small/{action}", label=label)


__all__ = [
    "CHECKER_JSON_SCHEMA",
    "CHECKER_SYSTEM_PROMPT",
    "CheckedNode",
    "CheckerConfig",
    "OpenRouterPlanChecker",
    "PlanCheck",
    "PlanCheckerError",
    "ScriptedPlanChecker",
    "build_check_context",
    "checker_text",
    "make_checker",
]
