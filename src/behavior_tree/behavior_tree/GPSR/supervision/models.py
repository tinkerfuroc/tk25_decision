"""Typed contracts shared by the GPSR execution supervisor.

The LLM boundary deliberately uses small, validated JSON documents.  Runtime
objects (``py_trees`` nodes, ROS messages and blackboard clients) never cross
that boundary.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
import hashlib
import json
import os
from pathlib import Path
from typing import Any, Mapping, Sequence


class StrEnum(str, Enum):
    def __str__(self) -> str:
        return self.value


class SupervisionMode(StrEnum):
    OFF = "off"
    SHADOW = "shadow"
    ACTIVE = "active"


class SuccessMode(StrEnum):
    HYBRID = "hybrid"
    OPTIMISTIC = "optimistic"


class EffectRisk(StrEnum):
    OBSERVATION = "observation"
    REVERSIBLE_MOTION = "reversible_motion"
    IRREVERSIBLE = "irreversible"


class ReportedStatus(StrEnum):
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class Verdict(StrEnum):
    ALL_CLEAR = "all_clear"
    RECOVERABLE = "recoverable"
    UNRECOVERABLE = "unrecoverable"
    UNCERTAIN = "uncertain"


class BtAssessment(StrEnum):
    AGREE = "agree"
    FALSE_SUCCESS = "false_success"
    FALSE_FAILURE = "false_failure"


class SubtaskStatus(StrEnum):
    ACHIEVED = "achieved"
    NOT_ACHIEVED = "not_achieved"
    UNKNOWN = "unknown"


class WorldChange(StrEnum):
    NONE = "none"
    NON_DESTRUCTIVE = "non_destructive"
    DESTRUCTIVE = "destructive"
    UNKNOWN = "unknown"


class Escalation(StrEnum):
    NONE = "none"
    LOCAL_RECOVERY = "local_recovery"
    GLOBAL_REPLAN = "global_replan"
    STOP = "stop"


class RecoveryKind(StrEnum):
    SCAN_VIEWS = "scan_views"
    REACQUIRE_OBJECT = "reacquire_object"
    RETRY_NAVIGATION = "retry_navigation"
    RELOCALIZE = "relocalize"
    ASK_HUMAN = "ask_human"


class GlobalAction(StrEnum):
    REPLAN_REMAINING = "replan_remaining"
    RELAX_GOAL = "relax_goal"
    ABORT_AND_REPORT = "abort_and_report"


class SupervisorUnavailable(RuntimeError):
    """Raised when a supervisor query cannot be completed."""


class SchemaError(ValueError):
    """Raised when an LLM response violates its typed contract."""


@dataclass(frozen=True)
class SupervisorConfig:
    mode: SupervisionMode = SupervisionMode.OFF
    success_mode: SuccessMode = SuccessMode.HYBRID
    model: str = "openai/gpt-5.6-luna"
    verify_effort: str = "medium"
    plan_effort: str = "high"
    max_recoveries: int = 3
    verify_timeout_s: float = 60.0
    plan_timeout_s: float = 120.0
    run_live_tests: bool = False

    @classmethod
    def from_env(cls, environ: Mapping[str, str] | None = None) -> "SupervisorConfig":
        env = os.environ if environ is None else environ
        mode = _enum(SupervisionMode, env.get("GPSR_SUPERVISION_MODE", "off"), "mode")
        success_mode = _enum(
            SuccessMode,
            env.get("GPSR_SUPERVISION_SUCCESS_MODE", "hybrid"),
            "success mode",
        )
        verify_effort = env.get("GPSR_SUPERVISOR_VERIFY_EFFORT", "medium").strip().lower()
        plan_effort = env.get("GPSR_SUPERVISOR_PLAN_EFFORT", "high").strip().lower()
        allowed_effort = {"none", "low", "medium", "high", "xhigh", "max"}
        if verify_effort not in allowed_effort or plan_effort not in allowed_effort:
            raise ValueError("supervisor reasoning effort must be none/low/medium/high/xhigh/max")
        max_recoveries = int(env.get("GPSR_SUPERVISOR_MAX_RECOVERIES", "3"))
        if max_recoveries < 1:
            raise ValueError("GPSR_SUPERVISOR_MAX_RECOVERIES must be positive")
        return cls(
            mode=mode,
            success_mode=success_mode,
            model=env.get("GPSR_SUPERVISOR_MODEL", "openai/gpt-5.6-luna").strip(),
            verify_effort=verify_effort,
            plan_effort=plan_effort,
            max_recoveries=max_recoveries,
            verify_timeout_s=float(env.get("GPSR_SUPERVISOR_VERIFY_TIMEOUT_S", "60")),
            plan_timeout_s=float(env.get("GPSR_SUPERVISOR_PLAN_TIMEOUT_S", "120")),
            run_live_tests=_truthy(env.get("GPSR_RUN_LIVE_LLM_TESTS", "0")),
        )


@dataclass(frozen=True)
class NodeContract:
    class_name: str
    effect: str
    risk: EffectRisk
    expected_postcondition: str
    evidence_modalities: tuple[str, ...] = ()
    blackboard_keys: tuple[str, ...] = ()
    allow_local_recovery: bool = True


@dataclass(frozen=True)
class ArtifactRef:
    role: str
    mime_type: str
    path: str | None
    sha256: str | None
    captured_at: str
    missing: bool = False
    metadata: Mapping[str, Any] = field(default_factory=dict)

    @classmethod
    def from_path(
        cls,
        *,
        role: str,
        mime_type: str,
        path: Path,
        captured_at: str,
        metadata: Mapping[str, Any] | None = None,
    ) -> "ArtifactRef":
        data = path.read_bytes()
        return cls(
            role=role,
            mime_type=mime_type,
            path=str(path),
            sha256=hashlib.sha256(data).hexdigest(),
            captured_at=captured_at,
            metadata=dict(metadata or {}),
        )

    @classmethod
    def absent(cls, role: str, captured_at: str, reason: str) -> "ArtifactRef":
        return cls(
            role=role,
            mime_type="application/octet-stream",
            path=None,
            sha256=None,
            captured_at=captured_at,
            missing=True,
            metadata={"reason": reason},
        )


@dataclass(frozen=True)
class CaptureRequest:
    checkpoint_id: str
    task_id: str
    subtask_id: str
    tree_revision: str
    plan_revision: int
    original_instruction: str
    subtask_goal: str
    terminal_node: Mapping[str, Any]
    next_node: Mapping[str, Any] | None
    subtask_tree: Mapping[str, Any]
    blackboard: Mapping[str, Any]
    execution_history: Sequence[Mapping[str, Any]]
    recovery_ledger: Sequence[Mapping[str, Any]]
    robot_pose: tuple[float, float, float] = (0.0, 0.0, 0.0)
    arm_joints: tuple[float, ...] = (0.0,) * 7


@dataclass(frozen=True)
class SnapshotBundle:
    request: CaptureRequest
    artifacts: tuple[ArtifactRef, ...]

    def to_prompt_dict(self) -> dict[str, Any]:
        return {
            "checkpoint_id": self.request.checkpoint_id,
            "task_id": self.request.task_id,
            "subtask_id": self.request.subtask_id,
            "tree_revision": self.request.tree_revision,
            "plan_revision": self.request.plan_revision,
            "original_instruction": self.request.original_instruction,
            "subtask_goal": self.request.subtask_goal,
            "terminal_node": _json_value(self.request.terminal_node),
            "next_node": _json_value(self.request.next_node),
            "subtask_tree": _json_value(self.request.subtask_tree),
            "blackboard": _json_value(self.request.blackboard),
            "execution_history": _json_value(self.request.execution_history),
            "recovery_ledger": _json_value(self.request.recovery_ledger),
            "artifacts": [_json_value(asdict(artifact)) for artifact in self.artifacts],
        }


@dataclass(frozen=True)
class VerificationDecision:
    checkpoint_id: str
    verdict: Verdict
    bt_assessment: BtAssessment
    subtask_status: SubtaskStatus
    world_change: WorldChange
    escalation: Escalation
    failure_category: str
    evidence: tuple[str, ...]
    rationale: str
    confidence: float

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "VerificationDecision":
        confidence = _number(raw, "confidence")
        if confidence < 0.0 or confidence > 1.0:
            raise SchemaError("confidence must be between 0 and 1")
        evidence = raw.get("evidence", [])
        if not isinstance(evidence, list) or not all(isinstance(item, str) for item in evidence):
            raise SchemaError("evidence must be an array of strings")
        return cls(
            checkpoint_id=_required_text(raw, "checkpoint_id"),
            verdict=_enum(Verdict, raw.get("verdict"), "verdict", SchemaError),
            bt_assessment=_enum(
                BtAssessment, raw.get("bt_assessment"), "bt_assessment", SchemaError
            ),
            subtask_status=_enum(
                SubtaskStatus, raw.get("subtask_status"), "subtask_status", SchemaError
            ),
            world_change=_enum(
                WorldChange, raw.get("world_change"), "world_change", SchemaError
            ),
            escalation=_enum(
                Escalation, raw.get("escalation"), "escalation", SchemaError
            ),
            failure_category=_required_text(raw, "failure_category", allow_empty=True),
            evidence=tuple(evidence),
            rationale=_required_text(raw, "rationale"),
            confidence=confidence,
        )


@dataclass(frozen=True)
class RecoveryProposal:
    checkpoint_id: str
    issue_id: str
    strategy_id: str
    kind: RecoveryKind
    arguments: Mapping[str, Any]
    rationale: str
    expected_evidence: tuple[str, ...]
    stop_conditions: tuple[str, ...]

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "RecoveryProposal":
        arguments = raw.get("arguments")
        if not isinstance(arguments, Mapping):
            raise SchemaError("arguments must be an object")
        return cls(
            checkpoint_id=_required_text(raw, "checkpoint_id"),
            issue_id=_required_text(raw, "issue_id"),
            strategy_id=_required_text(raw, "strategy_id"),
            kind=_enum(RecoveryKind, raw.get("kind"), "kind", SchemaError),
            arguments=dict(arguments),
            rationale=_required_text(raw, "rationale"),
            expected_evidence=_string_tuple(raw, "expected_evidence"),
            stop_conditions=_string_tuple(raw, "stop_conditions"),
        )


@dataclass(frozen=True)
class GlobalPlanDecision:
    checkpoint_id: str
    action: GlobalAction
    replacement_plan: tuple[Mapping[str, Any], ...]
    preserved_completed_steps: int
    relaxed_constraints: tuple[str, ...]
    rationale: str
    operator_message: str

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "GlobalPlanDecision":
        plan = raw.get("replacement_plan", [])
        if not isinstance(plan, list) or not all(isinstance(step, Mapping) for step in plan):
            raise SchemaError("replacement_plan must be an array of objects")
        preserved = raw.get("preserved_completed_steps", 0)
        if not isinstance(preserved, int) or isinstance(preserved, bool) or preserved < 0:
            raise SchemaError("preserved_completed_steps must be a non-negative integer")
        return cls(
            checkpoint_id=_required_text(raw, "checkpoint_id"),
            action=_enum(GlobalAction, raw.get("action"), "action", SchemaError),
            replacement_plan=tuple(dict(step) for step in plan),
            preserved_completed_steps=preserved,
            relaxed_constraints=_string_tuple(raw, "relaxed_constraints"),
            rationale=_required_text(raw, "rationale"),
            operator_message=_required_text(raw, "operator_message"),
        )


VERIFICATION_JSON_SCHEMA: dict[str, Any] = {
    "name": "gpsr_verification_decision",
    "strict": True,
    "schema": {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "checkpoint_id", "verdict", "bt_assessment", "subtask_status",
            "world_change", "escalation", "failure_category", "evidence",
            "rationale", "confidence",
        ],
        "properties": {
            "checkpoint_id": {"type": "string"},
            "verdict": {"enum": [item.value for item in Verdict]},
            "bt_assessment": {"enum": [item.value for item in BtAssessment]},
            "subtask_status": {"enum": [item.value for item in SubtaskStatus]},
            "world_change": {"enum": [item.value for item in WorldChange]},
            "escalation": {"enum": [item.value for item in Escalation]},
            "failure_category": {"type": "string"},
            "evidence": {"type": "array", "items": {"type": "string"}},
            "rationale": {"type": "string"},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
        },
    },
}


RECOVERY_JSON_SCHEMA: dict[str, Any] = {
    "name": "gpsr_local_recovery",
    "strict": True,
    "schema": {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "checkpoint_id", "issue_id", "strategy_id", "kind", "arguments",
            "rationale", "expected_evidence", "stop_conditions",
        ],
        "properties": {
            "checkpoint_id": {"type": "string"},
            "issue_id": {"type": "string"},
            "strategy_id": {"type": "string"},
            "kind": {"enum": [item.value for item in RecoveryKind]},
            "arguments": {"type": "object"},
            "rationale": {"type": "string"},
            "expected_evidence": {"type": "array", "items": {"type": "string"}},
            "stop_conditions": {"type": "array", "items": {"type": "string"}},
        },
    },
}


GLOBAL_PLAN_JSON_SCHEMA: dict[str, Any] = {
    "name": "gpsr_global_replan",
    "strict": True,
    "schema": {
        "type": "object",
        "additionalProperties": False,
        "required": [
            "checkpoint_id", "action", "replacement_plan",
            "preserved_completed_steps", "relaxed_constraints", "rationale",
            "operator_message",
        ],
        "properties": {
            "checkpoint_id": {"type": "string"},
            "action": {"enum": [item.value for item in GlobalAction]},
            "replacement_plan": {
                "type": "array",
                "items": {"type": "object"},
            },
            "preserved_completed_steps": {"type": "integer", "minimum": 0},
            "relaxed_constraints": {"type": "array", "items": {"type": "string"}},
            "rationale": {"type": "string"},
            "operator_message": {"type": "string"},
        },
    },
}


def issue_identity(
    *,
    subtask_goal: str,
    effect: str,
    failure_category: str,
    target: Any = None,
    location: Any = None,
) -> str:
    canonical = json.dumps(
        {
            "subtask_goal": subtask_goal.strip().lower(),
            "effect": effect.strip().lower(),
            "failure_category": failure_category.strip().lower(),
            "target": _json_value(target),
            "location": _json_value(location),
        },
        sort_keys=True,
        separators=(",", ":"),
    )
    return "issue-" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]


def _enum(enum_type, raw: Any, label: str, error_type=ValueError):
    try:
        return enum_type(str(raw))
    except (TypeError, ValueError) as exc:
        choices = ", ".join(item.value for item in enum_type)
        raise error_type(f"{label} must be one of: {choices}") from exc


def _required_text(raw: Mapping[str, Any], key: str, *, allow_empty: bool = False) -> str:
    value = raw.get(key)
    if not isinstance(value, str) or (not allow_empty and not value.strip()):
        raise SchemaError(f"{key} must be a {'possibly empty ' if allow_empty else ''}string")
    return value


def _number(raw: Mapping[str, Any], key: str) -> float:
    value = raw.get(key)
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise SchemaError(f"{key} must be a number")
    return float(value)


def _string_tuple(raw: Mapping[str, Any], key: str) -> tuple[str, ...]:
    value = raw.get(key, [])
    if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
        raise SchemaError(f"{key} must be an array of strings")
    return tuple(value)


def _truthy(value: str) -> bool:
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _json_value(value: Any, depth: int = 0) -> Any:
    if depth > 8:
        return "<max-depth>"
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, Mapping):
        return {str(key): _json_value(item, depth + 1) for key, item in value.items()}
    if isinstance(value, (list, tuple, set)):
        return [_json_value(item, depth + 1) for item in value]
    fields = getattr(value, "__dict__", None)
    if isinstance(fields, Mapping):
        return {
            str(key): _json_value(item, depth + 1)
            for key, item in fields.items()
            if not str(key).startswith("_")
        }
    return str(value)


__all__ = [
    "ArtifactRef",
    "BtAssessment",
    "CaptureRequest",
    "EffectRisk",
    "Escalation",
    "GlobalAction",
    "GlobalPlanDecision",
    "GLOBAL_PLAN_JSON_SCHEMA",
    "NodeContract",
    "RecoveryKind",
    "RecoveryProposal",
    "RECOVERY_JSON_SCHEMA",
    "ReportedStatus",
    "SchemaError",
    "SnapshotBundle",
    "SubtaskStatus",
    "SuccessMode",
    "SupervisionMode",
    "SupervisorConfig",
    "SupervisorUnavailable",
    "Verdict",
    "VerificationDecision",
    "VERIFICATION_JSON_SCHEMA",
    "WorldChange",
    "issue_identity",
]
