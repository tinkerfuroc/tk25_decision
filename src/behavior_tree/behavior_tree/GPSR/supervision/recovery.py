"""Recovery macro validation and distinct-attempt accounting."""
from __future__ import annotations

from dataclasses import asdict, dataclass
import hashlib
import json
import threading
from typing import Any, Callable, Mapping

from .models import (
    GlobalAction,
    GlobalPlanDecision,
    RecoveryKind,
    RecoveryProposal,
    SchemaError,
)


_ALLOWED_ARGUMENTS = {
    RecoveryKind.SCAN_VIEWS: frozenset({"angles", "perception_action"}),
    RecoveryKind.REACQUIRE_OBJECT: frozenset({"target", "viewpoints"}),
    RecoveryKind.RETRY_NAVIGATION: frozenset(
        {"target_location", "approach_offset_m", "attempts"}
    ),
    RecoveryKind.RELOCALIZE: frozenset({"method"}),
    RecoveryKind.ASK_HUMAN: frozenset({"prompt", "confirmation_required"}),
}


@dataclass
class RecoveryAttempt:
    issue_id: str
    strategy_id: str
    fingerprint: str
    kind: str
    arguments: Mapping[str, Any]
    state: str = "proposed"
    succeeded: bool | None = None


class RecoveryLedger:
    """Count only distinct, validated strategies that actually ran."""

    def __init__(self, max_distinct_failures: int = 3) -> None:
        if max_distinct_failures < 1:
            raise ValueError("max_distinct_failures must be positive")
        self.max_distinct_failures = max_distinct_failures
        self._attempts: dict[str, list[RecoveryAttempt]] = {}
        self._lock = threading.RLock()

    def register(self, proposal: RecoveryProposal) -> RecoveryAttempt:
        validate_recovery_macro(proposal)
        fingerprint = recovery_fingerprint(proposal)
        with self._lock:
            attempts = self._attempts.setdefault(proposal.issue_id, [])
            if any(attempt.fingerprint == fingerprint for attempt in attempts):
                raise SchemaError("duplicate recovery strategy for this issue")
            attempt = RecoveryAttempt(
                issue_id=proposal.issue_id,
                strategy_id=proposal.strategy_id,
                fingerprint=fingerprint,
                kind=proposal.kind.value,
                arguments=dict(proposal.arguments),
            )
            attempts.append(attempt)
            return attempt

    def mark_executed(self, issue_id: str, strategy_id: str) -> None:
        with self._lock:
            attempt = self._find(issue_id, strategy_id)
            if attempt.state not in {"proposed", "executing"}:
                raise ValueError(f"cannot execute recovery in state {attempt.state}")
            attempt.state = "executing"

    def mark_result(self, issue_id: str, strategy_id: str, *, succeeded: bool) -> None:
        with self._lock:
            attempt = self._find(issue_id, strategy_id)
            if attempt.state != "executing":
                raise ValueError("only an executed recovery can receive an outcome")
            attempt.state = "succeeded" if succeeded else "failed"
            attempt.succeeded = bool(succeeded)

    def failed_count(self, issue_id: str) -> int:
        with self._lock:
            return sum(
                1 for attempt in self._attempts.get(issue_id, ()) if attempt.succeeded is False
            )

    def exhausted(self, issue_id: str) -> bool:
        return self.failed_count(issue_id) >= self.max_distinct_failures

    def strategy_fingerprints(self, issue_id: str) -> frozenset[str]:
        with self._lock:
            return frozenset(
                attempt.fingerprint for attempt in self._attempts.get(issue_id, ())
            )

    def snapshot(self, issue_id: str | None = None) -> list[dict[str, Any]]:
        with self._lock:
            if issue_id is None:
                attempts = [
                    attempt
                    for issue_attempts in self._attempts.values()
                    for attempt in issue_attempts
                ]
            else:
                attempts = list(self._attempts.get(issue_id, ()))
            return [asdict(attempt) for attempt in attempts]

    def _find(self, issue_id: str, strategy_id: str) -> RecoveryAttempt:
        for attempt in self._attempts.get(issue_id, ()):
            if attempt.strategy_id == strategy_id:
                return attempt
        raise KeyError(f"unknown recovery strategy {issue_id}/{strategy_id}")


class RecoveryMacroCompiler:
    """Validate an LLM proposal then delegate to a trusted macro builder."""

    def __init__(
        self,
        builders: Mapping[RecoveryKind, Callable[[RecoveryProposal], Any]],
    ) -> None:
        self._builders = dict(builders)

    def compile(self, proposal: RecoveryProposal) -> Any:
        validate_recovery_macro(proposal)
        builder = self._builders.get(proposal.kind)
        if builder is None:
            raise SchemaError(f"no trusted builder registered for {proposal.kind.value}")
        return builder(proposal)


def validate_recovery_macro(proposal: RecoveryProposal) -> None:
    args = proposal.arguments
    allowed = _ALLOWED_ARGUMENTS[proposal.kind]
    unknown = set(args) - allowed
    missing = allowed - set(args)
    if unknown:
        raise SchemaError(f"unsupported {proposal.kind.value} arguments: {sorted(unknown)}")
    if missing:
        raise SchemaError(f"missing {proposal.kind.value} arguments: {sorted(missing)}")

    if proposal.kind is RecoveryKind.SCAN_VIEWS:
        _angles(args["angles"], "angles")
        _nonempty_text(args["perception_action"], "perception_action")
    elif proposal.kind is RecoveryKind.REACQUIRE_OBJECT:
        _nonempty_text(args["target"], "target")
        _angles(args["viewpoints"], "viewpoints")
    elif proposal.kind is RecoveryKind.RETRY_NAVIGATION:
        _nonempty_text(args["target_location"], "target_location")
        offset = _number(args["approach_offset_m"], "approach_offset_m")
        if offset < 0 or offset > 1.5:
            raise SchemaError("approach_offset_m must be between 0 and 1.5")
        if args["attempts"] != 1:
            raise SchemaError("retry_navigation attempts must equal 1")
    elif proposal.kind is RecoveryKind.RELOCALIZE:
        if args["method"] not in {
            "spin_in_place", "clear_costmaps", "return_to_last_pose"
        }:
            raise SchemaError("unsupported relocalization method")
    elif proposal.kind is RecoveryKind.ASK_HUMAN:
        prompt = _nonempty_text(args["prompt"], "prompt")
        if len(prompt) > 500:
            raise SchemaError("ask_human prompt is too long")
        if not isinstance(args["confirmation_required"], bool):
            raise SchemaError("confirmation_required must be boolean")


def recovery_fingerprint(proposal: RecoveryProposal) -> str:
    canonical = json.dumps(
        {"kind": proposal.kind.value, "arguments": proposal.arguments},
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def validate_global_decision(
    decision: GlobalPlanDecision,
    *,
    completed_steps: int,
    original_instruction: str,
    known_actions,
    known_locations=None,
) -> None:
    if decision.preserved_completed_steps != completed_steps:
        raise SchemaError(
            "global decision must preserve exactly the completed step count "
            f"({completed_steps})"
        )
    if decision.action is GlobalAction.ABORT_AND_REPORT:
        if decision.replacement_plan:
            raise SchemaError("abort_and_report cannot contain a replacement plan")
        return
    if not decision.replacement_plan:
        raise SchemaError(f"{decision.action.value} requires a replacement plan")
    from behavior_tree.GPSR.planner_validators import validate_plan

    valid, error = validate_plan(
        list(decision.replacement_plan),
        original_instruction,
        known_actions,
        known_locations=known_locations,
    )
    if not valid:
        raise SchemaError(f"invalid replacement GPSR plan: {error}")
    if decision.action is GlobalAction.RELAX_GOAL and not decision.relaxed_constraints:
        raise SchemaError("relax_goal must list at least one relaxed constraint")


def _angles(value: Any, label: str) -> None:
    if not isinstance(value, list) or not value or len(value) > 8:
        raise SchemaError(f"{label} must contain 1 to 8 pan/tilt pairs")
    for pair in value:
        if not isinstance(pair, list) or len(pair) != 2:
            raise SchemaError(f"{label} entries must be [pan_deg, tilt_deg]")
        pan = _number(pair[0], "pan_deg")
        tilt = _number(pair[1], "tilt_deg")
        if pan < -180 or pan > 180 or tilt < -90 or tilt > 90:
            raise SchemaError("pan/tilt angle is outside the safe schema bounds")


def _number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise SchemaError(f"{label} must be numeric")
    return float(value)


def _nonempty_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise SchemaError(f"{label} must be a non-empty string")
    return value


__all__ = [
    "RecoveryAttempt",
    "RecoveryLedger",
    "RecoveryMacroCompiler",
    "recovery_fingerprint",
    "validate_global_decision",
    "validate_recovery_macro",
]
