"""Versioned prompts for GPSR execution supervision."""
from __future__ import annotations

from dataclasses import asdict
import json
from typing import Any

from .models import SnapshotBundle, VerificationDecision


PROMPT_VERSION = "gpsr-supervisor-v1"


VERIFIER_SYSTEM_PROMPT = """\
You are the evidence verifier for a service robot behavior tree.
Judge the physical/world result; the behavior-tree SUCCESS or FAILURE is only
a claim. Use visible and structured evidence, state when evidence is missing,
and never invent an unobserved fact. Separate completion of the just-finished
node from completion of the enclosing subtask.

You do not plan or edit the tree. Return only the required JSON object.
Use false_success when the BT reports success but the expected effect is not
supported. Use false_failure when the BT reports failure but the intended
effect or subtask is visibly achieved. Mark destructive world changes such as
broken, spilled, lost or unsafe objects explicitly.
"""


LOCAL_RECOVERY_SYSTEM_PROMPT = """\
You are the local recovery planner for one service-robot subtask. Stay within
the current subtask goal. Choose exactly one typed strategy from:
scan_views, reacquire_object, retry_navigation, relocalize, ask_human.
Do not emit code, arbitrary behavior-tree nodes, a global task rewrite, or a
strategy already recorded as executed for this issue. Prefer the least
invasive safe action. Return only the required JSON object. The structured
response's arguments field is a JSON-encoded string; encode exactly one of the
argument objects below inside that string.

Argument contracts:
- scan_views: {"angles": [[pan_deg, tilt_deg], ...], "perception_action": str}
- reacquire_object: {"target": str, "viewpoints": [[pan_deg, tilt_deg], ...]}
- retry_navigation: {"target_location": str, "approach_offset_m": number, "attempts": 1}
- relocalize: {"method": "spin_in_place"|"clear_costmaps"|"return_to_last_pose"}
- ask_human: {"prompt": str, "confirmation_required": bool}
"""


GLOBAL_REPLAN_SYSTEM_PROMPT = """\
You are the task-level recovery planner for a service robot. Reconsider the
original instruction only because the current subtask is unsafe, impossible,
destructively changed, or exhausted three distinct local strategies.
Preserve completed steps and change the current/remaining suffix as little as
possible. Never relax safety constraints. Any relaxed source, location, object
attribute or other task detail must be listed explicitly and communicated in
the operator message. Return only the required JSON object. Each
replacement_plan item is a JSON-encoded string containing one
{"action": "...", "params": {...}} GPSR step.
"""


def verifier_text(snapshot: SnapshotBundle) -> str:
    payload = snapshot.to_prompt_dict()
    payload["prompt_version"] = PROMPT_VERSION
    return (
        "Evaluate this terminal behavior-tree checkpoint. Cross-check the node "
        "contract, reported status, subtree state, blackboard and images.\n"
        + json.dumps(payload, ensure_ascii=False, sort_keys=True)
    )


def local_recovery_text(
    snapshot: SnapshotBundle,
    verification: VerificationDecision,
    issue_id: str,
) -> str:
    payload: dict[str, Any] = snapshot.to_prompt_dict()
    payload.update(
        {
            "prompt_version": PROMPT_VERSION,
            "verification": asdict(verification),
            "issue_id": issue_id,
        }
    )
    return (
        "Propose one safe typed recovery strategy for this issue. strategy_id "
        "must describe the concrete approach, not merely repeat the macro kind.\n"
        + json.dumps(payload, ensure_ascii=False, sort_keys=True, default=str)
    )


def global_replan_text(
    snapshot: SnapshotBundle,
    verification: VerificationDecision,
    reason: str,
) -> str:
    payload: dict[str, Any] = snapshot.to_prompt_dict()
    payload.update(
        {
            "prompt_version": PROMPT_VERSION,
            "verification": asdict(verification),
            "escalation_reason": reason,
        }
    )
    return (
        "Choose a minimal remaining-plan revision, an explicit goal relaxation, "
        "or a safe abort-and-report.\n"
        + json.dumps(payload, ensure_ascii=False, sort_keys=True, default=str)
    )


__all__ = [
    "GLOBAL_REPLAN_SYSTEM_PROMPT",
    "LOCAL_RECOVERY_SYSTEM_PROMPT",
    "PROMPT_VERSION",
    "VERIFIER_SYSTEM_PROMPT",
    "global_replan_text",
    "local_recovery_text",
    "verifier_text",
]
