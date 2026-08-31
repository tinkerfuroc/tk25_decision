"""Fold a GPSR telemetry events.jsonl into per-task results."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

_SLOT_RE = re.compile(r"/task-(\d+)$")
_EXECUTOR_TASK_RE = re.compile(r"^executor task (\d+)$")
_NODE_STATUS_TO_TASK_STATUS = {"SUCCESS": "succeeded", "FAILURE": "failed"}
# Node ``feedback`` strings that carry real diagnostic evidence (captured live off a real
# t1-42 run, e.g. "precondition unmet: at_robot(laundry_desk) (INVALID)" on a leaf several
# levels under its "executor task N" ancestor) -- vs. routine noise like a "plan-file emit
# failed (ignored): ..." SUCCESS feedback that should not overwrite it.
#
# H3 (round-2 review, run 004): also recognises the H1/E2 escape-ladder's UNRECOVERABLE_
# ERROR_PREFIX/IDENTICAL_PLAN_ERROR_PREFIX markers (action_contracts.py -- "unrecoverable: no
# untried establisher for [...]" / "identical to failed plan: ...") and search_object's own
# "swept N of M spots" sweep-exhausted feedback -- without these, a genuine one of these
# diagnostics never overwrites diag_by_slot, so a node-derived failure's reason falls back to
# the bare, uninformative "executor node FAILURE" (e.g. run 004's detail).
_DIAG_RE = re.compile(
    r"(precondition unmet|postcondition unmet|error|unrecoverable|"
    r"swept \d+ of \d+ spots|identical to failed plan)",
    re.IGNORECASE,
)
# N4 (round-5 rerun fix, bench log flood): the K-round fail-fast guard's
# routine per-tick feedback -- "gpsr/mission_unrecoverable is not truthy" --
# contains the substring "unrecoverable" (from the blackboard KEY's own
# name), so _DIAG_RE above matches it too. That guard fires on EVERY tick
# of EVERY goto (almost always FAILURE -- the normal, expected case) and
# `diag_by_slot` keeps only the LAST match seen, so this routine noise
# (6805x in sim run 016's logs) silently overwrote whatever real diagnostic
# preceded it, and bench's run.json then reported the guard line as the
# failure detail instead of the true reason. Exclude this shape regardless
# of whether the guard node itself was also quieted (small_trees.py's
# BtNode_CheckBBTrue quiet_on_falsy) -- old logs captured before that fix
# still carry the noisy line verbatim.
_DIAG_EXCLUDE_RE = re.compile(r"is not truthy", re.IGNORECASE)


@dataclass
class TaskResult:
    slot: int
    status: str | None = None
    reason: str | None = None
    steps: list[tuple[str, str]] = field(default_factory=list)
    # K3 (task-K, live-manipulation sim findings, F2): the (action, method)
    # pairs for every step.finished that carried a "method" claim (e.g.
    # grasp's "autonomous" / "referee_fallback") -- separate from ``steps``
    # (kept exactly as-is: other consumers index it as (action, outcome)
    # pairs) so a referee-assisted step can be told apart from an autonomous
    # one without changing that shape.
    step_methods: list[tuple[str, str]] = field(default_factory=list)
    planner_errors: int = 0
    first_seen: str | None = None
    finished_at: str | None = None
    # I6 (round-3 adversarial review): the last INVALID/UNKNOWN `gate.verified`
    # reason per target_index -- a gate's rationale used to exist only in the
    # executor's transient feedback_message; this lets tier2 diagnostics show
    # WHY a gate failed (e.g. "postcondition:counted(drinks) INVALID (count
    # artifact target provenance mismatch)") instead of a bare status.
    gate_reasons: dict[int, str] = field(default_factory=dict)
    # L-4-M1 (round-3 fix2 review): the fact each `gate_reasons` entry was
    # recorded for, keyed the same way -- since J3 a postcondition check
    # verifies EVERY fact (one `gate.verified` per fact, "no more
    # fail-fast"), so a target with sources [answered(x) INVALID,
    # counted(y) VALID] must not have the INVALID reason popped just
    # because an UNRELATED fact on the same target went VALID in the same
    # tick. Not part of the public dict[int, str] shape (compare=False, so
    # it never affects TaskResult equality) -- purely an internal aid for
    # the pop-on-matching-fact rule below.
    _gate_reason_fact: dict[int, str] = field(default_factory=dict, repr=False, compare=False)
    # J14 (round-3 adversarial review, tier0 #7): the accepted split's full
    # target contracts (id/desc/pre/postconditions/depends_on/object/
    # location), from `split.accepted` -- lets tier0/tier2 reports show a
    # run's contracts, not just descriptions, for auditing split mistakes.
    split_targets: list[dict] | None = None


def slot_of(task_id: str | None) -> int | None:
    if not task_id:
        return None
    match = _SLOT_RE.search(task_id)
    return int(match.group(1)) if match else None


def _slot_for_node(node_id: str | None, executor_slot_by_id: dict[str, int]) -> int | None:
    """Which slot a (possibly nested) executor node belongs to.

    ``executor_slot_by_id`` only maps the TOP-level "executor task N" node ids (from
    ``tree.generated``); a real failure's diagnostic lives on a leaf several levels under
    that ancestor (e.g. "executor/root/7/2/0/13/0/0" under "executor/root/7/2/0/13"), so
    match by id-prefix, not exact id.
    """
    if not node_id:
        return None
    for eid, slot in executor_slot_by_id.items():
        if node_id == eid or node_id.startswith(eid + "/"):
            return slot
    return None


def parse_events(path: Path) -> dict[int, TaskResult]:
    """Fold telemetry into per-slot results.

    The production two-layer orchestrator flow (``_create_execute_slot_new``) never emits
    ``task.finished``/``step.finished`` — only the legacy flow does — so tier-1 groups run
    through it would otherwise never see a terminal status and burn their full timeout even
    when every task actually succeeded. As a fallback, also derive one from the "executor"
    tree's per-task ``DynamicExecutor`` node status:

    - a ``tree.generated`` event (``payload.kind == "executor"``) carries each node's ``name``
      (e.g. "executor task N", 1-based, same numbering as ``task_id``'s ``/task-N`` suffix)
      alongside its ``id``, once per (re)generation of that tree;
    - subsequent ``tree.node_states_changed`` events (``payload.tree_kind == "executor"``) only
      carry ``id``/``status`` per changed node, not ``name`` — so the id -> "executor task N"
      mapping from the most recent matching ``tree.generated`` is needed to interpret them.

    A terminal status is derived from SUCCESS/FAILURE there; RUNNING/INVALID are transient and
    ignored. A genuine ``task.finished`` event always wins over a node-derived status, in
    either order.

    When falling back to a node-derived status, ``TaskResult.reason`` is more than
    "executor node SUCCESS/FAILURE": any node under a task's subtree (not just the
    top-level "executor task N" node itself) whose ``feedback`` names a precondition/
    postcondition/error diagnostic (e.g. "precondition unmet: at_robot(laundry_desk)
    (INVALID)", captured off a real t1-42 run) is remembered per slot, and the LAST one
    seen replaces the bare status string -- real evidence instead of "it failed".
    """
    results: dict[int, TaskResult] = {}
    finalized: set[int] = set()
    executor_slot_by_id: dict[str, int] = {}
    diag_by_slot: dict[int, str] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue  # writer may be mid-line when we read
        kind = event.get("event_type")
        payload = event.get("payload") or {}

        if kind == "tree.generated" and payload.get("kind") == "executor":
            executor_slot_by_id = {}
            for node in payload.get("nodes", []):
                match = _EXECUTOR_TASK_RE.match(str(node.get("name") or ""))
                node_id = node.get("id") or node.get("node_id")
                if match and node_id:
                    executor_slot_by_id[node_id] = int(match.group(1))
            continue

        if kind == "tree.node_states_changed" and payload.get("tree_kind") == "executor":
            for node in payload.get("nodes", []):
                node_id = node.get("id") or node.get("node_id")
                feedback = node.get("feedback")
                if (
                    isinstance(feedback, str)
                    and feedback
                    and not _DIAG_EXCLUDE_RE.search(feedback)
                    and _DIAG_RE.search(feedback)
                ):
                    diag_slot = _slot_for_node(node_id, executor_slot_by_id)
                    if diag_slot is not None:
                        diag_by_slot[diag_slot] = feedback
                slot = executor_slot_by_id.get(node_id)
                if slot is None:
                    continue
                result = results.setdefault(slot, TaskResult(slot=slot))
                if result.first_seen is None:
                    result.first_seen = event.get("occurred_at")
                if slot in finalized:
                    continue
                task_status = _NODE_STATUS_TO_TASK_STATUS.get(node.get("status"))
                if task_status is None:  # RUNNING / INVALID — not terminal yet
                    continue
                result.status = task_status
                result.reason = diag_by_slot.get(slot) or f"executor node {node.get('status')}"
                result.finished_at = event.get("occurred_at")
            continue

        slot = slot_of(event.get("task_id"))
        if slot is None:
            continue
        result = results.setdefault(slot, TaskResult(slot=slot))
        if result.first_seen is None:
            result.first_seen = event.get("occurred_at")
        if kind == "step.finished":
            result.steps.append((str(payload.get("action")), str(payload.get("outcome"))))
            method = payload.get("method")
            if method:
                result.step_methods.append((str(payload.get("action")), str(method)))
        elif kind == "split.accepted":
            targets = payload.get("targets")
            if isinstance(targets, list):
                result.split_targets = targets
        elif kind == "planner.error":
            result.planner_errors += 1
        elif kind == "gate.verified":
            verdict = payload.get("verdict")
            fact = payload.get("fact")
            target_index = payload.get("target_index")
            if target_index is not None:
                try:
                    target_index = int(target_index)
                except (TypeError, ValueError):
                    target_index = None
            if target_index is not None:
                if verdict in ("INVALID", "UNKNOWN"):
                    result.gate_reasons[target_index] = (
                        f"{payload.get('phase')}:{fact} {verdict} "
                        f"({payload.get('reason')})"
                    )
                    result._gate_reason_fact[target_index] = fact
                elif verdict == "VALID":
                    # L-4 (round-3 fix review): a target that recovered on a
                    # later retry must not still show a stale failure reason
                    # in diagnostics -- pop it now that this target is
                    # VALID. L-4-M1 (round-3 fix2 review): since J3 verifies
                    # EVERY fact per gate tick ("no more fail-fast"), only
                    # pop when THIS VALID is for the SAME fact the stored
                    # reason was recorded for -- an unrelated fact's VALID
                    # in the same tick must not discard a still-standing
                    # INVALID for a different fact.
                    if result._gate_reason_fact.get(target_index) == fact:
                        result.gate_reasons.pop(target_index, None)
                        result._gate_reason_fact.pop(target_index, None)
        elif kind == "task.finished":
            result.status = payload.get("status")
            result.reason = payload.get("reason")
            result.finished_at = event.get("occurred_at")
            finalized.add(slot)
    return results
