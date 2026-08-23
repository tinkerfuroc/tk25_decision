"""Fold a GPSR telemetry events.jsonl into per-task results."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

_SLOT_RE = re.compile(r"/task-(\d+)$")
_EXECUTOR_TASK_RE = re.compile(r"^executor task (\d+)$")
_NODE_STATUS_TO_TASK_STATUS = {"SUCCESS": "succeeded", "FAILURE": "failed"}


@dataclass
class TaskResult:
    slot: int
    status: str | None = None
    reason: str | None = None
    steps: list[tuple[str, str]] = field(default_factory=list)
    planner_errors: int = 0
    first_seen: str | None = None
    finished_at: str | None = None


def slot_of(task_id: str | None) -> int | None:
    if not task_id:
        return None
    match = _SLOT_RE.search(task_id)
    return int(match.group(1)) if match else None


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
    """
    results: dict[int, TaskResult] = {}
    finalized: set[int] = set()
    executor_slot_by_id: dict[str, int] = {}
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
                result.reason = f"executor node {node.get('status')}"
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
        elif kind == "planner.error":
            result.planner_errors += 1
        elif kind == "task.finished":
            result.status = payload.get("status")
            result.reason = payload.get("reason")
            result.finished_at = event.get("occurred_at")
            finalized.add(slot)
    return results
