"""Fold a GPSR telemetry events.jsonl into per-task results."""
from __future__ import annotations

import json
import re
from dataclasses import dataclass, field
from pathlib import Path

_SLOT_RE = re.compile(r"/task-(\d+)$")


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
    results: dict[int, TaskResult] = {}
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            event = json.loads(line)
        except json.JSONDecodeError:
            continue  # writer may be mid-line when we read
        slot = slot_of(event.get("task_id"))
        if slot is None:
            continue
        result = results.setdefault(slot, TaskResult(slot=slot))
        if result.first_seen is None:
            result.first_seen = event.get("occurred_at")
        payload = event.get("payload") or {}
        kind = event.get("event_type")
        if kind == "step.finished":
            result.steps.append((str(payload.get("action")), str(payload.get("outcome"))))
        elif kind == "planner.error":
            result.planner_errors += 1
        elif kind == "task.finished":
            result.status = payload.get("status")
            result.reason = payload.get("reason")
            result.finished_at = event.get("occurred_at")
    return results
