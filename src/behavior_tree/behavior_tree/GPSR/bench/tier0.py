"""Tier 0: run corpus commands through the two-layer planner only (no ROS, no execution)."""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Iterable, Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.report import BenchResult
from behavior_tree.GPSR.planner_validators import validate_plan


def _call_with_timeout(fn: Callable[[], Any], timeout_s: float):
    """Run ``fn()`` on a daemon thread and enforce a hard wall-clock bound.

    ``GPSRPlanner.split_command`` is a synchronous LLM round-trip with no
    timeout of its own (the openai SDK's default is ~600s per attempt, up to
    ``max_attempts`` retries). Calling it directly on the bench's control
    thread means one slow/stuck request can stall the whole tier-0 run
    indefinitely. Running it here bounds the wait to ``timeout_s``; on
    timeout the worker thread is abandoned (never joined) rather than killed,
    matching the existing tolerance elsewhere in this bench for orphaned
    planner threads outliving a timed-out entry.
    """
    box: dict[str, Any] = {}

    def _target():
        try:
            box["value"] = fn()
        except Exception as exc:  # noqa: BLE001 - re-raised on the caller's thread
            box["error"] = exc

    thread = threading.Thread(target=_target, daemon=True)
    thread.start()
    thread.join(timeout_s)
    if thread.is_alive():
        raise TimeoutError(f"timed out after {timeout_s:.0f}s")
    if "error" in box:
        raise box["error"]
    return box["value"]


def plan_one(planner, slot: int, command: str, *, timeout_s: float):
    started = time.monotonic()
    try:
        targets = _call_with_timeout(lambda: planner.split_command(command), timeout_s)
    except TimeoutError:
        raise TimeoutError(f"split_command timed out after {timeout_s:.0f}s")
    remaining = max(0.0, timeout_s - (time.monotonic() - started))
    planner.request_plan_all(slot, targets, command=command)
    deadline = time.monotonic() + remaining
    while time.monotonic() < deadline:
        if planner.all_targets_ready(slot, len(targets)):
            break
        time.sleep(0.05)
    else:
        raise TimeoutError(f"planner not ready after {timeout_s:.0f}s")
    plans = [planner.get_action_plan(slot, i) for i in range(len(targets))]
    return targets, plans


def judge(entry: CorpusEntry, targets, plans, *, known_actions, known_locations, seconds) -> BenchResult:
    actions = [str(step.get("action")) for plan in plans for step in plan]
    base = dict(entry_id=entry.id, template=entry.template, feasibility=entry.feasibility,
                tier=0, seconds=seconds, plan=actions)
    if not targets:
        return BenchResult(verdict="FAIL", detail="split produced no targets", **base)
    for i, plan in enumerate(plans):
        if not plan:
            return BenchResult(verdict="FAIL", detail=f"empty plan for target {i}", **base)
        ok, message = validate_plan(plan, entry.text, known_actions, known_locations=known_locations)
        if not ok:
            return BenchResult(verdict="FAIL", detail=f"target {i}: {message}", **base)
    return BenchResult(verdict="PASS", **base)


def run_tier0(entries: Sequence[CorpusEntry], planner, *, known_actions: Iterable[str],
              known_locations: Iterable[str], timeout_s: float = 90.0,
              planner_factory: Callable[[], Any] | None = None) -> list[BenchResult]:
    known_actions = set(known_actions)
    known_locations = set(known_locations)
    results: list[BenchResult] = []
    for slot, entry in enumerate(entries):
        active_planner = planner_factory() if planner_factory is not None else planner
        active_planner.reset()
        started = time.monotonic()
        try:
            targets, plans = plan_one(active_planner, slot, entry.text, timeout_s=timeout_s)
        except TimeoutError as exc:
            results.append(BenchResult(entry.id, entry.template, entry.feasibility, 0, "TIMEOUT",
                                       str(exc), time.monotonic() - started))
            continue
        except Exception as exc:  # noqa: BLE001 - any planner failure is an ERROR verdict
            results.append(BenchResult(entry.id, entry.template, entry.feasibility, 0, "ERROR",
                                       f"{type(exc).__name__}: {exc}", time.monotonic() - started))
            continue
        results.append(judge(entry, targets, plans, known_actions=known_actions,
                             known_locations=known_locations, seconds=time.monotonic() - started))
    return results
