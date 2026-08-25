"""Tier 0: run corpus commands through the two-layer planner only (no ROS, no execution)."""
from __future__ import annotations

import threading
import time
from typing import Any, Callable, Iterable, Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.report import BenchResult
# _flatten_prior_plans is production's own cross-target validation context builder
# (planner.py:397); ported here (not re-implemented) so tier 0 mirrors exactly what
# plan_target validates against at planner.py:836-841.
from behavior_tree.GPSR.planner import _flatten_prior_plans
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
    # Check readiness once before the wait loop: if split_command already consumed the
    # whole budget, `remaining` is 0 and a `while time.monotonic() < deadline` loop would
    # exit immediately without ever polling, reporting TIMEOUT even when the plans are
    # already ready (e.g. an offline/mock planner that resolves synchronously).
    if not planner.all_targets_ready(slot, len(targets)):
        while time.monotonic() < deadline:
            if planner.all_targets_ready(slot, len(targets)):
                break
            time.sleep(0.05)
        else:
            raise TimeoutError(f"planner not ready after {timeout_s:.0f}s")
    plans = [planner.get_action_plan(slot, i) for i in range(len(targets))]
    return targets, plans


def _target_error(planner, slot: int, index: int):
    """Read the per-target planner error recorded on total-attempt exhaustion.

    ``GPSRPlanner`` has no public accessor for this (only ``_cache[(slot, index)]["error"]``,
    set at planner.py:877 alongside the ``_fallback_plan`` it stores) so this reads the private
    cache directly, tolerating planners (fakes) that don't have one at all.
    """
    cache = getattr(planner, "_cache", None)
    if not cache:
        return None
    entry = cache.get((slot, index))
    return entry.get("error") if entry else None


def judge(entry: CorpusEntry, targets, plans, planner, slot: int, *, known_actions,
          known_locations, seconds) -> BenchResult:
    actions = [str(step.get("action")) for plan in plans for step in plan]
    base = dict(entry_id=entry.id, template=entry.template, feasibility=entry.feasibility,
                tier=0, seconds=seconds, plan=actions)
    if not targets:
        return BenchResult(verdict="FAIL", detail="split produced no targets", **base)
    for i, plan in enumerate(plans):
        if not plan:
            return BenchResult(verdict="FAIL", detail=f"empty plan for target {i}", **base)
        target = targets[i] if i < len(targets) else {}
        desc = target.get("desc") if isinstance(target, dict) else str(target)
        # Mirror planner.py:836-841 exactly: validate THIS target's own description (not
        # the whole command) with the earlier targets' flattened intent seeded as prior_plan,
        # so a guide() after an earlier find_person() is not re-rejected across a target
        # boundary (planner_validators.validate_plan's own contract for prior_plan).
        prior_plan = _flatten_prior_plans(planner, slot, i)
        ok, message = validate_plan(plan, desc or "", known_actions,
                                    known_locations=known_locations, prior_plan=prior_plan)
        if not ok:
            return BenchResult(verdict="FAIL", detail=f"target {i}: {message}", **base)
    # A target whose planner attempts were all exhausted stores a guaranteed-valid
    # fallback acknowledgement plan (orchestrator.py:606 _fallback_plan) alongside the
    # error that caused it (planner.py:872-877) -- that plan passes validate_plan above
    # but is not a real answer, so score it FAIL using the recorded reason.
    for i in range(len(targets)):
        err = _target_error(planner, slot, i)
        if err:
            return BenchResult(verdict="FAIL", detail=f"planner exhausted attempts: {err}", **base)
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
        results.append(judge(entry, targets, plans, active_planner, slot, known_actions=known_actions,
                             known_locations=known_locations, seconds=time.monotonic() - started))
    return results
