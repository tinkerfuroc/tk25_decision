import os
os.environ.setdefault("BT_MOCK_MODE", "true")

import time

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier0 import run_tier0

KNOWN_ACTIONS = {"goto", "find_person", "announce", "count", "find_object", "guide"}
KNOWN_LOCATIONS = {"kitchen_table", "sofa", "kitchen"}


class FakePlanner:
    def __init__(self, plans_by_command, raise_for=(), never_ready=()):
        self.plans = plans_by_command
        self.raise_for = set(raise_for)
        self.never_ready = set(never_ready)
        self._slots = {}
        self._slot_context = {}

    def reset(self):
        self._slots.clear()
        self._slot_context.clear()

    def split_command(self, command, **kwargs):
        # M-4 (round-3 fix review): tier0.plan_one now passes slot=<idx> --
        # accept and ignore it, like production's split_command(slot=None).
        if command in self.raise_for:
            raise RuntimeError("llm down")
        return [{"id": f"t{i}", "desc": command} for i in range(len(self.plans[command]))]

    def request_plan_all(self, slot, targets, command=None):
        self._slots[slot] = command
        # Mirrors GPSRPlanner.request_plan_all storing the slot's target list (planner.py:912)
        # so _flatten_prior_plans (planner.py:397, ported into tier0.judge) has something to read.
        self._slot_context[slot] = {"command": command, "targets": targets}

    def _get_slot_context(self, slot):
        return self._slot_context.get(slot, {})

    def all_targets_ready(self, slot, n):
        return self._slots[slot] not in self.never_ready

    def get_action_plan(self, slot, i):
        return self.plans[self._slots[slot]][i]


def _entry(i, text, template="goToLoc"):
    return CorpusEntry(id=f"c{i}", seed=1, template=template, followups=(), category="people",
                       text=text, feasibility="A")


def test_tier0_verdicts():
    plans = {
        "go to the sofa then find a person": [[{"action": "goto", "params": {"location": "sofa"}},
                                               {"action": "find_person", "params": {}}]],
        "empty one": [[]],
        "bad action": [[{"action": "teleport", "params": {}}]],
        "slow one": [[{"action": "announce", "params": {"text": "hi"}}]],
    }
    planner = FakePlanner(plans, raise_for={"crash"}, never_ready={"slow one"})
    entries = [_entry(0, "go to the sofa then find a person"), _entry(1, "empty one"),
               _entry(2, "bad action"), _entry(3, "slow one"), _entry(4, "crash")]
    plans["crash"] = [[]]
    results = run_tier0(entries, planner, known_actions=KNOWN_ACTIONS,
                        known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    verdicts = {r.entry_id: r.verdict for r in results}
    assert verdicts == {"c0": "PASS", "c1": "FAIL", "c2": "FAIL", "c3": "TIMEOUT", "c4": "ERROR"}
    assert results[0].plan == ["goto", "find_person"]
    assert "empty" in results[1].detail
    assert "llm down" in results[4].detail
    assert all(r.tier == 0 for r in results)


def test_tier0_builds_a_fresh_planner_per_entry_via_factory():
    plans = {"go to the sofa": [[{"action": "goto", "params": {"location": "sofa"}}]]}
    calls = []

    def factory():
        calls.append(1)
        return FakePlanner(plans)

    entries = [_entry(i, "go to the sofa") for i in range(3)]
    results = run_tier0(entries, None, known_actions=KNOWN_ACTIONS,
                        known_locations=KNOWN_LOCATIONS, timeout_s=0.2, planner_factory=factory)
    assert len(calls) == 3
    assert all(r.verdict == "PASS" for r in results)


def test_tier0_judge_seeds_prior_plan_across_targets_like_production():
    """F1: judge must validate each target against ITS OWN desc with the earlier targets'
    flattened intent as prior_plan (mirrors planner.py:836-841), not the whole command with
    no prior_plan at all -- otherwise a legitimate guide() after an earlier find_person() is
    rejected across the target boundary."""
    two_targets = [
        {"id": "t0", "desc": "find the person in the kitchen", "depends_on": []},
        {"id": "t1", "desc": "guide them to the sofa", "depends_on": ["t0"]},
    ]
    command = "find the person in the kitchen then guide them to the sofa"
    two_target_plans = {command: [[{"action": "find_person", "params": {}}],
                                  [{"action": "guide", "params": {}}]]}

    class TwoTargetPlanner(FakePlanner):
        def split_command(self, command, **kwargs):
            return two_targets

    results = run_tier0([_entry(0, command)], TwoTargetPlanner(two_target_plans),
                        known_actions=KNOWN_ACTIONS, known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    assert results[0].verdict == "PASS"

    guide_alone = "guide them to the sofa"
    guide_alone_plans = {guide_alone: [[{"action": "guide", "params": {}}]]}
    results_alone = run_tier0([_entry(1, guide_alone)], FakePlanner(guide_alone_plans),
                              known_actions=KNOWN_ACTIONS, known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    assert results_alone[0].verdict == "FAIL"
    assert "guide" in results_alone[0].detail


def test_tier0_scores_a_fallback_plan_as_fail():
    """F3: a target whose planner attempts were all exhausted stores a guaranteed-valid
    fallback acknowledgement plan (orchestrator.py:606 _fallback_plan) alongside the error
    that caused it -- that plan passes validate_plan, but it is not a real answer."""
    command = "impossible command"
    fallback_text = ("I heard your command but could not work out a complete plan for it. "
                     "I will skip it for now.")
    plans = {command: [[{"action": "announce", "params": {"text": fallback_text}}]]}

    class FallbackPlanner(FakePlanner):
        def request_plan_all(self, slot, targets, command=None):
            super().request_plan_all(slot, targets, command=command)
            self._cache = {(slot, 0): {"error": "all 4 attempts failed (last reason: invalid json)"}}

    results = run_tier0([_entry(0, command)], FallbackPlanner(plans), known_actions=KNOWN_ACTIONS,
                        known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    assert results[0].verdict == "FAIL"
    assert "planner exhausted attempts" in results[0].detail


class HangingSplitPlanner:
    """A planner whose split_command never returns, like a stuck network call."""

    def reset(self):
        pass

    def split_command(self, command, **kwargs):
        time.sleep(60)
        raise AssertionError("should never get here in a bounded test")


def test_tier0_passes_the_entrys_slot_to_split_command():
    # M-4 (round-3 fix review): tier0's own consumer of split.accepted
    # telemetry/contract logs needs the slot to attribute a multi-entry run
    # correctly -- plan_one must pass slot=<idx> (production's
    # BtNode_SplitCommand does the same).
    plans = {"go to the sofa": [[{"action": "goto", "params": {"location": "sofa"}}]]}
    captured_slots = []

    class SlotCapturingPlanner(FakePlanner):
        def split_command(self, command, **kwargs):
            captured_slots.append(kwargs.get("slot"))
            return super().split_command(command, **kwargs)

    entries = [_entry(i, "go to the sofa") for i in range(3)]
    run_tier0(entries, SlotCapturingPlanner(plans), known_actions=KNOWN_ACTIONS,
             known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    assert captured_slots == [0, 1, 2]


def test_tier0_bounds_a_stuck_split_command():
    started = time.monotonic()
    entries = [_entry(0, "go to the sofa")]
    results = run_tier0(entries, HangingSplitPlanner(), known_actions=KNOWN_ACTIONS,
                        known_locations=KNOWN_LOCATIONS, timeout_s=0.2)
    elapsed = time.monotonic() - started
    assert results[0].verdict == "TIMEOUT"
    assert "split_command" in results[0].detail
    assert elapsed < 5.0
