import os
os.environ.setdefault("BT_MOCK_MODE", "true")

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.tier0 import run_tier0

KNOWN_ACTIONS = {"goto", "find_person", "announce", "count", "find_object"}
KNOWN_LOCATIONS = {"kitchen_table", "sofa", "kitchen"}


class FakePlanner:
    def __init__(self, plans_by_command, raise_for=(), never_ready=()):
        self.plans = plans_by_command
        self.raise_for = set(raise_for)
        self.never_ready = set(never_ready)
        self._slots = {}

    def reset(self):
        self._slots.clear()

    def split_command(self, command):
        if command in self.raise_for:
            raise RuntimeError("llm down")
        return [{"id": f"t{i}", "desc": command} for i in range(len(self.plans[command]))]

    def request_plan_all(self, slot, targets, command=None):
        self._slots[slot] = command

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
