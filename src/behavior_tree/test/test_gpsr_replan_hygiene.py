from __future__ import annotations

import py_trees
from py_trees.common import Status

from behavior_tree.GPSR.orchestrator import DynamicExecutor, _clean_plan
from behavior_tree.GPSR import planner as planner_mod


def test_clean_plan_drops_consecutive_duplicate_steps():
    raw = [
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "place", "params": {"location": "kitchen_table"}},
        {"action": "place", "params": {"location": "kitchen_table"}},
        {"action": "announce", "params": {"text": "done"}},
    ]
    cleaned, dropped = _clean_plan(raw)
    assert [s["action"] for s in cleaned] == ["goto", "place", "announce"]
    assert dropped == ["duplicate:place"]


def test_clean_plan_keeps_non_consecutive_repeats():
    raw = [
        {"action": "goto", "params": {"location": "a"}},
        {"action": "announce", "params": {"text": "x"}},
        {"action": "goto", "params": {"location": "a"}},
    ]
    cleaned, _ = _clean_plan(raw)
    assert len(cleaned) == 3


def test_canonical_plan_ignores_param_order():
    a = [{"action": "deliver", "params": {"object": "coke", "recipient": "Susan"}}]
    b = [{"action": "deliver", "params": {"recipient": "Susan", "object": "coke"}}]
    assert planner_mod._canonical_plan(a) == planner_mod._canonical_plan(b)


class _StubClient:  # never used; plan_target is driven via monkeypatched _call_llm
    pass


def test_plan_target_rejects_plan_identical_to_failed_one(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=3)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "put the coke on the kitchen table",
                          "targets": [{"id": "t0", "desc": "place the coke on the kitchen table",
                                       "object": "coke", "location": "kitchen_table", "depends_on": []}]}
    same = {"plan": [{"action": "place", "params": {"location": "kitchen_table"}}]}
    different = {"plan": [{"action": "goto", "params": {"location": "kitchen_table"}},
                          {"action": "announce", "params": {"text": "here"}}]}
    responses = iter([same, same, different])
    reasons = []

    def fake_call(client, system, user, temperature):
        reasons.append(user)
        return next(responses), None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))

    # Seed the cache as if `same` had been executed and failed.
    p._store(0, 0, "place the coke on the kitchen table", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    assert p._failed_plan(0, 0) == same["plan"]

    p.plan_target(0, 0, "place the coke on the kitchen table", command="put the coke on the kitchen table",
                  target_obj="coke", target_loc="kitchen_table", failure_reason="precondition unmet: at_robot(kitchen_table) (invalid)")
    assert p.get_action_plan(0, 0) == different["plan"]
    assert any("IDENTICAL" in r for r in reasons[1:])


def test_plan_target_accepts_identical_plan_on_final_attempt(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    same = {"plan": [{"action": "announce", "params": {"text": "x"}}]}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (same, None))
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="x")
    assert p.get_action_plan(0, 0) == same["plan"]  # honest hand-back, not the fallback


def test_executor_passes_leaf_feedback_as_replan_reason():
    leaf = py_trees.behaviours.Failure("gate")
    leaf.feedback_message = "precondition unmet: at_robot(kitchen_table) (invalid)"
    seq = py_trees.composites.Sequence("target", memory=True)
    seq.add_child(leaf)
    for _ in seq.tick():
        pass
    assert DynamicExecutor._last_child_feedback(seq) == leaf.feedback_message
