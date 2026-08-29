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


def test_alternatives_for_reason_lists_registry_establishers():
    text = planner_mod._alternatives_for_reason("postcondition unmet: object_seen(kitchen item) (UNKNOWN)")
    assert "find_object" in text and "search_object" in text
    assert "vlm_fallback" in text
    assert planner_mod._alternatives_for_reason("child FAILURE") == ""


def test_plan_target_marks_identical_final_attempt_with_error(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    same = {"plan": [{"action": "goto", "params": {"location": "living_room"}},
                     {"action": "find_object", "params": {"object": "kitchen item", "location": "living_room"}}]}
    prompts = []

    def fake_call(client, system, user, temperature):
        prompts.append(user)
        return same, None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="postcondition unmet: object_seen(kitchen item) (UNKNOWN)")
    assert p.is_target_ready(0, 0)
    assert p.get_action_plan(0, 0) == same["plan"]
    err = p.get_error(0, 0)
    assert err is not None and err.startswith(planner_mod.IDENTICAL_PLAN_ERROR_PREFIX)
    # the non-final rejection told the LLM what would be acceptable
    assert any("IDENTICAL" in u and "find_object" in u and "vlm_fallback" in u for u in prompts[1:])


def test_plan_target_identical_marker_does_not_nest_on_repeated_identical_replans(monkeypatch):
    # A target that keeps regenerating the same doomed plan across several
    # replan cycles gets re-invoked with failure_reason ALREADY prefixed by
    # a previous IDENTICAL_PLAN_ERROR_PREFIX marker. The composed error must
    # not stack the prefix a second time.
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

    nested_failure_reason = f"{planner_mod.IDENTICAL_PLAN_ERROR_PREFIX}: precondition unmet: at_robot(x) (invalid)"
    p.plan_target(0, 0, "d", command="c", failure_reason=nested_failure_reason)

    err = p.get_error(0, 0)
    assert err is not None
    assert err.count(planner_mod.IDENTICAL_PLAN_ERROR_PREFIX) == 1
    assert err == f"{planner_mod.IDENTICAL_PLAN_ERROR_PREFIX}: precondition unmet: at_robot(x) (invalid)"


def test_plan_target_different_plan_has_no_error(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    old = [{"action": "goto", "params": {"location": "a"}}]
    new = {"plan": [{"action": "vlm_fallback", "params": {"question": "where is it"}}]}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (new, None))
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", old, py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="x")
    assert p.get_error(0, 0) is None


def test_failed_plans_accumulate_across_invalidations():
    p = planner_mod.GPSRPlanner(max_attempts=2)
    a = [{"action": "goto", "params": {"location": "x"}}]
    b = [{"action": "vlm_fallback", "params": {"question": "q"}}]
    p._store(0, 0, "d", a, py_trees.behaviours.Success("s"), None)
    p._invalidate(0, 0)
    p._store(0, 0, "d", b, py_trees.behaviours.Success("s"), None)   # _store must carry failed_plans forward
    p._invalidate(0, 0)
    assert p._failed_plans(0, 0) == [a, b]
    assert p._failed_plan(0, 0) == b
    p._invalidate(0, 0)                                              # same plan again -> no duplicate
    assert p._failed_plans(0, 0) == [a, b]


def test_plan_target_rejects_plan_identical_to_an_earlier_failure(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=3)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    first = [{"action": "goto", "params": {"location": "living_room"}},
             {"action": "find_object", "params": {"object": "kitchen item", "location": "living_room"}}]
    second = [{"action": "vlm_fallback", "params": {"question": "q"}}]
    fresh = [{"action": "announce", "params": {"text": "cannot find it"}}]
    # history: first failed, then second failed
    p._store(0, 0, "d", first, py_trees.behaviours.Success("s"), None); p._invalidate(0, 0)
    p._store(0, 0, "d", second, py_trees.behaviours.Success("s"), None); p._invalidate(0, 0)
    responses = iter([{"plan": first}, {"plan": fresh}])
    reasons = []

    def fake_call(client, system, user, temperature):
        reasons.append(user)
        return next(responses), None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p.plan_target(0, 0, "d", command="c", failure_reason="postcondition unmet: object_seen(kitchen item) (UNKNOWN)")
    assert p.get_action_plan(0, 0) == fresh          # `first` was rejected even though the LAST failure was `second`
    assert p.get_error(0, 0) is None
    assert any("IDENTICAL" in u for u in reasons[1:])


def test_new_command_starts_with_empty_failed_plans(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=2)
    p._store(0, 0, "d", [{"action": "goto", "params": {"location": "x"}}], py_trees.behaviours.Success("s"), None)
    p._invalidate(0, 0)
    assert p._failed_plans(0, 0)
    # Drive the fresh-entry creation in request_plan_all directly (the pattern
    # used by test_gpsr_target_planner.py: monkeypatch plan_target so no
    # worker thread is spawned, then inspect the entry it just created).
    monkeypatch.setattr(p, "plan_target", lambda *a, **k: None)
    p.request_plan_all(0, ["new target"], command="new command")
    assert p._failed_plans(0, 0) == []
