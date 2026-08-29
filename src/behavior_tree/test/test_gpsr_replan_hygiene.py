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


# ---------------------------------------------------------------------------
# D1: _escape_plan -- deterministic escape when the LLM is stuck repeating
# an identical plan across the whole replan budget (battery run 004).
# ---------------------------------------------------------------------------

def test_escape_plan_object_seen_after_find_object_tries_search_object():
    failed_plans = [[
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}},
    ]]
    plan = planner_mod._escape_plan(
        {"id": "t0"}, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)", failed_plans,
    )
    assert plan == [
        {"action": "search_object", "params": {"object": "pudding_box", "location": "living_room"}},
    ]


def test_escape_plan_object_seen_returns_none_when_both_establishers_tried():
    # find_object AND search_object both already failed -- object_seen has
    # no other registry establisher, so there is nothing left to try.
    failed_plans = [
        [{"action": "goto", "params": {"location": "living_room"}},
         {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}}],
        [{"action": "goto", "params": {"location": "living_room"}},
         {"action": "search_object", "params": {"object": "pudding_box", "location": "living_room"}}],
    ]
    plan = planner_mod._escape_plan(
        {"id": "t0"}, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)", failed_plans,
    )
    assert plan is None


def test_escape_plan_person_found_after_find_person_has_no_untried_establisher():
    # person_found is established ONLY by find_person -- there is no second
    # strategy to escape to.
    failed_plans = [[{"action": "find_person", "params": {"person": "waving_person"}}]]
    plan = planner_mod._escape_plan(
        {"id": "t0"}, "", "",
        "postcondition unmet: person_found(waving_person) (UNKNOWN)", failed_plans,
    )
    assert plan is None


def test_escape_plan_held_after_grasp_has_no_untried_establisher():
    # `held` is established ONLY by `grasp`, which is already in the failed
    # history -- so there is no untried establisher and _escape_plan returns
    # None. This is DESPITE grasp itself `requires` object_seen: the
    # requires-chaining ("prepend its canonical establisher") logic only
    # runs once a CANDIDATE has been chosen -- with zero candidates here it
    # never triggers. See
    # test_escape_plan_prepends_canonical_establisher_for_an_untried_requires
    # below for the chaining logic actually firing.
    failed_plans = [[
        {"action": "find_object", "params": {"object": "spam"}},
        {"action": "grasp", "params": {"object": "spam"}},
    ]]
    plan = planner_mod._escape_plan(
        {"id": "t0"}, "spam", "",
        "precondition unmet: held(spam) (invalid)", failed_plans,
    )
    assert plan is None


def test_escape_plan_prepends_canonical_establisher_for_an_untried_requires():
    # grasp itself is untried, but it `requires` object_seen(object), which
    # is not yet established anywhere in the failed history (no
    # find_object/search_object) -- its canonical establisher (find_object)
    # must be prepended so the escape plan is actually executable.
    failed_plans = [[
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]]
    plan = planner_mod._escape_plan(
        {"id": "t0"}, "spam", "",
        "precondition unmet: held(spam) (invalid)", failed_plans,
    )
    assert plan == [
        {"action": "find_object", "params": {"object": "spam"}},
        {"action": "grasp", "params": {"object": "spam"}},
    ]


def test_escape_plan_returns_none_when_failure_reason_has_no_fact():
    assert planner_mod._escape_plan({"id": "t0"}, "", "", "child FAILURE", []) is None
    assert planner_mod._escape_plan({"id": "t0"}, "", "", "", []) is None


def test_plan_target_final_identical_attempt_escapes_to_untried_action(monkeypatch, capsys):
    # D1 integration (battery run 004): stubbed _call_llm returns the
    # identical [goto, find_object] plan on every attempt -- the cache must
    # hold the deterministic escape plan (search_object) with error None,
    # instead of burning the whole replan budget on IDENTICAL_PLAN_SKIPPED
    # markers.
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    same = {"plan": [{"action": "goto", "params": {"location": "living_room"}},
                     {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}}]}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (same, None))
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="postcondition unmet: object_seen(pudding_box) (UNKNOWN)")
    assert p.is_target_ready(0, 0)
    plan = p.get_action_plan(0, 0)
    assert [s["action"] for s in plan] == ["search_object"]
    assert plan[0]["params"]["object"] == "pudding_box"
    assert plan[0]["params"]["location"] == "living_room"
    assert p.get_error(0, 0) is None
    out = capsys.readouterr().out
    assert "LLM stuck on an identical plan -> deterministic escape" in out


def test_plan_target_final_identical_attempt_keeps_marker_when_escape_exhausted(monkeypatch):
    # The mirror case: find_object AND search_object have BOTH already
    # failed for this target, so _escape_plan has nothing left to try and
    # the existing IDENTICAL_PLAN marker path must still apply unchanged.
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    earlier = [{"action": "goto", "params": {"location": "living_room"}},
               {"action": "search_object", "params": {"object": "pudding_box", "location": "living_room"}}]
    same = {"plan": [{"action": "goto", "params": {"location": "living_room"}},
                     {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}}]}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (same, None))
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", earlier, py_trees.behaviours.Success("s"), None)
    p._invalidate(0, 0)
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("s"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="postcondition unmet: object_seen(pudding_box) (UNKNOWN)")
    assert p.is_target_ready(0, 0)
    assert [s["action"] for s in p.get_action_plan(0, 0)] == ["goto", "find_object"]
    err = p.get_error(0, 0)
    assert err is not None and err.startswith(planner_mod.IDENTICAL_PLAN_ERROR_PREFIX)


def test_plan_target_marks_identical_final_attempt_with_error(monkeypatch):
    # D1 update: with only find_object in the failed history, the final
    # identical attempt would now escape to search_object (see
    # test_plan_target_final_identical_attempt_escapes_to_untried_action)
    # instead of falling back to the marker -- so this test's history now
    # ALSO includes a failed search_object attempt, exhausting object_seen's
    # only two registry establishers, to keep exercising the marker path
    # this test is actually about (plus the non-final rejection prompt
    # content, still driven by the same object_seen failure_reason).
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    earlier_search = [{"action": "goto", "params": {"location": "living_room"}},
                      {"action": "search_object", "params": {"object": "kitchen item", "location": "living_room"}}]
    same = {"plan": [{"action": "goto", "params": {"location": "living_room"}},
                     {"action": "find_object", "params": {"object": "kitchen item", "location": "living_room"}}]}
    prompts = []

    def fake_call(client, system, user, temperature):
        prompts.append(user)
        return same, None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", earlier_search, py_trees.behaviours.Success("older"), None)
    p._invalidate(0, 0)
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
    #
    # D1: the fact this test parses out of the (stripped) reason is
    # at_robot(x), whose only registry establisher is goto -- the failed
    # plan includes goto too (not just announce) so `_escape_plan` finds
    # nothing untried and this test keeps exercising the marker path it is
    # actually about, instead of the new deterministic escape.
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    same = {"plan": [{"action": "goto", "params": {"location": "x"}},
                     {"action": "announce", "params": {"text": "x"}}]}

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


def test_plan_target_non_final_identical_rejection_uses_stripped_reason(monkeypatch):
    # L3 (Task B review): the non-final identical-attempt retry prompt must
    # show the STRIPPED reason (`identical_marker_reason`), never the raw
    # `failure_reason` -- which, on a repeated identical-replan cycle, is
    # itself already prefixed with IDENTICAL_PLAN_ERROR_PREFIX. Showing the
    # raw value nests the prefix in the prompt text ("... plan that just
    # failed (identical to failed plan: ...)").
    p = planner_mod.GPSRPlanner(max_attempts=3)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {"command": "c", "targets": [{"id": "t0", "desc": "d", "object": "", "location": "", "depends_on": []}]}
    same = {"plan": [{"action": "announce", "params": {"text": "x"}}]}
    different = {"plan": [{"action": "goto", "params": {"location": "a"}}]}
    responses = iter([same, same, different])
    prompts = []

    def fake_call(client, system, user, temperature):
        prompts.append(user)
        return next(responses), None

    monkeypatch.setattr(planner_mod, "_call_llm", fake_call)
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)

    nested_failure_reason = f"{planner_mod.IDENTICAL_PLAN_ERROR_PREFIX}: precondition unmet: at_robot(x) (invalid)"
    p.plan_target(0, 0, "d", command="c", failure_reason=nested_failure_reason)

    assert p.get_action_plan(0, 0) == different["plan"]
    non_final_prompts = prompts[1:]
    assert non_final_prompts, "expected at least one retry prompt after a non-final identical rejection"
    marker = "The previous attempt failed with: "
    for u in non_final_prompts:
        # Isolate the retry-reason sentence built from `last_reason` (the
        # non-final identical-rejection text) -- NOT the whole prompt, which
        # also echoes the raw `failure_reason` once in its unrelated
        # "Completed steps so far" state log.
        start = u.index(marker) + len(marker)
        failed_with_sentence = u[start:u.index("\n", start)]
        assert "IDENTICAL" in failed_with_sentence
        assert "precondition unmet: at_robot(x) (invalid)" in failed_with_sentence
        # The stripped reason must appear on its own -- the raw,
        # still-prefixed failure_reason ("identical to failed plan:
        # precondition unmet: ...") must not be nested inside it.
        assert planner_mod.IDENTICAL_PLAN_ERROR_PREFIX not in failed_with_sentence


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
