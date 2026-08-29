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
    target = {"id": "t0", "postconditions": ["object_seen(pudding_box)"]}
    failed_plans = [[
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}},
    ]]
    plan = planner_mod._escape_plan(
        target, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)", failed_plans,
    )
    assert plan == [
        {"action": "search_object", "params": {"object": "pudding_box", "location": "living_room"}},
    ]


def test_escape_plan_object_seen_returns_none_when_both_establishers_tried():
    # find_object AND search_object both already failed -- object_seen has
    # no other registry establisher, so there is nothing left to try.
    target = {"id": "t0", "postconditions": ["object_seen(pudding_box)"]}
    failed_plans = [
        [{"action": "goto", "params": {"location": "living_room"}},
         {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}}],
        [{"action": "goto", "params": {"location": "living_room"}},
         {"action": "search_object", "params": {"object": "pudding_box", "location": "living_room"}}],
    ]
    plan = planner_mod._escape_plan(
        target, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)", failed_plans,
    )
    assert plan is None


def test_escape_plan_person_found_after_find_person_has_no_untried_establisher():
    # person_found is established ONLY by find_person -- there is no second
    # strategy to escape to.
    target = {"id": "t0", "postconditions": ["person_found(waving_person)"]}
    failed_plans = [[{"action": "find_person", "params": {"person": "waving_person"}}]]
    plan = planner_mod._escape_plan(
        target, "", "",
        "postcondition unmet: person_found(waving_person) (UNKNOWN)", failed_plans,
    )
    assert plan is None


def test_escape_plan_held_after_grasp_has_no_untried_establisher():
    # `held` is established ONLY by `grasp`, which is already in the failed
    # history -- so there is no untried establisher and _escape_plan returns
    # None. `held(spam)` is also NOT one of this target's own postconditions
    # (its postcondition is delivered(spam,me)) -- M3 (Task D review): the
    # escape is postcondition-only, so this reason ("precondition unmet:
    # held(spam)") is skipped by case (1) even before the used-establisher
    # check matters, and case (2) (iterating the target's OWN postconditions)
    # never considers `held` either. The requires-chaining logic this test
    # used to exercise ("prepend grasp's canonical establisher") has been
    # REMOVED entirely (M3) -- the escape never chains anymore.
    target = {"id": "t0", "postconditions": ["delivered(spam,me)"]}
    failed_plans = [[
        {"action": "find_object", "params": {"object": "spam"}},
        {"action": "grasp", "params": {"object": "spam"}},
    ]]
    plan = planner_mod._escape_plan(
        target, "spam", "",
        "precondition unmet: held(spam) (invalid)", failed_plans,
    )
    assert plan is None


def test_escape_plan_derives_predicate_from_target_when_reason_has_no_fact():
    # H1 (battery run 004, Task D review): the leaf's raw feedback message
    # never contains a pred(args) fact at all (unlike a structured
    # postcondition-gate reason) -- case (1)'s regex/parse finds nothing, so
    # _escape_plan must fall back to the TARGET's own declared postconditions
    # (case (2)) and pick the first with an untried, eligible establisher.
    target = {
        "id": "t1", "desc": "locate a pudding_box in the bedroom",
        "object": "pudding_box", "location": "bedroom", "depends_on": [],
        "preconditions": [], "postconditions": ["object_seen(pudding_box)"],
    }
    failed_plans = [[
        {"action": "find_object", "params": {"object": "pudding_box", "location": "bedroom"}},
    ]]
    reason = (
        'ScanForGeneralist for brown pudding box failed status=1: '
        'no matches for "brown pudding box" via vlm_sam'
    )
    plan = planner_mod._escape_plan(target, "pudding_box", "bedroom", reason, failed_plans)
    assert plan == [
        {"action": "search_object", "params": {"object": "pudding_box", "location": "bedroom"}},
    ]


def test_escape_plan_resolves_location_from_nearest_ancestor():
    # L2 (Task D review): t1's own failed plan never navigated (the goto
    # lived in ancestor t0's plan instead) and t1 itself has no assigned
    # location -- the escape must fall back to the nearest dependency
    # ancestor's own `location` for the self-navigating `search_object` step.
    t0 = {
        "id": "t0", "desc": "go to the bedroom", "object": "", "location": "bedroom",
        "depends_on": [], "preconditions": [], "postconditions": ["at_robot(bedroom)"],
    }
    t1 = {
        "id": "t1", "desc": "locate a pudding_box", "object": "pudding_box", "location": "",
        "depends_on": ["t0"], "preconditions": [], "postconditions": ["object_seen(pudding_box)"],
    }
    failed_plans = [[{"action": "find_object", "params": {"object": "pudding_box"}}]]
    plan = planner_mod._escape_plan(
        t1, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)",
        failed_plans, all_targets=[t0, t1],
    )
    assert plan == [
        {"action": "search_object", "params": {"object": "pudding_box", "location": "bedroom"}},
    ]


def test_escape_plan_returns_none_when_resolved_location_is_not_known():
    # M-2 (round-2 review): the escape location is never checked against
    # known_locations -- if the resolved location has no pose (e.g. it was
    # reached via record_position, or is simply a typo/unrecognised room),
    # the materialised search_object(location=...) step is DOOMED to be
    # rejected by validate_plan's "unknown location" rule anyway. Passing
    # known_locations lets _escape_plan bail out early (None) instead of
    # wasting a validate_plan cycle on a step that can never pass.
    target = {"id": "t0", "postconditions": ["object_seen(pudding_box)"]}
    failed_plans = [[
        {"action": "goto", "params": {"location": "attic"}},
        {"action": "find_object", "params": {"object": "pudding_box", "location": "attic"}},
    ]]
    plan = planner_mod._escape_plan(
        target, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)", failed_plans,
        known_locations={"kitchen", "living_room"},
    )
    assert plan is None


def test_escape_plan_accepts_resolved_location_when_it_is_known():
    # Same shape as the case above, but "attic" IS in known_locations --
    # the escape must still fire normally.
    target = {"id": "t0", "postconditions": ["object_seen(pudding_box)"]}
    failed_plans = [[
        {"action": "goto", "params": {"location": "attic"}},
        {"action": "find_object", "params": {"object": "pudding_box", "location": "attic"}},
    ]]
    plan = planner_mod._escape_plan(
        target, "pudding_box", "",
        "postcondition unmet: object_seen(pudding_box) (UNKNOWN)", failed_plans,
        known_locations={"attic", "kitchen"},
    )
    assert plan == [
        {"action": "search_object", "params": {"object": "pudding_box", "location": "attic"}},
    ]


def test_escape_plan_returns_none_when_failure_reason_has_no_fact():
    # No fact parses out of the reason (case 1) AND the target itself
    # declares no postconditions to fall back to (case 2 has nothing to
    # iterate) -- there is nothing for _escape_plan to derive a predicate
    # from either way.
    target = {"id": "t0"}
    assert planner_mod._escape_plan(target, "", "", "child FAILURE", []) is None
    assert planner_mod._escape_plan(target, "", "", "", []) is None


# ---------------------------------------------------------------------------
# E2 (runs 003/004, 2026-08-29): when the escape ladder is fully exhausted --
# no registry action can establish ANY of the target's own postconditions
# without repeating one already tried -- the target must END honestly
# (UNRECOVERABLE_ERROR_PREFIX) instead of burning the whole replan budget on
# a run of IDENTICAL_PLAN_SKIPPED markers for plans the LLM cannot vary.
# _escape_no_untried_establisher_reason distinguishes this from the other
# reasons _escape_plan can come back empty (an eligible establisher exists
# but its escape step was rejected by validate_plan, or a self-navigating
# step's location could not be resolved) -- neither of those is unrecoverable.
# ---------------------------------------------------------------------------

def test_escape_unrecoverable_reason_is_none_when_an_untried_establisher_exists():
    target = {"id": "t0", "postconditions": ["object_seen(pudding_box)"]}
    failed_plans = [[
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}},
    ]]
    assert planner_mod._escape_no_untried_establisher_reason(target, failed_plans) is None


def test_escape_unrecoverable_reason_fires_when_all_establishers_are_tried():
    # find_object AND search_object -- object_seen's only two registry
    # establishers -- have both already failed.
    target = {"id": "t0", "postconditions": ["object_seen(pudding_box)"]}
    failed_plans = [
        [{"action": "find_object", "params": {"object": "pudding_box"}}],
        [{"action": "search_object", "params": {"object": "pudding_box"}}],
    ]
    reason = planner_mod._escape_no_untried_establisher_reason(target, failed_plans)
    assert reason is not None
    assert "object_seen(pudding_box)" in reason


def test_escape_unrecoverable_reason_fires_for_person_found_single_establisher():
    # person_found is established ONLY by find_person -- there is no second
    # strategy, so a single failed find_person already exhausts the ladder.
    target = {"id": "t0", "postconditions": ["person_found(liam)"]}
    failed_plans = [[{"action": "find_person", "params": {"person": "liam"}}]]
    reason = planner_mod._escape_no_untried_establisher_reason(target, failed_plans)
    assert reason is not None
    assert "person_found(liam)" in reason


def test_escape_unrecoverable_reason_is_none_without_declared_postconditions():
    # A target that declares no postconditions at all is not the
    # "escape ladder exhausted" case -- there is nothing to say is exhausted.
    assert planner_mod._escape_no_untried_establisher_reason({"id": "t0"}, []) is None


# ---------------------------------------------------------------------------
# H1 (round-2 review): the exhaustion helper must NOT apply _escape_plan's
# own materialisation arg-name filter (_ESCAPE_ALLOWED_ARG_NAMES) -- it
# answers "has this STRATEGY been tried", not "can _escape_plan materialise a
# step for it". Every one of answered(question)'s six establishers carries a
# `question` arg, which _ESCAPE_ALLOWED_ARG_NAMES excludes -- so the OLD
# helper (copying that filter) always returned "no untried establisher" for
# answered(...), marking the target UNRECOVERABLE after a single replan even
# though answer_question / count / announce / vlm_fallback / llm_fallback
# were never tried.
# ---------------------------------------------------------------------------

def test_escape_unrecoverable_reason_is_none_for_answered_with_only_ask_person_tried():
    target = {"id": "t0", "postconditions": ["answered(what_is_your_name)"]}
    failed_plans = [[
        {"action": "goto", "params": {"location": "living_room"}},
        {"action": "find_person", "params": {"person": "x"}},
        {"action": "ask_person", "params": {"question": "what_is_your_name"}},
    ]]
    assert planner_mod._escape_no_untried_establisher_reason(target, failed_plans) is None


def test_escape_unrecoverable_reason_fires_for_answered_when_every_establisher_tried():
    # All six of answered(question)'s registry establishers have been tried
    # -- NOW the ladder really is exhausted.
    target = {"id": "t0", "postconditions": ["answered(what_is_your_name)"]}
    failed_plans = [[
        {"action": "ask_person", "params": {"question": "q"}},
        {"action": "answer_question", "params": {"question": "q"}},
        {"action": "count", "params": {"object": "x", "question": "q"}},
        {"action": "announce", "params": {"question": "q"}},
        {"action": "vlm_fallback", "params": {"question": "q"}},
        {"action": "llm_fallback", "params": {"question": "q"}},
    ]]
    reason = planner_mod._escape_no_untried_establisher_reason(target, failed_plans)
    assert reason is not None
    assert "answered(what_is_your_name)" in reason


# ---------------------------------------------------------------------------
# L3 (round-2 review): helper tests were all single-postcondition -- pin the
# multi-postcondition shape (object_seen + held, the grasp/deliver targets
# M2 below is about).
# ---------------------------------------------------------------------------

def test_escape_unrecoverable_reason_multi_postcondition_is_none_with_untried_establisher():
    target = {"id": "t0", "postconditions": ["object_seen(spam)", "held(spam)"]}
    failed_plans = [[{"action": "find_object", "params": {"object": "spam"}}]]
    assert planner_mod._escape_no_untried_establisher_reason(target, failed_plans) is None


def test_escape_unrecoverable_reason_multi_postcondition_lists_both_facts_when_exhausted():
    target = {"id": "t0", "postconditions": ["object_seen(spam)", "held(spam)"]}
    failed_plans = [[
        {"action": "find_object", "params": {"object": "spam"}},
        {"action": "search_object", "params": {"object": "spam"}},
        {"action": "grasp", "params": {"object": "spam"}},
    ]]
    reason = planner_mod._escape_no_untried_establisher_reason(target, failed_plans)
    assert reason is not None
    assert "object_seen(spam)" in reason and "held(spam)" in reason


def test_plan_target_final_identical_attempt_escapes_to_untried_action(monkeypatch, capsys):
    # D1 integration (battery run 004): stubbed _call_llm returns the
    # identical [goto, find_object] plan on every attempt -- the cache must
    # hold the deterministic escape plan (search_object) with error None,
    # instead of burning the whole replan budget on IDENTICAL_PLAN_SKIPPED
    # markers.
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    # H1 (Task D review): the escape now derives from the TARGET's own
    # declared postconditions (M3: postcondition-only), so this target must
    # carry object_seen(pudding_box) -- matching the real run-004 target --
    # for the escape to fire at all.
    target = {"id": "t0", "desc": "d", "object": "", "location": "",
              "depends_on": [], "preconditions": [], "postconditions": ["object_seen(pudding_box)"]}
    p._slot_context[0] = {"command": "c", "targets": [target]}
    same = {"plan": [{"action": "goto", "params": {"location": "living_room"}},
                     {"action": "find_object", "params": {"object": "pudding_box", "location": "living_room"}}]}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (same, None))
    monkeypatch.setattr(planner_mod, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", same["plan"], py_trees.behaviours.Success("old"), None)
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c", failure_reason="postcondition unmet: object_seen(pudding_box) (UNKNOWN)",
                  target=target, all_targets=[target])
    assert p.is_target_ready(0, 0)
    plan = p.get_action_plan(0, 0)
    assert [s["action"] for s in plan] == ["search_object"]
    assert plan[0]["params"]["object"] == "pudding_box"
    assert plan[0]["params"]["location"] == "living_room"
    assert p.get_error(0, 0) is None
    out = capsys.readouterr().out
    assert "LLM stuck on an identical plan -> deterministic escape" in out
    assert "dropped []" in out


def test_plan_target_marks_unrecoverable_when_escape_ladder_is_exhausted(monkeypatch, capsys):
    # E2 (runs 003/004, 2026-08-29): find_object AND search_object have BOTH
    # already failed for this target -- object_seen's only two registry
    # establishers -- so _escape_plan has nothing left to try. This is the
    # truly UNRECOVERABLE case (not merely "identical again"): the target
    # must be marked so the executor ends it instead of burning the rest of
    # the replan budget on more IDENTICAL_PLAN_SKIPPED markers for a plan the
    # LLM cannot vary.
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {"id": "t0", "desc": "d", "object": "", "location": "",
              "depends_on": [], "preconditions": [], "postconditions": ["object_seen(pudding_box)"]}
    p._slot_context[0] = {"command": "c", "targets": [target]}
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
    p.plan_target(0, 0, "d", command="c", failure_reason="postcondition unmet: object_seen(pudding_box) (UNKNOWN)",
                  target=target, all_targets=[target])
    assert p.is_target_ready(0, 0)
    assert [s["action"] for s in p.get_action_plan(0, 0)] == ["goto", "find_object"]
    err = p.get_error(0, 0)
    assert err is not None and err.startswith(planner_mod.UNRECOVERABLE_ERROR_PREFIX)
    assert "object_seen(pudding_box)" in err
    out = capsys.readouterr().out
    assert "[plan:0:0] no untried establisher for" in out
    assert "object_seen(pudding_box)" in out
    assert "-> unrecoverable" in out


# ---------------------------------------------------------------------------
# M2 (round-2 review): a validation-rejected escape action must be recorded
# (`entry["tried_escapes"]`) so the SECOND identical replan sees the ladder
# as exhausted instead of re-materialising (and re-rejecting) the same
# doomed candidate forever -- the run-003/004 symptom E2 was meant to end,
# for a 2-postcondition target (the common grasp/deliver shape): a
# single-step escape can satisfy object_seen but never also held, so
# validate_plan always rejects it on coverage grounds.
# ---------------------------------------------------------------------------

def test_plan_target_records_rejected_escape_then_marks_unrecoverable_on_next_replan(monkeypatch):
    p = planner_mod.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {"id": "t0", "desc": "d", "object": "spam", "location": "kitchen",
              "depends_on": [], "preconditions": [],
              "postconditions": ["object_seen(spam)", "held(spam)"]}
    p._slot_context[0] = {"command": "c", "targets": [target]}
    failed_first = [{"action": "goto", "params": {"location": "kitchen"}},
                    {"action": "find_object", "params": {"object": "spam", "location": "kitchen"}},
                    {"action": "grasp", "params": {"object": "spam"}}]
    same = {"plan": list(failed_first)}
    monkeypatch.setattr(planner_mod, "_call_llm", lambda *a, **k: (same, None))

    def fake_validate(plan, desc, known_actions, **kwargs):
        # The deterministic escape (a lone search_object) is the ONLY plan
        # this fake rejects -- held(spam) stays uncovered (M-5: _escape_plan
        # only ever materialises one step) -- everything else (the
        # LLM-returned identical plan itself) validates fine.
        if [s["action"] for s in plan] == ["search_object"]:
            return False, "postcondition unmet: held(spam) (UNKNOWN)"
        return True, ""

    monkeypatch.setattr(planner_mod, "validate_plan", fake_validate)
    monkeypatch.setattr(planner_mod, "validate_plan_modifications", lambda *a, **k: (True, ""))
    p._store(0, 0, "d", failed_first, py_trees.behaviours.Success("s"), None)
    p._invalidate(0, 0)

    # Replan 1: identical -> _escape_plan picks search_object (untried) ->
    # rejected by validate_plan (held still uncovered) -> the "rejected by
    # validation" case is NOT unrecoverable -- ordinary IDENTICAL marker.
    p.plan_target(0, 0, "d", command="c",
                  failure_reason="postcondition unmet: object_seen(spam) (UNKNOWN)",
                  target=target, all_targets=[target])
    err1 = p.get_error(0, 0)
    assert err1 is not None and err1.startswith(planner_mod.IDENTICAL_PLAN_ERROR_PREFIX)

    # Replan 2: search_object is now a recorded tried_escape -- object_seen's
    # two registry establishers (find_object, search_object) AND held's only
    # establisher (grasp) have all been used -- the ladder is exhausted.
    p._invalidate(0, 0)
    p.plan_target(0, 0, "d", command="c",
                  failure_reason="postcondition unmet: object_seen(spam) (UNKNOWN)",
                  target=target, all_targets=[target])
    err2 = p.get_error(0, 0)
    assert err2 is not None and err2.startswith(planner_mod.UNRECOVERABLE_ERROR_PREFIX)
    assert "object_seen(spam)" in err2 and "held(spam)" in err2


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
