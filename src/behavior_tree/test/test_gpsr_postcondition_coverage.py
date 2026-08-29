"""C2 — a plan must cover its target's declared postconditions.

Regression coverage for the "locate them in the kitchen" defect (run 006,
meetNameAtLocThenFindInRm ERROR): the target's postcondition was
person_found(sarah), but nothing in the emitted plan calls find_person (or
any other establisher of person_found) -- the plan simply never establishes
its own target's postcondition, and nothing rejected it.

validate_plan gained a postconditions= kwarg: for each parsable
postcondition, at least one step's contract
(ACTION_CONTRACTS[action].establishes) must name that predicate.
"""
from __future__ import annotations

import py_trees
import pytest

from behavior_tree.GPSR import planner as planner_module
from behavior_tree.GPSR.planner_validators import validate_plan, uncovered_postcondition_reason


# ---------------------------------------------------------------------------
# Pure validate_plan(..., postconditions=...) checks.
# ---------------------------------------------------------------------------

def test_plan_without_establisher_is_rejected_naming_find_person():
    plan = [
        {"action": "follow", "params": {"person": "sarah"}},
        {"action": "goto", "params": {"location": "kitchen"}},
    ]
    ok, reason = validate_plan(
        plan, "locate sarah in the kitchen", {"follow", "goto"},
        postconditions=["person_found(sarah)"],
    )
    assert not ok
    assert "person_found(sarah)" in reason
    assert "find_person" in reason


def test_plan_with_find_person_is_accepted():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_person", "params": {"person": "sarah"}},
    ]
    ok, reason = validate_plan(
        plan, "locate sarah in the kitchen", {"goto", "find_person"},
        postconditions=["person_found(sarah)"],
    )
    assert ok, reason


def test_announce_covers_answered_postcondition():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "grasp", "params": {"object": "coke"}},
        {"action": "announce", "params": {"text": "Today is Saturday."}},
    ]
    ok, reason = validate_plan(
        # Avoid OPERATOR_REPORT_RE ("tell me") -- that rule is orthogonal to
        # postcondition coverage and would reject this plan for a different
        # reason (no goto(start_position) before the final announce).
        plan, "what day is it", {"goto", "grasp", "announce"},
        postconditions=["answered(what day is today)"],
    )
    assert ok, reason


def test_llm_fallback_alone_covers_answered_postcondition():
    # H1 (round-2 fix): the sim defect this brief is fixing -- "tell me what
    # day it is" plans llm_fallback(question=...) and the prompt explicitly
    # says do NOT add an announce after it. Bare llm_fallback must cover
    # answered(...) on its own now.
    plan = [{"action": "llm_fallback", "params": {"question": "what day is today"}}]
    ok, reason = validate_plan(
        plan, "tell me what day it is", {"llm_fallback"},
        postconditions=["answered(what day is today)"],
    )
    assert ok, reason


def test_goto_vlm_fallback_covers_answered_postcondition():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "vlm_fallback", "params": {"question": "what colour is the mug"}},
    ]
    ok, reason = validate_plan(
        plan, "what colour is the mug", {"goto", "vlm_fallback"},
        postconditions=["answered(what colour is the mug)"],
    )
    assert ok, reason


def test_goto_count_goto_announce_covers_answered_postcondition():
    # The t1-42 observed shape: answered(how many drinks there are on the
    # kitchen_table) planned as [goto, count, goto(start_position), announce()].
    plan = [
        {"action": "goto", "params": {"location": "kitchen_table"}},
        {"action": "count", "params": {"object": "drinks", "location": "kitchen_table"}},
        {"action": "goto", "params": {"location": "start_position"}},
        {"action": "announce", "params": {}},
    ]
    ok, reason = validate_plan(
        plan, "how many drinks are there on the kitchen table",
        {"goto", "count", "announce"},
        postconditions=["answered(how many drinks there are on the kitchen table)"],
    )
    assert ok, reason


def test_llm_fallback_added_to_bare_announce_whitelist():
    # H1: llm_fallback joins vlm_fallback/count/describe_person/ask_person as
    # a legitimate predecessor for a text-less announce (structural rule,
    # independent of postconditions).
    plan = [
        {"action": "llm_fallback", "params": {"question": "what day is today"}},
        {"action": "goto", "params": {"location": "start_position"}},
        {"action": "announce", "params": {}},
    ]
    ok, reason = validate_plan(
        plan, "please tell me what day it is",
        {"llm_fallback", "goto", "announce"},
    )
    assert ok, reason


def test_at_robot_excused_when_a_step_self_navigates_there():
    # place() self-establishes at_robot via its `location` param -- no
    # separate goto/establisher of at_robot(kitchen) is required.
    plan = [{"action": "place", "params": {"object": "cup", "location": "kitchen"}}]
    ok, reason = validate_plan(
        plan, "put the cup in the kitchen", {"place"},
        postconditions=["at_robot(kitchen)"],
    )
    assert ok, reason


def test_at_robot_coverage_requires_matching_destination_not_any_goto():
    # M-3 (round-2 review): uncovered_postcondition_reason's at_robot special
    # case was predicate-level ONLY (any self-navigating step anywhere
    # satisfied at_robot(X) for every X) -- a guard-reduced
    # [goto(laundry_desk)] passed coverage for at_robot(kitchen) just as
    # easily as at_robot(laundry_desk), only to fail the RUNTIME gate later
    # (last_nav_location mismatch). The destination must actually match.
    plan = [{"action": "goto", "params": {"location": "laundry_desk"}}]
    ok, reason = validate_plan(
        plan, "go to the kitchen", {"goto"},
        postconditions=["at_robot(kitchen)"],
    )
    assert not ok
    assert "at_robot(kitchen)" in reason


def test_at_robot_coverage_accepts_matching_goto_destination():
    plan = [{"action": "goto", "params": {"location": "kitchen"}}]
    ok, reason = validate_plan(
        plan, "go to the kitchen", {"goto"},
        postconditions=["at_robot(kitchen)"],
    )
    assert ok, reason


def test_no_postconditions_kwarg_is_unchanged_behaviour():
    plan = [{"action": "follow", "params": {"person": "sarah"}}]
    ok, reason = validate_plan(plan, "follow sarah", {"follow"})
    assert ok, reason


def test_empty_postconditions_iterable_is_a_noop():
    plan = [{"action": "follow", "params": {"person": "sarah"}}]
    ok, reason = validate_plan(
        plan, "follow sarah", {"follow"}, postconditions=[],
    )
    assert ok, reason


def test_unparsable_postcondition_is_skipped_not_rejected():
    plan = [{"action": "follow", "params": {"person": "sarah"}}]
    ok, reason = validate_plan(
        plan, "follow sarah", {"follow"},
        postconditions=["not a valid fact"],
    )
    assert ok, reason


def test_postcondition_check_runs_after_earlier_more_specific_rejections():
    # A placeholder-leak rejection (an earlier, more specific rule) must win
    # even when the plan ALSO fails to cover its postcondition.
    plan = [{"action": "announce", "params": {"text": "Today is <day>."}}]
    ok, reason = validate_plan(
        plan, "tell me what day it is", {"announce"},
        postconditions=["answered(what day is today)"],
    )
    assert not ok
    assert "placeholder" in reason


def test_uncovered_reason_lists_every_registry_establisher():
    reason = uncovered_postcondition_reason(
        [{"action": "goto", "params": {"location": "kitchen"}}],
        ["object_seen(coke)"],
    )
    assert reason is not None
    assert "find_object" in reason and "search_object" in reason


# ---------------------------------------------------------------------------
# plan_target wiring: postconditions come from the target dict.
# ---------------------------------------------------------------------------

def test_plan_target_rejects_plan_missing_its_own_postcondition_establisher(monkeypatch):
    p = planner_module.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: object())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {
        "id": "t0", "desc": "locate sarah in the kitchen", "object": "", "location": "kitchen",
        "depends_on": [], "preconditions": [], "postconditions": ["person_found(sarah)"],
    }
    bad = {"plan": [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "announce", "params": {"text": "here"}},
    ]}
    good = {"plan": [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_person", "params": {"person": "sarah"}},
    ]}
    responses = iter([bad, good])
    prompts = []

    def fake_call(client, system, user, temperature):
        prompts.append(user)
        return next(responses), None

    monkeypatch.setattr(planner_module, "_call_llm", fake_call)
    monkeypatch.setattr(planner_module, "validate_plan_modifications", lambda *a, **k: (True, ""))

    p.plan_target(
        0, 0, target["desc"], command="locate sarah in the kitchen",
        target=target, all_targets=[target],
    )

    assert p.get_action_plan(0, 0) == good["plan"]
    # The rejection reason from attempt 1 reached attempt 2's prompt.
    assert len(prompts) == 2
    assert "person_found(sarah)" in prompts[1]
    assert "find_person" in prompts[1]


def test_offline_mock_plan_target_path_ignores_postconditions():
    p = planner_module.GPSRPlanner(max_attempts=2)
    p._offline_mock = True
    target = {
        "id": "t0", "desc": "locate sarah in the kitchen", "object": "", "location": "kitchen",
        "depends_on": [], "preconditions": [], "postconditions": ["person_found(sarah)"],
    }
    p.plan_target(0, 0, target["desc"], command="locate sarah in the kitchen",
                  target=target, all_targets=[target])
    assert p.is_target_ready(0, 0)
    assert p.get_error(0, 0) is None


# ---------------------------------------------------------------------------
# replace_target_plan wiring: warns (does not block -- no retry loop here),
# and subtracts predicates already established by completed_steps.
# ---------------------------------------------------------------------------

def test_replace_target_plan_warns_when_remaining_plan_misses_postcondition(monkeypatch, capsys):
    p = planner_module.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {
        "id": "t0", "desc": "locate sarah in the kitchen", "object": "", "location": "kitchen",
        "depends_on": [], "preconditions": [], "postconditions": ["person_found(sarah)"],
    }
    p._slot_context[0] = {"command": "locate sarah in the kitchen", "targets": [target]}
    plan = [{"action": "goto", "params": {"location": "kitchen"}}]

    p.replace_target_plan(0, 0, plan, reason="supervisor global replan")

    out = capsys.readouterr().out
    assert "person_found(sarah)" in out
    assert "find_person" in out
    # Non-blocking: the plan is still installed.
    assert p.get_action_plan(0, 0) == plan


def test_replace_target_plan_subtracts_predicates_established_by_completed_steps(monkeypatch, capsys):
    p = planner_module.GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    target = {
        "id": "t0", "desc": "locate sarah in the kitchen", "object": "", "location": "kitchen",
        "depends_on": [], "preconditions": [], "postconditions": ["person_found(sarah)"],
    }
    p._slot_context[0] = {"command": "locate sarah in the kitchen", "targets": [target]}
    completed = [{"action": "find_person", "params": {"person": "sarah"}}]
    plan = [{"action": "approach_person", "params": {}}]

    p.replace_target_plan(0, 0, plan, reason="supervisor global replan", completed_steps=completed)

    out = capsys.readouterr().out
    assert "person_found(sarah)" not in out
    assert p.get_action_plan(0, 0) == plan
