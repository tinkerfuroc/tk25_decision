"""Lower-layer plans must stay inside their target's fact contract.

Regression coverage for the "bring me a spam from the laundry_desk" defect:
the lower layer planned t0 as [goto, grasp, deliver] and t1 as [deliver], so
the robot delivered twice. Two guards close the gap:

1. The lower-layer prompt now tells the LLM its target's own declared
   pre/postconditions AND which facts belong to sibling targets
   (``_build_lower_layer_user_prompt(..., contract=..., all_targets=...)``).
2. A deterministic, network-free guard (``_drop_foreign_contract_steps``)
   drops any step whose established fact belongs to a DIFFERENT target,
   even if the LLM ignores the prompt.
"""
from __future__ import annotations

import py_trees
import pytest

from behavior_tree.GPSR import planner as planner_module
from behavior_tree.GPSR.planner import (
    GPSRPlanner,
    _build_lower_layer_user_prompt,
    _drop_foreign_contract_steps,
)


@pytest.fixture(autouse=True)
def _clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


# ---------------------------------------------------------------------------
# Fixtures: the "bring me a spam from the laundry_desk" split.
# ---------------------------------------------------------------------------

T0 = {
    "id": "t0", "desc": "Get a spam from the laundry_desk", "object": "spam",
    "location": "laundry_desk", "depends_on": [],
    "preconditions": [], "postconditions": ["held(spam)"],
}
T1 = {
    "id": "t1", "desc": "Bring the spam to me", "object": "spam", "location": "",
    "depends_on": ["t0"],
    "preconditions": ["held(spam)"], "postconditions": ["delivered(spam,me)"],
}

# The move-plant DAG: t0 grabs the plant, t1 (independently) goes to the
# balcony, t2 places the plant on the balcony (depends on both).
MOVE_T0 = {
    "id": "t0", "desc": "Get the plant from the kitchen", "object": "plant",
    "location": "kitchen", "depends_on": [],
    "preconditions": [], "postconditions": ["held(plant)"],
}
MOVE_T1 = {
    "id": "t1", "desc": "Go to the balcony", "object": "", "location": "balcony",
    "depends_on": [],
    "preconditions": [], "postconditions": ["at_robot(balcony)"],
}
MOVE_T2 = {
    "id": "t2", "desc": "Place the plant on the balcony", "object": "plant",
    "location": "balcony", "depends_on": ["t0", "t1"],
    "preconditions": ["held(plant)", "at_robot(balcony)"],
    "postconditions": ["placed(plant,balcony)"],
}


# ---------------------------------------------------------------------------
# 1. Prompt: contract line + owned-by-sibling lines.
# ---------------------------------------------------------------------------

def test_prompt_without_contract_has_no_contract_block():
    prompt = _build_lower_layer_user_prompt(
        "bring me a spam from the laundry_desk", T0["desc"], "spam", "laundry_desk",
        [], [],
    )
    assert "fact contract" not in prompt
    assert "must establish" not in prompt


def test_prompt_with_contract_names_own_postcondition_and_sibling_owned_fact():
    prompt = _build_lower_layer_user_prompt(
        "bring me a spam from the laundry_desk", T0["desc"], "spam", "laundry_desk",
        [], [], contract=T0, all_targets=[T0, T1],
    )
    assert "This target's fact contract" in prompt
    # t0 must establish held(spam) — its own postcondition.
    must_establish_line = next(
        line for line in prompt.splitlines() if "must establish" in line
    )
    assert "held(spam)" in must_establish_line
    # delivered(spam,me) belongs to T1, and must be flagged as off-limits.
    # Labelled by the target's OWN id ("t1"), never by enumerate() position
    # over all_targets -- see IMPORTANT-2.
    owned_line = next(line for line in prompt.splitlines() if "owned by t1" in line)
    assert "delivered(spam,me)" in owned_line


def test_prompt_contract_for_t1_shows_requires_and_owned_by_ancestor():
    prompt = _build_lower_layer_user_prompt(
        "bring me a spam from the laundry_desk", T1["desc"], "spam", "",
        [T0], [], contract=T1, all_targets=[T0, T1],
    )
    requires_line = next(
        line for line in prompt.splitlines() if "requires" in line and "must establish" not in line
    )
    assert "held(spam)" in requires_line
    must_establish_line = next(
        line for line in prompt.splitlines() if "must establish" in line
    )
    assert "delivered(spam,me)" in must_establish_line
    # The prompt's "owned by" block still lists every OTHER target (ancestors
    # included) -- only the deterministic guard (_drop_foreign_contract_steps)
    # is restricted to successors (IMPORTANT-1). t0 is an ancestor (already
    # executed) but its held(spam) postcondition is still NOT one of t1's own
    # postconditions, so it is flagged "owned by t0" (labelled by id, per
    # IMPORTANT-2): the prompt still asks the model not to re-emit it, but the
    # reworded "requires" line (MINOR-1) makes clear this is advisory only --
    # the model MAY re-establish it if the failure reason says it is unmet.
    owned_line = next(line for line in prompt.splitlines() if "owned by t0" in line)
    assert "held(spam)" in owned_line


def test_prompt_requires_line_is_advisory_not_absolute():
    # MINOR-1: the old "do NOT re-establish" wording flatly contradicted a
    # replan whose failure_reason is "precondition unmet: held(spam)" (the
    # guard, before IMPORTANT-1, would then strip the very grasp step the
    # retry needs). The reworded line makes the "requires" state conditional
    # on the failure reason instead of an unconditional prohibition.
    prompt = _build_lower_layer_user_prompt(
        "bring me a spam from the laundry_desk", T1["desc"], "spam", "",
        [T0], [], contract=T1, all_targets=[T0, T1],
    )
    requires_line = next(
        line for line in prompt.splitlines() if "requires" in line and "must establish" not in line
    )
    assert "do NOT re-establish" not in requires_line
    assert "normally established by earlier targets" in requires_line
    assert "re-establish only if the failure reason below says it is unmet" in requires_line


def test_prompt_plan_only_sentence_renders_without_sibling_facts():
    # MINOR-6: a single-target command has an empty owned-by block (no other
    # targets in the slot), but the "Plan ONLY the steps that establish..."
    # sentence must still render -- it is what stops t0-style over-planning
    # even when there happen to be no siblings to list.
    solo = {
        "id": "t0", "desc": "Bring the spam to me", "object": "spam", "location": "",
        "depends_on": [], "preconditions": [], "postconditions": ["delivered(spam,me)"],
    }
    prompt = _build_lower_layer_user_prompt(
        "bring me a spam", solo["desc"], "spam", "",
        [], [], contract=solo, all_targets=[solo],
    )
    assert "Plan ONLY the steps that establish the \"must establish\" facts" in prompt
    assert "owned by" not in prompt


# ---------------------------------------------------------------------------
# 2. Deterministic guard: _drop_foreign_contract_steps.
# ---------------------------------------------------------------------------

def test_guard_drops_deliver_from_t0_plan():
    plan = [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(plan, T0, [T0, T1])
    assert [s["action"] for s in kept] == ["goto", "grasp"]
    assert dropped == ["contract:deliver->delivered(spam,me)"]


def test_guard_keeps_t1_deliver_plan_untouched():
    plan = [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]
    kept, dropped = _drop_foreign_contract_steps(plan, T1, [T0, T1])
    assert kept == plan
    assert dropped == []


def test_guard_t1_replan_keeps_grasp_for_ancestors_precondition():
    # IMPORTANT-1 regression: the foreign-fact set is ONLY targets AFTER
    # `target` in list order (all_targets[pos+1:]), never ancestors. t0
    # precedes t1, so t0's held(spam) postcondition must NOT be treated as
    # foreign when t1 replans -- e.g. after the precondition gate rejects
    # with "precondition unmet: held(spam)" (t0 was skipped or the object
    # was dropped) and the LLM correctly re-grasps as part of recovery.
    # Before this fix the guard dropped `grasp` here, leaving a plan that
    # could never satisfy the gate again.
    plan = [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "find_object", "params": {"object": "spam", "location": "laundry_desk"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(plan, T1, [T0, T1])
    assert [s["action"] for s in kept] == ["goto", "find_object", "grasp", "deliver"]
    assert dropped == []


def test_guard_returns_plan_unchanged_when_no_target_context():
    plan = [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]
    kept, dropped = _drop_foreign_contract_steps(plan, None, [T0, T1])
    assert kept == plan and dropped == []
    kept, dropped = _drop_foreign_contract_steps(plan, T1, None)
    assert kept == plan and dropped == []
    kept, dropped = _drop_foreign_contract_steps(plan, T1, [])
    assert kept == plan and dropped == []


def test_guard_move_plant_t0_keeps_everything_own_location_not_owned():
    # t0's own goto(kitchen) establishes at_robot(kitchen), which is NOT the
    # at_robot(balcony) fact t1 owns -- full facts are compared, not bare
    # predicates, so nothing is dropped.
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "plant", "location": "kitchen"}},
        {"action": "grasp", "params": {"object": "plant"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(plan, MOVE_T0, [MOVE_T0, MOVE_T1, MOVE_T2])
    assert kept == plan
    assert dropped == []


def test_guard_move_plant_t2_keeps_goto_balcony_since_t1_is_an_ancestor():
    # IMPORTANT-1 (flipped from the original expectation): t2's plan is
    # [goto balcony, place]. goto's established fact IS at_robot(balcony),
    # which IS a postcondition of t1 -- but t1 is a dependency ANCESTOR of
    # t2 (comes before it in list order, all_targets[pos+1:] for t2 is
    # empty), never a successor, so it is no longer in the foreign set and
    # goto is KEPT. Duplication of an ancestor's work is the "requires" /
    # "Prior targets ... do NOT repeat their work" prompt text's job (and,
    # if the model still gets it wrong, validate_plan's prior-plan seeding),
    # not this guard's -- the guard exists to catch a plan reaching AHEAD
    # into a fact only a later target should establish (e.g. the "bring me
    # a spam" double-delivery), which is a forward-looking failure mode.
    plan = [
        {"action": "goto", "params": {"location": "balcony"}},
        {"action": "place", "params": {"object": "plant", "location": "balcony"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(plan, MOVE_T2, [MOVE_T0, MOVE_T1, MOVE_T2])
    assert kept == plan
    assert dropped == []


def test_guard_only_establishes_counts_not_self_establishes():
    # `place`'s self_establishes(at_robot via location) never triggers a drop
    # by itself -- established_facts() only walks `establishes` templates.
    # Built with an explicit SUCCESSOR (not t2, which has none) that owns
    # at_robot(balcony), so this actually exercises the establishes-vs-
    # self_establishes distinction under the successors-only guard (a lone
    # place step establishing placed(plant,balcony), its OWN postcondition,
    # must be kept even though it also self-navigates through a location the
    # successor happens to own).
    self_nav_t0 = {
        "id": "t0", "desc": "Place the plant on the balcony", "object": "plant",
        "location": "balcony", "depends_on": [],
        "preconditions": [], "postconditions": ["placed(plant,balcony)"],
    }
    self_nav_t1 = {
        "id": "t1", "desc": "Go to the balcony", "object": "", "location": "balcony",
        "depends_on": [], "preconditions": [], "postconditions": ["at_robot(balcony)"],
    }
    plan = [{"action": "place", "params": {"object": "plant", "location": "balcony"}}]
    kept, dropped = _drop_foreign_contract_steps(plan, self_nav_t0, [self_nav_t0, self_nav_t1])
    assert kept == plan
    assert dropped == []


# ---------------------------------------------------------------------------
# 3. Integration: plan_target drops the LLM's foreign-contract step and
#    prints/traces it as `dropped`.
# ---------------------------------------------------------------------------

class _StubClient:
    pass


def test_plan_target_drops_llm_deliver_step_and_stores_reduced_plan(monkeypatch, capsys):
    p = GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {
        "command": "bring me a spam from the laundry_desk",
        "targets": [T0, T1],
    }
    llm_plan = {"plan": [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]}
    monkeypatch.setattr(planner_module, "_call_llm", lambda *a, **k: (llm_plan, None))
    monkeypatch.setattr(planner_module, "validate_plan", lambda *a, **k: (True, ""))
    monkeypatch.setattr(planner_module, "validate_plan_modifications", lambda *a, **k: (True, ""))

    p.plan_target(
        0, 0, T0["desc"], command="bring me a spam from the laundry_desk",
        target_obj="spam", target_loc="laundry_desk", target=T0, all_targets=[T0, T1],
    )

    assert [s["action"] for s in p.get_action_plan(0, 0)] == ["goto", "grasp"]
    out = capsys.readouterr().out
    assert "dropped ['contract:deliver->delivered(spam,me)']" in out


def test_replace_target_plan_keeps_original_plan_when_guard_empties_it(monkeypatch, capsys):
    # MINOR-7: a supervisor replacement plan for t0 that ONLY establishes
    # t1's fact (a premature deliver) would be dropped down to []. Installing
    # an empty plan is never acceptable -- keep the original (unfiltered)
    # plan instead and log a warning.
    p = GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {
        "command": "bring me a spam from the laundry_desk",
        "targets": [T0, T1],
    }
    plan = [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]

    p.replace_target_plan(0, 0, plan, reason="supervisor global replan")

    assert [s["action"] for s in p.get_action_plan(0, 0)] == ["deliver"]
    out = capsys.readouterr().out
    assert "contract-boundary guard emptied the plan" in out
