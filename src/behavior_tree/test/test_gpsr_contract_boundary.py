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
    _drop_dangling_goto_before_self_nav,
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

# The take-to-table-then-bring-to-me DAG (M1, Task D review): t0 grasps the
# spam, t1 places it on the table (which RETRACTS t0's held(spam)), t2
# delivers it to me -- and so must legitimately re-grasp.
TABLE_T0 = {
    "id": "t0", "desc": "Get a spam from the laundry_desk", "object": "spam",
    "location": "laundry_desk", "depends_on": [],
    "preconditions": [], "postconditions": ["held(spam)"],
}
TABLE_T1 = {
    "id": "t1", "desc": "Put the spam on the table", "object": "spam",
    "location": "table", "depends_on": ["t0"],
    "preconditions": ["held(spam)"], "postconditions": ["placed(spam,table)"],
}
TABLE_T2 = {
    "id": "t2", "desc": "Bring the spam to me", "object": "spam", "location": "",
    "depends_on": ["t1"],
    "preconditions": ["placed(spam,table)"], "postconditions": ["delivered(spam,me)"],
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
    # `failure_msg` is None here -- this is the INITIAL plan (D2, Task D
    # brief) -- so the "owned by" block lists every OTHER target, ancestors
    # included: `_build_lower_layer_user_prompt` passes
    # `include_ancestors=(failure_msg is None)` into `_render_contract_block`.
    # t0 is an ancestor (already executed) but its held(spam) postcondition
    # is still NOT one of t1's own postconditions, so it is flagged "owned by
    # t0" (labelled by id, per IMPORTANT-2): the prompt still asks the model
    # not to re-emit it, but the reworded "requires" line (MINOR-1) makes
    # clear this is advisory only -- the model MAY re-establish it if the
    # failure reason says it is unmet. A REPLAN (failure_msg set) drops this
    # ancestor line -- see test_prompt_replan_owned_by_block_excludes_ancestors.
    owned_line = next(line for line in prompt.splitlines() if "owned by t0" in line)
    assert "held(spam)" in owned_line


def test_prompt_replan_owned_by_block_excludes_ancestors():
    # D2: on a REPLAN (failure_msg set), _drop_foreign_contract_steps only
    # treats SUCCESSORS as foreign (an ancestor's fact may legitimately need
    # re-establishing during recovery) -- the advisory "owned by" prompt
    # block now follows the same rule, so it must NOT list t0 (t1's only
    # ancestor) as owning held(spam) here, even though it still would on the
    # initial plan (see test_prompt_contract_for_t1_shows_requires_and_owned_by_ancestor).
    prompt = _build_lower_layer_user_prompt(
        "bring me a spam from the laundry_desk", T1["desc"], "spam", "",
        [T0], [], "precondition unmet: held(spam) (invalid)",
        contract=T1, all_targets=[T0, T1],
    )
    assert not any("owned by t0" in line for line in prompt.splitlines())


def test_prompt_never_lists_a_non_dependency_preceding_target_as_owned():
    # I-1/I-2: t0 precedes t1 in list order but t1 does NOT declare t0 as a
    # dependency -- t0 is neither a SUCCESSOR nor a declared-dependency
    # ANCESTOR of t1, so _foreign_facts (shared by the guard and this
    # render) never attributes its fact to t1's "owned by" block, on
    # EITHER the initial plan or a replan. This is the render-side mirror
    # of test_guard_keeps_goto_when_ancestor_is_not_a_declared_dependency
    # (which pins the same invariant for the deterministic guard).
    initial_prompt = _build_lower_layer_user_prompt(
        "find the spam in the kitchen", KITCHEN_T1["desc"], "spam", "kitchen",
        [KITCHEN_T0], [],
        contract=KITCHEN_T1, all_targets=[KITCHEN_T0, KITCHEN_T1],
    )
    assert not any("owned by t0" in line for line in initial_prompt.splitlines())
    replan_prompt = _build_lower_layer_user_prompt(
        "find the spam in the kitchen", KITCHEN_T1["desc"], "spam", "kitchen",
        [KITCHEN_T0], [], "some failure",
        contract=KITCHEN_T1, all_targets=[KITCHEN_T0, KITCHEN_T1],
    )
    assert not any("owned by t0" in line for line in replan_prompt.splitlines())


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


def test_prompt_table_dag_does_not_list_retracted_ancestor_fact_as_owned():
    # I-2: t2's ancestors are t0 (held(spam)) and t1 (placed(spam,table),
    # which RETRACTS held(spam) via apply_fact_transitions). The guard
    # already accounts for this (M1, Task D review) -- the prompt's "owned
    # by" block must now agree: t2's initial prompt must NOT list
    # held(spam) as "owned by t0" (it no longer holds by the time t2 runs),
    # only placed(spam,table) as "owned by t1".
    prompt = _build_lower_layer_user_prompt(
        "take the spam to the table then bring it to me", TABLE_T2["desc"],
        "spam", "", [TABLE_T0, TABLE_T1], [],
        contract=TABLE_T2, all_targets=[TABLE_T0, TABLE_T1, TABLE_T2],
    )
    assert not any(
        "owned by t0" in line and "held(spam)" in line
        for line in prompt.splitlines()
    )
    owned_line = next(line for line in prompt.splitlines() if "owned by t1" in line)
    assert "placed(spam,table)" in owned_line


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


# The "find the spam in the kitchen" DAG (I-1, round-2 review): t1 does NOT
# declare t0 as a dependency even though t0 precedes it in list order and
# happens to establish at_robot(kitchen) -- the ancestor set for the guard
# AND the prompt must come from _dependency_ancestor_targets (declared
# depends_on only), never bare list position, or the guard wrongly assumes
# t0's fact is guaranteed for t1 while validate_plan's prior_plan seeding
# (also dependency-keyed) never agrees -- an unrecoverable initial plan.
KITCHEN_T0 = {
    "id": "t0", "desc": "Go to the kitchen", "object": "", "location": "kitchen",
    "depends_on": [], "preconditions": [], "postconditions": ["at_robot(kitchen)"],
}
KITCHEN_T1 = {
    "id": "t1", "desc": "Find the spam in the kitchen", "object": "spam",
    "location": "kitchen", "depends_on": [],
    "preconditions": [], "postconditions": ["object_seen(spam)"],
}


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


def test_guard_t1_initial_plan_drops_grasp_for_ancestors_fact_when_asked():
    # D2 (Task D brief) regression: t1 "Bring the spam to me" (depends_on t0
    # held(spam)) was planned as [goto, grasp, deliver] -- the whole of t0
    # again -- and executed twice. `include_ancestors=True` is what
    # `plan_target` now passes ONLY on the INITIAL plan (no failure_reason):
    # the DAG + precondition gate already guarantee t0's held(spam) holds, so
    # a fresh grasp is pure duplication, safe to drop. The default
    # (`include_ancestors=False`, exercised by
    # `test_guard_t1_replan_keeps_grasp_for_ancestors_precondition` above)
    # stays unchanged for the replan/recovery path.
    plan = [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(plan, T1, [T0, T1], include_ancestors=True)
    assert [s["action"] for s in kept] == ["goto", "deliver"]
    assert dropped == ["contract:grasp->held(spam)"]


def test_guard_ancestor_facts_retracted_by_a_later_ancestor_are_not_foreign():
    # M1 (Task D review): "take the spam to the table, then bring it to me".
    # t2's ancestors are t0 (held(spam)) and t1 (placed(spam,table), which
    # RETRACTS held(spam) for that object -- apply_fact_transitions drops
    # held(obj) once placed(obj,..)/delivered(obj,..) is established for it).
    # t2's INITIAL plan legitimately re-grasps (the spam is on the table, not
    # in the gripper); a plain union of ancestor postconditions would wrongly
    # treat held(spam) as still "guaranteed" (t0's fact, never retracted in a
    # naive union) and drop the grasp step, letting `deliver` run empty.
    plan = [
        {"action": "goto", "params": {"location": "table"}},
        {"action": "find_object", "params": {"object": "spam", "location": "table"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(
        plan, TABLE_T2, [TABLE_T0, TABLE_T1, TABLE_T2], include_ancestors=True,
    )
    assert [s["action"] for s in kept] == ["goto", "find_object", "grasp", "deliver"]
    assert dropped == []


def test_guard_keeps_goto_when_ancestor_is_not_a_declared_dependency():
    # I-1 regression: t0 precedes t1 in list order and its own postcondition
    # (at_robot(kitchen)) would be treated as a "guaranteed" ancestor fact by
    # a bare-list-position rule -- but t1 never declares t0 as a dependency
    # (depends_on: []), so the guard must NOT drop t1's own goto here. Before
    # the fix this dropped `goto`, leaving `find_object(location=kitchen)`
    # with no preceding goto -- validate_plan then rejects EVERY attempt
    # (nothing seeds at_robot(kitchen) via prior_plan either, since that is
    # also dependency-keyed), forcing a wasted fallback + replan cycle.
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_object", "params": {"object": "spam", "location": "kitchen"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(
        plan, KITCHEN_T1, [KITCHEN_T0, KITCHEN_T1], include_ancestors=True,
    )
    assert [s["action"] for s in kept] == ["goto", "find_object"]
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
# 2b. Deterministic cleanup: _drop_dangling_goto_before_self_nav (I-4).
# ---------------------------------------------------------------------------

def test_dangling_goto_dropped_when_immediately_before_self_navigator():
    # I-4 regression (run 000's t1 shape): the guard reduces
    # [goto, grasp, deliver] to [goto, deliver] (grasp establishes an
    # ancestor's held(spam)) -- but validate_plan's "no goto immediately
    # before a self-navigating action" rule then ALWAYS rejects [goto,
    # deliver] outright, burning a full LLM round-trip on the most common
    # template. This cleanup drops the now-dangling goto deterministically.
    plan = [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]
    kept, dropped = _drop_dangling_goto_before_self_nav(plan)
    assert kept == [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]
    assert dropped == ["contract:goto->dangling"]


def test_dangling_goto_cleanup_is_a_noop_when_goto_not_immediately_before_self_nav():
    plan = [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "find_object", "params": {"object": "spam"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]
    kept, dropped = _drop_dangling_goto_before_self_nav(plan)
    assert kept == plan
    assert dropped == []


def test_dangling_goto_cleanup_keeps_goto_before_a_non_self_navigator():
    plan = [
        {"action": "goto", "params": {"location": "kitchen"}},
        {"action": "find_person", "params": {"person": "sarah"}},
    ]
    kept, dropped = _drop_dangling_goto_before_self_nav(plan)
    assert kept == plan
    assert dropped == []


def test_dangling_goto_cleanup_empty_plan_is_a_noop():
    kept, dropped = _drop_dangling_goto_before_self_nav([])
    assert kept == []
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


def test_plan_target_initial_plan_drops_ancestors_grasp_step(monkeypatch, capsys):
    # D2/I-4 integration (round-2 review, run 000's t1 shape): t1's INITIAL
    # plan (no failure_reason) re-does t0's whole [goto, grasp, deliver] --
    # the ancestor's held(spam) is guaranteed by the DAG + precondition
    # gate, so `plan_target` drops the duplicated grasp on this path
    # (include_ancestors=True only when failure_reason is None), leaving
    # [goto, deliver]. UN-STUBBED validate_plan (I-4) then always rejects
    # [goto, deliver] outright (goto immediately before a self-navigating
    # deliver is redundant) -- so plan_target's dangling-goto cleanup must
    # ALSO drop that now-pointless goto, leaving a plan a real
    # validate_plan actually accepts: ["deliver"].
    p = GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    monkeypatch.setattr(planner_module, "KNOWN_LOCATIONS", {"laundry_desk": object()})
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
    monkeypatch.setattr(planner_module, "validate_plan_modifications", lambda *a, **k: (True, ""))

    p.plan_target(
        0, 1, T1["desc"], command="bring me a spam from the laundry_desk",
        target_obj="spam", target_loc="", target=T1, all_targets=[T0, T1],
    )

    assert [s["action"] for s in p.get_action_plan(0, 1)] == ["deliver"]
    out = capsys.readouterr().out
    assert "contract:grasp->held(spam)" in out
    assert "contract:goto->dangling" in out


def test_plan_target_replan_keeps_ancestors_grasp_step(monkeypatch, capsys):
    # D2: the SAME plan, but as a REPLAN (failure_reason set) -- e.g. the
    # precondition gate rejected with "precondition unmet: held(spam)"
    # because t0 was skipped. grasp must be kept for recovery
    # (include_ancestors=False whenever failure_reason is not None).
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
        0, 1, T1["desc"], command="bring me a spam from the laundry_desk",
        target_obj="spam", target_loc="", target=T1, all_targets=[T0, T1],
        failure_reason="precondition unmet: held(spam) (invalid)",
    )

    assert [s["action"] for s in p.get_action_plan(0, 1)] == ["goto", "grasp", "deliver"]


def test_replace_target_plan_keeps_ancestors_grasp_step(monkeypatch, capsys):
    # D2: the supervisor path (`replace_target_plan`) always passes
    # `include_ancestors=False` -- it never talks to the LLM and has no
    # initial-vs-replan distinction of its own, so it must keep behaving
    # exactly as before (ancestor facts are never dropped).
    p = GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {
        "command": "bring me a spam from the laundry_desk",
        "targets": [T0, T1],
    }
    plan = [
        {"action": "goto", "params": {"location": "laundry_desk"}},
        {"action": "grasp", "params": {"object": "spam"}},
        {"action": "deliver", "params": {"object": "spam", "recipient": "me"}},
    ]

    p.replace_target_plan(0, 1, plan, reason="supervisor global replan")

    assert [s["action"] for s in p.get_action_plan(0, 1)] == ["goto", "grasp", "deliver"]


def test_plan_target_initial_plan_attempt_two_still_lists_ancestors_as_owned(monkeypatch):
    # M2/N1 (Task D review): the prompt's "owned by" block must follow the
    # SAME include_ancestors rule as the _drop_foreign_contract_steps guard --
    # keyed on `plan_target`'s own `failure_reason` ARGUMENT, never on
    # `last_reason` (the retry-prompt text), which goes non-None as soon as
    # attempt 1 of the INITIAL plan is rejected for ANY reason (here, an
    # unrelated validator rejection) even though this is still the initial
    # plan (no `failure_reason` was ever passed to `plan_target`).
    p = GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "_new_client", lambda: _StubClient())
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {
        "command": "bring me a spam from the laundry_desk",
        "targets": [T0, T1],
    }
    llm_plan = {"plan": [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]}
    prompts = []

    def fake_call(client, system, user, temperature):
        prompts.append(user)
        return llm_plan, None

    monkeypatch.setattr(planner_module, "_call_llm", fake_call)
    validate_calls = {"n": 0}

    def fake_validate(*a, **k):
        validate_calls["n"] += 1
        if validate_calls["n"] == 1:
            return False, "some unrelated validator rejection"
        return True, ""

    monkeypatch.setattr(planner_module, "validate_plan", fake_validate)
    monkeypatch.setattr(planner_module, "validate_plan_modifications", lambda *a, **k: (True, ""))

    p.plan_target(
        0, 1, T1["desc"], command="bring me a spam from the laundry_desk",
        target_obj="spam", target_loc="", target=T1, all_targets=[T0, T1],
    )

    assert len(prompts) == 2
    assert any("owned by t0" in line for line in prompts[0].splitlines())
    # The regression: before the fix, attempt 2's prompt dropped this line
    # because `last_reason` (attempt 1's rejection text) was non-None.
    assert any("owned by t0" in line for line in prompts[1].splitlines())


def test_replace_target_plan_installs_the_guarded_empty_plan(monkeypatch, capsys):
    # M-1 (round-2 review): a supervisor replacement plan for t0 that ONLY
    # establishes t1's fact (a premature deliver) is dropped down to [] by
    # the contract-boundary guard -- exactly the premature-sibling-step
    # duplication the guard exists to remove. Re-installing the ORIGINAL
    # (unfiltered) plan here would re-create the double-delivery defect the
    # guard closes; install the guarded (empty) plan instead -- its
    # postcondition gate then fails cleanly (t0's own held(spam) is never
    # established) and the normal replan path takes over. Still logs the
    # same warning.
    p = GPSRPlanner(max_attempts=2)
    monkeypatch.setattr(p, "build_target_subtree", lambda *a, **k: py_trees.behaviours.Success("stub"))
    p._slot_context[0] = {
        "command": "bring me a spam from the laundry_desk",
        "targets": [T0, T1],
    }
    plan = [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]

    p.replace_target_plan(0, 0, plan, reason="supervisor global replan")

    assert p.get_action_plan(0, 0) == []
    out = capsys.readouterr().out
    assert "contract-boundary guard emptied the plan" in out
