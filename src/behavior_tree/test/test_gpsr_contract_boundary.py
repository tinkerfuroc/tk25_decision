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
    owned_line = next(line for line in prompt.splitlines() if "owned by T1" in line)
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
    # "Other targets" is not just successors -- t0 is an ancestor (already
    # executed) but its held(spam) postcondition is still NOT one of t1's own
    # postconditions, so it is flagged "owned by T0": never re-grasp it.
    owned_line = next(line for line in prompt.splitlines() if "owned by T0" in line)
    assert "held(spam)" in owned_line


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


def test_guard_move_plant_t2_drops_goto_balcony_because_t1_owns_it():
    # t2's plan is [goto balcony, place]. goto's established fact IS
    # at_robot(balcony), which is NOT one of t2's own postconditions
    # (placed(plant,balcony) only) and IS a postcondition of t1 -> dropped
    # by the same rule as the deliver case above.
    #
    # This is safe because `place` self-navigates (self_establishes at_robot
    # via its own `location` param, per action_contracts.py) -- it does not
    # need a preceding goto to reach at_robot(balcony) itself. So the
    # reduced plan [place] still establishes t2's own postcondition.
    plan = [
        {"action": "goto", "params": {"location": "balcony"}},
        {"action": "place", "params": {"object": "plant", "location": "balcony"}},
    ]
    kept, dropped = _drop_foreign_contract_steps(plan, MOVE_T2, [MOVE_T0, MOVE_T1, MOVE_T2])
    assert [s["action"] for s in kept] == ["place"]
    assert dropped == ["contract:goto->at_robot(balcony)"]


def test_guard_only_establishes_counts_not_self_establishes():
    # `place`'s self_establishes(at_robot via location) never triggers a drop
    # by itself -- established_facts() only walks `establishes` templates, so
    # a lone place step establishing placed(plant,balcony) (t2's OWN
    # postcondition) is always kept even though it also self-navigates
    # through a location another target (t1) happens to own.
    plan = [{"action": "place", "params": {"object": "plant", "location": "balcony"}}]
    kept, dropped = _drop_foreign_contract_steps(plan, MOVE_T2, [MOVE_T0, MOVE_T1, MOVE_T2])
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
