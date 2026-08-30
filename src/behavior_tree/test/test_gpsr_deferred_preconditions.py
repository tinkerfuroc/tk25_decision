from __future__ import annotations

import py_trees
import pytest
from py_trees.common import Access, Status

from behavior_tree.GPSR.orchestrator import (
    BtNode_TargetPreconditionCheck, BtNode_TargetPostconditionCheck, _TARGET_GATE_EVIDENCE_KEYS,
)
from behavior_tree.GPSR.planner import _normalise_targets
from behavior_tree.GPSR.small_trees import bb_keys


@pytest.fixture(autouse=True)
def _clean_blackboard():
    # The py_trees blackboard is process-global; other gate test modules assume
    # they start on a clean one, so never leak evidence (e.g. LAST_NAV_LOCATION)
    # written here into later-collected files.
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _writer():
    py_trees.blackboard.Blackboard.clear()
    w = py_trees.blackboard.Client(name="w")
    for key in (bb_keys.FACTS, bb_keys.DEFERRED_PRECONDITIONS, bb_keys.LAST_NAV_LOCATION):
        w.register_key(key, access=Access.WRITE)
        w.register_key(key, access=Access.READ)
    for _, key in _TARGET_GATE_EVIDENCE_KEYS:
        w.register_key(key, access=Access.WRITE)
    w.set(bb_keys.FACTS, [], overwrite=True)
    w.set(bb_keys.DEFERRED_PRECONDITIONS, [], overwrite=True)
    return w


PLACE_PLAN = [{"action": "place", "params": {"location": "kitchen_table"}}]


def _tick(node):
    node.setup()
    node.initialise()
    return node.update()


def test_self_satisfied_precondition_is_deferred_not_failed():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["at_robot(kitchen_table)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == ["at_robot(kitchen_table)"]
    assert "deferred" in gate.feedback_message


def test_non_self_satisfied_precondition_still_checked():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["held(coke)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.FAILURE
    assert "precondition unmet: held(coke)" in gate.feedback_message
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == []


def test_mixed_preconditions_defer_only_self_satisfied():
    w = _writer()
    w.set(bb_keys.FACTS, ["held(coke)"], overwrite=True)
    gate = BtNode_TargetPreconditionCheck("pre", ["held(coke)", "at_robot(kitchen_table)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == ["at_robot(kitchen_table)"]


def test_deferred_precondition_at_a_different_location_is_not_deferred():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["at_robot(office)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.FAILURE


GRASP_PLAN = [{"action": "grasp", "params": {"object": "coke"}}]
FIND_OBJECT_PLAN = [{"action": "find_object", "params": {"object": "coke"}}]


def test_precondition_predicate_established_by_own_plan_is_deferred_held():
    # J1(b): "held" is in grasp's `establishes` -- a held() precondition is
    # deferred at PREDICATE level (not just the self_establishes/at_robot
    # case), regardless of whether the object matches.
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["held(coke)"], 0, action_plan=GRASP_PLAN)
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == ["held(coke)"]


def test_precondition_predicate_established_by_own_plan_is_deferred_object_seen():
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["object_seen(coke)"], 0, action_plan=FIND_OBJECT_PLAN)
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == ["object_seen(coke)"]


GOTO_OFFICE_PLAN = [{"action": "goto", "params": {"location": "office"}}]


def test_at_robot_precondition_from_a_goto_to_a_different_location_is_checked_at_entry():
    # H-2 (round-3 fix review): goto's `establishes` tuple names at_robot
    # generically -- _plan_established_predicates must NOT put at_robot in
    # the general predicate-deferral set, or ANY plan with a goto (to any
    # location) would defer EVERY at_robot precondition, even one the plan's
    # own goto doesn't establish. Only the exact self_establishes match may
    # defer at_robot. Here the plan gotos to "office" but the precondition
    # is at_robot(kitchen_table), already true from an earlier target --
    # it must be CHECKED (not deferred) and pass via the ledger.
    w = _writer()
    w.set(bb_keys.FACTS, ["at_robot(kitchen_table)"], overwrite=True)
    gate = BtNode_TargetPreconditionCheck(
        "pre", ["at_robot(kitchen_table)"], 0, action_plan=GOTO_OFFICE_PLAN,
    )
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == []


def test_precondition_with_no_establisher_in_plan_still_fails_at_entry():
    # PLACE_PLAN's only action (place) does not establish held() -- this
    # precondition has no establisher in the target's own plan and must be
    # checked (and fail) at entry, not deferred.
    w = _writer()
    gate = BtNode_TargetPreconditionCheck("pre", ["held(coke)"], 0, action_plan=PLACE_PLAN)
    assert _tick(gate) is Status.FAILURE
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == []


def _post(completed):
    return BtNode_TargetPostconditionCheck(
        "post", [], 0, PLACE_PLAN, target_object="coke", completed_steps=completed,
    )


def test_postcondition_gate_verifies_deferred_with_nav_evidence():
    w = _writer()
    w.set(bb_keys.DEFERRED_PRECONDITIONS, ["at_robot(kitchen_table)"], overwrite=True)
    w.set(bb_keys.LAST_NAV_LOCATION, "kitchen_table", overwrite=True)
    gate = _post([])
    assert _tick(gate) is Status.SUCCESS
    assert "at_robot(kitchen_table)" in w.get(bb_keys.FACTS)


def test_postcondition_gate_fails_deferred_without_nav_evidence():
    w = _writer()
    w.set(bb_keys.DEFERRED_PRECONDITIONS, ["at_robot(kitchen_table)"], overwrite=True)
    w.set(bb_keys.LAST_NAV_LOCATION, "office", overwrite=True)
    gate = _post([])
    assert _tick(gate) is Status.FAILURE
    assert "at_robot(kitchen_table)" in gate.feedback_message


def test_h3_postcondition_gate_passes_own_held_from_a_completed_grasp():
    # H-3 (round-3 fix review) gate test: post [held(x), placed(x,t)], plan
    # [goto, place], completed [grasp(x)] -> SUCCESS. Before the fix,
    # `plan_target` never threaded `completed_steps` into
    # `build_target_subtree`, so this target's own `held(x)` postcondition
    # (J2: an OWN postcondition cannot use the established-fact ledger
    # shortcut) had no evidence at all after a replan -- it stayed UNKNOWN
    # forever and the target could never pass its own post gate.
    w = _writer()
    plan = [
        {"action": "goto", "params": {"location": "t"}},
        {"action": "place", "params": {"location": "t"}},
    ]
    completed = [{"action": "grasp", "params": {"object": "x"}}]
    gate = BtNode_TargetPostconditionCheck(
        "post", ["held(x)", "placed(x,t)"], 0, plan,
        target_object="x", completed_steps=completed,
    )
    assert _tick(gate) is Status.SUCCESS
    # M-5 documents that placed(x,t)'s commit retracts held(x) from the
    # ledger (you can't be holding something you just placed) -- that is the
    # CORRECT final state, not asserted here; what H-3 fixes is that the
    # target reaches SUCCESS at all.
    assert "placed(x,t)" in w.get(bb_keys.FACTS)


def test_j1_three_target_command_drops_retracted_precondition_end_to_end():
    # J1: "take a spam from the laundry_desk, put it on the kitchen_table,
    # then bring it to me" -- t2's held(spam) precondition was established by
    # t0 but retracted by t1's placed(spam,kitchen_table); the split must not
    # carry it forward as an acceptable precondition (the plan has to
    # re-establish it, e.g. by re-grasping).
    targets = _normalise_targets([
        {"desc": "Get a spam from the laundry_desk", "postconditions": ["held(spam)"]},
        {
            "desc": "Put the spam on the kitchen_table",
            "preconditions": ["held(spam)"],
            "postconditions": ["placed(spam,kitchen_table)"],
        },
        {
            "desc": "Bring the spam to me",
            "preconditions": ["held(spam)"],
            "postconditions": ["delivered(spam,me)"],
        },
    ])
    assert targets[1]["preconditions"] == ["held(spam)"]
    assert targets[2]["preconditions"] == []

    # The target-gate harness: t2's (now-empty) precondition list has
    # nothing left to check, so a plan that does NOT re-grasp cannot fake
    # its way past the gate -- there is no leftover held(spam) fact to lean
    # on.
    deliver_plan = [{"action": "deliver", "params": {"object": "spam", "recipient": "me"}}]
    w = _writer()
    gate = BtNode_TargetPreconditionCheck(
        "pre2", targets[2]["preconditions"], 2, action_plan=deliver_plan,
    )
    assert _tick(gate) is Status.SUCCESS
    assert w.get(bb_keys.DEFERRED_PRECONDITIONS) == []
