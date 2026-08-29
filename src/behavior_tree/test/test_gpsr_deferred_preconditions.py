from __future__ import annotations

import py_trees
import pytest
from py_trees.common import Access, Status

from behavior_tree.GPSR.orchestrator import (
    BtNode_TargetPreconditionCheck, BtNode_TargetPostconditionCheck, _TARGET_GATE_EVIDENCE_KEYS,
)
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
