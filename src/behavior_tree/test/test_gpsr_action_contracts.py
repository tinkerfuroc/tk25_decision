from __future__ import annotations

import pytest

from behavior_tree.GPSR import action_contracts as ac
from behavior_tree.GPSR.small_trees import ACTION_FACTORIES
from behavior_tree.GPSR.orchestrator import _TARGET_GATE_EVIDENCE_KEYS


def test_every_factory_has_a_contract_and_vice_versa():
    assert set(ac.ACTION_CONTRACTS) == set(ACTION_FACTORIES)


def test_records_are_known_evidence_names():
    known = {name for name, _ in _TARGET_GATE_EVIDENCE_KEYS}
    for contract in ac.ACTION_CONTRACTS.values():
        assert set(contract.records) <= known, contract.action


def test_contract_for_unknown_action_raises():
    with pytest.raises(KeyError):
        ac.contract_for("teleport")


def test_place_and_deliver_self_establish_at_robot():
    assert ac.contract_for("place").self_establishes == {"at_robot": "location"}
    assert ac.contract_for("deliver").self_establishes == {"at_robot": "recipient_location"}
    assert ac.contract_for("goto").self_establishes == {"at_robot": "location"}
    assert ac.contract_for("grasp").self_establishes == {}


def test_self_established_facts_from_step():
    assert ac.self_established_facts({"action": "place", "params": {"location": "kitchen table"}}) == ["at_robot(kitchen_table)"]
    assert ac.self_established_facts({"action": "deliver", "params": {"object": "coke", "recipient": "Susan"}}) == []
    assert ac.self_established_facts({"action": "grasp", "params": {"object": "coke"}}) == []


def test_self_navigating_destinations_excludes_goto():
    assert ac.self_navigating_destinations() == {
        "deliver": "recipient_location",
        "place": "location",
        "search_object": "location",
    }


def test_render_self_satisfied_rule_mentions_each_self_navigator():
    text = ac.render_self_satisfied_rule()
    for action in ("goto", "place", "deliver", "search_object"):
        assert action in text
    assert "at_robot" in text


import py_trees
from py_trees.common import Access

from behavior_tree.GPSR.orchestrator import materialise_params
from behavior_tree.GPSR.small_trees import bb_keys


@pytest.fixture(autouse=True)
def _clear_blackboard_after_materialise_tests():
    # _bb() below clears the shared py_trees Blackboard singleton on entry,
    # but leaves whatever it wrote behind on exit. Without a teardown clear
    # here, that state (LAST_NAV_LOCATION, TARGET_LOCATION, ...) leaks into
    # any test module that runs afterwards in the same pytest session and
    # doesn't clear the blackboard itself (e.g. test_gpsr_target_gates.py,
    # which alphabetically follows this file).
    yield
    py_trees.blackboard.Blackboard.clear()


def _bb():
    py_trees.blackboard.Blackboard.clear()
    bb = py_trees.blackboard.Client(name="t")
    for key in (bb_keys.LAST_NAV_LOCATION, bb_keys.TARGET_LOCATION, bb_keys.TARGET_POSE,
                bb_keys.GRASP_ASK_REFEREE, bb_keys.GRASP_REFEREE_LOCATION,
                bb_keys.GRASP_REFEREE_POSE, bb_keys.GRASP_REFEREE_IS_APPLIANCE,
                bb_keys.TARGET_OBJECT_NAME, bb_keys.TARGET_OBJECT_PROMPT,
                bb_keys.TARGET_PERSON_PROMPT, bb_keys.ANNOUNCE_TEXT, bb_keys.ASK_QUESTION,
                bb_keys.VLM_QUESTION, bb_keys.LLM_QUESTION, bb_keys.CURRENT_DYNLABEL):
        bb.register_key(key, access=Access.WRITE)
        bb.register_key(key, access=Access.READ)
    for key in (bb_keys.REPORT_INFO, bb_keys.START_POSE, bb_keys.DYNAMIC_LOCATIONS):
        bb.register_key(key, access=Access.READ)
    from behavior_tree.GPSR.orchestrator import SEARCH_POSE_KEYS
    for key in SEARCH_POSE_KEYS:
        bb.register_key(key, access=Access.WRITE)
    bb.set(bb_keys.LAST_NAV_LOCATION, "", overwrite=True)
    return bb


@pytest.mark.parametrize("action,params,expected", [
    ("goto", {"location": "kitchen_table"}, "kitchen_table"),
    ("place", {"location": "kitchen_table"}, "kitchen_table"),
    ("deliver", {"object": "coke", "recipient": "Susan", "recipient_location": "living_room"}, "living_room"),
    ("search_object", {"object": "coke", "location": "kitchen"}, "kitchen"),
])
def test_materialise_records_last_nav_for_self_navigators(action, params, expected):
    bb = _bb()
    materialise_params(bb, action, params)
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == expected


def test_materialise_does_not_touch_last_nav_for_grasp():
    bb = _bb()
    bb.set(bb_keys.LAST_NAV_LOCATION, "shelf", overwrite=True)
    materialise_params(bb, "grasp", {"object": "coke"})
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "shelf"


def test_materialise_deliver_without_location_leaves_last_nav():
    bb = _bb()
    bb.set(bb_keys.LAST_NAV_LOCATION, "office", overwrite=True)
    materialise_params(bb, "deliver", {"object": "coke", "recipient": "Susan"})
    assert bb.get(bb_keys.LAST_NAV_LOCATION) == "office"
