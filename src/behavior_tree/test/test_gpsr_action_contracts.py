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
