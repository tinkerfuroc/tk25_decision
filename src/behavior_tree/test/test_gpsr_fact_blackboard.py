"""GPSR fact blackboard key contract."""

from behavior_tree.GPSR.small_trees import bb_keys


def test_facts_blackboard_key_is_canonical():
    assert bb_keys.FACTS == "gpsr/facts"
