import py_trees
import pytest

from behavior_tree.GPSR.gpsr_runs.run_battery_commands import (
    COMMANDS,
    EXPECT,
    validate_target_contract,
)
from behavior_tree.GPSR.orchestrator import (
    BtNode_TargetPostconditionCheck,
    BtNode_TargetPreconditionCheck,
)


class FakePlanner:
    def __init__(self, subtrees):
        self.subtrees = subtrees

    def get_target_subtree(self, slot, index):
        return self.subtrees.get((slot, index))


def target(*, preconditions=None, postconditions=None, depends_on=None, target_id="t0"):
    return {
        "id": target_id,
        "desc": "move plant",
        "object": "plant",
        "location": "balcony",
        "depends_on": [] if depends_on is None else depends_on,
        "preconditions": [] if preconditions is None else preconditions,
        "postconditions": [] if postconditions is None else postconditions,
    }


def subtree(*, pre=0, post=0):
    root = py_trees.composites.Sequence("target", memory=True)
    for index in range(pre):
        root.add_child(BtNode_TargetPreconditionCheck(f"pre{index}", [], 0))
    root.add_child(py_trees.behaviours.Success("action"))
    for index in range(post):
        root.add_child(BtNode_TargetPostconditionCheck(f"post{index}", [], 0, []))
    return root


def violations(targets, tree):
    return validate_target_contract(targets, FakePlanner({(4, 0): tree}), 4)


def test_valid_target_with_both_gates_has_no_contract_violations():
    targets = [target(preconditions=["at_robot(kitchen)"], postconditions=["held(plant)"])]
    assert violations(targets, subtree(pre=1, post=1)) == []


def test_invalid_dag_is_reported():
    targets = [target(depends_on=["missing"])]
    result = violations(targets, subtree())
    assert any("unknown dependency" in violation for violation in result)


@pytest.mark.parametrize(
    ("source", "expected"),
    [
        ("unicorn(cup)", "unknown predicate"),
        ("placed(cup)", "expects 2 argument"),
        ("placed(cup,table) trailing", "no trailing text"),
    ],
)
def test_invalid_fact_sources_are_reported_with_source(source, expected):
    result = violations([target(preconditions=[source])], subtree(pre=1))
    assert any(expected in violation and source in violation for violation in result)


def test_missing_required_precondition_gate_is_reported():
    result = violations([target(preconditions=["held(plant)"])], subtree(pre=0))
    assert any("precondition gate" in violation and "exactly one" in violation for violation in result)


def test_missing_required_postcondition_gate_is_reported():
    result = violations([target(postconditions=["held(plant)"])], subtree(post=0))
    assert any("postcondition gate" in violation and "exactly one" in violation for violation in result)


def test_unexpected_pre_gate_when_conditions_empty_is_reported():
    result = violations([target()], subtree(pre=1))
    assert any("precondition gate" in violation and "zero" in violation for violation in result)


def test_unexpected_post_gate_when_conditions_empty_is_reported():
    result = violations([target()], subtree(post=1))
    assert any("postcondition gate" in violation and "zero" in violation for violation in result)


def test_duplicate_gates_are_reported():
    targets = [target(preconditions=["held(plant)"], postconditions=["held(plant)"])]
    result = violations(targets, subtree(pre=2, post=2))
    assert sum("precondition gate" in violation for violation in result) >= 1
    assert sum("postcondition gate" in violation for violation in result) >= 1
    assert any("exactly one" in violation for violation in result)


def test_missing_subtree_is_reported_without_exception():
    result = validate_target_contract([target()], FakePlanner({}), 4)
    assert any("subtree" in violation for violation in result)


def test_malformed_target_is_reported_without_exception():
    result = validate_target_contract(["not a target"], FakePlanner({(4, 0): subtree()}), 4)
    assert result
    assert any("not a mapping" in violation for violation in result)


def test_relocation_expectation_requires_three_validator_state_transition_targets():
    note, expected_count = EXPECT[2]
    assert expected_count == 3
    assert "validator" in note
    assert "state transition" in note


def test_dependency_expectation_uses_stable_target_id_not_numeric_index():
    note, _ = EXPECT[8]
    assert "depends_on=['t0']" in note
    assert "depends_on=0" not in note
    assert "then" in COMMANDS[8]
