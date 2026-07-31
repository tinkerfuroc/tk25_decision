import os
os.environ.setdefault("BT_MOCK_MODE", "true")
import py_trees  # noqa: E402
from behavior_tree.PickAndPlace import pick_and_place_rulebook as R  # noqa: E402


def test_root_is_memory_sequence_with_three_children():
    root = R.pickAndPlaceRulebook(place_policy="vlm")
    assert isinstance(root, py_trees.composites.Sequence)
    assert root.memory is True
    assert len(root.children) == 3  # constants, enter-arena, mission-parallel


def test_mission_is_success_on_one_parallel_with_deadline_and_sequence():
    root = R.pickAndPlaceRulebook()
    par = root.children[2]
    assert isinstance(par, py_trees.composites.Parallel)
    assert isinstance(par.policy, py_trees.common.ParallelPolicy.SuccessOnOne)
    guard, mission = par.children
    assert guard.__class__.__name__ == "BtNode_DeadlineGuard"
    assert isinstance(mission, py_trees.composites.Sequence)


def test_cleanup_loop_is_repeat_minus_one_over_unwrapped_pop():
    phase = R.phaseTableCleanup("vlm")
    repeat = phase.children[-1]
    assert isinstance(repeat, py_trees.decorators.Repeat)
    body = repeat.decorated
    # PopWorkItem must be UNWRAPPED (the only node allowed to FAIL the body).
    assert body.children[0].__class__.__name__ == "BtNode_PopWorkItem"
    assert isinstance(body.children[1], py_trees.decorators.FailureIsSuccess)


def test_breakfast_table_is_the_frozen_four():
    assert [row[0] for row in R.BREAKFAST] == [
        "bowl",
        "spoon",
        "cornflakes",
        "milk",
    ]
