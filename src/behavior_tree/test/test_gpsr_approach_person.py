import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.GPSR.small_trees import (  # noqa: E402
    PERSON_APPROACH_DESIRED_DISTANCE_M,
    PERSON_APPROACH_MAX_DISTANCE_M,
    PERSON_APPROACH_MIN_DISTANCE_M,
    PERSON_APPROACH_TIMEOUT_SEC,
    create_approach_person,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach  # noqa: E402


def _find_approach_node(root):
    for child in root.iterate():
        if isinstance(child, BtNode_Approach):
            return child
    return None


def test_approach_person_calls_real_navigation():
    root = create_approach_person()
    assert isinstance(root, py_trees.composites.Sequence)
    node = _find_approach_node(root)
    assert node is not None, "create_approach_person no longer drives BtNode_Approach"
    assert node.desired_distance == PERSON_APPROACH_DESIRED_DISTANCE_M
    assert node.min_distance == PERSON_APPROACH_MIN_DISTANCE_M
    assert node.max_distance == PERSON_APPROACH_MAX_DISTANCE_M
    assert node.timeout_sec == PERSON_APPROACH_TIMEOUT_SEC
    # NOTE: bb_target_key is NOT a readable attribute on BtNode_Approach —
    # ActionHandler.__init__ only uses it to wire a blackboard-client remap
    # to "goal"; asserting on it would raise AttributeError. The blackboard
    # wiring is exercised by the `gpsr-test-approach-person` ros2 run dev
    # runner (test_uncovered_actions.py), not this pure-Python test.


def test_approach_person_distance_bounds_satisfy_planner_guard():
    # approach_planner's solve rejects any goal where
    # not (min <= desired <= max) with STATUS_INVALID_REQUEST
    # (approach_planner/algorithm.py:184), which would disable its entire
    # attempt-2 costmap recompute. Pin the ordering so nobody re-breaks it
    # by "simplifying" the goal back to desired-only.
    assert (
        PERSON_APPROACH_MIN_DISTANCE_M
        <= PERSON_APPROACH_DESIRED_DISTANCE_M
        <= PERSON_APPROACH_MAX_DISTANCE_M
    )
    # 1.3 = approach_planner's tuned 1.0 m default + the requested 0.3 m.
    assert PERSON_APPROACH_DESIRED_DISTANCE_M == 1.3
    assert PERSON_APPROACH_TIMEOUT_SEC == 45.0
