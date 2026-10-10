"""approach_person must send the person as a person target (issue-2 2d).

The barrier check treats a target as a person only via `target_is_person` or
z <= 0.05; sending the raw 3-D centroid (z ~1 m) made the person's own
head/shoulders read as a wall between the robot and the person.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1, with the issue-2 overlay sourced
(tinker_nav_msgs must have GoToApproach.Goal.target_is_person).
"""
from geometry_msgs.msg import PointStamped

from behavior_tree.GPSR import small_trees
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach


def _approach_node(tree):
    return next(n for n in tree.iterate()
                if isinstance(n, BtNode_Approach) and n.name == "approach person")


def test_approach_person_targets_the_flat_nav_pose_as_a_person():
    node = _approach_node(small_trees.create_approach_person())
    assert node.bb_target_key == small_trees.bb_keys.PERSON_NAV_POSE
    assert node.target_is_person is True


def test_goal_carries_target_is_person():
    node = BtNode_Approach("a", bb_target_key="x", target_is_person=True)
    assert node._build_goal(PointStamped()).target_is_person is True


def test_default_approach_is_not_a_person():
    node = BtNode_Approach("a", bb_target_key="x")
    assert node._build_goal(PointStamped()).target_is_person is False
