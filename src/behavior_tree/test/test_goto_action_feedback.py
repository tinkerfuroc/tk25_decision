"""BtNode_GotoAction's success feedback must name what it reached.

Node feedback feeds a run's contact sheets — "goal accepted :)" (the RUNNING
message from ActionHandler.regular_update) says nothing about *where* the
robot was heading, which made a battery run's logs hard to skim after the
fact (battery run s2026-002, 2026-08-28 family of fixes). On SUCCESS,
process_result() now reports "reached '<label>'" using the resolved goal
pose's position when available, falling back to the blackboard key the goal
came from (e.g. in mock mode, where send_goal() never stashes a pose copy).

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""

from __future__ import annotations

import sys
from pathlib import Path
from unittest import mock

SRC = Path(__file__).resolve().parents[1]
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import action_msgs.msg as action_msgs  # noqa: E402
import py_trees  # noqa: E402
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion  # noqa: E402
from std_msgs.msg import Header  # noqa: E402


def _make_pose(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _make_goto_node(key, goal, result_status):
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction

    node = BtNode_GotoAction.__new__(BtNode_GotoAction)
    node._key = key
    node.blackboard = mock.Mock()
    node.blackboard.goal = goal
    node.result_status = result_status
    node.feedback_message = ""
    return node


def test_success_feedback_reports_resolved_pose():
    goal = _make_pose(1.2, 3.4)
    node = _make_goto_node("gpsr/target_pose", goal, action_msgs.GoalStatus.STATUS_SUCCEEDED)

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    assert node.feedback_message == "reached '1.20, 3.40'"


def test_success_feedback_falls_back_to_bb_key_without_a_pose():
    # Mock mode never populates a pose on the goal blackboard key; the
    # message should still name *something* meaningful (the key it read).
    node = _make_goto_node("gpsr/target_pose", None, action_msgs.GoalStatus.STATUS_SUCCEEDED)

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    assert node.feedback_message == "reached 'gpsr/target_pose'"


def test_aborted_feedback_is_unchanged():
    node = _make_goto_node("gpsr/target_pose", None, action_msgs.GoalStatus.STATUS_ABORTED)

    status = node.process_result()

    assert status == py_trees.common.Status.FAILURE
    assert node.feedback_message == "action aborted"
