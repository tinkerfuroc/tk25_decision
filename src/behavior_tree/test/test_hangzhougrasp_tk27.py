"""TK27 action contracts for hangzhougrasp: Pick, MoveToSavedPose, gripper open."""
from types import SimpleNamespace

import py_trees
import pytest
from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from tk27_manipulation_msgs.action import MoveToSavedPose, Pick

from behavior_tree.hangzhougrasp.tk27_nodes import (
    BtNode_Tk27GripperOpen, BtNode_Tk27Pick, BtNode_Tk27SavedPose,
)

S = py_trees.common.Status
PROMPT_KEY = "hg_target_label"


@pytest.fixture(autouse=True)
def clear_bb():
    py_trees.blackboard.Blackboard.storage.clear()
    yield
    py_trees.blackboard.Blackboard.storage.clear()


def _live(leaf):
    """Real (non-mock) leaf whose goals are captured instead of sent."""
    leaf.mock_mode = False
    goals = []
    leaf.send_goal_request = goals.append
    return goals


def _set_prompt(value):
    py_trees.blackboard.Blackboard.set("/" + PROMPT_KEY, value)


def _pick():
    return BtNode_Tk27Pick("Pick selected object", PROMPT_KEY,
                           "/manipulation/pick", "high")


def _finish(leaf, ros_status, result):
    leaf.result_status = ros_status
    leaf.result_message = SimpleNamespace(result=result)
    return leaf.process_result()


def test_pick_contract():
    leaf = _pick()
    goals = _live(leaf)
    _set_prompt(" coke ")
    leaf.send_goal()
    assert leaf.action_type is Pick
    assert leaf.action_name == "/manipulation/pick"
    assert len(goals) == 1 and isinstance(goals[0], Pick.Goal)
    assert goals[0].prompt == "coke"
    assert goals[0].plan_effort == "high"


@pytest.mark.parametrize("ros_status, success, status, expected", [
    (GoalStatus.STATUS_SUCCEEDED, True, 0, S.SUCCESS),
    (GoalStatus.STATUS_ABORTED, True, 0, S.FAILURE),
    (GoalStatus.STATUS_SUCCEEDED, False, 0, S.FAILURE),
    (GoalStatus.STATUS_SUCCEEDED, True, 2, S.FAILURE),
])
def test_pick_success_requires_all_three(ros_status, success, status, expected):
    leaf = _pick()
    result = Pick.Result(success=success, status=status,
                         error_msg="PLANNING_FAILED detail")
    assert _finish(leaf, ros_status, result) == expected
    if expected == S.FAILURE:
        assert "PLANNING_FAILED detail" in leaf.feedback_message


@pytest.mark.parametrize("prompt", ["  ", None])
def test_pick_empty_prompt_sends_no_goal(prompt):
    leaf = _pick()
    goals = _live(leaf)
    if prompt is not None:
        _set_prompt(prompt)
    leaf.send_goal()
    assert goals == []
    assert leaf.update() == S.FAILURE
    assert "prompt" in leaf.feedback_message


def test_saved_pose_contract():
    leaf = BtNode_Tk27SavedPose("Arm to table pose", "pick45",
                                "/manipulation/move_to_saved_pose")
    goals = _live(leaf)
    leaf.send_goal()
    assert leaf.action_type is MoveToSavedPose
    assert leaf.action_name == "/manipulation/move_to_saved_pose"
    assert isinstance(goals[0], MoveToSavedPose.Goal)
    assert goals[0].pose_name == "pick45"
    assert goals[0].avoid_collisions is False


@pytest.mark.parametrize("ros_status, success, status, expected", [
    (GoalStatus.STATUS_SUCCEEDED, True, 0, S.SUCCESS),
    (GoalStatus.STATUS_SUCCEEDED, False, 1, S.FAILURE),   # FILE_NOT_FOUND
    (GoalStatus.STATUS_SUCCEEDED, False, 9, S.FAILURE),   # POSTCONDITION_FAILED
    (GoalStatus.STATUS_ABORTED, True, 0, S.FAILURE),
])
def test_saved_pose_result(ros_status, success, status, expected):
    leaf = BtNode_Tk27SavedPose("Arm", "navigation", "/manipulation/move_to_saved_pose")
    result = MoveToSavedPose.Result(success=success, status=status,
                                    error_msg="pose file missing")
    assert _finish(leaf, ros_status, result) == expected
    if expected == S.FAILURE:
        assert "pose file missing" in leaf.feedback_message


def _gripper():
    return BtNode_Tk27GripperOpen("Open gripper before pick",
                                  "/xarm_gripper/gripper_action", 0.0, 0.025, 50.0)


def test_gripper_open_contract():
    leaf = _gripper()
    goals = _live(leaf)
    leaf.send_goal()
    assert leaf.action_type is GripperCommand
    assert leaf.action_name == "/xarm_gripper/gripper_action"
    assert isinstance(goals[0], GripperCommand.Goal)
    assert goals[0].command.position == 0.0
    assert goals[0].command.max_effort == 50.0


@pytest.mark.parametrize("ros_status, reached, stalled, position, expected", [
    (GoalStatus.STATUS_SUCCEEDED, True, False, 0.01, S.SUCCESS),
    (GoalStatus.STATUS_SUCCEEDED, False, False, 0.0, S.FAILURE),
    (GoalStatus.STATUS_SUCCEEDED, False, True, 0.0, S.FAILURE),
    (GoalStatus.STATUS_SUCCEEDED, True, False, 0.03, S.FAILURE),
    (GoalStatus.STATUS_ABORTED, True, False, 0.0, S.FAILURE),
    (GoalStatus.STATUS_CANCELED, True, False, 0.0, S.FAILURE),
])
def test_gripper_open_strict_result(ros_status, reached, stalled, position, expected):
    leaf = _gripper()
    result = GripperCommand.Result(position=position, effort=0.0,
                                   stalled=stalled, reached_goal=reached)
    assert _finish(leaf, ros_status, result) == expected
    if expected == S.FAILURE:
        assert "reached_goal" in leaf.feedback_message


def test_mock_config_names():
    assert BtNode_Tk27Pick.mock_config_name == "BtNode_Grasp"
    assert BtNode_Tk27SavedPose.mock_config_name == "BtNode_MoveArmSingle"
    assert BtNode_Tk27GripperOpen.mock_config_name == "BtNode_GripperAction"
