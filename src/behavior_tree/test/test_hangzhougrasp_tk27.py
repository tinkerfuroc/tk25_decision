"""TK27 action contracts for hangzhougrasp: Pick, MoveToSavedPose, gripper open."""
import time
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


# ---- Task 2: blackboard helper leaves ----
from behavior_tree.hangzhougrasp.custom_nodes import (  # noqa: E402
    BtNode_ClearKey, BtNode_ScanSawObjects, BtNode_SelectNearestLabel,
)

SCAN_KEY = "hg_scan_result"


def _scan(*labels, status=0):
    return SimpleNamespace(status=status,
                           objects=[SimpleNamespace(cls=c) for c in labels])


def _bb(key):
    return py_trees.blackboard.Blackboard.storage.get("/" + key, "<unset>")


def _tick(node):
    node.tick_once()
    return node.status


def test_clear_key_sets_none():
    py_trees.blackboard.Blackboard.set("/" + SCAN_KEY, _scan("coke"))
    assert _tick(BtNode_ClearKey("Clear previous scan result", SCAN_KEY)) == S.SUCCESS
    assert _bb(SCAN_KEY) is None


@pytest.mark.parametrize("value, expected", [
    ("<unset>", S.FAILURE),
    (None, S.FAILURE),
    (_scan("coke", status=1), S.FAILURE),
    (_scan(), S.FAILURE),
    (_scan("coke"), S.SUCCESS),
])
def test_scan_saw_objects(value, expected):
    if value != "<unset>":
        py_trees.blackboard.Blackboard.set("/" + SCAN_KEY, value)
    assert _tick(BtNode_ScanSawObjects("Scan saw objects?", SCAN_KEY)) == expected


def _select():
    return BtNode_SelectNearestLabel("Select nearest label", SCAN_KEY, PROMPT_KEY)


def test_select_nearest_label_takes_first():
    py_trees.blackboard.Blackboard.set("/" + SCAN_KEY, _scan(" cup ", "coke"))
    assert _tick(_select()) == S.SUCCESS
    assert _bb(PROMPT_KEY) == "cup"


def test_select_nearest_label_rejects_blank():
    py_trees.blackboard.Blackboard.set("/" + SCAN_KEY, _scan("  "))
    assert _tick(_select()) == S.FAILURE
    assert _bb(PROMPT_KEY) == ""


@pytest.mark.parametrize("value", [None, _scan(), _scan("coke", status=3)])
def test_select_nearest_label_clears_old_label(value):
    _set_prompt("coke")
    py_trees.blackboard.Blackboard.set("/" + SCAN_KEY, value)
    assert _tick(_select()) == S.FAILURE
    assert _bb(PROMPT_KEY) == ""


# ---- final review: real-mode lifecycle ----
from unittest.mock import Mock  # noqa: E402

from tk27_manipulation_msgs.msg import TaskProgress  # noqa: E402


def _all_leaves():
    return [_pick(),
            BtNode_Tk27SavedPose("Arm", "navigation", "/manipulation/move_to_saved_pose"),
            _gripper()]


def _running(leaf):
    """A live leaf mid-goal, with a node stub and spied logger / cancel dispatch."""
    leaf.mock_mode = False
    leaf.node = SimpleNamespace(get_logger=lambda: Mock())
    leaf.logger = Mock()
    leaf._dispatch_cancel_request = Mock()
    leaf.initialise = lambda: None
    leaf.status = S.RUNNING
    leaf.goal_handle = Mock(accepted=True)
    leaf.send_goal_future = Mock()
    leaf.get_result_future = Mock(done=Mock(return_value=True))
    leaf._cancel_requested = leaf._cancel_pending = False
    leaf.result_status = None
    leaf.result_status_string = None
    leaf.result_message = None
    leaf.action_status = 0
    leaf.counter = 0
    leaf.action_timeout_ticks = 0
    leaf.last_feedback_time = time.time()
    leaf.feedback_timeout = 1e9
    leaf._no_goal_reason = None
    return leaf


def test_pick_feedback_does_not_crash():
    leaf = _running(_pick())
    msg = SimpleNamespace(feedback=Pick.Feedback(
        progress=TaskProgress(stage=6, detail="Acquiring target observation")))
    leaf.feedback_callback(msg)
    assert leaf.feedback_message == "Acquiring target observation"
    assert leaf.action_status == 0


def _fail_result(leaf):
    if isinstance(leaf, BtNode_Tk27GripperOpen):
        return GripperCommand.Result(position=0.5, reached_goal=False)
    return leaf.action_type.Result(success=False, status=2, error_msg="server said no")


@pytest.mark.parametrize("index", [0, 1, 2])
def test_failure_message_survives_stop_and_no_cancel_of_finished_goal(index):
    leaf = _running(_all_leaves()[index])
    leaf.result_status = GoalStatus.STATUS_SUCCEEDED
    leaf.result_message = SimpleNamespace(result=_fail_result(leaf))
    leaf.tick_once()  # update -> FAILURE, then stop() -> terminate() while RUNNING
    assert leaf.status == S.FAILURE
    assert "cancelling" not in leaf.feedback_message
    assert ("server said no" in leaf.feedback_message
            or "reached_goal=False" in leaf.feedback_message)
    leaf._dispatch_cancel_request.assert_not_called()


@pytest.mark.parametrize("index", [0, 1, 2])
def test_rejected_goal_logs_error(index):
    leaf = _running(_all_leaves()[index])
    leaf.goal_handle = Mock(accepted=False)
    assert leaf.update() == S.FAILURE
    leaf.logger.error.assert_called_once()
    assert "rejected" in leaf.logger.error.call_args[0][0]


@pytest.mark.parametrize("index", [0, 1, 2])
def test_failed_goal_request_fails_instead_of_hanging(index):
    leaf = _running(_all_leaves()[index])
    leaf.goal_handle = None
    future = Mock()
    future.result.return_value = None
    future.exception.return_value = RuntimeError("transport down")
    leaf.goal_response_callback(future)
    assert leaf.update() == S.FAILURE
    assert "transport down" in leaf.feedback_message
    leaf.logger.error.assert_called_once()


def test_running_goal_is_still_cancelled_on_interrupt():
    leaf = _running(_pick())
    leaf.stop(S.INVALID)  # e.g. a higher-priority branch preempts a live Pick
    assert leaf._cancel_requested


# ---- Orbbec head: strict settle ----
import math  # noqa: E402

from behavior_tree.hangzhougrasp.custom_nodes import BtNode_HeadTilt  # noqa: E402


def _head(tilt=20.0):
    head = BtNode_HeadTilt(name="Head to table scan tilt", x=0.0, y=tilt)
    head.mock_mode = False
    head._skip_settle = False
    head._cmd_pan_deg, head._cmd_tilt_deg = 0.0, tilt
    head._settle_count = 0
    head._command_at = time.monotonic()
    head._feedback_received = 0.0
    return head


def _state(tilt_deg, ok=True):
    return SimpleNamespace(feedback_ok=ok, pan_rad=0.0, tilt_rad=math.radians(tilt_deg))


def test_head_mock_config_name():
    assert BtNode_HeadTilt.mock_config_name == "BtNode_TurnPanTilt"


def test_head_succeeds_on_fresh_in_tolerance_feedback():
    head = _head()
    head._settle_deadline = time.time() + 5
    head._state_cb(_state(20.3))
    assert head.update() == S.SUCCESS


@pytest.mark.parametrize("case", ["off_target", "no_feedback", "stale"])
def test_head_fails_after_deadline_without_settled_feedback(case):
    head = _head()
    head._settle_deadline = time.time() - 0.1  # already past: base reports SUCCESS
    if case == "off_target":
        head._state_cb(_state(28.0))
    elif case == "stale":
        head._state_cb(_state(20.0))
        head._command_at = time.monotonic() + 1  # feedback predates the command
    assert head.update() == S.FAILURE
    assert "did not settle" in head.feedback_message


def test_head_waits_while_deadline_open():
    head = _head()
    head._settle_deadline = time.time() + 5
    head._state_cb(_state(28.0))
    assert head.update() == S.RUNNING
