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


def test_canceled_navigation_is_failure():
    node = _make_goto_node("gpsr/target_pose", None, action_msgs.GoalStatus.STATUS_CANCELED)

    status = node.process_result()

    assert status == py_trees.common.Status.FAILURE
    assert node.feedback_message == "action canceled"


def test_stalled_navigation_times_out_and_cancels_once(monkeypatch):
    """An accepted-but-stalled Nav2 goal must release the retry wrapper."""
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction

    class _CancelFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    cancel_future = _CancelFuture()
    goal_handle = mock.Mock(accepted=True)
    goal_handle.cancel_goal_async.return_value = cancel_future

    node = BtNode_GotoAction.__new__(BtNode_GotoAction)
    node.action_timeout_sec = 5.0
    node.feedback_stall_sec = 5.0
    node.wall_backstop_sec = 20.0
    node._nav_time_s = None
    node._last_feedback_at = None
    node._action_started_at = 100.0
    node._timeout_cancel_requested = False
    node.feedback_message = ""
    node.qualified_name = "test/goto"
    node.node = mock.Mock()
    node.goal_handle = goal_handle
    node.result_status = None

    monkeypatch.setattr(
        "behavior_tree.TemplateNodes.Navigation.time.monotonic", lambda: 106.0
    )

    status = node.update()

    assert status == py_trees.common.Status.FAILURE
    goal_handle.cancel_goal_async.assert_called_once_with()
    assert node._timeout_cancel_requested is True
    assert node.feedback_message.startswith("navigation action timeout after 5.0s")

    # A retry decorator can tick the failed child again before re-entry; do
    # not submit a second cancel request for the same timed-out goal.
    node.update()
    goal_handle.cancel_goal_async.assert_called_once_with()


def test_stalled_approach_times_out_and_cancels_once(monkeypatch):
    """An accepted GoToApproach goal must not hold delivery forever."""
    from behavior_tree.TemplateNodes.Navigation import BtNode_Approach

    class _CancelFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    cancel_future = _CancelFuture()
    goal_handle = mock.Mock(accepted=True)
    goal_handle.cancel_goal_async.return_value = cancel_future

    node = BtNode_Approach.__new__(BtNode_Approach)
    node.action_deadline_sec = 5.0
    node._action_started_at = 100.0
    node._timeout_cancel_requested = False
    node.feedback_message = ""
    node.qualified_name = "test/approach"
    node.node = mock.Mock()
    node.goal_handle = goal_handle
    node.result_status = None

    monkeypatch.setattr(
        "behavior_tree.TemplateNodes.Navigation.time.monotonic", lambda: 106.0
    )

    status = node.update()

    assert status == py_trees.common.Status.FAILURE
    goal_handle.cancel_goal_async.assert_called_once_with()
    assert node._timeout_cancel_requested is True
    assert node.feedback_message == "approach action timeout after 5.0s"

    node.update()
    goal_handle.cancel_goal_async.assert_called_once_with()


def test_stale_retry_callbacks_do_not_update_current_navigation():
    """Canceled goals must not complete the replacement retry attempt."""
    from behavior_tree.TemplateNodes.ActionBase import ActionHandler
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction

    node = BtNode_GotoAction.__new__(BtNode_GotoAction)
    current_send = object()
    current_result = object()
    node.send_goal_future = current_send
    node.get_result_future = current_result

    with mock.patch.object(ActionHandler, "goal_response_callback") as goal_cb:
        node.goal_response_callback(object())
        goal_cb.assert_not_called()
    with mock.patch.object(ActionHandler, "get_result_callback") as result_cb:
        node.get_result_callback(object())
        result_cb.assert_not_called()


# ---------------------------------------------------------------------------
# Issue report 2026-10-11 Bug A: the 120 s budget counts Nav2's own clock.
# The sim ran at RTF ~0.3, so a 120 s wall deadline gave Nav2 ~36 sim-s and
# cancelled a still-moving robot 0.2 m short of side_table_02 (battery
# t2-full-20261010 goals 7492/7613/3808). bt_navigator's feedback
# navigation_time is on its node clock (sim time in the sim, real time on
# the robot); wall clock only guards a silent Nav2 and a frozen sim clock.
# ---------------------------------------------------------------------------

def _timed_goto(monkeypatch, wall_now, timeout=120.0):
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction

    class _CancelFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    goal_handle = mock.Mock(accepted=True, goal_id="goal-A")
    goal_handle.cancel_goal_async.return_value = _CancelFuture()
    node = BtNode_GotoAction.__new__(BtNode_GotoAction)
    node.action_timeout_sec = timeout
    node.feedback_stall_sec = timeout
    node.wall_backstop_sec = 4.0 * timeout
    node._action_started_at = 0.0
    node._timeout_cancel_requested = False
    node._nav_time_s = None
    node._last_feedback_at = None
    node.feedback_message = ""
    node.qualified_name = "test/goto"
    node.node = mock.Mock()
    node.goal_handle = goal_handle
    node.result_status = None
    node.process_feedback = lambda feedback: None
    clock = {"t": wall_now}
    monkeypatch.setattr("behavior_tree.TemplateNodes.Navigation.time.monotonic", lambda: clock["t"])
    monkeypatch.setattr("behavior_tree.TemplateNodes.Navigation.time.time", lambda: clock["t"])
    return node, clock


def _nav_feedback(nav_s, goal_id="goal-A"):
    from types import SimpleNamespace
    sec = int(nav_s)
    return SimpleNamespace(
        goal_id=goal_id,
        feedback=SimpleNamespace(
            navigation_time=SimpleNamespace(sec=sec, nanosec=int((nav_s - sec) * 1e9))),
    )


def test_slow_sim_goal_is_not_cancelled_while_nav_time_is_under_budget(monkeypatch):
    from behavior_tree.TemplateNodes.ActionBase import ActionHandler

    node, clock = _timed_goto(monkeypatch, wall_now=0.0)
    clock["t"] = 125.0                       # 125 wall-s ...
    node.feedback_callback(_nav_feedback(37.5))   # ... but only 37.5 sim-s
    with mock.patch.object(ActionHandler, "update", return_value=py_trees.common.Status.RUNNING):
        assert node.update() == py_trees.common.Status.RUNNING
    node.goal_handle.cancel_goal_async.assert_not_called()


def test_goal_times_out_on_nav2_navigation_time(monkeypatch):
    node, clock = _timed_goto(monkeypatch, wall_now=0.0)
    clock["t"] = 400.0
    node.feedback_callback(_nav_feedback(120.2))
    assert node.update() == py_trees.common.Status.FAILURE
    node.goal_handle.cancel_goal_async.assert_called_once_with()
    assert node.feedback_message.startswith("navigation action timeout after 120.0s")
    assert "navigation time" in node.feedback_message


def test_silent_nav2_still_times_out_on_wall_clock(monkeypatch):
    node, clock = _timed_goto(monkeypatch, wall_now=0.0)
    clock["t"] = 10.0
    node.feedback_callback(_nav_feedback(3.0))
    clock["t"] = 130.0                       # 120 wall-s without any feedback
    assert node.update() == py_trees.common.Status.FAILURE
    assert "no Nav2 feedback" in node.feedback_message


def test_frozen_nav_clock_hits_the_wall_backstop(monkeypatch):
    node, clock = _timed_goto(monkeypatch, wall_now=0.0)
    clock["t"] = 480.0
    node.feedback_callback(_nav_feedback(20.0))   # feedback flows, clock stuck
    assert node.update() == py_trees.common.Status.FAILURE
    assert "wall" in node.feedback_message


def test_feedback_from_a_previous_goal_is_ignored(monkeypatch):
    node, clock = _timed_goto(monkeypatch, wall_now=0.0)
    clock["t"] = 5.0
    node.feedback_callback(_nav_feedback(119.0, goal_id="goal-OLD"))
    assert node._nav_time_s is None and node._last_feedback_at is None
