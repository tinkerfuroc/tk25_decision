"""Regression tests for the live GPSR gripper action client."""
from __future__ import annotations

import time
from types import SimpleNamespace

import action_msgs.msg as action_msgs
import py_trees

from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction


def _node(*, timeout: float = 45.0) -> BtNode_GripperAction:
    return BtNode_GripperAction("gripper", open_gripper=False, action_timeout_sec=timeout)


def test_gripper_result_rejects_aborted_goal():
    node = _node()
    node.result_status = action_msgs.GoalStatus.STATUS_ABORTED

    assert node.process_result() == py_trees.common.Status.FAILURE
    assert "STATUS_ABORTED" in node.feedback_message


def test_gripper_result_accepts_stalled_success_goal():
    # The simulator reports a physical stall as a successful GripperCommand
    # action result, matching the hardware driver's close semantics.
    node = _node()
    node.result_status = action_msgs.GoalStatus.STATUS_SUCCEEDED

    assert node.process_result() == py_trees.common.Status.SUCCESS


def test_gripper_timeout_cancels_an_accepted_goal(monkeypatch):
    node = _node(timeout=0.5)
    # Keep this test independent of the developer's BT_MOCK_CONFIG.
    node.mock_mode = False
    node.send_goal = lambda: None
    node.initialise()
    node.send_goal_future = object()
    node.goal_handle = SimpleNamespace(accepted=True)
    node.result_status = None
    node.last_feedback_time = time.time() - 1.0
    cancelled = []
    monkeypatch.setattr(node, "send_cancel_request", lambda: cancelled.append(True))

    assert node.update() == py_trees.common.Status.FAILURE
    assert cancelled == [True]
    assert "timed out" in node.feedback_message


def test_gripper_timeout_must_be_positive():
    try:
        _node(timeout=0.0)
    except ValueError as exc:
        assert "timeout" in str(exc)
    else:
        raise AssertionError("non-positive gripper timeout was accepted")


def test_action_timeout_before_acceptance_cancels_after_handle_arrives():
    """A timed-out grasp must not survive a delayed goal-response callback."""
    from unittest import mock

    from behavior_tree.TemplateNodes.ActionBase import ActionHandler

    class _CancelFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    class _ResultFuture:
        def add_done_callback(self, callback):
            self.callback = callback

    handle = mock.Mock(accepted=True)
    handle.cancel_goal_async.return_value = _CancelFuture()
    handle.get_result_async.return_value = _ResultFuture()

    node = ActionHandler.__new__(ActionHandler)
    node._cancel_pending = False
    node._cancel_sent = False
    node.goal_handle = None
    node.node = mock.Mock()
    node.qualified_name = "test/grasp"

    # The timeout happens while the server is still deciding whether to
    # accept the goal, so the first cancellation is deferred.
    node.send_cancel_request()
    assert node._cancel_pending is True

    response_future = mock.Mock()
    response_future.result.return_value = handle
    node.goal_response_callback(response_future)

    handle.cancel_goal_async.assert_called_once_with()
    assert node._cancel_pending is False
    assert node._cancel_sent is True
