# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import time

import py_trees
import pytest
import rclpy.action

from behavior_tree.TemplateNodes.ActionBase import ActionHandler


class _Logger:
    def debug(self, *_args, **_kwargs):
        pass


class _Node:
    def get_logger(self):
        return _Logger()


class _Future:
    def __init__(self, result=None):
        self._result = result
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        return self._result

    def done(self):
        return False


class _GoalHandle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.cancel_calls = 0
        self.result_future = _Future()

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _Future()

    def get_result_async(self):
        return self.result_future


def _handler(action_timeout_ticks=0):
    handler = ActionHandler(
        name="test action",
        action_type=object,
        action_name="test_action",
        key=None,
        action_timeout_ticks=action_timeout_ticks,
    )
    handler.mock_mode = False
    handler.node = _Node()
    handler.send_goal_future = _Future()
    handler.get_result_future = None
    handler.result_status = None
    handler.last_feedback_time = time.time()
    handler.feedback_timeout = 1000.0
    handler.counter = 0
    return handler


def test_action_timeout_cancels_once_before_failure():
    handler = _handler(action_timeout_ticks=1)
    goal_handle = _GoalHandle()
    handler.goal_handle = goal_handle
    handler.counter = 1
    handler.status = py_trees.common.Status.RUNNING

    assert handler.update() == py_trees.common.Status.FAILURE
    assert goal_handle.cancel_calls == 1

    handler.terminate(py_trees.common.Status.FAILURE)
    assert goal_handle.cancel_calls == 1


def test_feedback_timeout_cancels_once_before_failure():
    handler = _handler()
    goal_handle = _GoalHandle()
    handler.goal_handle = goal_handle
    handler.last_feedback_time = 0.0
    handler.feedback_timeout = 0.0

    assert handler.update() == py_trees.common.Status.FAILURE
    assert goal_handle.cancel_calls == 1


@pytest.mark.parametrize(
    "new_status",
    (py_trees.common.Status.FAILURE, py_trees.common.Status.INVALID),
)
def test_running_terminal_transition_cancels_once(new_status):
    handler = _handler()
    goal_handle = _GoalHandle()
    handler.goal_handle = goal_handle
    handler.status = py_trees.common.Status.RUNNING

    handler.terminate(new_status)
    handler.terminate(new_status)

    assert goal_handle.cancel_calls == 1


def test_running_to_success_does_not_cancel():
    handler = _handler()
    goal_handle = _GoalHandle()
    handler.goal_handle = goal_handle
    handler.status = py_trees.common.Status.RUNNING

    handler.terminate(py_trees.common.Status.SUCCESS)

    assert goal_handle.cancel_calls == 0


@pytest.mark.parametrize(
    "prior_status",
    (
        py_trees.common.Status.SUCCESS,
        py_trees.common.Status.FAILURE,
        py_trees.common.Status.INVALID,
    ),
)
def test_non_running_prior_state_does_not_cancel(prior_status):
    handler = _handler()
    goal_handle = _GoalHandle()
    handler.goal_handle = goal_handle
    handler.status = prior_status

    handler.terminate(py_trees.common.Status.INVALID)

    assert goal_handle.cancel_calls == 0


def test_cancel_before_acceptance_is_dispatched_once_after_acceptance():
    handler = _handler()
    handler.goal_handle = None

    handler.send_cancel_request()
    handler.send_cancel_request()

    goal_handle = _GoalHandle()
    handler.goal_response_callback(_Future(goal_handle))

    assert goal_handle.cancel_calls == 1
    assert handler._cancel_pending is False


def test_cancel_idempotency_resets_for_each_goal():
    handler = _handler()
    first_goal = _GoalHandle()
    handler.goal_handle = first_goal

    handler.send_cancel_request()
    handler.send_cancel_request()
    assert first_goal.cancel_calls == 1

    handler.send_goal = lambda: None
    handler.initialise()
    second_goal = _GoalHandle()
    handler.goal_handle = second_goal

    handler.send_cancel_request()
    handler.send_cancel_request()
    assert second_goal.cancel_calls == 1


def test_setup_forces_mock_mode_for_mock_action_type(monkeypatch):
    mock_action_type = type(
        "VisionAction",
        (),
        {"__module__": "behavior_tree.mock_messages"},
    )
    handler = ActionHandler(
        name="mock action",
        action_type=mock_action_type,
        action_name="mock_action",
        key=None,
    )
    handler.mock_mode = False
    handler._mock_input_controller = type(
        "InputController",
        (),
        {"configure": lambda self, _config: None, "start": lambda self: None},
    )()

    def fail_action_client(*_args, **_kwargs):
        pytest.fail("setup attempted to construct an ActionClient for a mock type")

    monkeypatch.setattr(rclpy.action, "ActionClient", fail_action_client)

    handler.setup(node=_Node())

    assert handler.mock_mode is True
    assert handler.action_client is None
