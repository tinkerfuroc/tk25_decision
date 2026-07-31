# Copyright 2026 Tinker
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
"""Contracts for DetectWaving action clients in behavior-tree nodes."""

import sys
from types import SimpleNamespace

import action_msgs.msg as action_msgs
import py_trees
import pytest

from behavior_tree.interfaces import messages
from behavior_tree.components.following.wave_reseed_cycle import _ActionPayloadFuture

# GPSR's legacy module configures the optional OpenAI SDK at import time.
sys.modules.setdefault("openai", SimpleNamespace())

from behavior_tree.GPSR.custom_nodes import BtNode_ScanForWavingPersonNew
from behavior_tree.Restaurant.custumNodes import BtNode_DetectCallingCustomer
from behavior_tree.nodes.ActionBase import ActionHandler
from behavior_tree.nodes.Vision import BtNode_ScanForWavingPerson


@pytest.fixture(autouse=True)
def _clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _vision_node():
    return BtNode_ScanForWavingPerson(
        "scan", "all_wavers", "closest_waver", 4.5)


def _gpsr_node():
    return BtNode_ScanForWavingPersonNew(
        "scan", "all_wavers", "closest_waver", 4.5)


def _restaurant_node():
    return BtNode_DetectCallingCustomer("scan", "all_wavers")


@pytest.mark.parametrize(
    "node",
    [_vision_node(), _gpsr_node(), _restaurant_node()],
)
def test_waving_nodes_use_action_with_preserved_endpoint(node):
    assert isinstance(node, ActionHandler)
    assert node.action_type is messages.DetectWavingAction
    assert node.action_name == "detect_waving_persons"
    assert node.action_timeout_ticks == 0


@pytest.mark.parametrize("node", [_vision_node(), _gpsr_node()])
def test_scan_goal_preserves_detection_parameters(monkeypatch, node):
    node.mock_mode = False
    sent = []
    monkeypatch.setattr(node, "send_goal_request", sent.append)

    node.send_goal()

    assert len(sent) == 1
    assert isinstance(sent[0], messages.DetectWavingAction.Goal)
    assert sent[0].threshold_meters == 4.5
    assert sent[0].target_frame == "map"


class _ManualFuture:
    def __init__(self):
        self._callbacks = []
        self._result = None

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def result(self):
        return self._result

    def resolve(self, result):
        self._result = result
        for callback in self._callbacks:
            callback(self)


class _GoalHandle:
    accepted = True

    def __init__(self):
        self.result_future = _ManualFuture()
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return _ManualFuture()


def test_action_payload_future_unwraps_result():
    send_future = _ManualFuture()
    future = _ActionPayloadFuture(send_future)
    goal_handle = _GoalHandle()
    payload = SimpleNamespace(status=0, waving_boxes=[])

    send_future.resolve(goal_handle)
    goal_handle.result_future.resolve(SimpleNamespace(result=payload))

    assert future.done()
    assert future.result() is payload


def test_action_payload_future_cancels_after_late_goal_acceptance():
    send_future = _ManualFuture()
    future = _ActionPayloadFuture(send_future)
    goal_handle = _GoalHandle()

    assert future.cancel() is True
    send_future.resolve(goal_handle)

    assert goal_handle.cancel_calls == 1


def test_vision_result_writes_people_and_closest():
    node = _vision_node()
    writes = {}
    node.bb_write_client = SimpleNamespace(
        set=lambda key, value, **_kwargs: writes.update({key: value}))
    person = SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        point=SimpleNamespace(x=1.0, y=2.0, z=3.0),
    )
    node.result_status = action_msgs.GoalStatus.STATUS_SUCCEEDED
    node.result_message = SimpleNamespace(result=SimpleNamespace(
        status=0,
        error_msg="",
        waving_persons=[person],
        rgb_image=None,
        segments=[],
    ))

    assert node.process_result() == py_trees.common.Status.SUCCESS
    assert writes["all_wavers"] == [person]
    assert writes["closest_waver"] is person
