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

from types import SimpleNamespace

import action_msgs.msg as action_msgs
import py_trees
import pytest

from behavior_tree.interfaces import messages
from behavior_tree.nodes.ActionBase import ActionHandler
from behavior_tree.nodes.Vision import BtNode_ObjectScan


@pytest.fixture(autouse=True)
def _clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _make_node(**kwargs):
    return BtNode_ObjectScan(
        name="Object scan",
        bb_key="test_scan_result",
        vocabulary=[" apple ", "", "cup", "  "],
        **kwargs,
    )


def _set_result(node, *, goal_status, status=0, error_msg="", found_labels=None):
    node.result_status = goal_status
    node.result_message = SimpleNamespace(
        result=SimpleNamespace(
            status=status,
            error_msg=error_msg,
            found_labels=list(found_labels or []),
        )
    )


def test_constructs_action_handler_with_preserved_endpoint_and_no_tick_timeout():
    default_node = _make_node()
    node = _make_node(service_name="custom_object_scan")

    assert default_node.action_name == "object_scan"
    assert isinstance(node, ActionHandler)
    assert node.action_type is messages.ObjectScanAction
    assert node.action_name == "custom_object_scan"
    assert node.action_timeout_ticks == 0


def test_send_goal_preserves_camera_and_normalized_vocabulary(monkeypatch):
    node = _make_node(use_orbbec=False)
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert len(sent_goals) == 1
    goal = sent_goals[0]
    assert isinstance(goal, messages.ObjectScanAction.Goal)
    assert goal.camera == "realsense"
    assert goal.vocabulary == ["apple", "cup"]
    assert node.feedback_message == "Initialized ObjectScan (2 classes)"


def test_mock_goal_preserves_empty_packed_result(monkeypatch):
    node = _make_node()
    node.mock_mode = True
    node.bb_write_client = node.attach_blackboard_client(name="ObjectScanTest")
    node.bb_write_client.register_key(
        node.bb_key,
        access=py_trees.common.Access.WRITE,
    )

    node.send_goal()

    result = py_trees.blackboard.Blackboard.get("/test_scan_result")
    assert node.send_goal_future.done()
    assert result.status == 0
    assert result.objects == []
    assert node.feedback_message == "MOCK: ObjectScan (2 classes)"
    monkeypatch.setattr(
        node,
        "wait_for_keypress_in_mock",
        lambda: py_trees.common.Status.RUNNING,
    )
    assert node.update() == py_trees.common.Status.RUNNING


def test_success_packs_found_labels_for_existing_consumers():
    node = _make_node()
    node.bb_write_client = node.attach_blackboard_client(name="ObjectScanTest")
    node.bb_write_client.register_key(
        node.bb_key,
        access=py_trees.common.Access.WRITE,
    )
    _set_result(
        node,
        goal_status=action_msgs.GoalStatus.STATUS_SUCCEEDED,
        found_labels=["apple", "cup"],
    )

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    result = py_trees.blackboard.Blackboard.get("/test_scan_result")
    assert result.status == 0
    assert [item.cls for item in result.objects] == ["apple", "cup"]
    assert all(item.segment is None for item in result.objects)
    assert node.feedback_message == (
        "ObjectScan found 2, stored to test_scan_result"
    )


@pytest.mark.parametrize(
    ("goal_status", "payload_status"),
    [
        (action_msgs.GoalStatus.STATUS_ABORTED, 0),
        (action_msgs.GoalStatus.STATUS_SUCCEEDED, 6),
    ],
)
def test_rejects_action_and_payload_failures(goal_status, payload_status):
    node = _make_node()
    _set_result(
        node,
        goal_status=goal_status,
        status=payload_status,
        error_msg="scan failed",
        found_labels=["unexpected"],
    )

    assert node.process_result() == py_trees.common.Status.FAILURE
    assert not py_trees.blackboard.Blackboard.exists("/test_scan_result")
