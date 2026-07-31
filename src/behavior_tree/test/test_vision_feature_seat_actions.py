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
from sensor_msgs.msg import Image

from behavior_tree.interfaces import messages
from behavior_tree.nodes.ActionBase import ActionHandler
from behavior_tree.nodes.Vision import (
    BtNode_FeatureExtraction,
    BtNode_SeatRecommend,
)


@pytest.fixture(autouse=True)
def _clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _make_feature_node(**kwargs):
    return BtNode_FeatureExtraction(
        name="Feature extraction",
        bb_dest_key="test_features",
        bb_image_key="test_comparison_image",
        **kwargs,
    )


def _make_seat_node(**kwargs):
    return BtNode_SeatRecommend(
        name="Seat recommendation",
        bb_dest_key="test_recommendation",
        bb_source_key="test_persons",
        **kwargs,
    )


def _set_result(node, *, goal_status, status=0, error_msg="", **fields):
    node.result_status = goal_status
    node.result_message = SimpleNamespace(
        result=SimpleNamespace(status=status, error_msg=error_msg, **fields)
    )


@pytest.mark.parametrize(
    ("node", "action_type", "endpoint"),
    [
        (_make_feature_node(), messages.FeatureExtractionAction, "feature_extraction_service"),
        (_make_seat_node(), messages.SeatRecommendationAction, "seat_recommend_service"),
    ],
)
def test_nodes_use_actions_with_preserved_endpoints_and_no_tick_timeout(
    node, action_type, endpoint
):
    assert isinstance(node, ActionHandler)
    assert node.action_type is action_type
    assert node.action_name == endpoint
    assert node.action_timeout_ticks == 0


def test_feature_extraction_goal_preserves_camera_selection(monkeypatch):
    node = _make_feature_node(
        service_name="custom_feature_endpoint",
        use_orbbec=False,
    )
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert node.action_name == "custom_feature_endpoint"
    assert len(sent_goals) == 1
    assert isinstance(sent_goals[0], messages.FeatureExtractionAction.Goal)
    assert sent_goals[0].camera == "realsense"


def test_feature_extraction_mock_preserves_blackboard_writes(monkeypatch):
    node = _make_feature_node()
    node.mock_mode = True

    node.send_goal()

    assert node.send_goal_future.done()
    assert node.blackboard.features == "[mock features]"
    assert isinstance(node.blackboard.comparison_image, Image)
    monkeypatch.setattr(
        node,
        "wait_for_keypress_in_mock",
        lambda: py_trees.common.Status.RUNNING,
    )
    assert node.update() == py_trees.common.Status.RUNNING


def test_feature_extraction_success_writes_result():
    image = Image(width=640, height=480)
    node = _make_feature_node()
    _set_result(
        node,
        goal_status=action_msgs.GoalStatus.STATUS_SUCCEEDED,
        feature="blue shirt",
        comparison_image=image,
    )

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    assert node.blackboard.features == "blue shirt"
    assert node.blackboard.comparison_image is image
    assert node.feedback_message == "Features: blue shirt | image: 640x480"


@pytest.mark.parametrize(
    ("goal_status", "payload_status"),
    [
        (action_msgs.GoalStatus.STATUS_ABORTED, 0),
        (action_msgs.GoalStatus.STATUS_SUCCEEDED, 7),
    ],
)
def test_feature_extraction_rejects_action_and_payload_failures(
    goal_status, payload_status
):
    node = _make_feature_node()
    _set_result(
        node,
        goal_status=goal_status,
        status=payload_status,
        error_msg="extraction failed",
        feature="unexpected",
        comparison_image=Image(),
    )

    assert node.process_result() == py_trees.common.Status.FAILURE
    assert not py_trees.blackboard.Blackboard.exists("/test_features")
    assert not py_trees.blackboard.Blackboard.exists("/test_comparison_image")


def test_seat_goal_preserves_camera_and_excludes_newest_person(monkeypatch):
    persons = [
        SimpleNamespace(name="host", features="host features"),
        SimpleNamespace(name="guest one", features="guest one features"),
        SimpleNamespace(name="new guest", features="new guest features"),
    ]
    py_trees.blackboard.Blackboard.set("/test_persons", persons)
    node = _make_seat_node(use_orbbec=False)
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert len(sent_goals) == 1
    goal = sent_goals[0]
    assert isinstance(goal, messages.SeatRecommendationAction.Goal)
    assert goal.camera == "realsense"
    assert goal.names == ["host", "guest one"]
    assert goal.features == ["host features", "guest one features"]


def test_seat_goal_uses_empty_person_lists_when_source_is_none(monkeypatch):
    py_trees.blackboard.Blackboard.set("/test_persons", None)
    node = _make_seat_node()
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert sent_goals[0].names == []
    assert sent_goals[0].features == []


def test_seat_mock_preserves_recommendation_write(monkeypatch):
    node = _make_seat_node()
    node.mock_mode = True

    node.send_goal()

    assert node.send_goal_future.done()
    assert node.blackboard.recommendation == (
        "Dear guest, I recommend the seat on the left side of the table."
    )
    monkeypatch.setattr(
        node,
        "wait_for_keypress_in_mock",
        lambda: py_trees.common.Status.RUNNING,
    )
    assert node.update() == py_trees.common.Status.RUNNING


def test_seat_success_writes_prefixed_recommendation():
    node = _make_seat_node()
    _set_result(
        node,
        goal_status=action_msgs.GoalStatus.STATUS_SUCCEEDED,
        recommendation="Please take the chair by the window.",
    )

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    assert node.blackboard.recommendation == (
        "Dear guest, Please take the chair by the window."
    )
    assert node.feedback_message == "Recommendation: Please take the chair by the window."


@pytest.mark.parametrize(
    ("goal_status", "payload_status"),
    [
        (action_msgs.GoalStatus.STATUS_ABORTED, 0),
        (action_msgs.GoalStatus.STATUS_SUCCEEDED, 9),
    ],
)
def test_seat_recommendation_rejects_action_and_payload_failures(
    goal_status, payload_status
):
    node = _make_seat_node()
    _set_result(
        node,
        goal_status=goal_status,
        status=payload_status,
        error_msg="recommendation failed",
        recommendation="unexpected",
    )

    assert node.process_result() == py_trees.common.Status.FAILURE
    assert not py_trees.blackboard.Blackboard.exists("/test_recommendation")
