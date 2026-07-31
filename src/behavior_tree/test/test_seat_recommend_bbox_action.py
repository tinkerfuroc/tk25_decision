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
from geometry_msgs.msg import PointStamped

from behavior_tree.interfaces import messages
from behavior_tree.nodes.ActionBase import ActionHandler
from behavior_tree.nodes.Vision import BtNode_SeatRecommendBbox


@pytest.fixture(autouse=True)
def _clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _make_node(**kwargs):
    return BtNode_SeatRecommendBbox(
        name="Seat recommendation bbox",
        bb_recommendation_key="test_recommendation",
        bb_bbox_key="test_bbox",
        bb_point_key="test_point",
        bb_source_key="test_persons",
        **kwargs,
    )


def _set_result(node, *, goal_status, status=0, error_msg="", **fields):
    node.result_status = goal_status
    node.result_message = SimpleNamespace(
        result=SimpleNamespace(status=status, error_msg=error_msg, **fields)
    )


def test_node_uses_action_with_preserved_endpoint_and_no_tick_timeout():
    default_node = _make_node()
    node = _make_node(service_name="custom_seat_bbox_endpoint")

    assert default_node.action_name == "seat_recommend_bbox_service"
    assert isinstance(node, ActionHandler)
    assert node.action_type is messages.SeatRecommendBboxAction
    assert node.action_name == "custom_seat_bbox_endpoint"
    assert node.action_timeout_ticks == 0


def test_goal_preserves_inputs_and_excludes_newest_person(monkeypatch):
    py_trees.blackboard.Blackboard.set(
        "/test_persons",
        [
            SimpleNamespace(name="host", features="host features"),
            SimpleNamespace(name="guest one", features="guest one features"),
            SimpleNamespace(name="new guest", features="new guest features"),
        ],
    )
    known_seats = ["left chair", "right chair"]
    node = _make_node(
        use_orbbec=False,
        target_frame="map",
        known_seats=known_seats,
    )
    known_seats.append("late mutation")
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert len(sent_goals) == 1
    goal = sent_goals[0]
    assert isinstance(goal, messages.SeatRecommendBboxAction.Goal)
    assert goal.camera == "realsense"
    assert goal.names == ["host", "guest one"]
    assert goal.features == ["host features", "guest one features"]
    assert goal.target_frame == "map"
    assert goal.known_seats == ["left chair", "right chair"]
    assert node.feedback_message == "Initialized seat-bbox recommendation"


def test_goal_uses_empty_person_lists_when_source_is_none(monkeypatch):
    py_trees.blackboard.Blackboard.set("/test_persons", None)
    node = _make_node()
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert sent_goals[0].names == []
    assert sent_goals[0].features == []


def test_mock_preserves_recommendation_bbox_and_centroid(monkeypatch):
    node = _make_node(target_frame="map")
    node.mock_mode = True

    node.send_goal()

    assert node.send_goal_future.done()
    assert node.blackboard.recommendation == (
        "Dear guest, I recommend the seat on the left side of the table."
    )
    assert isinstance(node.blackboard.bbox, messages.BoundingBox)
    assert (
        node.blackboard.bbox.xmin,
        node.blackboard.bbox.ymin,
        node.blackboard.bbox.xmax,
        node.blackboard.bbox.ymax,
    ) == (300, 200, 380, 280)
    assert node.blackboard.point.header.frame_id == "map"
    assert (
        node.blackboard.point.point.x,
        node.blackboard.point.point.y,
        node.blackboard.point.point.z,
    ) == (1.0, 0.5, 0.6)
    assert node.feedback_message == "MOCK: Seat-bbox recommendation completed"
    monkeypatch.setattr(
        node,
        "wait_for_keypress_in_mock",
        lambda: py_trees.common.Status.RUNNING,
    )
    assert node.update() == py_trees.common.Status.RUNNING


def test_success_writes_prefixed_recommendation_bbox_and_centroid():
    bbox = messages.BoundingBox()
    centroid = PointStamped()
    node = _make_node()
    _set_result(
        node,
        goal_status=action_msgs.GoalStatus.STATUS_SUCCEEDED,
        recommendation="Please take the chair by the window.",
        bbox=bbox,
        centroid=centroid,
    )

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    assert node.blackboard.recommendation == (
        "Dear guest, Please take the chair by the window."
    )
    assert node.blackboard.bbox is bbox
    assert node.blackboard.point is centroid
    assert node.feedback_message == "Recommendation: Please take the chair by the window."


@pytest.mark.parametrize(
    ("goal_status", "payload_status"),
    [
        (action_msgs.GoalStatus.STATUS_ABORTED, 0),
        (action_msgs.GoalStatus.STATUS_SUCCEEDED, 9),
    ],
)
def test_rejects_action_and_payload_failures(goal_status, payload_status):
    node = _make_node()
    _set_result(
        node,
        goal_status=goal_status,
        status=payload_status,
        error_msg="recommendation failed",
        recommendation="unexpected",
        bbox=messages.BoundingBox(),
        centroid=PointStamped(),
    )

    assert node.process_result() == py_trees.common.Status.FAILURE
    assert not py_trees.blackboard.Blackboard.exists("/test_recommendation")
    assert not py_trees.blackboard.Blackboard.exists("/test_bbox")
    assert not py_trees.blackboard.Blackboard.exists("/test_point")
