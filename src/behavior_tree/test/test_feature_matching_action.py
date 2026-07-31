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
from behavior_tree.nodes.Vision import BtNode_FeatureMatching


@pytest.fixture(autouse=True)
def _clear_blackboard():
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


def _make_node(**kwargs):
    return BtNode_FeatureMatching(
        name="Feature matching",
        bb_dest_key="test_centroids",
        bb_persons_key="test_persons",
        **kwargs,
    )


def _set_persons(*persons):
    py_trees.blackboard.Blackboard.set("/test_persons", list(persons))


def _person(features, comparison_image=None):
    return SimpleNamespace(
        features=features,
        comparison_image=comparison_image,
    )


def _set_result(node, *, goal_status, status=0, error_msg="", centroids=None):
    node.result_status = goal_status
    node.result_message = SimpleNamespace(
        result=SimpleNamespace(
            status=status,
            error_msg=error_msg,
            centroids=list(centroids or []),
        )
    )


def test_constructs_action_handler_with_preserved_endpoint_and_no_tick_timeout():
    node = _make_node()

    assert isinstance(node, ActionHandler)
    assert node.action_type is messages.FeatureMatchingAction
    assert node.action_name == "feature_matching_service"
    assert node.action_timeout_ticks == 0


def test_send_goal_maps_persons_and_applies_both_trims(monkeypatch):
    first_image = Image()
    middle_image = Image()
    last_image = Image()
    _set_persons(
        _person("host", first_image),
        _person("guest", middle_image),
        _person("newest", last_image),
    )
    node = _make_node(
        use_orbbec=False,
        max_distance=4.5,
        target_frame="map",
        trim_first_person=True,
        trim_last_person=True,
    )
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert len(sent_goals) == 1
    goal = sent_goals[0]
    assert isinstance(goal, messages.FeatureMatchingAction.Goal)
    assert goal.camera == "realsense"
    assert goal.features == ["guest"]
    assert goal.comparison_images == [middle_image]
    assert goal.max_distance == 4.5
    assert goal.target_frame == "map"


def test_send_goal_substitutes_empty_image_for_missing_comparison(monkeypatch):
    _set_persons(_person("guest", None))
    node = _make_node(trim_last_person=False)
    node.mock_mode = False
    sent_goals = []
    monkeypatch.setattr(node, "send_goal_request", sent_goals.append)

    node.send_goal()

    assert isinstance(sent_goals[0].comparison_images[0], Image)


def test_mock_goal_preserves_synthetic_centroids_and_padding(monkeypatch):
    _set_persons(_person("host"), _person("guest-1"), _person("guest-2"))
    node = _make_node(trim_first_person=True, trim_last_person=False)
    node.mock_mode = True

    node.send_goal()

    assert node.send_goal_future.done()
    assert node.blackboard.centroids[0] is None
    assert len(node.blackboard.centroids) == 3
    assert all(
        centroid.header.frame_id == "base_link"
        for centroid in node.blackboard.centroids[1:]
    )
    monkeypatch.setattr(
        node,
        "wait_for_keypress_in_mock",
        lambda: py_trees.common.Status.RUNNING,
    )
    assert node.update() == py_trees.common.Status.RUNNING


def test_successful_action_result_writes_padded_centroids():
    node = _make_node(trim_first_person=True, trim_last_person=False)
    _set_result(
        node,
        goal_status=action_msgs.GoalStatus.STATUS_SUCCEEDED,
        centroids=["guest-1-centroid", "guest-2-centroid"],
    )

    status = node.process_result()

    assert status == py_trees.common.Status.SUCCESS
    assert node.blackboard.centroids == [
        None,
        "guest-1-centroid",
        "guest-2-centroid",
    ]


def test_pre_result_action_failure_clears_centroids():
    node = _make_node(trim_last_person=False)
    node.mock_mode = False
    node.blackboard.centroids = ["stale-centroid"]
    node.send_goal_future = None

    status = node.update()

    assert status == py_trees.common.Status.FAILURE
    assert node.blackboard.centroids == []


@pytest.mark.parametrize(
    ("goal_status", "payload_status"),
    [
        (action_msgs.GoalStatus.STATUS_ABORTED, 0),
        (action_msgs.GoalStatus.STATUS_SUCCEEDED, 7),
    ],
)
def test_failed_action_or_payload_clears_centroids(goal_status, payload_status):
    node = _make_node(trim_last_person=False)
    node.blackboard.centroids = ["stale-centroid"]
    _set_result(
        node,
        goal_status=goal_status,
        status=payload_status,
        error_msg="matching failed",
        centroids=["unexpected-centroid"],
    )

    status = node.process_result()

    assert status == py_trees.common.Status.FAILURE
    assert node.blackboard.centroids == []
