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

"""Unit tests for BtNode_PublishFollowGoal (publishes the follow target)."""

import py_trees
import pytest
from geometry_msgs.msg import PointStamped

from behavior_tree.FollowPerson.nodes import BtNode_PublishFollowGoal

POS_KEY = "track/person_position"
LOST_KEY = "track/target_lost"


class _FakePublisher:
    """Records published messages."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _writer():
    client = py_trees.blackboard.Client(name="goal_writer")
    client.register_key(key=POS_KEY, access=py_trees.common.Access.WRITE)
    client.register_key(key=LOST_KEY, access=py_trees.common.Access.WRITE)
    return client


def _make_node(fake_pub):
    node = BtNode_PublishFollowGoal(
        name="PublishFollowGoal",
        topic="/follow_target",
        pos_key=POS_KEY,
        lost_key=LOST_KEY,
    )
    node.inject_publisher(fake_pub)
    return node


def _point(x=1.0, y=2.0, z=3.0, frame="map"):
    p = PointStamped()
    p.header.frame_id = frame
    p.point.x = x
    p.point.y = y
    p.point.z = z
    return p


def test_publishes_when_not_lost_and_position_present():
    fake = _FakePublisher()
    writer = _writer()
    node = _make_node(fake)

    pos = _point()
    writer.set(LOST_KEY, False, overwrite=True)
    writer.set(POS_KEY, pos, overwrite=True)

    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert len(fake.published) == 1
    assert fake.published[0] is pos


def test_no_publish_when_target_lost():
    fake = _FakePublisher()
    writer = _writer()
    node = _make_node(fake)

    writer.set(LOST_KEY, True, overwrite=True)
    writer.set(POS_KEY, _point(), overwrite=True)

    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert fake.published == []


def test_no_publish_when_position_missing():
    fake = _FakePublisher()
    writer = _writer()
    node = _make_node(fake)

    writer.set(LOST_KEY, False, overwrite=True)
    writer.set(POS_KEY, None, overwrite=True)

    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert fake.published == []


def test_no_publish_when_blackboard_unset():
    fake = _FakePublisher()
    node = _make_node(fake)
    # Nothing written to the blackboard.
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert fake.published == []
