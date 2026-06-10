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

"""Tree-assembly test for create_follow_person_tree (no ROS graph / no setup)."""

import py_trees

from behavior_tree.FollowPerson.follow_person import create_follow_person_tree
from behavior_tree.FollowPerson.nodes import (
    BtNode_PublishFollowGoal,
    BtNode_ReacqAnnounce,
)
from behavior_tree.TemplateNodes.TrackPersonAction import BtNode_TrackPersonAction


def test_root_is_parallel_success_on_all():
    root = create_follow_person_tree()
    assert isinstance(root, py_trees.composites.Parallel)
    assert isinstance(root.policy, py_trees.common.ParallelPolicy.SuccessOnAll)
    assert root.policy.synchronise is False


def test_children_are_track_action_and_sequence():
    root = create_follow_person_tree()
    assert len(root.children) == 2

    child_a, child_b = root.children
    assert isinstance(child_a, BtNode_TrackPersonAction)
    assert isinstance(child_b, py_trees.composites.Sequence)
    assert child_b.memory is False


def test_sequence_children_are_publish_then_announce():
    root = create_follow_person_tree()
    _, sequence = root.children
    assert len(sequence.children) == 2
    publish, announce = sequence.children
    assert isinstance(publish, BtNode_PublishFollowGoal)
    assert isinstance(announce, BtNode_ReacqAnnounce)


def test_tree_built_without_setup():
    # Building the tree must not require a ROS graph; nodes are unset until
    # setup() is called by the runner. This call alone must not raise.
    root = create_follow_person_tree()
    assert root is not None
