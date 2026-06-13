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
from behavior_tree.FollowPerson.nodes import BtNode_ReacqAnnounce
from behavior_tree.TemplateNodes.FollowAction import BtNode_FollowAction
from behavior_tree.TemplateNodes.TrackPersonAction import BtNode_TrackPersonAction


def test_root_is_parallel_success_on_all():
    root = create_follow_person_tree()
    assert isinstance(root, py_trees.composites.Parallel)
    assert isinstance(root.policy, py_trees.common.ParallelPolicy.SuccessOnAll)
    assert root.policy.synchronise is False


def test_children_are_track_follow_and_reactions():
    root = create_follow_person_tree()
    # Three long-running children: tracker, follow executive, reactions.
    assert len(root.children) == 3

    track, follow, reactions = root.children
    assert isinstance(track, BtNode_TrackPersonAction)
    assert isinstance(follow, BtNode_FollowAction)
    assert isinstance(reactions, py_trees.composites.Sequence)
    assert reactions.memory is False


def test_follow_action_is_a_top_level_parallel_child():
    # The follow executive is long-running, so it sits beside the tracker under
    # the Parallel root — NOT inside the per-tick reactions Sequence.
    root = create_follow_person_tree()
    _, _, reactions = root.children
    assert not any(
        isinstance(child, BtNode_FollowAction) for child in reactions.children
    )


def test_reactions_sequence_holds_only_announce():
    root = create_follow_person_tree()
    _, _, reactions = root.children
    assert len(reactions.children) == 1
    (announce,) = reactions.children
    assert isinstance(announce, BtNode_ReacqAnnounce)


def test_tree_built_without_setup():
    # Building the tree must not require a ROS graph; nodes are unset until
    # setup() is called by the runner. This call alone must not raise.
    root = create_follow_person_tree()
    assert root is not None


def test_tree_with_nav_includes_follow_child():
    root = create_follow_person_tree(enable_navigation=True)
    names = [c.name for c in root.children]
    assert "Follow Navigation" in names
    assert len(root.children) == 3   # Track Person, Follow Navigation, Follow Reactions


def test_tree_no_nav_omits_follow_child():
    root = create_follow_person_tree(enable_navigation=False)
    names = [c.name for c in root.children]
    assert "Follow Navigation" not in names
    assert "Track Person" in names           # tracking still present
    assert len(root.children) == 2           # Track Person, Follow Reactions
