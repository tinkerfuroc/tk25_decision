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

#
# Follow-person behaviour tree
# ============================
#
# Builds the follow-person tree: a Parallel root that keeps the real
# ``/track_person`` action alive (child A) while, in parallel, publishing the
# follow target and speaking reacquisition guidance (child B).
#
#   Parallel(SuccessOnAll, synchronise=False)
#   ├── BtNode_TrackPersonAction        (writes the track/* blackboard keys)
#   └── Sequence(memory=False)
#       ├── BtNode_PublishFollowGoal    (publishes /follow_target)
#       └── BtNode_ReacqAnnounce        (reacq-driven voice announcements)
#
# Child A refreshes the blackboard before child B reads it each cycle. If the
# action terminates (permanent loss / abort) it returns FAILURE, the Parallel
# returns FAILURE, and the follow process ends cleanly.
#

import py_trees

from behavior_tree.TemplateNodes.TrackPersonAction import BtNode_TrackPersonAction
from behavior_tree.FollowPerson.nodes import (
    BtNode_PublishFollowGoal,
    BtNode_ReacqAnnounce,
)


def create_follow_person_tree(target_frame: str = "") -> py_trees.behaviour.Behaviour:
    """Build and return the follow-person tree root.

    Args:
        target_frame: TF frame for the tracked position output. Defaults to
            ``""`` (empty) so the tracker keeps the camera frame and performs
            **no** per-frame TF lookup. Do NOT set this to ``"map"`` (or any
            frame) unless a TF source for that frame is actually running: the
            tracker blocks ~0.2 s/frame on a lookup that never resolves, which
            collapses the tracking loop from ~30 Hz to ~5 Hz and starves
            reacquisition. The follow demo (dummy_nav stub, no nav stack) has no
            ``map`` frame, so ``""`` is the correct default.

    Returns:
        The Parallel root behaviour (compatible with ``run_tree``).
    """
    track = BtNode_TrackPersonAction(
        name="Track Person",
        target_frame=target_frame,
    )

    publish_goal = BtNode_PublishFollowGoal(name="Publish Follow Goal")
    announce = BtNode_ReacqAnnounce(name="Reacq Announce")

    reactions = py_trees.composites.Sequence(
        name="Follow Reactions",
        memory=False,
        children=[publish_goal, announce],
    )

    root = py_trees.composites.Parallel(
        name="Follow Person",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[track, reactions],
    )
    return root
