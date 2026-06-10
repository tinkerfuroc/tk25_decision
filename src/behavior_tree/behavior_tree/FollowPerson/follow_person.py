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
# Builds the follow-person tree: a Parallel root with three children, each
# long-running and independent.
#
#   Parallel(SuccessOnAll, synchronise=False)
#   ├── BtNode_TrackPersonAction        (keeps /track_person alive; writes track/*)
#   ├── BtNode_FollowAction             (keeps follow_server alive; writes follow/*)
#   └── Sequence(memory=False)
#       └── BtNode_ReacqAnnounce        (reacq-driven voice announcements)
#
# The tracker (child A) refreshes the ``track/*`` blackboard keys, the follow
# executive node (child B) keeps the navigation ``Follow`` action alive and
# refreshes the ``follow/*`` keys, and the reactions Sequence (child C) reacts
# to the tracker state each tick. The navigation stack now consumes the
# tracker's ``/target_points`` topic directly, so there is no per-tick follow
# goal publisher — ``BtNode_FollowAction`` drives navigation via the long-running
# ``follow_server`` action instead.
#
# If either long-running action terminates (permanent loss / abort) it returns
# FAILURE, the Parallel returns FAILURE, and the follow process ends cleanly.
#

import py_trees

from behavior_tree.TemplateNodes.TrackPersonAction import BtNode_TrackPersonAction
from behavior_tree.TemplateNodes.FollowAction import BtNode_FollowAction
from behavior_tree.FollowPerson.nodes import (
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

    follow = BtNode_FollowAction(
        name="Follow Navigation",
        use_breadcrumbs=True,
        timeout=0.0,
    )
    announce = BtNode_ReacqAnnounce(name="Reacq Announce")

    reactions = py_trees.composites.Sequence(
        name="Follow Reactions",
        memory=False,
        children=[announce],
    )

    root = py_trees.composites.Parallel(
        name="Follow Person",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=[track, follow, reactions],
    )
    return root
