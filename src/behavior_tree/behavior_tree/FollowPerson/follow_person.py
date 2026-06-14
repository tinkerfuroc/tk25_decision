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


def create_follow_person_tree(
    target_frame: str | None = None,
    enable_navigation: bool = True,
    use_breadcrumbs: bool = False,
) -> py_trees.behaviour.Behaviour:
    """Build and return the follow-person tree root.

    Args:
        target_frame: TF frame for the tracked position output, written to the
            ``TrackPerson`` goal so the tracker transforms ``/target_points``
            into this frame before publishing. Defaults to ``None``, which
            resolves by mode: ``"map"`` when ``enable_navigation`` is True,
            ``""`` (camera frame) when it is False.

            Rationale — emit the person point in ``map`` directly so every
            consumer (including ``follow_server``, whose ``working_frame`` is
            also ``map``) reads a map-frame point with no further TF transform.
            That is only safe when a ``map`` TF source is live: with navigation
            enabled the follow pipeline runs Nav2 + localization (AMCL/SLAM), so
            ``map``→camera resolves immediately. In the vision+audio-only mode
            (``enable_navigation=False``) there is NO localization, so requesting
            ``map`` would make the tracker block ~0.2 s/frame on a failing lookup
            — collapsing the ~30 Hz loop to ~5 Hz, starving reacquisition, and
            dropping every point (the v2.2.6 regression). There it stays in the
            camera frame. Pass an explicit string to override either default
            (e.g. ``""`` to force camera frame even with nav, or ``"odom"`` for
            out-of-arena SLAM following where ``follow_server.working_frame`` is
            ``odom``). (The legacy ``dummy_nav`` stub on ``/follow_target`` no
            longer participates — its topic has no publisher since the
            2026-06-10 rewire.)
        enable_navigation: When True (default) the tree includes the
            ``BtNode_FollowAction`` child that drives ``follow_server`` (full
            pipeline). When False the tree is built WITHOUT that child
            (``Parallel[track, reactions]``) — the vision+audio-only mode: the
            tracker stays alive and the reacq announcer fires, but the robot base
            never moves (no ``Follow`` goal is dispatched). Nothing reads
            ``follow/*``, so the no-nav tree is self-consistent.
        use_breadcrumbs: Route the robot through the person's own dropped trail
            (``follow_server`` NavigateThroughPoses). Defaults to ``False`` for
            **open following**: in open space the single-goal standoff pursuit
            (NavigateToPose, re-planned to the person's live position at 2 Hz)
            tracks a moving person directly, and the long, accumulated breadcrumb
            corridor otherwise pins the robot to a stale route it never advances
            on (observed on long open routes: crumbs pile to the cap while the
            robot fails to pursue). Set ``True`` only for cluttered/doorway
            following, where threading the person's exact trail is what gets the
            robot through the gap. Ignored when ``enable_navigation`` is False.

    Returns:
        The Parallel root behaviour (compatible with ``run_tree``).
    """
    # Resolve the tracker output frame by mode (see the target_frame docstring):
    # map when navigation+localization are up, camera frame otherwise. An
    # explicit target_frame (incl. "") always wins.
    if target_frame is None:
        target_frame = "map" if enable_navigation else ""

    track = BtNode_TrackPersonAction(
        name="Track Person",
        target_frame=target_frame,
    )
    announce = BtNode_ReacqAnnounce(name="Reacq Announce")
    reactions = py_trees.composites.Sequence(
        name="Follow Reactions",
        memory=False,
        children=[announce],
    )

    children = [track]
    if enable_navigation:
        follow = BtNode_FollowAction(
            name="Follow Navigation",
            use_breadcrumbs=use_breadcrumbs,
            timeout=0.0,
        )
        children.append(follow)
    children.append(reactions)

    root = py_trees.composites.Parallel(
        name="Follow Person",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=children,
    )
    return root
