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
#   ├── FailureIsRunning(BtNode_TrackPersonAction)                  (/track_person; track/*)
#   ├── FailureIsRunning(BtNode_FollowAction)                       (follow_server; follow/*)
#   └── FailureIsRunning(Sequence[ReacqAnnounce, RecoveryScan])     (reacq-driven reactions)
#
# The tracker (child A) refreshes the ``track/*`` blackboard keys, the follow
# executive node (child B) keeps the navigation ``Follow`` action alive and
# refreshes the ``follow/*`` keys, and the reactions Sequence (child C) reacts
# to the tracker state each tick: ``BtNode_ReacqAnnounce`` speaks the PASSIVE
# slow-down nudge, and ``BtNode_RecoveryScan`` (NEEDS_HELP-gated) runs the
# two-pass head-scan recovery — ask the person to stop and sweep the head for
# re-lock (Pass 1), then ask the operator to raise a hand and sweep again with
# wave-reseed active (Pass 2), repeating Pass 2 until re-lock — the active
# escape from the indefinite NEEDS_HELP hold. The navigation stack now consumes the
# tracker's ``/target_points`` topic directly, so there is no per-tick follow
# goal publisher — ``BtNode_FollowAction`` drives navigation via the long-running
# ``follow_server`` action instead.
#
# NEVER MID-ABORT: every child is wrapped in ``FailureIsRunning`` so a child's
# action terminating (permanent loss -> reacq INACTIVE -> follow_server
# ``ABORTED_TARGET_LOST``; or ``ABORTED_NAV_FAILED``) becomes RUNNING, never
# FAILURE. The ``SuccessOnAll`` Parallel therefore cannot fail, so the tree STAYS
# ALIVE; the wrapped action re-initialises and re-dispatches its goal on the next
# tick, so the tracker re-acquires and the follow resumes when the person returns
# (e.g. after walking past the depth sensor's ~5.5 m range). The follow ends only
# on an explicit external stop, never on a transient loss.
#

import py_trees

from behavior_tree.nodes.TrackPersonAction import BtNode_TrackPersonAction
from behavior_tree.nodes.FollowAction import BtNode_FollowAction
from behavior_tree.components.following.nodes import (
    BtNode_ReacqAnnounce,
    BtNode_RecoveryScan,
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
    # Two-pass head-scan recovery owns the NEEDS_HELP loop the announcer's
    # PASSIVE nudge precedes: ask the person to stop and sweep the head for
    # re-lock (Pass 1), then ask the operator to raise a hand and sweep again
    # with wave-reseed active (Pass 2), repeating Pass 2 until re-lock — the
    # active escape from the indefinite NEEDS_HELP hold. SUCCESS when idle,
    # RUNNING while scanning; NEEDS_HELP-gated, inert otherwise.
    recovery_scan = BtNode_RecoveryScan(name="Recovery Scan")
    reactions = py_trees.composites.Sequence(
        name="Follow Reactions",
        memory=False,
        children=[announce, recovery_scan],
    )

    # NEVER MID-ABORT — keep the follow tree alive through transient losses.
    # SuccessOnAll returns FAILURE the instant ANY child fails, which would end
    # the whole follow. A child fails when its long-running action aborts: the
    # TrackPerson action ending (the person walks past the depth sensor's ~5.5 m
    # range / the RGB lock drops) makes the tracker publish reacq INACTIVE, which
    # drives follow_server to ABORTED_TARGET_LOST -> BtNode_FollowAction FAILURE.
    # Wrapping each child in FailureIsRunning converts that FAILURE to RUNNING, so
    # the Parallel never fails and the tree stays alive; on the next tick the
    # wrapped action re-initialises and re-dispatches, so the tracker re-acquires
    # and the follow resumes when the person is back in range. (A Sequence wrapper
    # would NOT help — a Sequence still propagates a child's FAILURE.) NOTE:
    # re-dispatching TrackPerson re-seeds the lock, so a lost operator may be
    # re-acquired as whoever is in frame — identity persistence on re-acquire is a
    # tracker concern, not the BT's.
    def _stay_alive(node):
        return py_trees.decorators.FailureIsRunning(
            name=f"{node.name} (stay alive)", child=node)

    children = [_stay_alive(track)]
    if enable_navigation:
        follow = BtNode_FollowAction(
            name="Follow Navigation",
            use_breadcrumbs=use_breadcrumbs,
            timeout=0.0,
        )
        children.append(_stay_alive(follow))
    children.append(_stay_alive(reactions))

    root = py_trees.composites.Parallel(
        name="Follow Person",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
        children=children,
    )
    return root
