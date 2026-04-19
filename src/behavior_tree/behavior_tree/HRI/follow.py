"""Follow-person subtree for HRI.

Composes tk26_vision's ``TrackPerson`` action (now publishing ``PointStamped``
natively on ``/target_points``) with tk26_nav's ``Follow`` (``tracking_server``)
into a lost-sight-tolerant behaviour. Loss recovery is expressed via atomic
condition/state leaves + built-in composites and decorators — no branching
semantics live inside any single leaf.

Tree shape::

    Retry(n_restart)
      └── Parallel(SuccessOnSelected[follow_branch])
             ├── vision_branch : Repeat(-1, BtNode_TrackPersonAction)
             ├── loss_updater  : BtNode_UpdateLossElapsed
             ├── dedup_reset   : BtNode_WriteBBIfVisible(
             │                       follow_short_announced, False)
             ├── long_sentinel : Sequence(memory=True)
             │      ├── BtNode_LossElapsedAtLeast(long_sec)
             │      ├── BtNode_Announce(announce_long)
             │      └── py_trees.behaviours.Failure()
             └── follow_branch : Retry(n_nav_retry)
                    └── Sequence(memory=True)
                          ├── BtNode_IsTargetVisible
                          └── Parallel(SuccessOnSelected[nav])
                                 ├── nav : BtNode_FollowAction
                                 └── short_guard : Sequence(memory=False)
                                        ├── BtNode_LossElapsedAtLeast(short_sec)
                                        ├── BtNode_FlagIsFalse(follow_short_announced)
                                        ├── BtNode_Announce(announce_short)
                                        ├── BtNode_SetFlag(follow_short_announced, True)
                                        └── py_trees.behaviours.Running()
"""

from __future__ import annotations

import time
from typing import Any

import py_trees
import action_msgs.msg as action_msgs  # GoalStatus

from behavior_tree.messages import Follow, TrackPerson
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.TemplateNodes.Audio import BtNode_Announce


# Blackboard keys — root-scoped per .claude/rules/behavior-tree.md.
BB_TARGET_LOST = "follow_target_lost"
BB_TARGET_POSITION = "follow_target_position"
BB_TRACK_ID = "follow_track_id"
BB_TRANSFORM_OK = "follow_transform_ok"

# Written by the loss-state updater, consumed by the watchdog gates.
BB_LOSS_ELAPSED_SEC = "follow_lost_elapsed_sec"
BB_SHORT_ANNOUNCED = "follow_short_announced"


# -----------------------------------------------------------------------------
# Drain-pending-cancel mixin
# -----------------------------------------------------------------------------

class _DrainPriorCancelMixin:
    """Serialise cancel → re-init for action wrappers under Repeat/Retry.

    ``ActionHandler.send_cancel_request`` (``ActionBase.py:605-617``) discards
    the cancel future, and ``ActionHandler.initialise`` immediately fires a
    fresh ``send_goal_async`` after a re-tick. When the outer ``Retry`` here
    restarts the parallel, the new goal therefore races the still-pending
    cancel of the old one. This mixin captures both the cancel future and the
    old result future on terminate, and defers ``send_goal`` until both have
    resolved (or a bounded timeout elapses).

    Why both futures: the cancel response just acknowledges the request was
    accepted. The server still treats the goal as active until the result
    future fires with ``STATUS_CANCELED``. Single-active-goal servers (e.g.
    ``person_track_node``) reject a new goal that arrives in that window.
    """

    # Subclasses may override.
    _drain_timeout_sec: float = 1.5

    def _drain_init(self):
        self._pending_cancel_future = None
        self._pending_result_future = None
        self._needs_drain = False
        self._drain_deadline = None

    def _capture_cancel(self):
        if self.goal_handle is None:
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_response_callback)
        self._pending_cancel_future = cancel_future
        # Hold a reference so we can wait for STATUS_CANCELED to land on the
        # server side. The old get_result_callback will fire into the still-
        # current self.result_status field; we defer initialise()'s reset
        # until after that fires so the write is harmless.
        self._pending_result_future = self.get_result_future
        self.feedback_message = "cancelling prior goal"

    def _drain_pending(self) -> bool:
        cf = self._pending_cancel_future
        rf = self._pending_result_future
        if cf is None and rf is None:
            return False
        cancel_done = cf is None or cf.done()
        result_done = rf is None or rf.done()
        if cancel_done and result_done:
            self._pending_cancel_future = None
            self._pending_result_future = None
            return False
        if self._drain_deadline is not None and time.time() >= self._drain_deadline:
            self.node.get_logger().warning(
                f"[{self.qualified_name}] cancel drain timed out after "
                f"{self._drain_timeout_sec:.1f}s "
                f"(cancel_done={cancel_done}, result_done={result_done}); "
                f"sending new goal regardless"
            )
            self._pending_cancel_future = None
            self._pending_result_future = None
            return False
        return True

    def _reset_action_state_without_send(self):
        """Mirror ``ActionHandler.initialise`` reset, but skip ``send_goal()``."""
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.result_message = None
        self.result_status = None
        self.result_status_string = None
        self.action_status = 0
        self.action_stage = None
        self.last_feedback_time = time.time()
        self.feedback_timeout = 10000.0
        self.counter = 0

    def get_result_callback(self, future):
        # rclpy ``Future.done()`` returns True as soon as the result is set,
        # but done-callbacks may be invoked later (deferred to the executor).
        # The OLD goal's callback can therefore fire AFTER we have already
        # reset state and sent a new goal, poisoning self.result_status with
        # the cancelled goal's terminal status. Discard those stale calls.
        if future is not self.get_result_future:
            return
        super().get_result_callback(future)


# -----------------------------------------------------------------------------
# Action / visibility leaves
# -----------------------------------------------------------------------------

class BtNode_TrackPersonAction(_DrainPriorCancelMixin, ActionHandler):
    """Wrap ``tk26_vision/track_person`` and fan feedback onto the blackboard.

    Feedback schema is ``{target_lost, target_track_id,
    is_transformation_successful, target_position}`` — not the canonical
    ``{stage, stage_name, status, delay_limit}``. ``feedback_callback`` is
    overridden to force ``action_status=0`` and keep a wide feedback timeout,
    matching the pattern used by ``BtNode_MaintainEyeContact`` in Vision.py.
    """

    def __init__(
        self,
        name: str,
        action_name: str = "track_person",
        target_frame: str = "map",
        target_point_topic: str = "/target_points",
        feedback_timeout_secs: float = 30.0,
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(name, TrackPerson, action_name, None, wait_for_server_timeout_sec)
        self._drain_init()
        self._target_frame = target_frame
        self._target_point_topic = target_point_topic
        self._feedback_timeout_secs = feedback_timeout_secs

        self._bb = self.attach_blackboard_client(name=self.name)
        for key in (BB_TARGET_LOST, BB_TARGET_POSITION, BB_TRACK_ID, BB_TRANSFORM_OK):
            self._bb.register_key(
                key=key,
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
            )

    def send_goal(self):
        if self.mock_mode:
            self.feedback_message = "MOCK: track_person goal sent"

            class _MockFuture:
                def done(self):
                    return True

            self.send_goal_future = _MockFuture()
            # Seed the blackboard so the follow branch is never blocked in mock.
            self._bb.set(BB_TARGET_LOST, False, overwrite=True)
            self._bb.set(BB_TRANSFORM_OK, True, overwrite=True)
            return

        goal = TrackPerson.Goal()
        goal.target_frame = self._target_frame
        goal.target_point_topic = self._target_point_topic
        goal.return_rgb_img = False
        goal.return_depth_img = False
        goal.return_segment = False
        goal.debug = False
        self.send_goal_request(goal)
        self.feedback_message = (
            f"track_person goal sent (frame={self._target_frame}, "
            f"topic={self._target_point_topic})"
        )

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        self._bb.set(
            BB_TARGET_LOST,
            bool(getattr(feedback, "target_lost", True)),
            overwrite=True,
        )
        self._bb.set(
            BB_TRACK_ID,
            int(getattr(feedback, "target_track_id", -1)),
            overwrite=True,
        )
        self._bb.set(
            BB_TRANSFORM_OK,
            bool(getattr(feedback, "is_transformation_successful", False)),
            overwrite=True,
        )
        pos = getattr(feedback, "target_position", None)
        if pos is not None:
            self._bb.set(BB_TARGET_POSITION, pos, overwrite=True)
        self.feedback_message = (
            f"track: lost={self._bb.get(BB_TARGET_LOST)} "
            f"id={self._bb.get(BB_TRACK_ID)}"
        )

    def process_result(self):
        if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "track_person SUCCESS (will re-issue)"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = (
            f"track_person ended with status {self.result_status_string}"
        )
        return py_trees.common.Status.FAILURE

    def initialise(self):
        cf = self._pending_cancel_future
        rf = self._pending_result_future
        cancel_done = cf is None or cf.done()
        result_done = rf is None or rf.done()
        if not (cancel_done and result_done):
            # State is intentionally NOT reset here — the old get_result
            # callback may still fire and we want it to land in the old
            # fields. update() resets right before sending the new goal.
            self._needs_drain = True
            self._drain_deadline = time.time() + self._drain_timeout_sec
            self.feedback_message = "draining prior cancel before new goal"
            return
        self._pending_cancel_future = None
        self._pending_result_future = None
        self._needs_drain = False
        self._drain_deadline = None
        super().initialise()

    def update(self):
        if self._needs_drain:
            if self._drain_pending():
                self.feedback_message = "draining prior cancel"
                return py_trees.common.Status.RUNNING
            self._needs_drain = False
            self._drain_deadline = None
            self._reset_action_state_without_send()
            self.send_goal()
            if self.send_goal_future is None:
                self.feedback_message = "send_goal failed after drain"
                return py_trees.common.Status.FAILURE
            self.last_feedback_time = time.time()
        return super().update()

    def terminate(self, new_status: py_trees.common.Status):
        if not self.mock_mode and self.goal_handle is not None:
            self._capture_cancel()
            # Neuter the base ActionHandler.terminate's own cancel — we just
            # sent ours. Keep self.get_result_future intact: the old result
            # callback firing into still-live fields is harmless and we wait
            # for it as part of the drain.
            self.goal_handle = None
        super().terminate(new_status)


class BtNode_FollowAction(ActionHandler):
    """Wrap tk26_nav's ``tracking_server`` (``tinker_nav_msgs/Follow``).

    Feedback schema is ``{status, point_header, nav_goal_header}`` — not
    canonical. Override matches ``BtNode_TrackPersonAction``.
    Terminal mapping: server ``goal_handle.succeed()`` (REACHED) → SUCCESS;
    anything else → FAILURE.
    """

    def __init__(
        self,
        name: str,
        action_name: str = "tracking_server",
        follow_timeout_sec: float = 2.0,
        feedback_timeout_secs: float = 30.0,
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(name, Follow, action_name, None, wait_for_server_timeout_sec)
        # self._drain_init()
        self._follow_timeout_sec = follow_timeout_sec
        self._feedback_timeout_secs = feedback_timeout_secs

    def send_goal(self):
        if self.mock_mode:
            self.feedback_message = "MOCK: follow goal sent"

            class _MockFuture:
                def done(self):
                    return True

            self.send_goal_future = _MockFuture()
            return

        goal = Follow.Goal()
        goal.timeout = float(self._follow_timeout_sec)
        self.send_goal_request(goal)
        self.feedback_message = (
            f"follow goal sent (timeout={self._follow_timeout_sec}s)"
        )

    def feedback_callback(self, msg: Any):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        status = getattr(feedback, "status", "")
        self.feedback_message = f"follow: status={status}"

    def process_result(self):
        if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "follow reached"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = (
            f"follow ended with status {self.result_status_string}"
        )
        return py_trees.common.Status.FAILURE

    # def initialise(self):
    #     cf = self._pending_cancel_future
    #     rf = self._pending_result_future
    #     cancel_done = cf is None or cf.done()
    #     result_done = rf is None or rf.done()
    #     if not (cancel_done and result_done):
    #         self._needs_drain = True
    #         self._drain_deadline = time.time() + self._drain_timeout_sec
    #         self.feedback_message = "draining prior cancel before new goal"
    #         return
    #     self._pending_cancel_future = None
    #     self._pending_result_future = None
    #     self._needs_drain = False
    #     self._drain_deadline = None
    #     super().initialise()

    # def update(self):
    #     if self._needs_drain:
    #         if self._drain_pending():
    #             self.feedback_message = "draining prior cancel"
    #             return py_trees.common.Status.RUNNING
    #         self._needs_drain = False
    #         self._drain_deadline = None
    #         self._reset_action_state_without_send()
    #         self.send_goal()
    #         if self.send_goal_future is None:
    #             self.feedback_message = "send_goal failed after drain"
    #             return py_trees.common.Status.FAILURE
    #         self.last_feedback_time = time.time()
    #     return super().update()

    def terminate(self, new_status: py_trees.common.Status):
        if not self.mock_mode and self.goal_handle is not None:
            # self._capture_cancel()
            # self.goal_handle = None
            self.send_cancel_request()
        super().terminate(new_status)


class BtNode_IsTargetVisible(py_trees.behaviour.Behaviour):
    """SUCCESS once the target is visible; RUNNING while lost.

    Reads ``follow_target_lost`` written by :class:`BtNode_TrackPersonAction`.
    """

    def __init__(self, name: str = "IsTargetVisible"):
        super().__init__(name=name)
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(
            key=BB_TARGET_LOST,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", BB_TARGET_LOST),
        )

    def update(self):
        try:
            lost = self._bb.get(BB_TARGET_LOST)
        except KeyError:
            self.feedback_message = "awaiting first feedback"
            return py_trees.common.Status.RUNNING
        if lost:
            self.feedback_message = "target lost"
            return py_trees.common.Status.RUNNING
        self.feedback_message = "target visible"
        return py_trees.common.Status.SUCCESS


# -----------------------------------------------------------------------------
# Atomic watchdog state leaves — each does ONE thing.
# -----------------------------------------------------------------------------

class BtNode_UpdateLossElapsed(py_trees.behaviour.Behaviour):
    """Maintain ``follow_lost_elapsed_sec`` from ``follow_target_lost``.

    Responsibility: write the elapsed-loss timer. Always RUNNING.
    """

    def __init__(self, name: str = "UpdateLossElapsed"):
        super().__init__(name=name)
        self._first_lost_ts = None
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(
            key=BB_TARGET_LOST,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", BB_TARGET_LOST),
        )
        self._bb.register_key(
            key=BB_LOSS_ELAPSED_SEC,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", BB_LOSS_ELAPSED_SEC),
        )

    def initialise(self):
        self._first_lost_ts = None
        self._bb.set(BB_LOSS_ELAPSED_SEC, 0.0, overwrite=True)

    def update(self):
        try:
            lost = self._bb.get(BB_TARGET_LOST)
        except KeyError:
            lost = True
        if not lost:
            self._first_lost_ts = None
            self._bb.set(BB_LOSS_ELAPSED_SEC, 0.0, overwrite=True)
            self.feedback_message = "visible"
            return py_trees.common.Status.RUNNING
        now = time.time()
        if self._first_lost_ts is None:
            self._first_lost_ts = now
        elapsed = now - self._first_lost_ts
        self._bb.set(BB_LOSS_ELAPSED_SEC, elapsed, overwrite=True)
        self.feedback_message = f"lost {elapsed:.1f}s"
        return py_trees.common.Status.RUNNING


class BtNode_LossElapsedAtLeast(py_trees.behaviour.Behaviour):
    """Gate: SUCCESS iff ``follow_lost_elapsed_sec >= threshold_sec``.

    Responsibility: compare the elapsed timer against a threshold.
    """

    def __init__(self, name: str, threshold_sec: float):
        super().__init__(name=name)
        self._threshold_sec = float(threshold_sec)
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(
            key=BB_LOSS_ELAPSED_SEC,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", BB_LOSS_ELAPSED_SEC),
        )

    def update(self):
        try:
            elapsed = float(self._bb.get(BB_LOSS_ELAPSED_SEC))
        except (KeyError, TypeError, ValueError):
            elapsed = 0.0
        if elapsed >= self._threshold_sec:
            self.feedback_message = f"{elapsed:.1f}s ≥ {self._threshold_sec:.1f}s"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"{elapsed:.1f}s < {self._threshold_sec:.1f}s"
        return py_trees.common.Status.RUNNING


class BtNode_WriteBBIfVisible(py_trees.behaviour.Behaviour):
    """Write a value to a BB key while target is visible; always RUNNING.

    Responsibility: reset downstream flags when the target re-appears.
    """

    def __init__(self, name: str, key: str, value: Any):
        super().__init__(name=name)
        self._key = key
        self._value = value
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(
            key=BB_TARGET_LOST,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", BB_TARGET_LOST),
        )
        self._bb.register_key(
            key=key,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )

    def update(self):
        try:
            lost = self._bb.get(BB_TARGET_LOST)
        except KeyError:
            lost = True
        if not lost:
            self._bb.set(self._key, self._value, overwrite=True)
            self.feedback_message = f"visible: {self._key}={self._value!r}"
        else:
            self.feedback_message = "lost: no-op"
        return py_trees.common.Status.RUNNING


class BtNode_FlagIsFalse(py_trees.behaviour.Behaviour):
    """Gate: SUCCESS while a BB bool is False/unset, RUNNING while True.

    Responsibility: dedup — let downstream fire while the flag is clear.
    """

    def __init__(self, name: str, key: str):
        super().__init__(name=name)
        self._key = key
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(
            key=key,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )

    def update(self):
        try:
            value = self._bb.get(self._key)
        except KeyError:
            value = False
        if bool(value):
            self.feedback_message = f"{self._key}=True → gate closed"
            return py_trees.common.Status.RUNNING
        self.feedback_message = f"{self._key}=False → passing"
        return py_trees.common.Status.SUCCESS


class BtNode_SetFlag(py_trees.behaviour.Behaviour):
    """Latch a BB value once. Returns SUCCESS on the first tick after writing.

    Responsibility: write a single BB variable.
    """

    def __init__(self, name: str, key: str, value: Any = True):
        super().__init__(name=name)
        self._key = key
        self._value = value
        self._bb = self.attach_blackboard_client(name=name)
        self._bb.register_key(
            key=key,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )

    def update(self):
        self._bb.set(self._key, self._value, overwrite=True)
        self.feedback_message = f"{self._key}={self._value!r}"
        return py_trees.common.Status.SUCCESS


# -----------------------------------------------------------------------------
# Factory
# -----------------------------------------------------------------------------

def createFollowPerson(cfg: dict) -> py_trees.behaviour.Behaviour:
    """Assemble the follow-person subtree with two-stage loss recovery.

    All thresholds / action names / audio strings come from *cfg* (see
    ``HRI/config.py:FOLLOW_CONFIG`` for the canonical dictionary).
    """
    # --- vision branch: keep TrackPerson alive through the whole session ---
    vision_branch = py_trees.decorators.Repeat(
        name="TrackPerson keep-alive",
        child=BtNode_TrackPersonAction(
            name="TrackPerson",
            action_name=cfg.get("track_action_name", "track_person"),
            target_frame=cfg.get("target_frame", "map"),
            target_point_topic=cfg.get("target_point_topic", "/target_points"),
        ),
        num_success=-1,
    )

    # --- loss state ---
    loss_updater = BtNode_UpdateLossElapsed(name="update loss timer")
    dedup_reset = BtNode_WriteBBIfVisible(
        name="reset short-dedup on visible",
        key=BB_SHORT_ANNOUNCED,
        value=False,
    )

    # --- long sentinel: announce then FAIL to trigger outer restart ---
    long_sentinel = py_trees.composites.Sequence(
        name="long-lost → restart",
        memory=True,
        children=[
            BtNode_LossElapsedAtLeast(
                name=f"≥ {cfg.get('lost_long_sec', 15.0):.1f}s lost?",
                threshold_sec=float(cfg.get("lost_long_sec", 15.0)),
            ),
            BtNode_Announce(
                name="say come back",
                bb_source=None,
                message=cfg.get(
                    "announce_long",
                    "I've lost you. Please come back in front of Tinker.",
                ),
            ),
            py_trees.behaviours.Failure(name="trigger restart"),
        ],
    )
    long_sentinel_retry = py_trees.decorators.Retry("infinite retry", long_sentinel, -1)

    # --- follow branch: wait-visible → follow + short guard ---
    nav = BtNode_FollowAction(
        name="FollowAction",
        action_name=cfg.get("follow_action_name", "tracking_server"),
        follow_timeout_sec=float(cfg.get("follow_timeout_sec", 2.0)),
    )
    short_guard = py_trees.composites.Sequence(
        name="short-lost → please wait",
        memory=True,
        children=[
            BtNode_LossElapsedAtLeast(
                name=f"≥ {cfg.get('lost_short_sec', 5.0):.1f}s lost?",
                threshold_sec=float(cfg.get("lost_short_sec", 5.0)),
            ),
            # BtNode_FlagIsFalse(
            #     name="not yet announced?",
            #     key=BB_SHORT_ANNOUNCED,
            # ),
            BtNode_Announce(
                name="say wait",
                bb_source=None,
                message=cfg.get(
                    "announce_short",
                    "Please stop and wait for Tinker to catch up.",
                ),
            ),
            # BtNode_SetFlag(
            #     name="mark short announced",
            #     key=BB_SHORT_ANNOUNCED,
            #     value=True,
            # ),
            # py_trees.behaviours.Running(name="idle"),
        ],
    )
    short_guard_repeat = py_trees.decorators.Retry("infinite retry", short_guard, -1)
    active = py_trees.composites.Parallel(
        name="FollowActive",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(children=[nav]),
        children=[nav, short_guard_repeat],
    )

    follow_seq = py_trees.composites.Sequence(
        name="FollowSequence",
        memory=True,
        children=[
            BtNode_IsTargetVisible(name="wait until visible"),
            active,
        ],
    )
    follow_branch = py_trees.decorators.Retry(
        name="follow retry (transient nav LOST)",
        child=follow_seq,
        num_failures=int(cfg.get("n_nav_retry", 3)),
    )

    # --- outer parallel: vision + loss-state + long sentinel + follow branch ---
    outer_parallel = py_trees.composites.Parallel(
        name="FollowPerson",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[follow_branch],
        ),
        children=[
            vision_branch,
            loss_updater,
            dedup_reset,
            long_sentinel_retry,
            follow_branch,
        ],
    )

    return py_trees.decorators.Retry(
        name="FollowPerson restart (reanchor ReID)",
        child=outer_parallel,
        num_failures=int(cfg.get("n_restart", 2)),
    )


# -----------------------------------------------------------------------------
# Standalone entry point
# -----------------------------------------------------------------------------

def main():
    """Run the follow subtree in isolation for threshold tuning."""
    from behavior_tree.runtime import run_tree
    from .config import FOLLOW_CONFIG

    def build():
        root = py_trees.composites.Sequence(name="Follow-only test", memory=True)
        root.add_child(createFollowPerson(FOLLOW_CONFIG))
        return root

    run_tree(build, period_ms=500.0, title="Follow-only")
