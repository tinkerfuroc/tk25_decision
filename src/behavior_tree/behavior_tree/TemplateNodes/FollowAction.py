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
# Follow Action Node Module
# =========================
#
# This module provides a behavior tree node for the continuous person-following
# navigation action (``tinker_nav_msgs/action/Follow``, served by the follow
# executive at ``follow_server``). It inherits from ActionHandler and adapts it
# for continuous-action behavior — the action runs until the follow executive
# decides the person is stationary (SUCCESS) or until the parent tree cancels.
#
# It is the navigation-side twin of ``BtNode_TrackPersonAction``: the tracker
# node keeps ``/track_person`` alive and refreshes the ``track/*`` blackboard
# keys, while this node keeps ``follow_server`` alive and refreshes the
# ``follow/*`` keys. Both sit beside each other under the follow-person Parallel
# root, neither gating the other.
#
# Classes
# -------
# FollowFeedbackBuffer
#     Thread-safe buffer for accumulating Follow feedback between behavior tree
#     ticks (feedback arrives faster than the tree ticks).
# BtNode_FollowAction
#     Continuous follow-navigation node with blackboard output.
#
# Blackboard Keys (configurable)
# ------------------------------
# follow/state     - int (uint8): follow executive state (mirrors
#                    following.state_machine.FollowState)
# follow/distance  - float: distance to the person in metres (-1.0 if unknown)
# follow/reacq     - int (uint8): passthrough reacquisition state
#                    (0 TRACKING / 1 PASSIVE / 2 NEEDS_HELP / 255 INACTIVE)
# follow/goal_held - bool: True on a tick where the follow executive reused its
#                    cached fallback goal or skipped dispatch (no fresh reachable
#                    standoff); False otherwise. Lets the tree announce / wait
#                    while the executive is holding rather than actively closing.
#
# Mock Mode
# ---------
# Supports KEYPRESS and IMMEDIATE modes via mock_config.json (registered under
# the ``navigation`` subsystem). In mock mode the node seeds synthetic
# blackboard values (state=TRACKING, distance, reacq=TRACKING) and stays RUNNING
# per the package's continuous-action mock conventions.
#

import threading
import time
from typing import Any

import py_trees
from py_trees.common import Status

from behavior_tree.TemplateNodes.ActionBase import ActionHandler

# Follow feedback state code mirrored for mock synthetic values: the follow
# executive reports TRACKING (1) while actively following. Kept as a literal so
# this module does not depend on the navigation package at import time.
MOCK_FOLLOW_STATE_TRACKING = 1
# Passthrough reacquisition state code for "actively tracking".
REACQ_TRACKING = 0


class FollowFeedbackBuffer:
    """Thread-safe buffer for storing Follow feedback between behavior tree ticks.

    The follow executive publishes feedback (~10 Hz) faster than the behavior
    tree ticks (~4 Hz). This buffer accumulates feedback and hands the latest
    state to the node on every tick.

    Attributes:
        _lock: Thread lock for safe concurrent access.
        _state: Latest follow-executive state (uint8).
        _distance: Latest distance to the person in metres (-1.0 if unknown).
        _reacq: Latest passthrough reacquisition state (uint8).
        _breadcrumbs_pending: Latest pending-breadcrumb count.
        _goal_held: Latest goal-held flag (True when the executive reused its
            cached fallback goal or skipped dispatch this tick).
        _feedback_count: Total feedbacks received (for debugging).
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state: int = 0
        self._distance: float = -1.0
        self._reacq: int = REACQ_TRACKING
        self._breadcrumbs_pending: int = 0
        self._goal_held: bool = False
        self._feedback_count: int = 0

    def update(self, feedback) -> None:
        """Update buffer with new Follow feedback (thread-safe).

        Args:
            feedback: Follow feedback message.
        """
        with self._lock:
            self._feedback_count += 1
            self._state = int(getattr(feedback, "state", 0))
            self._distance = float(getattr(feedback, "distance_to_person", -1.0))
            self._reacq = int(getattr(feedback, "reacq_state", REACQ_TRACKING))
            self._breadcrumbs_pending = int(
                getattr(feedback, "breadcrumbs_pending", 0)
            )
            # getattr default keeps this safe against older follow_server builds
            # whose Follow.action has no goal_held field (additive feedback field).
            self._goal_held = bool(getattr(feedback, "goal_held", False))

    def get_state(self) -> tuple:
        """Get the current follow state (thread-safe).

        Returns:
            Tuple of (state, distance, reacq, breadcrumbs_pending, goal_held,
            feedback_count).
        """
        with self._lock:
            return (
                self._state,
                self._distance,
                self._reacq,
                self._breadcrumbs_pending,
                self._goal_held,
                self._feedback_count,
            )

    def clear(self) -> None:
        """Clear buffer (thread-safe)."""
        with self._lock:
            self._state = 0
            self._distance = -1.0
            self._reacq = REACQ_TRACKING
            self._breadcrumbs_pending = 0
            self._goal_held = False
            self._feedback_count = 0


class BtNode_FollowAction(ActionHandler):
    """Continuous follow-navigation using the ``Follow`` ROS2 action.

    Inherits from ActionHandler but adapts it for continuous-action behavior:
    the follow executive runs the goal until the person is stationary (the
    server succeeds) or the parent tree cancels.

    Behavior:
        - Returns RUNNING while the executive is following.
        - Writes feedback to the blackboard on every tick.
        - Returns SUCCESS when the action succeeds (person stationary, robot
          parked behind) or when cancelled (normal termination).
        - Returns FAILURE if the action aborts or the goal is rejected.

    Blackboard Output:
        - follow/state: follow executive state (uint8).
        - follow/distance: distance to the person in metres (-1.0 if unknown).
        - follow/reacq: passthrough reacquisition state (uint8).
        - follow/goal_held: True when the executive reused its cached fallback
          goal or skipped dispatch this tick (no fresh reachable standoff).

    Mock Mode:
        - Seeds synthetic blackboard values (state=TRACKING, reacq=TRACKING).
        - KEYPRESS mode: Runs until the parent tree cancels.
        - IMMEDIATE mode: Auto-completes after a few ticks.

    Example:
        >>> follow = BtNode_FollowAction(
        ...     name="Follow Navigation",
        ...     use_breadcrumbs=True,
        ...     timeout=0.0,
        ... )
        >>> # Use beside the tracker node under the follow Parallel root.
    """

    # Default blackboard keys
    DEFAULT_BB_KEY_STATE = "follow/state"
    DEFAULT_BB_KEY_DISTANCE = "follow/distance"
    DEFAULT_BB_KEY_REACQ = "follow/reacq"
    DEFAULT_BB_KEY_GOAL_HELD = "follow/goal_held"

    def __init__(
        self,
        name: str,
        use_breadcrumbs: bool = True,
        timeout: float = 0.0,
        standoff_distance: float = 0.0,
        bb_key_state: str = DEFAULT_BB_KEY_STATE,
        bb_key_distance: str = DEFAULT_BB_KEY_DISTANCE,
        bb_key_reacq: str = DEFAULT_BB_KEY_REACQ,
        bb_key_goal_held: str = DEFAULT_BB_KEY_GOAL_HELD,
        action_name: str = "follow_server",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        """Initialize the Follow action node.

        Args:
            name: Node name for the behavior tree.
            use_breadcrumbs: Route the robot through the person's own trail.
            timeout: Wall-clock seconds; 0 = no limit.
            standoff_distance: Standoff distance in metres; 0 = server default.
            bb_key_state: Blackboard key for the follow state.
            bb_key_distance: Blackboard key for the distance-to-person.
            bb_key_reacq: Blackboard key for the reacquisition state.
            bb_key_goal_held: Blackboard key for the goal-held flag.
            action_name: ROS2 action server name.
            wait_for_server_timeout_sec: Timeout for server connection.
        """
        # Initialize ActionHandler with None key (we manage the blackboard
        # ourselves) and None action_type (set in setup() from messages.py).
        super().__init__(
            name=name,
            action_type=None,
            action_name=action_name,
            key=None,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )

        # Goal configuration
        self.use_breadcrumbs = use_breadcrumbs
        self.timeout = timeout
        self.standoff_distance = standoff_distance

        # Blackboard keys
        self.bb_key_state = bb_key_state
        self.bb_key_distance = bb_key_distance
        self.bb_key_reacq = bb_key_reacq
        self.bb_key_goal_held = bb_key_goal_held

        # Feedback buffer for handling the fast feedback rate
        self._feedback_buffer = FollowFeedbackBuffer()

        # Statistics
        self._total_feedbacks = 0

        # Blackboard writer (setup in setup())
        self._bb_writer = None

        # Follow action type (imported in setup())
        self._follow_action = None

    def setup(self, **kwargs):
        """Setup action client and blackboard.

        Imports the Follow action type (handles mock fallback) and registers the
        blackboard keys this node writes.
        """
        # Import Follow action type from messages.py (handles mock fallback).
        from behavior_tree.messages import Follow
        self._follow_action = Follow
        self.action_type = Follow

        # Call parent setup (creates the action client if not in mock mode).
        super().setup(**kwargs)

        # Setup blackboard writer.
        self._bb_writer = self.attach_blackboard_client(name=f"{self.name}_bb")
        self._bb_writer.register_key(
            self.bb_key_state, access=py_trees.common.Access.WRITE
        )
        self._bb_writer.register_key(
            self.bb_key_distance, access=py_trees.common.Access.WRITE
        )
        self._bb_writer.register_key(
            self.bb_key_reacq, access=py_trees.common.Access.WRITE
        )
        self._bb_writer.register_key(
            self.bb_key_goal_held, access=py_trees.common.Access.WRITE
        )

        self.logger.debug(f"Setup complete for {self.name}")

    def initialise(self):
        """Initialize for a new follow session."""
        # Clear feedback buffer FIRST.
        self._feedback_buffer.clear()
        self._total_feedbacks = 0

        # Seed the blackboard with defaults BEFORE super().initialise()
        # (super().initialise() calls send_goal() which sets mock state).
        self._bb_writer.set(self.bb_key_state, 0, overwrite=True)
        self._bb_writer.set(self.bb_key_distance, -1.0, overwrite=True)
        self._bb_writer.set(self.bb_key_reacq, REACQ_TRACKING, overwrite=True)
        self._bb_writer.set(self.bb_key_goal_held, False, overwrite=True)

        # Now call parent — in mock mode send_goal() overwrites with mock data.
        super().initialise()

        self.logger.debug(f"Initialized {self.name}")

    def send_goal(self):
        """Send the Follow goal to the action server."""
        # Handle mock mode.
        if self.mock_mode:
            self.feedback_message = "MOCK: Follow goal sent"

            # Create a mock future that appears sent.
            class MockFuture:
                def done(self):
                    return True

            self.send_goal_future = MockFuture()

            # Seed synthetic blackboard data (executive is "following").
            self._bb_writer.set(
                self.bb_key_state, MOCK_FOLLOW_STATE_TRACKING, overwrite=True
            )
            self._bb_writer.set(self.bb_key_distance, 1.5, overwrite=True)
            self._bb_writer.set(self.bb_key_reacq, REACQ_TRACKING, overwrite=True)
            self._bb_writer.set(self.bb_key_goal_held, False, overwrite=True)

            self.logger.info("MOCK: Follow initialized with synthetic state")
            return

        # Real mode — send the actual goal.
        goal = self._follow_action.Goal()
        goal.timeout = float(self.timeout)
        goal.use_breadcrumbs = bool(self.use_breadcrumbs)
        goal.standoff_distance = float(self.standoff_distance)

        self.send_goal_request(goal)
        self.feedback_message = (
            f"Sent Follow goal (use_breadcrumbs={self.use_breadcrumbs}, "
            f"timeout={self.timeout})"
        )
        self.logger.info(self.feedback_message)

    def feedback_callback(self, msg: Any):
        """Process feedback from the Follow action.

        Overrides ActionHandler.feedback_callback() to handle Follow-specific
        feedback fields (state / distance_to_person / reacq_state).

        Args:
            msg: Feedback message wrapper from the action server.
        """
        feedback = msg.feedback
        self.last_feedback_time = time.time()

        # Update feedback buffer (thread-safe).
        self._feedback_buffer.update(feedback)
        self._total_feedbacks += 1

        # Don't call super() — continuous action handles feedback differently.

    def regular_update(self) -> Status:
        """Called while the action is running.

        Writes buffered feedback to the blackboard and returns RUNNING to keep
        the follow active.

        Returns:
            Always returns RUNNING to maintain continuous following.
        """
        state, distance, reacq, breadcrumbs_pending, goal_held, count = (
            self._feedback_buffer.get_state()
        )

        # Update the blackboard.
        self._bb_writer.set(self.bb_key_state, state, overwrite=True)
        self._bb_writer.set(self.bb_key_distance, distance, overwrite=True)
        self._bb_writer.set(self.bb_key_reacq, reacq, overwrite=True)
        self._bb_writer.set(self.bb_key_goal_held, goal_held, overwrite=True)

        if distance >= 0.0:
            self.feedback_message = (
                f"Following (state {state}, {distance:.2f} m, reacq {reacq}, "
                f"{breadcrumbs_pending} crumbs pending"
                f"{', HOLDING' if goal_held else ''})"
            )
        else:
            self.feedback_message = (
                f"Following (state {state}, distance unknown, reacq {reacq})"
            )

        # Always return RUNNING — continuous action never completes here.
        return py_trees.common.Status.RUNNING

    def process_result(self) -> Status:
        """Handle the action result (only called on success/cancel/abort).

        For continuous actions, this is only reached when:
            - Goal succeeded (person stationary — SUCCESS).
            - Goal was cancelled (normal termination — SUCCESS).
            - Goal was aborted (unexpected termination — FAILURE).

        Returns:
            SUCCESS if cancelled or succeeded, FAILURE otherwise.
        """
        from behavior_tree.messages import action_msgs

        if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = "Follow completed (person stationary)"
            self.logger.info("Follow action succeeded")
            return py_trees.common.Status.SUCCESS
        elif self.result_status == action_msgs.GoalStatus.STATUS_CANCELED:
            self.feedback_message = "Follow cancelled successfully"
            self.logger.info("Follow action cancelled")
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Follow failed with status {self.result_status}"
            self.logger.warning(
                f"Follow action failed with status {self.result_status}"
            )
            return py_trees.common.Status.FAILURE

    def update(self) -> Status:
        """Main update method — overrides ActionHandler.update() for mock mode.

        In mock mode, maintains continuous following behavior until cancelled.
        In real mode, delegates to the parent update logic.

        Returns:
            Current node status (RUNNING, SUCCESS, or FAILURE).
        """
        # In mock mode, keep RUNNING until the parent tree cancels us. The
        # parent controls the lifecycle via cancellation (INVALID -> terminate),
        # so we never self-complete here; a mock "success" keypress is folded
        # back into RUNNING to preserve the continuous-action contract.
        if self.mock_mode:
            # Use the keypress interaction from the base class.
            keypress_result = self.wait_for_keypress_in_mock()

            # For continuous following, override SUCCESS to keep running — the
            # parent tree controls lifecycle via cancellation.
            if keypress_result == py_trees.common.Status.SUCCESS:
                self.feedback_message = (
                    "MOCK: Following active (parent tree controls termination)"
                )
                return py_trees.common.Status.RUNNING

            return keypress_result

        # Real mode — use the parent's update logic.
        return super().update()

    def terminate(self, new_status: Status) -> None:
        """Handle node termination.

        Called when the node is interrupted or completed. Logs final statistics.

        Args:
            new_status: The status the node is transitioning to.
        """
        self.logger.info(
            f"Follow terminated with status {new_status}, "
            f"received {self._total_feedbacks} feedbacks"
        )
        super().terminate(new_status)
