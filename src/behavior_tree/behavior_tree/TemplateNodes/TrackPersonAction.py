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
# TrackPerson Action Node Module
# ===============================
#
# This module provides a behavior tree node for continuous person tracking
# using the TrackPerson ROS2 action. It inherits from ActionHandler and adapts
# it for continuous action behavior.
#
# Classes
# -------
# FeedbackBuffer
#     Thread-safe buffer for accumulating feedback between behavior tree ticks.
# BtNode_TrackPersonAction
#     Continuous person tracking node with blackboard output.
#
# Blackboard Keys (configurable)
# ------------------------------
# track/person_position    - PointStamped: Target position in target_frame
# track/person_id          - int: Tracking ID assigned by action server
# track/target_lost        - bool: True if target is currently lost
# track/transform_success  - bool: True if TF transformation succeeded
#
# Mock Mode
# ---------
# Supports KEYPRESS and IMMEDIATE modes via mock_config.json.
# In mock mode, simulates a stationary target at (2.0, 0.0, 1.0) in target_frame.
#

import threading
import copy
import time
from typing import Optional, Any

import py_trees
from py_trees.common import Status

from behavior_tree.TemplateNodes.ActionBase import ActionHandler


class FeedbackBuffer:
    """
    Thread-safe buffer for storing feedback messages between behavior tree ticks.

    Since TrackPerson action sends feedback faster than behavior tree ticks
    (typically 30Hz feedback vs 4Hz ticks), this buffer accumulates all
    feedback messages and provides the latest valid state when queried.

    Attributes:
        _lock: Thread lock for safe concurrent access
        _latest_position: Most recent valid target position
        _track_id: Current tracking ID
        _target_lost: Whether target is currently lost
        _transform_success: Whether TF transformation succeeded
        _feedback_count: Total feedbacks received (for debugging)
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._latest_position: Optional[Any] = None
        self._track_id: int = -1
        self._target_lost: bool = True
        self._transform_success: bool = False
        self._feedback_count: int = 0

    def update(self, feedback) -> None:
        """
        Update buffer with new feedback (thread-safe).

        Args:
            feedback: TrackPerson feedback message
        """
        with self._lock:
            self._feedback_count += 1
            self._target_lost = feedback.target_lost
            self._track_id = feedback.target_track_id
            self._transform_success = feedback.is_transformation_successful

            # Only update position if transformation succeeded and target not lost
            if feedback.is_transformation_successful and not feedback.target_lost:
                if feedback.target_position is not None:
                    self._latest_position = copy.deepcopy(feedback.target_position)

    def get_state(self) -> tuple:
        """
        Get current tracking state (thread-safe).

        Returns:
            Tuple of (position, track_id, target_lost, transform_success, feedback_count)
        """
        with self._lock:
            return (
                copy.deepcopy(self._latest_position),
                self._track_id,
                self._target_lost,
                self._transform_success,
                self._feedback_count
            )

    def clear(self) -> None:
        """Clear buffer (thread-safe)."""
        with self._lock:
            self._latest_position = None
            self._track_id = -1
            self._target_lost = True
            self._transform_success = False
            self._feedback_count = 0


class BtNode_TrackPersonAction(ActionHandler):
    """
    Continuous person tracking using the TrackPerson ROS2 action.

    This node inherits from ActionHandler but adapts it for continuous action
    behavior where the action runs indefinitely until cancelled.

    Behavior:
        - Returns RUNNING while actively tracking
        - Writes feedback to blackboard on every tick
        - Returns SUCCESS when cancelled (normal termination)
        - Returns FAILURE if action aborts or goal is rejected

    Blackboard Output:
        - position: Latest target position (PointStamped)
        - track_id: Tracking ID from action server
        - target_lost: Whether target is currently lost
        - transform_success: Whether TF transformation succeeded

    Mock Mode:
        - Simulates tracking with stationary target at (2.0, 0.0, 1.0)
        - KEYPRESS mode: Runs until parent tree cancels
        - IMMEDIATE mode: Auto-completes after a few ticks

    Example:
        >>> track_node = BtNode_TrackPersonAction(
        ...     name="Track Person",
        ...     target_frame="map"
        ... )
        >>> # Use in behavior tree - will run until cancelled
    """

    # Default blackboard keys
    DEFAULT_BB_KEY_POSITION = "track/person_position"
    DEFAULT_BB_KEY_TRACK_ID = "track/person_id"
    DEFAULT_BB_KEY_TARGET_LOST = "track/target_lost"
    DEFAULT_BB_KEY_TRANSFORM_SUCCESS = "track/transform_success"

    def __init__(
        self,
        name: str,
        target_frame: str = "map",
        bb_key_position: str = DEFAULT_BB_KEY_POSITION,
        bb_key_track_id: str = DEFAULT_BB_KEY_TRACK_ID,
        bb_key_target_lost: str = DEFAULT_BB_KEY_TARGET_LOST,
        bb_key_transform_success: str = DEFAULT_BB_KEY_TRANSFORM_SUCCESS,
        return_rgb: bool = False,
        return_depth: bool = False,
        return_segment: bool = False,
        debug: bool = False,
        action_name: str = "track_person",
        wait_for_server_timeout_sec: float = -3.0
    ):
        """
        Initialize TrackPerson action node.

        Args:
            name: Node name for behavior tree
            target_frame: TF frame for position output (default: "map")
            bb_key_position: Blackboard key for target position
            bb_key_track_id: Blackboard key for tracking ID
            bb_key_target_lost: Blackboard key for target lost flag
            bb_key_transform_success: Blackboard key for transform success
            return_rgb: Request RGB image in feedback
            return_depth: Request depth image in feedback
            return_segment: Request segmentation in feedback
            debug: Enable debug visualization on server
            action_name: ROS2 action server name
            wait_for_server_timeout_sec: Timeout for server connection
        """
        # Initialize ActionHandler with None key (we handle blackboard ourselves)
        # Pass None for action_type - will be set in setup()
        super().__init__(
            name=name,
            action_type=None,
            action_name=action_name,
            key=None,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec
        )

        # Store configuration
        self.target_frame = target_frame
        self.bb_key_position = bb_key_position
        self.bb_key_track_id = bb_key_track_id
        self.bb_key_target_lost = bb_key_target_lost
        self.bb_key_transform_success = bb_key_transform_success
        self.return_rgb = return_rgb
        self.return_depth = return_depth
        self.return_segment = return_segment
        self.debug = debug

        # Feedback buffer for handling fast feedback rate
        self._feedback_buffer = FeedbackBuffer()

        # Statistics
        self._total_feedbacks = 0

        # Blackboard writer (setup in setup())
        self._bb_writer = None

        # TrackPerson action type (imported in setup())
        self._track_person_action = None

    def setup(self, **kwargs):
        """
        Setup action client and blackboard.

        Imports TrackPerson action type and sets up blackboard clients.
        """
        # Import TrackPerson action type from messages.py (handles mock fallback)
        from behavior_tree.messages import TrackPerson
        self._track_person_action = TrackPerson
        self.action_type = TrackPerson

        # Call parent setup (creates action client if not in mock mode)
        super().setup(**kwargs)

        # Setup blackboard writer
        self._bb_writer = self.attach_blackboard_client(name=f"{self.name}_bb")
        self._bb_writer.register_key(
            self.bb_key_position,
            access=py_trees.common.Access.WRITE
        )
        self._bb_writer.register_key(
            self.bb_key_track_id,
            access=py_trees.common.Access.WRITE
        )
        self._bb_writer.register_key(
            self.bb_key_target_lost,
            access=py_trees.common.Access.WRITE
        )
        self._bb_writer.register_key(
            self.bb_key_transform_success,
            access=py_trees.common.Access.WRITE
        )

        self.logger.debug(f"Setup complete for {self.name}")

    def initialise(self):
        """Initialize for new tracking session."""
        # Clear feedback buffer FIRST
        self._feedback_buffer.clear()
        self._total_feedbacks = 0

        # Initialize blackboard with default values BEFORE super().initialise()
        # (super().initialise() calls send_goal() which sets mock state)
        self._bb_writer.set(self.bb_key_target_lost, True, overwrite=True)
        self._bb_writer.set(self.bb_key_transform_success, False, overwrite=True)
        self._bb_writer.set(self.bb_key_track_id, -1, overwrite=True)

        # Now call parent - in mock mode, send_goal() will overwrite with valid mock data
        super().initialise()

        self.logger.debug(f"Initialized {self.name}")

    def send_goal(self):
        """Send TrackPerson goal to action server."""
        # Handle mock mode
        if self.mock_mode:
            self.feedback_message = "MOCK: TrackPerson goal sent"
            # Create mock future that appears sent
            class MockFuture:
                def done(self):
                    return True
            self.send_goal_future = MockFuture()

            # Initialize mock blackboard data
            self._bb_writer.set(self.bb_key_target_lost, False, overwrite=True)
            self._bb_writer.set(self.bb_key_transform_success, True, overwrite=True)
            self._bb_writer.set(self.bb_key_track_id, 1, overwrite=True)

            # Create mock position
            from geometry_msgs.msg import PointStamped
            mock_pos = PointStamped()
            mock_pos.header.frame_id = self.target_frame
            mock_pos.point.x = 2.0
            mock_pos.point.y = 0.0
            mock_pos.point.z = 1.0
            self._bb_writer.set(self.bb_key_position, mock_pos, overwrite=True)

            self.logger.info(f"MOCK: TrackPerson initialized with mock position")
            return

        # Real mode - send actual goal
        goal = self._track_person_action.Goal()
        goal.target_frame = self.target_frame
        goal.return_rgb_img = self.return_rgb
        goal.return_depth_img = self.return_depth
        goal.return_segment = self.return_segment
        goal.debug = self.debug

        self.send_goal_request(goal)
        self.feedback_message = f"Sent TrackPerson goal (target_frame={self.target_frame})"
        self.logger.info(f"Sent TrackPerson goal with target_frame={self.target_frame}")

    def feedback_callback(self, msg: Any):
        """
        Process feedback from TrackPerson action.

        Overrides ActionHandler.feedback_callback() to handle
        TrackPerson-specific feedback fields.

        Args:
            msg: Feedback message from action server
        """
        feedback = msg.feedback
        self.last_feedback_time = time.time()

        # Update feedback buffer (thread-safe)
        self._feedback_buffer.update(feedback)
        self._total_feedbacks += 1

        # Don't call super() - we handle feedback differently for continuous action

    def regular_update(self) -> Status:
        """
        Called while action is running.

        Writes buffered feedback to blackboard and returns RUNNING
        to keep the action active.

        Returns:
            Always returns RUNNING to maintain continuous tracking
        """
        # Get latest state from buffer
        position, track_id, target_lost, transform_success, count = \
            self._feedback_buffer.get_state()

        # Update blackboard
        self._bb_writer.set(self.bb_key_track_id, track_id, overwrite=True)
        self._bb_writer.set(self.bb_key_target_lost, target_lost, overwrite=True)
        self._bb_writer.set(self.bb_key_transform_success, transform_success, overwrite=True)

        if position is not None:
            self._bb_writer.set(self.bb_key_position, position, overwrite=True)

        # Build feedback message
        if position is not None and not target_lost:
            pos = position.point
            self.feedback_message = (
                f"Tracking ID {track_id} at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) "
                f"[{position.header.frame_id}]"
            )
        elif target_lost:
            self.feedback_message = f"Target lost (ID {track_id})"
        elif not transform_success:
            self.feedback_message = f"Transform failed to {self.target_frame}"
        else:
            self.feedback_message = f"Tracking active, waiting for position..."

        # Always return RUNNING - continuous action never completes
        return py_trees.common.Status.RUNNING

    def process_result(self) -> Status:
        """
        Handle action result (only called on cancel/abort).

        For continuous actions, this is only called when:
            - Goal was cancelled (SUCCESS - normal termination)
            - Goal was aborted (FAILURE - unexpected termination)

        Returns:
            SUCCESS if cancelled or succeeded, FAILURE otherwise
        """
        # Import action_msgs for status codes
        from behavior_tree.messages import action_msgs

        if self.result_status == action_msgs.GoalStatus.STATUS_CANCELED:
            self.feedback_message = "Tracking cancelled successfully"
            self.logger.info("TrackPerson action cancelled")
            return py_trees.common.Status.SUCCESS
        elif self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
            # Server returned success (unusual for continuous action)
            self.feedback_message = "Tracking completed"
            self.logger.info("TrackPerson action succeeded")
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Tracking failed with status {self.result_status}"
            self.logger.warn(f"TrackPerson action failed with status {self.result_status}")
            return py_trees.common.Status.FAILURE

    def update(self) -> Status:
        """
        Main update method - overrides ActionHandler.update() for mock mode.

        In mock mode, maintains continuous tracking behavior until cancelled.
        In real mode, delegates to parent update logic.

        Returns:
            Current node status (RUNNING, SUCCESS, or FAILURE)
        """
        # In mock mode, we need continuous behavior
        if self.mock_mode:
            # For continuous tracking in mock mode, we want to keep running
            # until the parent tree cancels us (INVALID status)

            # Check if we're being terminated
            if self.status == py_trees.common.Status.INVALID:
                return py_trees.common.Status.SUCCESS

            # Use the keypress interaction from base class
            keypress_result = self.wait_for_keypress_in_mock()

            # For continuous tracking, override SUCCESS to keep running
            # The parent tree controls lifecycle via cancellation
            if keypress_result == py_trees.common.Status.SUCCESS:
                # User pressed success key - but we keep running for continuous action
                self.feedback_message = "MOCK: Tracking active (parent tree controls termination)"
                return py_trees.common.Status.RUNNING

            return keypress_result

        # Real mode - use parent's update logic
        return super().update()

    def terminate(self, new_status: Status) -> None:
        """
        Handle node termination.

        Called when node is interrupted or completed. Logs final statistics.

        Args:
            new_status: The status the node is transitioning to
        """
        self.logger.info(
            f"TrackPerson terminated with status {new_status}, "
            f"received {self._total_feedbacks} feedbacks"
        )
        super().terminate(new_status)
