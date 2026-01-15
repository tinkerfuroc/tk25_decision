#!/usr/bin/env python3
"""
Standalone Test Script for Follow Behavior Tree

This script is a self-contained test that doesn't depend on the full behavior_tree
package, avoiding import errors from other modules.

Usage:
    python3 standalone_test.py [mode]
    
    Modes: full, track_only, until_stopped (default: until_stopped)

Author: TinkerFuroc
"""

import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import action_msgs.msg as action_msgs

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
from nav2_msgs.action import NavigateToPose

# Import the TrackPerson action
from tinker_vision_msgs_26.action import TrackPerson

import sys
import time
import copy
import math
import threading
from typing import Any, List, Optional


# ============================================================================
# Constants
# ============================================================================

BB_KEY_TRACK_POSITION = "track/person_position"
BB_KEY_TRACK_ID = "track/person_id"
BB_KEY_TARGET_LOST = "track/target_lost"
BB_KEY_TRANSFORM_SUCCESS = "track/transform_success"
BB_KEY_FOLLOW_GOAL = "track/follow_goal"

DEFAULT_TARGET_FRAME = "map"
TICK_PERIOD_SEC = 0.250


# ============================================================================
# FeedbackBuffer - Thread-safe buffer for feedback between ticks
# ============================================================================

class FeedbackBuffer:
    """
    Thread-safe buffer for storing feedback messages between behavior tree ticks.
    
    Since the behavior tree ticks at 250ms intervals but feedback may arrive
    faster (e.g., 10Hz = 100ms), this buffer accumulates all feedback messages
    and provides the latest valid state when queried.
    """
    
    def __init__(self):
        self._lock = threading.Lock()
        self._feedbacks: List[Any] = []
        self._latest_valid_position: Optional[PointStamped] = None
        self._latest_transform_success: bool = False
        self._target_lost: bool = False
        self._track_id: int = -1
        self._feedback_count: int = 0
        
    def add_feedback(self, feedback: Any) -> None:
        """Add a feedback message to the buffer (thread-safe)."""
        with self._lock:
            self._feedbacks.append(feedback)
            self._feedback_count += 1
            
            # Update latest valid position if transform was successful
            if hasattr(feedback, 'is_transformation_successful') and feedback.is_transformation_successful:
                if hasattr(feedback, 'target_position'):
                    self._latest_valid_position = copy.deepcopy(feedback.target_position)
                    self._latest_transform_success = True
            
            # Always update these fields
            if hasattr(feedback, 'target_lost'):
                self._target_lost = feedback.target_lost
            if hasattr(feedback, 'target_track_id'):
                self._track_id = feedback.target_track_id
                
    def get_latest_state(self) -> dict:
        """
        Get the latest state and clear the buffer (thread-safe).
        
        Returns a dict with:
            - position: Latest valid PointStamped or None
            - transform_success: Whether last transform was successful
            - target_lost: Whether target is lost
            - track_id: Current track ID
            - feedback_count: Number of feedbacks received since last query
        """
        with self._lock:
            state = {
                'position': copy.deepcopy(self._latest_valid_position),
                'transform_success': self._latest_transform_success,
                'target_lost': self._target_lost,
                'track_id': self._track_id,
                'feedback_count': len(self._feedbacks)
            }
            # Clear the feedback list but keep the latest state
            self._feedbacks.clear()
            return state
            
    def clear(self) -> None:
        """Clear all buffered data."""
        with self._lock:
            self._feedbacks.clear()
            self._latest_valid_position = None
            self._latest_transform_success = False
            self._target_lost = False
            self._track_id = -1


# ============================================================================
# BtNode_TrackPersonAction - Main tracking action node
# ============================================================================

class BtNode_TrackPersonAction(py_trees.behaviour.Behaviour):
    """
    Behavior tree node that sends a TrackPerson action goal and processes feedback.
    
    This node:
    1. Sends a TrackPerson goal with the specified target_frame
    2. Buffers feedback messages between ticks (handles faster-than-tick feedback)
    3. Writes the latest valid position to the blackboard
    4. Returns RUNNING while tracking, SUCCESS if target lost, FAILURE on errors
    """
    
    def __init__(
        self,
        name: str,
        action_name: str = "track_person",
        target_frame: str = DEFAULT_TARGET_FRAME,
        bb_key_position: str = BB_KEY_TRACK_POSITION,
        bb_key_transform_success: str = BB_KEY_TRANSFORM_SUCCESS
    ):
        super().__init__(name)
        self.action_name = action_name
        self.target_frame = target_frame
        self.bb_key_position = bb_key_position
        self.bb_key_transform_success = bb_key_transform_success
        
        # Action client state
        self._action_client: Optional[ActionClient] = None
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._result_message = None
        self._result_status = None
        
        # Feedback buffer for handling multiple feedbacks per tick
        self._feedback_buffer = FeedbackBuffer()
        
        # ROS node reference
        self._node: Optional[Node] = None
        
    def setup(self, **kwargs) -> None:
        """Setup the action client."""
        try:
            self._node = kwargs.get('node')
            if self._node is None:
                # Try to get from py_trees_ros
                self._node = py_trees_ros.utilities.get_node()
                
            self._action_client = ActionClient(
                self._node,
                TrackPerson,
                self.action_name
            )
            self.logger.info(f"Waiting for action server '{self.action_name}'...")
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"Action server '{self.action_name}' not available")
            self.logger.info(f"Connected to action server '{self.action_name}'")
        except Exception as e:
            raise RuntimeError(f"Failed to setup TrackPersonAction: {e}")
            
    def initialise(self) -> None:
        """Initialize on first tick - send the goal."""
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._result_message = None
        self._result_status = None
        self._feedback_buffer.clear()
        
        # Create and send goal
        goal = TrackPerson.Goal()
        goal.target_frame = self.target_frame
        
        self.logger.info(f"Sending TrackPerson goal (target_frame: {self.target_frame})")
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
    def _goal_response_callback(self, future) -> None:
        """Handle goal response, proceed to listen for result if accepted."""
        if future.result() is None:
            self.logger.warning(f"Goal request failed: {future.exception()}")
            return
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.warning("TrackPerson goal rejected")
            return
        self.logger.info("TrackPerson goal accepted")
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
        
    def _get_result_callback(self, future) -> None:
        """Handle action result."""
        self._result_message = future.result()
        self._result_status = future.result().status
        
    def _feedback_callback(self, feedback_msg) -> None:
        """
        Handle feedback - add to buffer for processing on next tick.
        This may be called multiple times between behavior tree ticks.
        """
        self._feedback_buffer.add_feedback(feedback_msg.feedback)
        
    def update(self) -> py_trees.common.Status:
        """
        Process buffered feedback and update blackboard.
        Called once per behavior tree tick (every 250ms).
        """
        # Check for errors
        if self._send_goal_future is None:
            self.feedback_message = "No goal sent"
            return py_trees.common.Status.FAILURE
        if self._goal_handle is not None and not self._goal_handle.accepted:
            self.feedback_message = "Goal rejected"
            return py_trees.common.Status.FAILURE
        
        # Check if action completed
        if self._result_status is not None:
            self.feedback_message = "Tracking action completed"
            return py_trees.common.Status.SUCCESS
            
        # Still waiting for goal response (callback not yet called)
        if self._goal_handle is None:
            self.feedback_message = "Waiting for goal acceptance..."
            return py_trees.common.Status.RUNNING
            
        # Action is running, process buffered feedback
        state = self._feedback_buffer.get_latest_state()
        
        # Write to blackboard
        blackboard = py_trees.blackboard.Blackboard()
        
        if state['position'] is not None:
            blackboard.set(self.bb_key_position, state['position'])
            blackboard.set(self.bb_key_transform_success, state['transform_success'])
            
        blackboard.set(BB_KEY_TARGET_LOST, state['target_lost'])
        blackboard.set(BB_KEY_TRACK_ID, state['track_id'])
        
        # Build feedback message
        if state['position'] is not None:
            pos = state['position'].point
            self.feedback_message = (
                f"Tracking ID:{state['track_id']} "
                f"pos:({pos.x:.2f},{pos.y:.2f},{pos.z:.2f}) "
                f"transform:{state['transform_success']} "
                f"lost:{state['target_lost']} "
                f"fb_count:{state['feedback_count']}"
            )
        else:
            self.feedback_message = (
                f"Tracking ID:{state['track_id']} "
                f"no valid position yet "
                f"lost:{state['target_lost']}"
            )
            
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Cancel the action if we're being terminated."""
        if self._goal_handle is not None and self._goal_handle.accepted and self._result_status is None:
            self.logger.info("Cancelling TrackPerson action")
            self._goal_handle.cancel_goal_async()
        self._feedback_buffer.clear()


# ============================================================================
# BtNode_ProcessTrackPosition - Process position and create nav goal
# ============================================================================

class BtNode_ProcessTrackPosition(py_trees.behaviour.Behaviour):
    """
    Process the tracked person's position and create a navigation goal.
    
    Reads the position from blackboard, applies follow distance offset,
    and writes a PoseStamped goal for navigation.
    """
    
    def __init__(
        self,
        name: str,
        follow_distance: float = 1.0,
        bb_key_position: str = BB_KEY_TRACK_POSITION,
        bb_key_goal: str = BB_KEY_FOLLOW_GOAL,
        bb_key_transform_success: str = BB_KEY_TRANSFORM_SUCCESS
    ):
        super().__init__(name)
        self.follow_distance = follow_distance
        self.bb_key_position = bb_key_position
        self.bb_key_goal = bb_key_goal
        self.bb_key_transform_success = bb_key_transform_success
        
    def update(self) -> py_trees.common.Status:
        """Process position and create navigation goal."""
        blackboard = py_trees.blackboard.Blackboard()
        
        # Check if transform was successful (use try/except for missing key)
        try:
            transform_success = blackboard.get(self.bb_key_transform_success)
        except KeyError:
            transform_success = None
            
        if transform_success is None or not transform_success:
            self.feedback_message = "Transform not successful yet, skipping navigation"
            return py_trees.common.Status.SUCCESS  # Don't fail, just skip
            
        # Get position from blackboard
        try:
            position: Optional[PointStamped] = blackboard.get(self.bb_key_position)
        except KeyError:
            position = None
            
        if position is None:
            self.feedback_message = "No position data available"
            return py_trees.common.Status.SUCCESS  # Don't fail, just skip
            
        # Calculate goal position (stop at follow_distance from person)
        px, py = position.point.x, position.point.y
        distance = math.sqrt(px*px + py*py)
        
        if distance < self.follow_distance:
            self.feedback_message = f"Already within follow distance ({distance:.2f}m)"
            return py_trees.common.Status.SUCCESS
            
        # Calculate point at follow_distance from person, towards robot
        scale = (distance - self.follow_distance) / distance
        goal_x = px * scale
        goal_y = py * scale
        
        # Calculate orientation to face the person
        yaw = math.atan2(py - goal_y, px - goal_x)
        
        # Create PoseStamped goal
        goal = PoseStamped()
        goal.header = position.header
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simplified, only z rotation)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Write to blackboard
        blackboard.set(self.bb_key_goal, goal)
        
        self.feedback_message = f"Goal: ({goal_x:.2f}, {goal_y:.2f}) facing person at ({px:.2f}, {py:.2f})"
        return py_trees.common.Status.SUCCESS


# ============================================================================
# BtNode_CheckTargetStopped - Check if person has stopped moving
# ============================================================================

class BtNode_CheckTargetStopped(py_trees.behaviour.Behaviour):
    """
    Check if the tracked person has stopped moving for a duration.
    
    Returns SUCCESS when person has been stationary for stationary_duration,
    RUNNING while still monitoring, FAILURE if target lost.
    """
    
    def __init__(
        self,
        name: str,
        stationary_threshold: float = 0.3,  # meters
        stationary_duration: float = 3.0,   # seconds
        bb_key_position: str = BB_KEY_TRACK_POSITION
    ):
        super().__init__(name)
        self.stationary_threshold = stationary_threshold
        self.stationary_duration = stationary_duration
        self.bb_key_position = bb_key_position
        
        self._last_position: Optional[Point] = None
        self._stationary_start: Optional[float] = None
        
    def initialise(self) -> None:
        """Reset state on start."""
        self._last_position = None
        self._stationary_start = None
        
    def update(self) -> py_trees.common.Status:
        """Check if person is stationary."""
        blackboard = py_trees.blackboard.Blackboard()
        
        # Check if target is lost (handle missing key)
        try:
            target_lost = blackboard.get(BB_KEY_TARGET_LOST)
        except KeyError:
            target_lost = False
            
        if target_lost:
            self.feedback_message = "Target lost"
            return py_trees.common.Status.FAILURE
            
        # Get current position (handle missing key)
        try:
            position: Optional[PointStamped] = blackboard.get(self.bb_key_position)
        except KeyError:
            position = None
            
        if position is None:
            self.feedback_message = "No position data yet"
            return py_trees.common.Status.RUNNING
            
        current_pos = position.point
        current_time = time.time()
        
        # First position
        if self._last_position is None:
            self._last_position = copy.deepcopy(current_pos)
            self._stationary_start = current_time
            self.feedback_message = "Started monitoring position"
            return py_trees.common.Status.RUNNING
            
        # Calculate movement
        dx = current_pos.x - self._last_position.x
        dy = current_pos.y - self._last_position.y
        movement = math.sqrt(dx*dx + dy*dy)
        
        if movement > self.stationary_threshold:
            # Person moved, reset timer
            self._last_position = copy.deepcopy(current_pos)
            self._stationary_start = current_time
            self.feedback_message = f"Person moved {movement:.2f}m, resetting"
            return py_trees.common.Status.RUNNING
            
        # Check if stationary long enough
        stationary_time = current_time - self._stationary_start
        if stationary_time >= self.stationary_duration:
            self.feedback_message = f"Person stationary for {stationary_time:.1f}s - STOPPED"
            return py_trees.common.Status.SUCCESS
            
        self.feedback_message = f"Person stationary for {stationary_time:.1f}s / {self.stationary_duration:.1f}s"
        return py_trees.common.Status.RUNNING


# ============================================================================
# BtNode_SimpleNavigate - Simple navigation node
# ============================================================================

class BtNode_SimpleNavigate(py_trees.behaviour.Behaviour):
    """
    Simple navigation node that sends NavigateToPose goals.
    """
    
    def __init__(
        self,
        name: str,
        action_name: str = "navigate_to_pose",
        bb_key_goal: str = BB_KEY_FOLLOW_GOAL
    ):
        super().__init__(name)
        self.action_name = action_name
        self.bb_key_goal = bb_key_goal
        
        self._action_client: Optional[ActionClient] = None
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._result_message = None
        self._result_status = None
        self._node: Optional[Node] = None
        
    def setup(self, **kwargs) -> None:
        """Setup the action client."""
        try:
            self._node = kwargs.get('node')
            if self._node is None:
                self._node = py_trees_ros.utilities.get_node()
                
            self._action_client = ActionClient(
                self._node,
                NavigateToPose,
                self.action_name
            )
            self.logger.info(f"Waiting for action server '{self.action_name}'...")
            if not self._action_client.wait_for_server(timeout_sec=10.0):
                raise RuntimeError(f"Action server '{self.action_name}' not available")
            self.logger.info(f"Connected to action server '{self.action_name}'")
        except Exception as e:
            raise RuntimeError(f"Failed to setup NavigateAction: {e}")
            
    def initialise(self) -> None:
        """Send navigation goal."""
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._result_message = None
        self._result_status = None
        
        blackboard = py_trees.blackboard.Blackboard()
        try:
            goal_pose: Optional[PoseStamped] = blackboard.get(self.bb_key_goal)
        except KeyError:
            goal_pose = None
        
        if goal_pose is None:
            self.logger.warning("No goal pose on blackboard")
            return
            
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose
        
        self.logger.info(f"Sending NavigateToPose goal: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})")
        
        self._send_goal_future = self._action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._goal_response_callback)
        
    def _goal_response_callback(self, future) -> None:
        """Handle goal response."""
        if future.result() is None:
            self.logger.warning(f"Goal request failed: {future.exception()}")
            return
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.logger.warning("NavigateToPose goal rejected")
            return
        self.logger.info("NavigateToPose goal accepted")
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
        
    def _get_result_callback(self, future) -> None:
        """Handle action result."""
        self._result_message = future.result()
        self._result_status = future.result().status
        
    def update(self) -> py_trees.common.Status:
        """Check navigation status."""
        # No goal was sent (no goal pose on blackboard)
        if self._send_goal_future is None:
            self.feedback_message = "No goal pose available"
            return py_trees.common.Status.SUCCESS  # Return SUCCESS to not fail the tree
        
        # Goal was rejected
        if self._goal_handle is not None and not self._goal_handle.accepted:
            self.feedback_message = "Goal rejected"
            return py_trees.common.Status.FAILURE
        
        # Action completed
        if self._result_status is not None:
            self.feedback_message = "Navigation completed"
            return py_trees.common.Status.SUCCESS
            
        # Still waiting for goal acceptance
        if self._goal_handle is None:
            self.feedback_message = "Waiting for goal acceptance..."
            return py_trees.common.Status.RUNNING
            
        self.feedback_message = "Navigating..."
        return py_trees.common.Status.RUNNING
        
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Cancel navigation if needed."""
        if self._goal_handle is not None and self._goal_handle.accepted and self._result_status is None:
            self.logger.info("Cancelling navigation")
            self._goal_handle.cancel_goal_async()


# ============================================================================
# BtNode_CheckGoalAvailable - Guard condition for navigation
# ============================================================================

class BtNode_CheckGoalAvailable(py_trees.behaviour.Behaviour):
    """
    Guard condition that checks if a valid navigation goal is available.
    Returns SUCCESS if goal exists, FAILURE otherwise.
    Used as a guard in a Selector to skip navigation when no goal.
    """
    
    def __init__(
        self,
        name: str,
        bb_key_goal: str = BB_KEY_FOLLOW_GOAL
    ):
        super().__init__(name)
        self.bb_key_goal = bb_key_goal
        
    def update(self) -> py_trees.common.Status:
        """Check if goal is available."""
        blackboard = py_trees.blackboard.Blackboard()
        try:
            goal_pose = blackboard.get(self.bb_key_goal)
            if goal_pose is not None:
                self.feedback_message = "Goal available"
                return py_trees.common.Status.SUCCESS
        except KeyError:
            pass
        
        self.feedback_message = "No goal yet"
        return py_trees.common.Status.FAILURE


# ============================================================================
# BtNode_SkipNavigation - Dummy node that always succeeds (skip navigation)
# ============================================================================

class BtNode_SkipNavigation(py_trees.behaviour.Behaviour):
    """Dummy node that always succeeds - used to skip navigation when no goal."""
    
    def __init__(self, name: str = "Skip Navigation"):
        super().__init__(name)
        
    def update(self) -> py_trees.common.Status:
        self.feedback_message = "Skipping navigation (no goal)"
        return py_trees.common.Status.SUCCESS


# ============================================================================
# Tree Creation Functions
# ============================================================================

def createFollowPersonUntilStopped(
    follow_distance: float = 1.5,
    stationary_threshold: float = 0.3,
    stationary_duration: float = 5.0,
    target_frame: str = DEFAULT_TARGET_FRAME,
    action_name: str = "track_person",
    nav_action_name: str = "navigate_to_pose"
) -> py_trees.behaviour.Behaviour:
    """
    Create a behavior tree that follows a person until they stop.
    
    Tree structure:
    
    Parallel (SuccessOnOne) "Follow Until Stopped"
    ├── Sequence "Track and Follow" (keeps running)
    │   └── Parallel (SuccessOnAll) "Track Parallel"
    │       ├── Track Person (action, keeps running)
    │       └── Sequence "Process and Navigate"
    │           ├── Process Position (processes feedback, always succeeds)
    │           └── Selector "Navigate If Goal" (guard pattern)
    │               ├── Sequence "Do Navigate"
    │               │   ├── Check Goal Available (guard)
    │               │   └── Navigate to Follow (action)
    │               └── Skip Navigation (fallback, always succeeds)
    └── Check Stopped (succeeds when person stopped)
    """
    root = py_trees.composites.Parallel(
        name="Follow Until Stopped",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    
    # Branch 1: Track and follow
    follow_sequence = py_trees.composites.Sequence(name="Track and Follow", memory=False)
    
    track_parallel = py_trees.composites.Parallel(
        name="Track Parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    
    track = BtNode_TrackPersonAction(
        name="Track Person",
        action_name=action_name,
        target_frame=target_frame
    )
    
    process_and_nav = py_trees.composites.Sequence(name="Process and Navigate", memory=False)
    
    process = BtNode_ProcessTrackPosition(
        name="Process Position",
        follow_distance=follow_distance
    )
    
    # Guard pattern: Selector with guard condition
    # If guard succeeds (goal available) -> do navigation
    # If guard fails (no goal) -> skip navigation (fallback succeeds)
    navigate_if_goal = py_trees.composites.Selector(name="Navigate If Goal", memory=False)
    
    do_navigate = py_trees.composites.Sequence(name="Do Navigate", memory=False)
    check_goal = BtNode_CheckGoalAvailable(name="Check Goal")
    navigate = BtNode_SimpleNavigate(
        name="Navigate to Follow",
        action_name=nav_action_name
    )
    do_navigate.add_children([check_goal, navigate])
    
    skip_nav = BtNode_SkipNavigation(name="Skip Nav")
    
    navigate_if_goal.add_children([do_navigate, skip_nav])
    
    process_and_nav.add_children([process, navigate_if_goal])
    track_parallel.add_children([track, process_and_nav])
    follow_sequence.add_child(track_parallel)
    
    # Branch 2: Check if stopped
    check_stopped = BtNode_CheckTargetStopped(
        name="Check Stopped",
        stationary_threshold=stationary_threshold,
        stationary_duration=stationary_duration
    )
    
    root.add_children([follow_sequence, check_stopped])
    
    return root


def createTrackOnly(
    target_frame: str = DEFAULT_TARGET_FRAME,
    action_name: str = "track_person",
    timeout: float = 30.0
) -> py_trees.behaviour.Behaviour:
    """Create a simple tracking-only tree for testing."""
    root = py_trees.composites.Sequence(name="Track Only", memory=True)
    
    track = BtNode_TrackPersonAction(
        name="Track Person",
        action_name=action_name,
        target_frame=target_frame
    )
    
    # Add timeout
    timed_track = py_trees.decorators.Timeout(
        name=f"Track for {timeout}s",
        child=track,
        duration=timeout
    )
    
    root.add_child(timed_track)
    return root


# ============================================================================
# Test Node
# ============================================================================

class FollowTestNode(Node):
    """ROS2 node for running the Follow behavior tree test."""
    
    def __init__(self, test_mode: str = 'until_stopped'):
        super().__init__('follow_standalone_test')
        
        self.declare_parameter('mode', test_mode)
        self.test_mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.get_logger().info(f'Creating Follow test tree (mode: {self.test_mode})...')
        
        # Create behavior tree based on mode
        if self.test_mode == 'track_only':
            self.root = createTrackOnly()
        else:  # 'until_stopped' or default
            self.root = createFollowPersonUntilStopped(
                follow_distance=1.5,
                stationary_threshold=0.3,
                stationary_duration=5.0
            )
        
        self.tick_count = 0
        self.start_time = time.time()
        
        self.get_logger().info(f'Tree created, will use tick_tock for timing')
        
    def post_tick_handler(self, tree):
        """Called after each tick to print status."""
        self.tick_count += 1
        status = tree.root.status
        
        # Log status every second (4 ticks at 250ms)
        if self.tick_count % 4 == 0:
            elapsed = time.time() - self.start_time
            self.get_logger().info(
                f'Tick {self.tick_count} ({elapsed:.1f}s): {status.name}'
            )
            # Print tree
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
            
        if status == py_trees.common.Status.SUCCESS:
            self.get_logger().info('Behavior tree completed SUCCESSFULLY!')
        elif status == py_trees.common.Status.FAILURE:
            self.get_logger().error('Behavior tree FAILED!')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line args for mode
    test_mode = 'until_stopped'
    if len(sys.argv) > 1 and not sys.argv[1].startswith('--'):
        test_mode = sys.argv[1]
        
    print(f'\n{"="*60}')
    print(f'Follow Behavior Tree STANDALONE Test')
    print(f'Mode: {test_mode}')
    print(f'{"="*60}\n')
    
    # Create a helper node just to get parameters
    helper_node = FollowTestNode(test_mode)
    
    try:
        # Create py_trees_ros tree - this creates its own internal node
        tree = py_trees_ros.trees.BehaviourTree(
            root=helper_node.root,
            unicode_tree_debug=True
        )
        
        # Setup the tree (this waits for action servers)
        print("Setting up behavior tree...")
        tree.setup(node_name="follow_test_tree", timeout=15.0)
        print("Behavior tree setup complete")
        
        # Use tick_tock with post_tick_handler for timing
        # period_ms=250 means 4Hz tick rate
        tree.tick_tock(period_ms=250.0, post_tick_handler=helper_node.post_tick_handler)
        
        # Spin the tree's node - this is what processes callbacks!
        print("Starting behavior tree, spinning tree.node...")
        rclpy.spin(tree.node)
        
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'tree' in locals():
            tree.shutdown()
        helper_node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
