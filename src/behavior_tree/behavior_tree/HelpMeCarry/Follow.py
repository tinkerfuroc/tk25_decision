"""
Follow.py - Behavior tree nodes for following a person using the TrackPerson action.

This module provides behavior tree nodes for following a person until they indicate
they have reached their destination.

Author: TinkerFuroc
"""

import py_trees
from typing import Any, List, Optional
import time
import copy
import math
import threading

from rclpy.node import Node
from rclpy.action import ActionClient
import action_msgs.msg as action_msgs

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

# Import the TrackPerson action
from tinker_vision_msgs_26.action import TrackPerson

from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WaitTicks


# Blackboard keys
BB_KEY_TRACK_POSITION = "track/person_position"
BB_KEY_TRACK_ID = "track/person_id"
BB_KEY_TARGET_LOST = "track/target_lost"
BB_KEY_TRANSFORM_SUCCESS = "track/transform_success"
BB_KEY_FOLLOW_GOAL = "track/follow_goal"

# Default target frame for transformation
DEFAULT_TARGET_FRAME = "map"

# Tick period in seconds (250ms per tick)
TICK_PERIOD_SEC = 0.250


class FeedbackBuffer:
    """
    Thread-safe buffer for storing feedback messages between behavior tree ticks.
    
    Since the action server may send feedback faster than the behavior tree ticks
    (250ms), this buffer accumulates all feedback and provides the latest valid
    position when queried.
    """
    
    def __init__(self):
        self._lock = threading.Lock()
        self._feedbacks: List[Any] = []
        self._latest_valid_position: Optional[PointStamped] = None
        self._latest_track_id: int = -1
        self._target_lost: bool = True
        self._transform_success: bool = False
        self._feedback_count: int = 0
        
    def add_feedback(self, feedback) -> None:
        """Add a feedback message to the buffer (thread-safe)."""
        with self._lock:
            self._feedbacks.append(feedback)
            self._feedback_count += 1
            
            # Update latest state
            self._target_lost = feedback.target_lost
            self._latest_track_id = feedback.target_track_id
            self._transform_success = feedback.is_transformation_successful
            
            # Only update position if transformation was successful and target not lost
            if feedback.is_transformation_successful and not feedback.target_lost:
                if feedback.target_position is not None:
                    self._latest_valid_position = copy.deepcopy(feedback.target_position)
                    
    def get_latest_state(self) -> tuple:
        """
        Get the latest tracking state (thread-safe).
        
        Returns:
            Tuple of (position, track_id, target_lost, transform_success, feedback_count)
        """
        with self._lock:
            return (
                copy.deepcopy(self._latest_valid_position),
                self._latest_track_id,
                self._target_lost,
                self._transform_success,
                self._feedback_count
            )
            
    def get_and_clear_feedbacks(self) -> List[Any]:
        """Get all buffered feedbacks and clear the buffer (thread-safe)."""
        with self._lock:
            feedbacks = self._feedbacks.copy()
            self._feedbacks.clear()
            return feedbacks
            
    def clear(self) -> None:
        """Clear all buffered data (thread-safe)."""
        with self._lock:
            self._feedbacks.clear()
            self._latest_valid_position = None
            self._latest_track_id = -1
            self._target_lost = True
            self._transform_success = False
            self._feedback_count = 0


class BtNode_TrackPersonAction(py_trees.behaviour.Behaviour):
    """
    Behavior tree node that wraps the TrackPerson action.
    
    This node starts tracking a person and continuously updates the blackboard
    with the target's position. It returns RUNNING while actively tracking,
    SUCCESS when tracking completes (goal cancelled), and FAILURE if tracking fails.
    
    The node handles feedback buffering to accumulate all feedback messages
    received between behavior tree ticks (250ms intervals). It requests the
    action server to transform positions to the map frame.
    
    Features:
    - Requests position transformation to map frame
    - Handles is_transformation_successful field
    - Buffers multiple feedback messages between ticks
    - Updates blackboard with latest valid position
    """
    
    def __init__(self,
                 name: str,
                 action_name: str = "track_person",
                 target_frame: str = DEFAULT_TARGET_FRAME,
                 bb_key_position: str = BB_KEY_TRACK_POSITION,
                 bb_key_track_id: str = BB_KEY_TRACK_ID,
                 bb_key_target_lost: str = BB_KEY_TARGET_LOST,
                 bb_key_transform_success: str = BB_KEY_TRANSFORM_SUCCESS,
                 return_rgb: bool = False,
                 return_depth: bool = False,
                 return_segment: bool = False,
                 debug: bool = False,
                 wait_for_server_timeout_sec: float = 10.0):
        """
        Initialize the TrackPerson action node.
        
        Args:
            name: Name of the behavior node
            action_name: Name of the ROS action server
            target_frame: Target frame for position transformation (default: "map")
            bb_key_position: Blackboard key to store target position (PointStamped)
            bb_key_track_id: Blackboard key to store track ID
            bb_key_target_lost: Blackboard key to store target lost status
            bb_key_transform_success: Blackboard key to store transformation success status
            return_rgb: Whether to return RGB image in feedback
            return_depth: Whether to return depth image in feedback
            return_segment: Whether to return segmentation mask in feedback
            debug: Enable debug visualization on action server
            wait_for_server_timeout_sec: Timeout for waiting for action server
        """
        super().__init__(name=name)
        
        self.action_name = action_name
        self.target_frame = target_frame
        self.bb_key_position = bb_key_position
        self.bb_key_track_id = bb_key_track_id
        self.bb_key_target_lost = bb_key_target_lost
        self.bb_key_transform_success = bb_key_transform_success
        
        self.return_rgb = return_rgb
        self.return_depth = return_depth
        self.return_segment = return_segment
        self.debug = debug
        self.wait_for_server_timeout_sec = wait_for_server_timeout_sec
        
        # Action client state
        self.node: Node = None
        self.action_client: ActionClient = None
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        
        # Result state
        self.result_status = None
        self.result_message = None
        
        # Tracking state
        self.tracking_active = False
        self.last_feedback_time = None
        
        # Feedback buffer for handling multiple feedbacks between ticks
        self.feedback_buffer = FeedbackBuffer()
        
        # Statistics
        self.total_feedbacks_received = 0
        self.successful_transforms = 0
        
    def setup(self, **kwargs: Any) -> None:
        """Setup the action client."""
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__)
            raise KeyError(error_message) from e
        
        # Create action client
        self.action_client = ActionClient(
            self.node,
            TrackPerson,
            self.action_name
        )
        
        # Wait for action server
        self.node.get_logger().info(f"Waiting for action server '{self.action_name}'...")
        if not self.action_client.wait_for_server(timeout_sec=self.wait_for_server_timeout_sec):
            self.node.get_logger().error(f"Action server '{self.action_name}' not available!")
            raise RuntimeError(f"Action server '{self.action_name}' not available")
        self.node.get_logger().info(f"Connected to action server '{self.action_name}'")
        
        # Setup blackboard
        self.bb_client = self.attach_blackboard_client(name=f"{self.name}_bb")
        self.bb_client.register_key(self.bb_key_position, access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(self.bb_key_track_id, access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(self.bb_key_target_lost, access=py_trees.common.Access.WRITE)
        self.bb_client.register_key(self.bb_key_transform_success, access=py_trees.common.Access.WRITE)
        
        self.logger.debug("Setup complete")
        
    def initialise(self) -> None:
        """Start the tracking action."""
        self.logger.debug("Initializing TrackPersonAction")
        
        # Reset state
        self.goal_handle = None
        self.send_goal_future = None
        self.get_result_future = None
        self.result_status = None
        self.result_message = None
        self.tracking_active = False
        self.last_feedback_time = time.time()
        
        # Clear feedback buffer
        self.feedback_buffer.clear()
        self.total_feedbacks_received = 0
        self.successful_transforms = 0
        
        # Initialize blackboard values
        self.bb_client.set(self.bb_key_target_lost, True, overwrite=True)
        self.bb_client.set(self.bb_key_transform_success, False, overwrite=True)
        
        # Create and send goal with target_frame for transformation
        goal = TrackPerson.Goal()
        goal.target_frame = self.target_frame  # Request transformation to map frame
        goal.return_rgb_img = self.return_rgb
        goal.return_depth_img = self.return_depth
        goal.return_segment = self.return_segment
        goal.debug = self.debug
        
        self.send_goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        self.send_goal_future.add_done_callback(self._goal_response_callback)
        
        self.feedback_message = f"Sending tracking goal (target_frame={self.target_frame})..."
        
    def update(self) -> py_trees.common.Status:
        """
        Check tracking status and update blackboard.
        
        This method processes all buffered feedback messages received since
        the last tick and updates the blackboard with the latest valid state.
        """
        self.logger.debug("Updating TrackPersonAction")
        
        # Check if goal was sent
        if self.send_goal_future is None:
            self.feedback_message = "No goal sent"
            return py_trees.common.Status.FAILURE
            
        # Check if goal was rejected
        if self.goal_handle is not None and not self.goal_handle.accepted:
            self.feedback_message = "Goal rejected by action server"
            return py_trees.common.Status.FAILURE
            
        # Check if we have a result
        if self.result_status is not None:
            if self.result_status == action_msgs.GoalStatus.STATUS_SUCCEEDED:
                self.feedback_message = f"Tracking completed: {self.result_message}"
                return py_trees.common.Status.SUCCESS
            elif self.result_status == action_msgs.GoalStatus.STATUS_CANCELED:
                self.feedback_message = "Tracking was cancelled"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = f"Tracking failed with status: {self.result_status}"
                return py_trees.common.Status.FAILURE
        
        # Process buffered feedbacks and update blackboard
        self._process_buffered_feedbacks()
                
        # Still tracking
        return py_trees.common.Status.RUNNING
        
    def _process_buffered_feedbacks(self) -> None:
        """Process all buffered feedbacks and update blackboard with latest state."""
        # Get latest state from buffer
        position, track_id, target_lost, transform_success, feedback_count = \
            self.feedback_buffer.get_latest_state()
        
        # Update statistics
        self.total_feedbacks_received = feedback_count
        
        # Update blackboard
        self.bb_client.set(self.bb_key_target_lost, target_lost, overwrite=True)
        self.bb_client.set(self.bb_key_track_id, track_id, overwrite=True)
        self.bb_client.set(self.bb_key_transform_success, transform_success, overwrite=True)
        
        if position is not None:
            self.bb_client.set(self.bb_key_position, position, overwrite=True)
            pos = position.point
            frame = position.header.frame_id
            self.feedback_message = (
                f"Tracking ID {track_id} at ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}) "
                f"in frame [{frame}] | {feedback_count} feedbacks"
            )
        elif target_lost:
            self.feedback_message = f"Target lost! | {feedback_count} feedbacks received"
        elif not transform_success:
            self.feedback_message = f"Transform to {self.target_frame} failed | {feedback_count} feedbacks"
        else:
            self.feedback_message = f"Waiting for valid position... | {feedback_count} feedbacks"
        
    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Cancel tracking if interrupted."""
        self.logger.debug(f"Terminating with status {new_status}")
        
        if (self.status == py_trees.common.Status.RUNNING and
            new_status == py_trees.common.Status.INVALID):
            self._cancel_tracking()
            
    def _feedback_callback(self, feedback_msg) -> None:
        """
        Handle feedback from the action server.
        
        This callback may be called multiple times between behavior tree ticks.
        All feedbacks are buffered and processed during the next update().
        """
        feedback = feedback_msg.feedback
        self.last_feedback_time = time.time()
        
        # Add to buffer - will be processed during next update()
        self.feedback_buffer.add_feedback(feedback)
        
        # Log for debugging (optional - can be verbose)
        if feedback.is_transformation_successful and not feedback.target_lost:
            self.successful_transforms += 1
            
    def _goal_response_callback(self, future) -> None:
        """Handle goal response."""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.feedback_message = "Goal rejected"
            return
            
        self.tracking_active = True
        self.feedback_message = f"Goal accepted, tracking started (frame={self.target_frame})"
        
        # Setup result callback
        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._result_callback)
        
    def _result_callback(self, future) -> None:
        """Handle action result."""
        result = future.result()
        self.result_status = result.status
        self.result_message = result.result.message if hasattr(result.result, 'message') else "Completed"
        self.tracking_active = False
        
    def _cancel_tracking(self) -> None:
        """Cancel the current tracking goal."""
        if self.goal_handle is not None and self.tracking_active:
            self.feedback_message = "Cancelling tracking..."
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_callback)
            
    def _cancel_callback(self, future) -> None:
        """Handle cancel response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.feedback_message = "Tracking cancelled successfully"
        else:
            self.feedback_message = "Failed to cancel tracking"


class BtNode_ProcessTrackPosition(py_trees.behaviour.Behaviour):
    """
    Process tracking position updates and convert to navigation goals.
    
    This node reads the target position from the blackboard and converts it
    to a PoseStamped goal suitable for navigation. It maintains the last
    valid position and calculates a position offset for following.
    
    The node checks is_transformation_successful to ensure the position
    is in the correct frame before using it.
    """
    
    def __init__(self,
                 name: str,
                 bb_key_position: str = BB_KEY_TRACK_POSITION,
                 bb_key_target_lost: str = BB_KEY_TARGET_LOST,
                 bb_key_transform_success: str = BB_KEY_TRANSFORM_SUCCESS,
                 bb_key_goal: str = BB_KEY_FOLLOW_GOAL,
                 follow_distance: float = 1.0,
                 update_threshold: float = 0.3):
        """
        Initialize the process track position node.
        
        Args:
            name: Name of the behavior node
            bb_key_position: Blackboard key to read target position
            bb_key_target_lost: Blackboard key to read target lost status
            bb_key_transform_success: Blackboard key to read transformation success status
            bb_key_goal: Blackboard key to write navigation goal
            follow_distance: Distance to maintain from the target
            update_threshold: Minimum movement to trigger goal update
        """
        super().__init__(name=name)
        
        self.bb_key_position = bb_key_position
        self.bb_key_target_lost = bb_key_target_lost
        self.bb_key_transform_success = bb_key_transform_success
        self.bb_key_goal = bb_key_goal
        self.follow_distance = follow_distance
        self.update_threshold = update_threshold
        
        self.last_goal: PoseStamped = None
        self.last_valid_position: PointStamped = None
        
    def setup(self, **kwargs: Any) -> None:
        """Setup blackboard access."""
        try:
            self.node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs"
            raise KeyError(error_message) from e
            
        self.bb_read = self.attach_blackboard_client(name=f"{self.name}_read")
        self.bb_read.register_key(self.bb_key_position, access=py_trees.common.Access.READ)
        self.bb_read.register_key(self.bb_key_target_lost, access=py_trees.common.Access.READ)
        self.bb_read.register_key(self.bb_key_transform_success, access=py_trees.common.Access.READ)
        
        self.bb_write = self.attach_blackboard_client(name=f"{self.name}_write")
        self.bb_write.register_key(self.bb_key_goal, access=py_trees.common.Access.WRITE)
        
    def initialise(self) -> None:
        """Reset state."""
        self.feedback_message = "Initializing position processor"
        
    def update(self) -> py_trees.common.Status:
        """Process position and create navigation goal."""
        try:
            target_lost = self.bb_read.get(self.bb_key_target_lost)
            transform_success = self.bb_read.get(self.bb_key_transform_success)
            
            if target_lost:
                self.feedback_message = "Target lost, using last known position"
                if self.last_goal is not None:
                    return py_trees.common.Status.RUNNING
                return py_trees.common.Status.FAILURE
            
            if not transform_success:
                self.feedback_message = "Transform failed, using last known position"
                if self.last_goal is not None:
                    return py_trees.common.Status.RUNNING
                return py_trees.common.Status.FAILURE
                
            position: PointStamped = self.bb_read.get(self.bb_key_position)
        except KeyError:
            self.feedback_message = "No position data available yet"
            return py_trees.common.Status.RUNNING
        
        # Validate position is in expected frame (should be map frame)
        if position.header.frame_id != "map":
            self.feedback_message = f"Position not in map frame (got {position.header.frame_id})"
            if self.last_goal is not None:
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.FAILURE
            
        # Store last valid position
        self.last_valid_position = copy.deepcopy(position)
            
        # Calculate goal position (behind the target)
        goal = self._calculate_follow_goal(position)
        
        # Check if we need to update
        if self.last_goal is not None:
            dist = self._distance(goal, self.last_goal)
            if dist < self.update_threshold:
                self.feedback_message = f"Position change too small ({dist:.2f}m), skipping update"
                return py_trees.common.Status.RUNNING
                
        # Update goal
        self.last_goal = goal
        self.bb_write.set(self.bb_key_goal, goal, overwrite=True)
        
        pos = goal.pose.position
        self.feedback_message = f"Updated follow goal to ({pos.x:.2f}, {pos.y:.2f}) in map frame"
        return py_trees.common.Status.SUCCESS
        
    def _calculate_follow_goal(self, position: PointStamped) -> PoseStamped:
        """Calculate a following goal position behind the target."""
        goal = PoseStamped()
        goal.header = position.header
        goal.header.stamp = self.node.get_clock().now().to_msg()
        
        # Calculate direction from robot to target
        x, y = position.point.x, position.point.y
        dist = math.sqrt(x*x + y*y)
        
        if dist > 0.01:
            # Normalize and apply follow distance
            scale = (dist - self.follow_distance) / dist
            goal.pose.position.x = x * scale
            goal.pose.position.y = y * scale
        else:
            goal.pose.position.x = x
            goal.pose.position.y = y
            
        goal.pose.position.z = 0.0
        
        # Calculate orientation to face the target
        yaw = math.atan2(y, x)
        goal.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.orientation.w = math.cos(yaw / 2)
        
        return goal
        
    def _distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate distance between two poses."""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)


class BtNode_CheckTargetStopped(py_trees.behaviour.Behaviour):
    """
    Check if the tracked target has stopped moving.
    
    Returns SUCCESS when the target has been stationary (within threshold)
    for the specified duration. Returns RUNNING while monitoring, and
    FAILURE if position data is unavailable.
    
    The node checks is_transformation_successful to ensure the position
    is valid before using it for stationary detection.
    """
    
    def __init__(self,
                 name: str,
                 bb_key_position: str = BB_KEY_TRACK_POSITION,
                 bb_key_target_lost: str = BB_KEY_TARGET_LOST,
                 bb_key_transform_success: str = BB_KEY_TRANSFORM_SUCCESS,
                 stationary_threshold: float = 0.3,
                 stationary_duration: float = 3.0):
        """
        Initialize the check target stopped node.
        
        Args:
            name: Name of the behavior node
            bb_key_position: Blackboard key to read target position
            bb_key_target_lost: Blackboard key to read target lost status
            bb_key_transform_success: Blackboard key to read transformation success status
            stationary_threshold: Maximum movement (meters) to be considered stationary
            stationary_duration: Time (seconds) target must be stationary
        """
        super().__init__(name=name)
        
        self.bb_key_position = bb_key_position
        self.bb_key_target_lost = bb_key_target_lost
        self.bb_key_transform_success = bb_key_transform_success
        self.stationary_threshold = stationary_threshold
        self.stationary_duration = stationary_duration
        
        self.anchor_position: PointStamped = None
        self.anchor_time: float = None
        
    def setup(self, **kwargs: Any) -> None:
        """Setup blackboard access."""
        try:
            self.node = kwargs['node']
        except KeyError as e:
            raise KeyError("didn't find 'node' in setup's kwargs") from e
            
        self.bb_read = self.attach_blackboard_client(name=f"{self.name}_read")
        self.bb_read.register_key(self.bb_key_position, access=py_trees.common.Access.READ)
        self.bb_read.register_key(self.bb_key_target_lost, access=py_trees.common.Access.READ)
        self.bb_read.register_key(self.bb_key_transform_success, access=py_trees.common.Access.READ)
        
    def initialise(self) -> None:
        """Reset stationary detection state."""
        self.anchor_position = None
        self.anchor_time = None
        self.feedback_message = "Starting stationary detection"
        
    def update(self) -> py_trees.common.Status:
        """Check if target has stopped."""
        try:
            target_lost = self.bb_read.get(self.bb_key_target_lost)
            transform_success = self.bb_read.get(self.bb_key_transform_success)
            
            if target_lost:
                # Reset anchor when target is lost
                self.anchor_position = None
                self.anchor_time = None
                self.feedback_message = "Target lost, resetting anchor"
                return py_trees.common.Status.RUNNING
            
            if not transform_success:
                # Don't reset anchor on transform failure, just skip this update
                self.feedback_message = "Transform failed, skipping update"
                return py_trees.common.Status.RUNNING
                
            position: PointStamped = self.bb_read.get(self.bb_key_position)
        except KeyError:
            self.feedback_message = "No position data available"
            return py_trees.common.Status.RUNNING
        
        # Validate position is in map frame
        if position.header.frame_id != "map":
            self.feedback_message = f"Position not in map frame (got {position.header.frame_id})"
            return py_trees.common.Status.RUNNING
            
        current_time = time.time()
        
        # First position - set anchor
        if self.anchor_position is None:
            self.anchor_position = copy.deepcopy(position)
            self.anchor_time = current_time
            self.feedback_message = "Set anchor position in map frame"
            return py_trees.common.Status.RUNNING
            
        # Check movement from anchor
        dist = self._distance(position, self.anchor_position)
        
        if dist > self.stationary_threshold:
            # Target moved - reset anchor
            self.anchor_position = copy.deepcopy(position)
            self.anchor_time = current_time
            self.feedback_message = f"Target moved {dist:.2f}m, reset anchor"
            return py_trees.common.Status.RUNNING
            
        # Target is stationary - check duration
        stationary_time = current_time - self.anchor_time
        
        if stationary_time >= self.stationary_duration:
            self.feedback_message = f"Target stopped for {stationary_time:.1f}s"
            return py_trees.common.Status.SUCCESS
            
        self.feedback_message = f"Target stationary for {stationary_time:.1f}s / {self.stationary_duration:.1f}s"
        return py_trees.common.Status.RUNNING
        
    def _distance(self, pos1: PointStamped, pos2: PointStamped) -> float:
        """Calculate distance between two positions."""
        dx = pos1.point.x - pos2.point.x
        dy = pos1.point.y - pos2.point.y
        return math.sqrt(dx*dx + dy*dy)


def createFollowPersonUntilStopped(
    follow_distance: float = 1.0,
    stationary_threshold: float = 0.3,
    stationary_duration: float = 3.0,
    target_frame: str = DEFAULT_TARGET_FRAME,
    action_name: str = "track_person",
    nav_action_name: str = "navigate_to_pose"
) -> py_trees.behaviour.Behaviour:
    """
    Create a behavior tree that follows a person until they stop.
    
    The tree runs tracking and navigation in parallel, and checks
    if the target has stopped moving. When the target stops, the
    tree returns SUCCESS.
    
    The tracking requests position transformation to the map frame for
    proper navigation and stationary detection.
    
    Args:
        follow_distance: Distance to maintain from the target
        stationary_threshold: Movement threshold for stationary detection
        stationary_duration: Time target must be stationary
        target_frame: Target frame for position transformation (default: "map")
        action_name: Name of the TrackPerson action server
        nav_action_name: Name of the navigation action server
        
    Returns:
        Root behavior node for the follow-until-stopped tree
    """
    root = py_trees.composites.Parallel(
        name="Follow Until Stopped",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    
    # Tracking branch - runs continuously, requests map frame transformation
    track = BtNode_TrackPersonAction(
        name="Track Person",
        action_name=action_name,
        target_frame=target_frame,
        bb_key_position=BB_KEY_TRACK_POSITION,
        bb_key_track_id=BB_KEY_TRACK_ID,
        bb_key_target_lost=BB_KEY_TARGET_LOST,
        bb_key_transform_success=BB_KEY_TRANSFORM_SUCCESS
    )
    track_repeat = py_trees.decorators.FailureIsRunning(
        name="Keep Tracking",
        child=track
    )
    
    # Following branch - process position and navigate
    follow_seq = py_trees.composites.Sequence(name="Follow Sequence", memory=True)
    
    process_pos = BtNode_ProcessTrackPosition(
        name="Process Position",
        bb_key_position=BB_KEY_TRACK_POSITION,
        bb_key_target_lost=BB_KEY_TARGET_LOST,
        bb_key_transform_success=BB_KEY_TRANSFORM_SUCCESS,
        bb_key_goal=BB_KEY_FOLLOW_GOAL,
        follow_distance=follow_distance
    )
    
    goto = BtNode_GotoAction(
        name="Navigate to Target",
        key=BB_KEY_FOLLOW_GOAL,
        action_name=nav_action_name,
        wait_for_server_timeout_sec=-3
    )
    
    follow_seq.add_child(process_pos)
    follow_seq.add_child(py_trees.decorators.FailureIsRunning(
        name="Navigate",
        child=goto
    ))
    
    follow_repeat = py_trees.decorators.Repeat(
        name="Repeat Follow",
        child=follow_seq,
        num_success=-1  # Infinite
    )
    
    # Stationary detection branch - returns SUCCESS when target stops
    check_stopped = BtNode_CheckTargetStopped(
        name="Check Target Stopped",
        bb_key_position=BB_KEY_TRACK_POSITION,
        bb_key_target_lost=BB_KEY_TARGET_LOST,
        bb_key_transform_success=BB_KEY_TRANSFORM_SUCCESS,
        stationary_threshold=stationary_threshold,
        stationary_duration=stationary_duration
    )
    
    root.add_children([track_repeat, follow_repeat, check_stopped])
    
    return root


def createFollowPersonWithConfirmation(
    follow_distance: float = 1.0,
    target_frame: str = DEFAULT_TARGET_FRAME,
    action_name: str = "track_person",
    nav_action_name: str = "navigate_to_pose"
) -> py_trees.behaviour.Behaviour:
    """
    Create a behavior tree that follows a person and asks for confirmation.
    
    The tree follows the person while periodically asking if they have reached
    their destination. When confirmed, it stops following.
    
    Args:
        follow_distance: Distance to maintain from the target
        target_frame: Target frame for position transformation (default: "map")
        action_name: Name of the TrackPerson action server
        nav_action_name: Name of the navigation action server
        
    Returns:
        Root behavior node for follow-with-confirmation tree
    """
    root = py_trees.composites.Parallel(
        name="Follow With Confirmation",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne()
    )
    
    # Tracking branch - requests map frame transformation
    track = BtNode_TrackPersonAction(
        name="Track Person",
        action_name=action_name,
        target_frame=target_frame,
        bb_key_position=BB_KEY_TRACK_POSITION,
        bb_key_track_id=BB_KEY_TRACK_ID,
        bb_key_target_lost=BB_KEY_TARGET_LOST,
        bb_key_transform_success=BB_KEY_TRANSFORM_SUCCESS
    )
    track_repeat = py_trees.decorators.FailureIsRunning(
        name="Keep Tracking",
        child=track
    )
    
    # Following branch
    follow_seq = py_trees.composites.Sequence(name="Follow Sequence", memory=True)
    
    process_pos = BtNode_ProcessTrackPosition(
        name="Process Position",
        bb_key_position=BB_KEY_TRACK_POSITION,
        bb_key_target_lost=BB_KEY_TARGET_LOST,
        bb_key_transform_success=BB_KEY_TRANSFORM_SUCCESS,
        bb_key_goal=BB_KEY_FOLLOW_GOAL,
        follow_distance=follow_distance
    )
    
    goto = BtNode_GotoAction(
        name="Navigate to Target",
        key=BB_KEY_FOLLOW_GOAL,
        action_name=nav_action_name,
        wait_for_server_timeout_sec=-3
    )
    
    follow_seq.add_child(process_pos)
    follow_seq.add_child(py_trees.decorators.FailureIsRunning(
        name="Navigate",
        child=goto
    ))
    
    follow_repeat = py_trees.decorators.Repeat(
        name="Repeat Follow",
        child=follow_seq,
        num_success=-1
    )
    
    # Confirmation branch - periodically asks if destination reached
    confirm_seq = py_trees.composites.Sequence(name="Confirmation Loop", memory=True)
    
    # Wait before asking (50 ticks at 250ms = 12.5 seconds)
    wait = BtNode_WaitTicks(name="Wait Before Asking", ticks=50)
    
    # Ask if reached destination
    ask = BtNode_Announce(
        name="Ask If Reached",
        bb_source=None,
        message="Have you reached your destination?"
    )
    
    # Get confirmation
    confirm = BtNode_GetConfirmation(name="Get Confirmation")
    
    confirm_seq.add_child(wait)
    confirm_seq.add_child(ask)
    confirm_seq.add_child(confirm)
    
    # Retry until confirmed
    confirm_retry = py_trees.decorators.Retry(
        name="Retry Confirmation",
        child=confirm_seq,
        num_failures=-1  # Infinite retries
    )
    
    root.add_children([track_repeat, follow_repeat, confirm_retry])
    
    return root


def createFollowPersonComplete(
    follow_distance: float = 1.0,
    stationary_threshold: float = 0.3,
    stationary_duration: float = 5.0,
    target_frame: str = DEFAULT_TARGET_FRAME,
    action_name: str = "track_person",
    nav_action_name: str = "navigate_to_pose"
) -> py_trees.behaviour.Behaviour:
    """
    Create a complete follow behavior that:
    1. Follows the person (with position in map frame)
    2. When person stops, asks if they reached destination
    3. If not confirmed, continues following
    4. If confirmed, returns SUCCESS
    
    Note: The behavior tree should be ticked at 250ms intervals (4 Hz).
    The TrackPerson action may send feedback faster than this, and all
    feedback messages are buffered and processed on each tick.
    
    Args:
        follow_distance: Distance to maintain from the target
        stationary_threshold: Movement threshold for stationary detection
        stationary_duration: Time target must be stationary before asking
        target_frame: Target frame for position transformation (default: "map")
        action_name: Name of the TrackPerson action server
        nav_action_name: Name of the navigation action server
        
    Returns:
        Root behavior node for the complete follow tree
    """
    root = py_trees.composites.Sequence(name="Complete Follow Person", memory=True)
    
    # Announce start
    root.add_child(BtNode_Announce(
        name="Announce Start Following",
        bb_source=None,
        message="I will follow you now. Please let me know when you have reached your destination."
    ))
    
    # Main follow loop - retry until confirmation
    follow_loop = py_trees.composites.Sequence(name="Follow Loop", memory=True)
    
    # Follow until person stops (with map frame transformation)
    follow_until_stopped = createFollowPersonUntilStopped(
        follow_distance=follow_distance,
        stationary_threshold=stationary_threshold,
        stationary_duration=stationary_duration,
        target_frame=target_frame,
        action_name=action_name,
        nav_action_name=nav_action_name
    )
    follow_loop.add_child(follow_until_stopped)
    
    # Person has stopped - ask for confirmation
    follow_loop.add_child(BtNode_Announce(
        name="Ask If Reached",
        bb_source=None,
        message="Have you reached your destination?"
    ))
    
    # Get confirmation - FAILURE means not yet reached, continue following
    follow_loop.add_child(BtNode_GetConfirmation(name="Get Confirmation"))
    
    # Retry until confirmed
    follow_retry = py_trees.decorators.Retry(
        name="Retry Until Confirmed",
        child=follow_loop,
        num_failures=-1
    )
    
    root.add_child(follow_retry)
    
    # Announce completion
    root.add_child(BtNode_Announce(
        name="Announce Arrived",
        bb_source=None,
        message="Great! You have reached your destination. Now please show me which bag to pick up."
    ))
    
    return root