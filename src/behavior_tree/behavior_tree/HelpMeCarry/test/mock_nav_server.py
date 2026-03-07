#!/usr/bin/env python3
"""
Mock NavigateToPose Action Server for Testing

This module provides a mock Nav2 NavigateToPose action server that simulates
navigation behavior for testing the Follow behavior tree. It also publishes
a static transform from the camera frame to the map frame.

Usage:
    ros2 run behavior_tree mock_nav_server

Author: TinkerFuroc
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TransformStamped
from builtin_interfaces.msg import Duration

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

import math
import time
import threading


class MockNavigateToPoseServer(Node):
    """
    Mock NavigateToPose action server that simulates Nav2 navigation behavior.
    
    Features:
    - Accepts navigation goals
    - Simulates robot movement with configurable speed
    - Publishes feedback with current position and distance remaining
    - Publishes static transform from camera_link to map frame
    - Publishes dynamic transform from base_link to map frame (robot position)
    """
    
    def __init__(self):
        super().__init__('mock_navigate_to_pose_server')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)  # m/s
        self.declare_parameter('angular_speed', 1.0)  # rad/s
        self.declare_parameter('goal_tolerance', 0.1)  # meters
        self.declare_parameter('feedback_rate', 10.0)  # Hz
        self.declare_parameter('simulate_failure_rate', 0.0)  # probability of failure (0-1)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.simulate_failure_rate = self.get_parameter('simulate_failure_rate').value
        
        # Current robot pose (start at origin)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.pose_lock = threading.Lock()
        
        # Action server
        self._action_server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Transform broadcasters
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.dynamic_tf_broadcaster = TransformBroadcaster(self)
        
        # Publish static transforms
        self._publish_static_transforms()
        
        # Timer for publishing dynamic transforms (robot position)
        self.tf_timer = self.create_timer(0.05, self._publish_robot_transform)  # 20 Hz
        
        self.get_logger().info('Mock NavigateToPose server started')
        self.get_logger().info(f'  Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'  Angular speed: {self.angular_speed} rad/s')
        self.get_logger().info(f'  Goal tolerance: {self.goal_tolerance} m')
        
    def _publish_static_transforms(self):
        """Publish static transforms for camera frames."""
        transforms = []
        
        # Map -> odom (identity for simplicity)
        t_map_odom = TransformStamped()
        t_map_odom.header.stamp = self.get_clock().now().to_msg()
        t_map_odom.header.frame_id = 'map'
        t_map_odom.child_frame_id = 'odom'
        t_map_odom.transform.translation.x = 0.0
        t_map_odom.transform.translation.y = 0.0
        t_map_odom.transform.translation.z = 0.0
        t_map_odom.transform.rotation.w = 1.0
        transforms.append(t_map_odom)
        
        # base_link -> camera_link (orbbec camera mounted on robot)
        # Assuming camera is mounted 0.5m forward and 0.3m up from base_link
        t_base_camera = TransformStamped()
        t_base_camera.header.stamp = self.get_clock().now().to_msg()
        t_base_camera.header.frame_id = 'base_link'
        t_base_camera.child_frame_id = 'camera_link'
        t_base_camera.transform.translation.x = 0.5  # 50cm forward
        t_base_camera.transform.translation.y = 0.0
        t_base_camera.transform.translation.z = 0.3  # 30cm up
        t_base_camera.transform.rotation.w = 1.0
        transforms.append(t_base_camera)
        
        # camera_link -> camera_color_optical_frame (typical camera convention)
        # Rotate so Z points forward (into the scene)
        t_camera_optical = TransformStamped()
        t_camera_optical.header.stamp = self.get_clock().now().to_msg()
        t_camera_optical.header.frame_id = 'camera_link'
        t_camera_optical.child_frame_id = 'camera_color_optical_frame'
        t_camera_optical.transform.translation.x = 0.0
        t_camera_optical.transform.translation.y = 0.0
        t_camera_optical.transform.translation.z = 0.0
        # Rotation: x=-90deg, y=0, z=-90deg (standard optical frame convention)
        t_camera_optical.transform.rotation.x = -0.5
        t_camera_optical.transform.rotation.y = 0.5
        t_camera_optical.transform.rotation.z = -0.5
        t_camera_optical.transform.rotation.w = 0.5
        transforms.append(t_camera_optical)
        
        # Also create orbbec-specific frames
        t_orbbec = TransformStamped()
        t_orbbec.header.stamp = self.get_clock().now().to_msg()
        t_orbbec.header.frame_id = 'camera_link'
        t_orbbec.child_frame_id = 'camera_color_frame'
        t_orbbec.transform.rotation.w = 1.0
        transforms.append(t_orbbec)
        
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms: map->odom, base_link->camera_link, camera_link->optical_frame')
        
    def _publish_robot_transform(self):
        """Publish dynamic transform for robot position (odom -> base_link)."""
        with self.pose_lock:
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self.current_x
            t.transform.translation.y = self.current_y
            t.transform.translation.z = 0.0
            # Convert yaw to quaternion
            t.transform.rotation.z = math.sin(self.current_yaw / 2)
            t.transform.rotation.w = math.cos(self.current_yaw / 2)
            
        self.dynamic_tf_broadcaster.sendTransform(t)
        
    def goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        self.get_logger().info(f'Received goal request: x={goal_request.pose.pose.position.x:.2f}, '
                               f'y={goal_request.pose.pose.position.y:.2f}')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    async def execute_callback(self, goal_handle):
        """Execute the navigation goal."""
        self.get_logger().info('Executing navigation goal...')
        
        goal = goal_handle.request
        target_x = goal.pose.pose.position.x
        target_y = goal.pose.pose.position.y
        
        # Calculate target yaw from quaternion
        qz = goal.pose.pose.orientation.z
        qw = goal.pose.pose.orientation.w
        target_yaw = 2.0 * math.atan2(qz, qw)
        
        feedback_msg = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        
        # Simulate random failure
        import random
        if random.random() < self.simulate_failure_rate:
            self.get_logger().warn('Simulating navigation failure!')
            goal_handle.abort()
            return result
        
        # Navigation loop
        rate_period = 1.0 / self.feedback_rate
        
        while rclpy.ok():
            # Check for cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return result
                
            with self.pose_lock:
                # Calculate distance and angle to goal
                dx = target_x - self.current_x
                dy = target_y - self.current_y
                distance = math.sqrt(dx*dx + dy*dy)
                angle_to_goal = math.atan2(dy, dx)
                
                # Check if goal reached
                if distance < self.goal_tolerance:
                    self.get_logger().info(f'Goal reached! Final position: ({self.current_x:.2f}, {self.current_y:.2f})')
                    goal_handle.succeed()
                    return result
                    
                # Calculate movement for this time step
                dt = rate_period
                
                # First rotate towards goal
                angle_diff = self._normalize_angle(angle_to_goal - self.current_yaw)
                if abs(angle_diff) > 0.1:  # ~6 degrees
                    # Rotate
                    rotation = self.angular_speed * dt
                    if abs(angle_diff) < rotation:
                        self.current_yaw = angle_to_goal
                    else:
                        self.current_yaw += rotation * (1 if angle_diff > 0 else -1)
                    self.current_yaw = self._normalize_angle(self.current_yaw)
                else:
                    # Move forward
                    move_dist = min(self.linear_speed * dt, distance)
                    self.current_x += move_dist * math.cos(self.current_yaw)
                    self.current_y += move_dist * math.sin(self.current_yaw)
                    
                # Update feedback
                feedback_msg.current_pose.header.stamp = self.get_clock().now().to_msg()
                feedback_msg.current_pose.header.frame_id = 'map'
                feedback_msg.current_pose.pose.position.x = self.current_x
                feedback_msg.current_pose.pose.position.y = self.current_y
                feedback_msg.current_pose.pose.orientation.z = math.sin(self.current_yaw / 2)
                feedback_msg.current_pose.pose.orientation.w = math.cos(self.current_yaw / 2)
                feedback_msg.distance_remaining = distance
                
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().debug(f'Distance remaining: {distance:.2f}m')
            
            # Sleep
            time.sleep(rate_period)
            
        # If we get here, something went wrong
        goal_handle.abort()
        return result
        
    def _normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
        
    def set_robot_pose(self, x, y, yaw=0.0):
        """Set the robot's current pose (for testing)."""
        with self.pose_lock:
            self.current_x = x
            self.current_y = y
            self.current_yaw = yaw
        self.get_logger().info(f'Robot pose set to: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)')


def main(args=None):
    rclpy.init(args=args)
    
    server = MockNavigateToPoseServer()
    
    # Use multi-threaded executor to handle concurrent goals
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
