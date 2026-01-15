#!/usr/bin/env python3
"""
Mock TrackPerson Action Server for Testing

This module provides a mock TrackPerson action server that simulates
person tracking for testing the Follow behavior tree.

It publishes simulated person positions that can be controlled via
keyboard or pre-programmed paths.

Usage:
    ros2 run behavior_tree mock_track_server

Author: TinkerFuroc
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from tinker_vision_msgs_26.action import TrackPerson
from geometry_msgs.msg import PointStamped, TransformStamped
from std_msgs.msg import Header

from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

import math
import time
import threading


class MockTrackPersonServer(Node):
    """
    Mock TrackPerson action server that simulates person tracking.
    
    Features:
    - Simulates a person walking in front of the robot
    - Supports transformation to requested target frame (e.g., map)
    - Configurable person movement patterns
    - Feedback rate configurable
    """
    
    def __init__(self):
        super().__init__('mock_track_person_server')
        
        # Parameters
        self.declare_parameter('feedback_rate', 10.0)  # Hz
        self.declare_parameter('initial_person_x', 2.0)  # meters in camera frame
        self.declare_parameter('initial_person_y', 0.0)
        self.declare_parameter('initial_person_z', 0.0)
        self.declare_parameter('person_speed', 0.3)  # m/s
        self.declare_parameter('movement_pattern', 'walk_forward')  # walk_forward, circle, stationary, random
        
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.person_speed = self.get_parameter('person_speed').value
        self.movement_pattern = self.get_parameter('movement_pattern').value
        
        # Person position in camera_link frame
        self.person_x = self.get_parameter('initial_person_x').value
        self.person_y = self.get_parameter('initial_person_y').value
        self.person_z = self.get_parameter('initial_person_z').value
        self.person_lock = threading.Lock()
        
        # Track ID
        self.track_id = 1
        self.target_lost = False
        
        # TF buffer for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action server
        self._action_server = ActionServer(
            self,
            TrackPerson,
            'track_person',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Movement simulation timer
        self.movement_timer = self.create_timer(0.1, self._update_person_position)  # 10 Hz
        self.movement_time = 0.0
        
        # Keyboard control subscriber (optional)
        # self.create_subscription(...)
        
        self.get_logger().info('Mock TrackPerson server started')
        self.get_logger().info(f'  Feedback rate: {self.feedback_rate} Hz')
        self.get_logger().info(f'  Movement pattern: {self.movement_pattern}')
        self.get_logger().info(f'  Person speed: {self.person_speed} m/s')
        
    def _update_person_position(self):
        """Update person position based on movement pattern."""
        dt = 0.1  # Timer period
        self.movement_time += dt
        
        with self.person_lock:
            if self.movement_pattern == 'walk_forward':
                # Person walks forward (increasing X in camera frame -> away from robot)
                # In map frame, this means the person moves in the direction the robot is facing
                self.person_x += self.person_speed * dt
                
            elif self.movement_pattern == 'walk_left':
                # Person walks to the left
                self.person_y += self.person_speed * dt
                
            elif self.movement_pattern == 'circle':
                # Person walks in a circle
                radius = 3.0
                angular_speed = self.person_speed / radius
                angle = angular_speed * self.movement_time
                self.person_x = radius * math.cos(angle) + 3.0
                self.person_y = radius * math.sin(angle)
                
            elif self.movement_pattern == 'stationary':
                # Person stands still
                pass
                
            elif self.movement_pattern == 'stop_after_5s':
                # Person walks for 5 seconds then stops
                if self.movement_time < 5.0:
                    self.person_x += self.person_speed * dt
                    
            elif self.movement_pattern == 'random':
                # Random walk
                import random
                self.person_x += (random.random() - 0.5) * self.person_speed * dt * 2
                self.person_y += (random.random() - 0.5) * self.person_speed * dt * 2
                
    def goal_callback(self, goal_request):
        """Accept or reject a goal request."""
        self.get_logger().info(f'Received tracking goal request')
        self.get_logger().info(f'  target_frame: {goal_request.target_frame}')
        self.get_logger().info(f'  return_rgb: {goal_request.return_rgb_img}')
        self.get_logger().info(f'  debug: {goal_request.debug}')
        return GoalResponse.ACCEPT
        
    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
        
    async def execute_callback(self, goal_handle):
        """Execute the tracking goal - continuously publish person position."""
        self.get_logger().info('Starting person tracking...')
        
        goal = goal_handle.request
        target_frame = goal.target_frame if goal.target_frame else 'camera_link'
        
        feedback_msg = TrackPerson.Feedback()
        result = TrackPerson.Result()
        
        rate_period = 1.0 / self.feedback_rate
        
        while rclpy.ok():
            # Check for cancel
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Tracking canceled')
                result.status = 0
                result.message = "Tracking canceled by request"
                return result
                
            with self.person_lock:
                # Create position in camera_link frame
                camera_point = PointStamped()
                camera_point.header.stamp = self.get_clock().now().to_msg()
                camera_point.header.frame_id = 'camera_link'
                camera_point.point.x = self.person_x
                camera_point.point.y = self.person_y
                camera_point.point.z = self.person_z
                
            # Transform to target frame if requested
            transform_success = False
            target_point = camera_point
            
            if target_frame and target_frame != 'camera_link':
                try:
                    # Wait for transform with timeout
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,
                        'camera_link',
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    
                    # Transform the point
                    target_point = tf2_geometry_msgs.do_transform_point(camera_point, transform)
                    target_point.header.frame_id = target_frame
                    transform_success = True
                    
                except Exception as e:
                    self.get_logger().warn(f'Transform failed: {e}')
                    transform_success = False
            else:
                transform_success = True  # No transform needed
                
            # Build feedback message
            feedback_msg.target_lost = self.target_lost
            feedback_msg.target_track_id = self.track_id
            feedback_msg.is_transformation_successful = transform_success
            feedback_msg.target_position = target_point
            
            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            
            if transform_success and not self.target_lost:
                self.get_logger().debug(
                    f'Person at ({target_point.point.x:.2f}, {target_point.point.y:.2f}) '
                    f'in frame [{target_frame}]'
                )
            
            # Sleep
            time.sleep(rate_period)
            
        # If we get here, node is shutting down
        result.status = 1
        result.message = "Tracking ended"
        goal_handle.succeed()
        return result
        
    def set_person_position(self, x, y, z=0.0):
        """Set the person's position in camera frame (for testing)."""
        with self.person_lock:
            self.person_x = x
            self.person_y = y
            self.person_z = z
        self.get_logger().info(f'Person position set to: ({x:.2f}, {y:.2f}, {z:.2f}) in camera_link')
        
    def set_target_lost(self, lost: bool):
        """Set whether the target is lost."""
        self.target_lost = lost
        self.get_logger().info(f'Target lost: {lost}')
        
    def set_movement_pattern(self, pattern: str):
        """Change the movement pattern."""
        self.movement_pattern = pattern
        self.movement_time = 0.0  # Reset time
        self.get_logger().info(f'Movement pattern changed to: {pattern}')


def main(args=None):
    rclpy.init(args=args)
    
    server = MockTrackPersonServer()
    
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
