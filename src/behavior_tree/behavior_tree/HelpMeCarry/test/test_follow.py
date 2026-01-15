#!/usr/bin/env python3
"""
Test Script for Follow Behavior Tree

This script creates and runs the Follow behavior tree for testing purposes.
It connects to the mock servers for navigation and tracking.

Usage:
    ros2 run behavior_tree test_follow

Author: TinkerFuroc
"""

import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node

import sys
import time

# Add the behavior tree package to path if needed
from behavior_tree.HelpMeCarry.Follow import (
    createFollowPersonUntilStopped,
    createFollowPersonComplete,
    BtNode_TrackPersonAction,
    BtNode_ProcessTrackPosition,
    BtNode_CheckTargetStopped,
    BB_KEY_TRACK_POSITION,
    BB_KEY_FOLLOW_GOAL,
    BB_KEY_TRANSFORM_SUCCESS,
    DEFAULT_TARGET_FRAME
)


def create_test_tree(test_mode: str = 'full') -> py_trees.behaviour.Behaviour:
    """
    Create a test behavior tree.
    
    Args:
        test_mode: 'full' for complete follow behavior, 
                   'track_only' for just tracking,
                   'until_stopped' for follow until stopped
    """
    if test_mode == 'track_only':
        # Just track without navigation
        root = py_trees.composites.Sequence(name="Test Track Only", memory=True)
        
        track = BtNode_TrackPersonAction(
            name="Track Person",
            action_name="track_person",
            target_frame="map",
            bb_key_position=BB_KEY_TRACK_POSITION,
            bb_key_transform_success=BB_KEY_TRANSFORM_SUCCESS
        )
        
        # Run tracking for a while
        root.add_child(py_trees.decorators.Timeout(
            name="Track for 30s",
            child=track,
            duration=30.0
        ))
        
        return root
        
    elif test_mode == 'until_stopped':
        return createFollowPersonUntilStopped(
            follow_distance=1.5,
            stationary_threshold=0.3,
            stationary_duration=5.0,
            target_frame="map",
            action_name="track_person",
            nav_action_name="navigate_to_pose"
        )
        
    else:  # 'full'
        return createFollowPersonComplete(
            follow_distance=1.5,
            stationary_threshold=0.3,
            stationary_duration=5.0,
            target_frame="map",
            action_name="track_person",
            nav_action_name="navigate_to_pose"
        )


class FollowTestNode(Node):
    """ROS2 node for running the Follow behavior tree test."""
    
    def __init__(self, test_mode: str = 'full'):
        super().__init__('follow_test_node')
        
        # Declare and get parameter
        self.declare_parameter('mode', test_mode)
        self.test_mode = self.get_parameter('mode').get_parameter_value().string_value
        
        self.get_logger().info(f'Creating Follow test tree (mode: {self.test_mode})...')
        
        # Create behavior tree
        self.root = create_test_tree(self.test_mode)
        
        # Create py_trees_ros tree
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=self.root,
            unicode_tree_debug=True
        )
        
        # Setup the tree
        try:
            self.tree.setup(timeout=15.0)
            self.get_logger().info('Behavior tree setup complete')
        except Exception as e:
            self.get_logger().error(f'Failed to setup behavior tree: {e}')
            raise
            
        # Create timer for ticking the tree (250ms = 4Hz)
        self.tick_period = 0.250  # seconds
        self.tick_timer = self.create_timer(self.tick_period, self.tick_tree)
        
        self.tick_count = 0
        self.start_time = time.time()
        
        self.get_logger().info(f'Starting behavior tree (tick period: {self.tick_period}s)')
        
    def tick_tree(self):
        """Tick the behavior tree."""
        self.tick_count += 1
        
        # Tick the tree
        self.tree.tick()
        
        # Get status
        status = self.root.status
        
        # Log status periodically
        if self.tick_count % 4 == 0:  # Every second
            elapsed = time.time() - self.start_time
            self.get_logger().info(
                f'Tick {self.tick_count} ({elapsed:.1f}s): {status.name} - {self.root.feedback_message}'
            )
            
        # Check if tree completed
        if status == py_trees.common.Status.SUCCESS:
            self.get_logger().info('Behavior tree completed SUCCESSFULLY!')
            self.tick_timer.cancel()
        elif status == py_trees.common.Status.FAILURE:
            self.get_logger().error('Behavior tree FAILED!')
            self.tick_timer.cancel()
            
    def shutdown(self):
        """Shutdown the behavior tree."""
        self.tree.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line args
    test_mode = 'full'
    if len(sys.argv) > 1:
        test_mode = sys.argv[1]
        
    print(f'\n{"="*60}')
    print(f'Follow Behavior Tree Test')
    print(f'Mode: {test_mode}')
    print(f'{"="*60}\n')
    
    try:
        node = FollowTestNode(test_mode)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
    except Exception as e:
        print(f'Error: {e}')
        raise
    finally:
        if 'node' in locals():
            node.shutdown()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
