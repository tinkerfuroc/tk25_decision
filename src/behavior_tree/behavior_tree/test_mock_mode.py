#!/usr/bin/env python3
"""
Test script to demonstrate mock mode functionality.
This can run with or without Tinker packages installed.
"""
import py_trees
import py_trees_ros
import rclpy

from behavior_tree.config import get_config, is_mock_mode
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.MockableNodes import create_mockable_node

try:
    from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
    from behavior_tree.TemplateNodes.Audio import BtNode_Announce
    from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
except ImportError as e:
    print(f"Warning: Could not import some template nodes: {e}")
    print("This is expected if dependencies are missing. Continuing with mocks...")


def create_test_tree():
    """
    Create a simple test tree that demonstrates mock mode.
    """
    root = py_trees.composites.Sequence(name="Mock Mode Test", memory=True)
    
    # Print configuration status
    config = get_config()
    print("\n" + "="*60)
    print("Mock Mode Test Script")
    print("="*60)
    config.print_status()
    print("\nThis script will:")
    print("1. Show configuration status")
    print("2. Attempt an 'announcement' (mock or real)")
    print("3. Attempt 'navigation' (mock or real)")
    print("4. Attempt 'arm movement' (mock or real)")
    print("\nPress 's' to advance through mock actions.")
    print("="*60 + "\n")
    
    # Start message
    root.add_child(BtNode_WaitKeyboardPress("Press 's' to start test", 's'))
    
    # Test 1: Audio announcement
    print("\n[Test 1] Audio Announcement...")
    if not is_mock_mode():
        try:
            announce_node = BtNode_Announce(
                name="Test Announcement",
                bb_source=None,
                message="Mock mode test running with real hardware"
            )
            root.add_child(announce_node)
        except:
            root.add_child(BtNode_WaitKeyboardPress(
                "MOCK: Audio announcement complete", 's'
            ))
    else:
        root.add_child(BtNode_WaitKeyboardPress(
            "MOCK: Audio announcement complete", 's'
        ))
    
    # Test 2: Navigation
    print("[Test 2] Navigation...")
    if not is_mock_mode():
        try:
            # In real mode, would use actual navigation
            root.add_child(BtNode_WaitKeyboardPress(
                "Real nav not configured in test, press 's'", 's'
            ))
        except:
            root.add_child(BtNode_WaitKeyboardPress(
                "MOCK: Navigation complete", 's'
            ))
    else:
        root.add_child(BtNode_WaitKeyboardPress(
            "MOCK: Navigation complete", 's'
        ))
    
    # Test 3: Arm movement
    print("[Test 3] Arm Movement...")
    if not is_mock_mode():
        try:
            # In real mode, would use actual arm control
            root.add_child(BtNode_WaitKeyboardPress(
                "Real arm control not configured in test, press 's'", 's'
            ))
        except:
            root.add_child(BtNode_WaitKeyboardPress(
                "MOCK: Arm movement complete", 's'
            ))
    else:
        root.add_child(BtNode_WaitKeyboardPress(
            "MOCK: Arm movement complete", 's'
        ))
    
    # End message
    root.add_child(BtNode_WaitKeyboardPress("Test complete! Press 's' to exit", 's'))
    
    return root


def main():
    """Main entry point."""
    # Initialize ROS2
    rclpy.init(args=None)
    
    # Create the test tree
    root = create_test_tree()
    
    # Visualize the tree structure
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)
    
    # Create ROS2 behavior tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="mock_mode_test", timeout=15)
    
    # Print function for tree status
    def print_tree(tree):
        print(py_trees.display.unicode_tree(
            root=tree.root, 
            show_status=True
        ))
    
    # Run the tree
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        print("\n\nTest interrupted by user")
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
        print("\n" + "="*60)
        print("Mock Mode Test Complete")
        print("="*60)


if __name__ == '__main__':
    main()
