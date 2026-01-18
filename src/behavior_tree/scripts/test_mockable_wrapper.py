"""
Example demonstrating how to use create_mockable_node() wrapper.

When BT_MOCK_MODE=true, nodes are automatically substituted with keyboard press + blackboard handling.
When BT_MOCK_MODE=false, the original nodes are used unchanged.

Usage:
    # Without mock mode (requires hardware):
    ros2 run behavior_tree test_mockable_wrapper
    
    # With mock mode (keyboard substitution):
    BT_MOCK_MODE=true ros2 run behavior_tree test_mockable_wrapper
"""

import py_trees
import rclpy
from behavior_tree.config import is_mock_mode
from behavior_tree.TemplateNodes.MockableNodes import create_mockable_node

# Import your actual node classes
from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle


def create_test_tree():
    """
    Create a simple behavior tree with mockable nodes.
    """
    
    # Initialize blackboard with required data
    blackboard = py_trees.blackboard.Client()
    blackboard.register_key(key="target_object", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="scan_result", access=py_trees.common.Access.WRITE)
    blackboard.set("target_object", "bottle", overwrite=True)
    
    # Create root sequence
    root = py_trees.composites.Sequence("Test Sequence", memory=False)
    
    # Create nodes using the wrapper - they automatically substitute in mock mode
    announce_node = create_mockable_node(
        BtNode_Announce(
            name="announce_start",
            sentence="Starting object detection"
        ),
        key_name="s"  # Press 's' to continue
    )
    
    scan_node = create_mockable_node(
        BtNode_ScanFor(
            name="scan_for_object",
            bb_source_key="target_object",
            bb_dest_key="scan_result",
            category="objects",
            read=True
        ),
        key_name="s"
    )
    
    announce_done = create_mockable_node(
        BtNode_Announce(
            name="announce_done",
            sentence="Detection complete"
        ),
        key_name="s"
    )
    
    # Add nodes to tree
    root.add_children([announce_node, scan_node, announce_done])
    
    return root


def main():
    """Main entry point"""
    rclpy.init()
    
    # Show mode
    if is_mock_mode():
        print("=" * 60)
        print("🔑 MOCK MODE ENABLED - Using keyboard substitutions")
        print("   Press 's' to advance through each step")
        print("=" * 60)
    else:
        print("=" * 60)
        print("🤖 REAL MODE - Using actual hardware/services")
        print("=" * 60)
    
    # Create and setup tree
    root = create_test_tree()
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(timeout=15)
    
    # Run tree
    try:
        for i in range(10):  # Max iterations
            print(f"\n--- Tick {i+1} ---")
            tree.tick()
            
            # Check if done
            if root.status == py_trees.common.Status.SUCCESS:
                print("\n✅ Tree completed successfully!")
                break
            elif root.status == py_trees.common.Status.FAILURE:
                print("\n❌ Tree failed!")
                break
                
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        tree.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
