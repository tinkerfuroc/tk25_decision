import py_trees
import py_trees_ros
import rclpy
from behavior_tree.Receptionist.customNodes import BtNode_HeadTracking, BtNode_HeadTrackingAction
from behavior_tree.visualization import create_post_tick_visualizer

head_tracking = py_trees.decorators.Repeat(name="repeat head tracking", child=py_trees.decorators.FailureIsSuccess("f is s", BtNode_HeadTracking(name="Follow guest2 head", service_name="follow_head_service")), num_success = -1)
head_tracking_action = BtNode_HeadTrackingAction(name="Follow guest head action", actionName="follow_head_action")

def main(args=None):
    rclpy.init(args=None)

    root = head_tracking_action

    # make it a ros tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="root_node", timeout=15)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Follow Head Test",
    )

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown() 
