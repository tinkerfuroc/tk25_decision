import py_trees
import py_trees_ros
import rclpy

from .HelpMeCarry.Track import createFollowPerson
from .Constants import PRINT_BLACKBOARD, PRINT_DEBUG

def test_follow():
    rclpy.init(args=None)

    root = createFollowPerson()

    # make it a ros tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        if PRINT_BLACKBOARD:
            print(py_trees.display.unicode_blackboard())

    if PRINT_DEBUG:
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()

def draw_follow():
    root = createFollowPerson()
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)

if __name__ == "__main__":
    draw_follow()