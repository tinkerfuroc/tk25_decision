import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.TeleopNodes import BtNode_MoveArmTeleop
from behavior_tree.TemplateNodes.Manipulation import BtNode_Drop
from behavior_tree.visualization import create_post_tick_visualizer
from geometry_msgs.msg import PointStamped

def main():
    rclpy.init(args=None)

    root = BtNode_MoveArmTeleop(
        name="TestMoveArmTeleop",
        dof=7,
        cartesian_command_in_topic="/servo_server/delta_twist_cmds",
        joint_command_in_topic="/servo_server/delta_joint_cmds",
        gripper_command_in_topic="/gripper_servo_cmd",
        command_frame="link_base",
    )
    # root = BtNode_Drop(
    #     name="TestDrop",
    #     bb_source=None,
    #     bin_point=PointStamped()
    # )

    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="test_move_arm_teleop_node", timeout=15)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Move Arm Teleop Behavior Tree",
    )

    tree.tick_tock(period_ms=100.0, post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
