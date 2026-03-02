import py_trees
import py_trees_ros

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_DoorDetection, BtNode_TurnPanTilt, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place, BtNode_GripperAction
from behavior_tree.visualization import create_post_tick_visualizer

import math, time
import json
import os

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

ARM_POS_GRASP = [x / 180 * math.pi for x in [0.0, 13.4, -8.7, 83.0, 8.0, 72.3, 0.3]]

KEY_ARM_GRASP = "arm_pose_grasp"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"

KEY_OBJECT = "detected_object"
KEY_GRASP_POSE = "grasp_pose"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"

def createConstantWriter():
    root = py_trees.composites.Sequence(name="Write Constants", memory=True)
    root.add_child(BtNode_WriteToBlackboard(name="Write arm pose for grasp", bb_namespace='',  bb_source=None, bb_key=KEY_ARM_GRASP, object=ARM_POS_GRASP))
    root.add_child(BtNode_WriteToBlackboard(name="Write grasp announcement", bb_namespace='',  bb_source=None, bb_key=KEY_GRASP_ANNOUNCEMENT, object="I will try to grasp the apple now."))
    return root


def createGraspOnce(retry_times=5):
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_GRASP, add_octomap=False))

    root.add_child(BtNode_FindObj(name=f"find bottle", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object="bottle", target_object_cls="bottle"))

    root.add_child(BtNode_Grasp(f"Grasp object on table", bb_source=KEY_OBJECT, action_name=grasp_service_name))
 
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_GRASP, add_octomap=False))

    return py_trees.decorators.Retry(name=f"retry {retry_times} times", child=root, num_failures=retry_times)

def main():
    rclpy.init()
    root = py_trees.composites.Sequence(name="Grasp Intel Demo", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(createGraspOnce())
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="tree_node", timeout=15)
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Grasp Intel Demo",
    )

    # if PRINT_DEBUG:
    #     py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()
