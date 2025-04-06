import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Drop

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

prompt_list = "can . bottle . carton . container . cup . glass . jug . mug . pitcher . pot . sack . tub . tube . vase . box . bag . bowl . canister . jar"

POS_SHELF = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                        )
POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                        )

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -0.4187, 0.0, 1.709, 0.0, 1.343, 0.0]]
ARM_POS_PLACING = [x / 180 * math.pi for x in [-87.6, -18.0, 8.3, 42.4, 1.6, -56.1, -20]]

KEY_POS_SHELF = "pos_shelf"
KEY_POS_TABLE = "pos_table"

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_PLACING = "arm_placing"

KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_node"

def createConstantWriter():
    root = py_trees.composites.Sequence("Root")
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", key_dest=KEY_ARM_SCAN, key_value=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", key_dest=KEY_ARM_NAVIGATING, key_value=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing", key_dest=KEY_ARM_PLACING, key_value=ARM_POS_PLACING))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", key_dest=KEY_POS_SHELF, key_value=POS_SHELF))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", key_dest=KEY_POS_TABLE, key_value=POS_TABLE))
    return root

def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    root.add_child(BtNode_FindObj(name="find object", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object="green bottle", target_object_cls='green bottle'))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source="", message="grasping green water bottle"))
    parallel_grasp.add_child(BtNode_Grasp("Grasp trash", bb_source=KEY_OBJECT, service_name=grasp_service_name))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArmSingle("Move arm to scan middle", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN_MIDDLE))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return root

