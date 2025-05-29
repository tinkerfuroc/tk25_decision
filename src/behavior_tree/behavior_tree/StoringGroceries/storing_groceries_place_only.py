import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place, BtNode_GripperAction
from .customNodes import BtNode_CategorizeGrocery, BtNode_FindObjTable, BtNode_GraspWithPose

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

prompt_list = "can . bottle . apple"

#14.949769937531444
POS_SHELF = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=15.226666017642406, y=-0.4662523345900057, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.1661321085018473, w=0.9861035049753806))
                        )
POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=12.203, y=-2.165, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.9736709, w=0.227958))
                        )
POINT_SHELF_LEFT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                 point=Point(x=12.6343, y=5.4323167, z=0.706))
POINT_SHELF_RIGHT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                    point=Point(x=12.6343, y=5.4323167, z=0.706))

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]
ARM_POS_PLACING = ARM_POS_NAVIGATING
# ARM_POS_PLACING = [x / 180 * math.pi for x in [-87.6, -18.0, 8.3, 42.4, 1.6, -56.1, -20]]
GRASP_POSE_DUMMY = Pose(position=Point(x=15.226666017642406, y=-0.4662523345900057, z=0.0),
                        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0))

N_LAYERS = 2

KEY_POS_SHELF = "pos_shelf"
KEY_POS_TABLE = "pos_table"
KEY_POINT_SHELF_LEFT = "point_shelf_left"
KEY_POINT_SHELF_RIGHT = "point_shelf_right"

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_PLACING = "arm_placing"

KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"
KEY_ENV_POINTS = "env_points"

KEY_GRASP_POSE = "grasp_pose"
KEY_TARGET_FRAME = "target_frame"

KEY_PROMPT = "prompt"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"
point_target_frame = "base_link"

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing", bb_namespace="", bb_source=None, bb_key=KEY_ARM_PLACING, object=ARM_POS_PLACING))
    root.add_child(BtNode_WriteToBlackboard("Write Dummy Grasp Pose", bb_namespace="", bb_source=None, bb_key=KEY_GRASP_POSE, object=GRASP_POSE_DUMMY))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF, object=POS_SHELF))
    root.add_child(BtNode_WriteToBlackboard("Write Target Frame", bb_namespace="", bb_source=None, bb_key=KEY_TARGET_FRAME, object=point_target_frame))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list))
    root.add_child(BtNode_WriteToBlackboard("Write Point shelf left", bb_namespace="", bb_source=None, bb_key=KEY_POINT_SHELF_LEFT, object=POINT_SHELF_LEFT))
    root.add_child(BtNode_WriteToBlackboard("Write Point shelf right", bb_namespace="", bb_source=None, bb_key=KEY_POINT_SHELF_RIGHT, object=POINT_SHELF_RIGHT))
    return root

def createPlaceOnShelf():
    root = py_trees.composites.Sequence(name="Place object", memory=True)
    root.add_child(BtNode_CategorizeGrocery("Categorize object", n_layers=N_LAYERS, bb_key_prompt=KEY_PROMPT, 
                                            bb_key_image=KEY_TABLE_IMG, bb_key_segment=KEY_OBJ_SEG, 
                                            bb_target_frame=KEY_TARGET_FRAME, bb_key_result_point=KEY_POINT_PLACE, 
                                            bb_key_env_points=KEY_ENV_POINTS, bb_key_shelf_left=KEY_POINT_SHELF_LEFT, bb_key_shelf_right=KEY_POINT_SHELF_RIGHT,))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    # announce placing on shelf
    root.add_child(BtNode_Announce(name="Announce placing on shelf", bb_source=None, message="Placing on shelf"))
    root.add_child(BtNode_Place(name="Place object on shelf", bb_key_point=KEY_POINT_PLACE, bb_key_pose=KEY_GRASP_POSE, bb_key_env_points=KEY_ENV_POINTS, action_name=place_service_name))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    return root

def createPlaceOnce():
    root = py_trees.composites.Sequence(name="Place once", memory=True)
    # root.add_child(BtNode_GripperAction(name="close gripper", open_gripper=False))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(BtNode_FindObjTable("Find object", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    root.add_child(createPlaceOnShelf())
    return root

def createStoreGroceriesPlaceOnly():
    root = py_trees.composites.Sequence(name="Store groceries (placing only)", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(name="Announce starting placing", bb_source=None, message="Starting placing"))
    # TODO: repeat retry storing groceries 1 time
    retry_store = py_trees.decorators.Retry(name=f"retry 1 times", child=createPlaceOnce(), num_failures=1)
    root.add_child(py_trees.decorators.Repeat(name="repeat 1 times", child=retry_store, num_success=1))
    root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Placing task complete"))
    return root