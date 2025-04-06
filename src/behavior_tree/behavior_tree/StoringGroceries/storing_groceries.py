import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place
from .customNodes import BtNode_CategorizeGrocery, BtNode_FindObjTable, BtNode_GraspWithPose

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

N_LAYERS = 4

KEY_POS_SHELF = "pos_shelf"
KEY_POS_TABLE = "pos_table"

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_PLACING = "arm_placing"

KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"

KEY_GRASP_POSE = "grasp_pose"

KEY_PROMPT = "prompt"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"

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
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", key_dest=KEY_PROMPT, key_value=prompt_list))
    return root

def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    root.add_child(BtNode_FindObjTable("Find object on table", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, service_name=grasp_service_name))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return root

def createPlaceOnShelf():
    root = py_trees.composites.Sequence(name="Place object", memory=True)
    root.add_child(BtNode_CategorizeGrocery("Categorize object", n_layers=N_LAYERS, bb_key_prompt=KEY_PROMPT, 
                                            bb_key_image=KEY_TABLE_IMG, bb_key_segment=KEY_OBJ_SEG, 
                                            bb_target_frame="map", bb_key_result_point=KEY_POINT_PLACE
                                            ))
    root.add_child(BtNode_Place(name="Place object on shelf", bb_key_point=KEY_POINT_PLACE, bb_key_pose=KEY_GRASP_POSE, service_name=place_service_name))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return root

def createGoToShelf():
    root = py_trees.composites.Sequence(name="Go to shelf", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to shelf", bb_source=None, message="Going to shelf"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to kitchen", KEY_POS_SHELF), num_failures=10))
    return root

def createGoToTable():
    root = py_trees.composites.Sequence(name="Go to table", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to table", bb_source=None, message="Going to table"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to kitchen", KEY_POS_TABLE), num_failures=10))
    return root

def createStoreOnce():
    root = py_trees.composites.Sequence(name="Store groceries once", memory=True)
    root.add_child(createGoToTable())
    root.add_child(createGraspOnce())
    root.add_child(createGoToShelf())
    root.add_child(createPlaceOnShelf())
    return root

def createStoreGroceries():
    root = py_trees.composites.Sequence(name="Store groceries", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(name="Announce starting storing groceries", bb_source=None, message="Starting storing groceries"))
    # TODO: repeat retry storing groceries five times
    for i in range(5):
        root.add_child(py_trees.decorators.Retry(name=f"retry {i}", child=createStoreOnce(), num_failures=5))
    root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Storing groceries task complete"))
    return root