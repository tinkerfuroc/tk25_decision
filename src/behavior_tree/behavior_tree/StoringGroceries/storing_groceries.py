import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_DoorDetection
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place, BtNode_GripperAction
from .customNodes import BtNode_CategorizeGrocery, BtNode_FindObjTable, BtNode_GraspWithPose

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

categories = {"chip": "blue and pink oreo box",
            "biscuit": "yellow chips can",
            "lays": "red chips can",
            "cookie": "black and green cookie box",
            "bread": "white bread",
            "sprite": "green sprite bottle",
            "cola": "black cola bottle",
            "orange juice": "orange bottle",
            "water": "clear water bottle",
            "dishsoap": "yellow and blue bottle",
            "handwash": "white handwash bottle",
            "shampoo": "blue shampoo bottle",
            "cereal bowl": "blue bowl"}

prompt_list = "water bottle . bread . cookie box . white hand sanitizer . yellow chip can . yellow dish washer bottle . green sprite" +\
                "black cola . blue shampoo"
USE_GRASP_DUMMY = False

TRY_TWICE = False
DO_PLACE = False

#14.949769937531444
POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=-2.6210129261016846, y=-2.6210129261016846, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
POS_TABLE2 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=-2.6210129261016846, y=-2.6210129261016846, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
POS_TABLE3 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=-2.6210129261016846, y=-2.6210129261016846, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=1.0, w=0.0))
                        )
# POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                         pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
#                                   orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
#                         )
# POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                         pose=Pose(position=Point(x=0.42952810404850056, y=-0.772284168894167, z=0.0),
#                                   orientation=Quaternion(x=0.0, y=0.0, z=0.041104151942754144, w=0.9991548672218271))
#                         )
# POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                         pose=Pose(position=Point(x=1.9408609917631492, y=-1.544248683233972, z=0.0),
#                                   orientation=Quaternion(x=0.0, y=0.0, z=-0.9999828747238424, w=0.005852372086610969))
#                         )
POS_SHELF = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        # pose=Pose(position=Point(x=-0.2942876962347504, y=0.7816651007796609, z=0.0),
                        pose=Pose(position=Point(x=-0.2942876962347504, y=0.6316651007796609, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.732980291613576, w=0.680249874))
                        )

POINT_PLACE = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                          point=Point(x=-0.266, y=1.297, z=0.706))

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN_MIDDLE = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 60.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]
ARM_POS_PLACING = ARM_POS_NAVIGATING
# ARM_POS_PLACING = [x / 180 * math.pi for x in [-87.6, -18.0, 8.3, 42.4, 1.6, -56.1, -20]]
GRASP_POSE_DUMMY = Pose(position=Point(x=15.226666017642406, y=-0.4662523345900057, z=0.0),
                        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0))

N_LAYERS = 2

KEY_POS_SHELF = "pos_shelf"
KEY_POS_TABLE = "pos_table"
KEY_POS_TABLE2 = "pos_table2"
KEY_POS_TABLE3 = "pos_table3"

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_GRASPING_MIDDLE = "arm_placing_middle"
KEY_ARM_PLACING = "arm_placing"

KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"
KEY_POINT_PLACE_DUMMY = "point_place_dummy"
KEY_ENV_POINTS = "env_points"

KEY_GRASP_POSE_DUMMY = "grasp_pose_dummy"
KEY_GRASP_POSE = "grasp_pose"
KEY_TARGET_FRAME = "target_frame"

KEY_PROMPT = "prompt"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"

KEY_DOOR_STATUS = "door_status"

KEY_PLACE_REASON = "place_reason"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"
point_target_frame = "base_link"

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter arena", memory=True)
    
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))
    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering arena"))
    # parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", service_name="move_base", target_pose=POS_TABLE, target_frame=point_target_frame)))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_POS_TABLE), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing Middle", bb_namespace="", bb_source=None, bb_key=KEY_ARM_GRASPING_MIDDLE, object=ARM_POS_SCAN_MIDDLE))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing", bb_namespace="", bb_source=None, bb_key=KEY_ARM_PLACING, object=ARM_POS_PLACING))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF, object=POS_SHELF))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE, object=POS_TABLE))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE2, object=POS_TABLE2))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE3, object=POS_TABLE3))
    root.add_child(BtNode_WriteToBlackboard("Write Target Frame", bb_namespace="", bb_source=None, bb_key=KEY_TARGET_FRAME, object=point_target_frame))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list))
    root.add_child(BtNode_WriteToBlackboard("Write Grasp Pose", bb_namespace="", bb_source=None, bb_key=KEY_GRASP_POSE_DUMMY, object=GRASP_POSE_DUMMY))
    root.add_child(BtNode_WriteToBlackboard("Write Point Place", bb_namespace="", bb_source=None, bb_key=KEY_POINT_PLACE, object=POINT_PLACE))
    return root

def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    # root.add_child(BtNode_MoveArmSingle("Move arm to find middle", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_GRASPING_MIDDLE, add_octomap=True))
    parallel_move_arm = py_trees.composites.Parallel("Move arm to find object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_move_arm.add_child(BtNode_Announce(name="Announce moving arm", bb_source="", message="Moving arm to find object"))
    parallel_move_arm.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    root.add_child(parallel_move_arm)
    # find object on table
    root.add_child(BtNode_FindObjTable("Find object on table", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, service_name=grasp_service_name))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createPlaceOnShelf():
    root = py_trees.composites.Sequence(name="Place object", memory=True)
    # move arm to navigating position
    root.add_child(BtNode_MoveArmSingle("Move arm to navigating for easier scanning", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    # move arm to scan position
    scan_parallel = py_trees.composites.Parallel("Scan object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    scan_parallel.add_child(BtNode_Announce(name="Announce scanning", bb_source=None, message="Scanning shelf to determine where to place object"))
    scan_parallel.add_child(BtNode_CategorizeGrocery("Categorize object", n_layers=N_LAYERS, bb_key_prompt=KEY_PROMPT, 
                                            bb_key_image=KEY_TABLE_IMG, bb_key_segment=KEY_OBJ_SEG, 
                                            bb_target_frame=KEY_TARGET_FRAME, bb_key_result_point=KEY_POINT_PLACE, 
                                            bb_key_env_points=KEY_ENV_POINTS, bb_key_reason=KEY_PLACE_REASON))
    root.add_child(scan_parallel)
    root.add_child(BtNode_MoveArmSingle("Move arm to scan", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    # announce placing on shelf
    place_parallel = py_trees.composites.Parallel("Place object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    place_parallel.add_child(BtNode_Announce(name="Announce placing on shelf", bb_source=KEY_PLACE_REASON))
    # place_parallel.add_child(BtNode_Announce(name="Announce placing on shelf", bb_source=None, message="Attempting to place on shelf"))
    
    if DO_PLACE:
        place_sequence = py_trees.composites.Selector("Sequence of place", memory=True)
        place_sequence.add_child(BtNode_Place(name="Place object on shelf", 
                                            bb_key_point=KEY_POINT_PLACE, 
                                            bb_key_pose=KEY_GRASP_POSE_DUMMY if USE_GRASP_DUMMY else KEY_GRASP_POSE, 
                                            bb_key_env_points=KEY_ENV_POINTS, 
                                            service_name=place_service_name))
        if TRY_TWICE:
            place_sequence.add_child(BtNode_Place(name="Place object on shelf", 
                                                bb_key_point=KEY_POINT_PLACE, 
                                                bb_key_pose=KEY_GRASP_POSE_DUMMY, 
                                                bb_key_env_points=KEY_ENV_POINTS, 
                                                service_name=place_service_name))
            place_parallel.add_child(py_trees.decorators.Retry(name="retry 2 times", child=place_sequence, num_failures=2))
        else:
            place_parallel.add_child(place_sequence)
    root.add_child(place_parallel)
    # Not sure if this works
    root.add_child(BtNode_GripperAction("open gripper", True))
    root.add_child(BtNode_Announce(name="Announce placing complete", bb_source=None, message="Placing on shelf complete"))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createGoToShelf():
    root = py_trees.composites.Sequence(name="Go to shelf", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to shelf", bb_source=None, message="Going to shelf"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to shelf", KEY_POS_SHELF), num_failures=10))
    return root

def createGoToTable(KEY_TABLE_POS):
    root = py_trees.composites.Sequence(name="Go to table", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to table", bb_source=None, message="Going to table"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to table", KEY_TABLE_POS), num_failures=10))
    return root

def createStoreOnce(KEY_TABLE_POS):
    root = py_trees.composites.Sequence(name="Store groceries once", memory=True)
    root.add_child(createGoToTable(KEY_TABLE_POS))
    root.add_child(createGraspOnce())
    root.add_child(createGoToShelf())
    root.add_child(createPlaceOnShelf())
    # root.add_child(createGoToTable())
    return root

def createStoreGroceries():
    root = py_trees.composites.Sequence(name="Store groceries", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(name="Announce starting storing groceries", bb_source=None, message="Starting storing groceries"))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(createEnterArena())
    retry_store = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(KEY_POS_TABLE), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store, num_success=2))
    retry_store2 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(KEY_POS_TABLE2), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store2, num_success=2))
    retry_store3 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(KEY_POS_TABLE3), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store3, num_success=2))
    root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Storing groceries task complete"))
    return root
