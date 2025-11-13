import py_trees
from typing import List

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_TTSCN, BtNode_Announce, BtNode_GetConfirmation, BtNode_PhraseExtraction, BtNode_GraspRequest
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_DoorDetection, BtNode_TurnPanTilt, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place, BtNode_GripperAction
from .customNodes import BtNode_CategorizeGrocery, BtNode_FindObjTable, BtNode_GraspWithPose, BtNode_Confirm, BtNode_WriteGrid

import math, time
import json

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/yanglaozhucan/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

# categories = {"chip": "blue and pink oreo box",
#             "biscuit": "yellow chips can",
#             "lays": "red chips can",
#             "cookie": "black and green cookie box",
#             "bread": "white bread",
#             "sprite": "green sprite bottle",
#             "cola": "black cola bottle",
#             "orange juice": "orange bottle",
#             "water": "clear water bottle",
#             "dishsoap": "yellow and blue bottle",
#             "handwash": "white handwash bottle",
#             "shampoo": "blue shampoo bottle",
#             "cereal bowl": "blue bowl"
#             }

# prompt_drinks = "green sprite bottle . black cola bottle . orange fanta bottle . clear water bottle . white milk box"
# prompt_food = "pink oreo box . yellow chips can . red chips can . white bread"
# prompt_utilities = "blue dishsoap bottle . white handwash bottle . blue shampoo bottle . blue bowl"
prompt_drugs = "aspirin box . ibuprofen box . acetaminophen box" # depends on the drugs given

prompt_list = prompt_drugs 

USE_GRASP_DUMMY = False

TRY_TWICE = False
DO_PLACE = True
DO_NAV = True


POS_CHECK_GRID = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_check_grid"]["point"]["x"], y=constants["pose_check_grid"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_check_grid"]["odrugsrientation"]["x"], 
                                                            y=constants["pose_check_grid"]["orientation"]["y"], 
                                                            z=constants["pose_check_grid"]["orientation"]["z"], 
                                                            w=constants["pose_check_grid"]["orientation"]["w"]))
                        )

POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_table1"]["point"]["x"], y=constants["pose_table1"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_table1"]["orientation"]["x"], 
                                                            y=constants["pose_table1"]["orientation"]["y"], 
                                                            z=constants["pose_table1"]["orientation"]["z"], 
                                                            w=constants["pose_table1"]["orientation"]["w"]))
                        )
# if needed to go to different tables or different position of a table
# POS_TABLE2 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                         pose=Pose(position=Point(x=constants["pose_table2"]["point"]["x"], y=constants["pose_table2"]["point"]["y"], z=0.0),
#                                     orientation=Quaternion(x=constants["pose_table2"]["orientation"]["x"], 
#                                                                 y=constants["pose_table2"]["orientation"]["y"], 
#                                                                 z=constants["pose_table2"]["orientation"]["z"], 
#                                                                 w=constants["pose_table2"]["orientation"]["w"]))
#                             )
# POS_TABLE3 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                         pose=Pose(position=Point(x=constants["pose_table3"]["point"]["x"], y=constants["pose_table3"]["point"]["y"], z=0.0),
#                                     orientation=Quaternion(x=constants["pose_table3"]["orientation"]["x"], 
#                                                                 y=constants["pose_table3"]["orientation"]["y"], 
#                                                                 z=constants["pose_table3"]["orientation"]["z"], 
#                                                                 w=constants["pose_table3"]["orientation"]["w"]))
                            # )
POS_SHELF1 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf1"]["point"]["x"], y=constants["pose_shelf1"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf1"]["orientation"]["x"], 
                                                                y=constants["pose_shelf1"]["orientation"]["y"], 
                                                                z=constants["pose_shelf1"]["orientation"]["z"], 
                                                                w=constants["pose_shelf1"]["orientation"]["w"]))
                        )
POS_SHELF2 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf2"]["point"]["x"], y=constants["pose_shelf2"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf2"]["orientation"]["x"], 
                                                                y=constants["pose_shelf2"]["orientation"]["y"], 
                                                                z=constants["pose_shelf2"]["orientation"]["z"], 
                                                                w=constants["pose_shelf2"]["orientation"]["w"]))
                        )

POS_SHELF3 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf3"]["point"]["x"], y=constants["pose_shelf3"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf3"]["orientation"]["x"], 
                                                                y=constants["pose_shelf3"]["orientation"]["y"], 
                                                                z=constants["pose_shelf3"]["orientation"]["z"], 
                                                                w=constants["pose_shelf3"]["orientation"]["w"]))
                        )   

POS_SHELF_LEFT = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf_left"]["point"]["x"], y=constants["pose_shelf_left"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf_left"]["orientation"]["x"], 
                                                                y=constants["pose_shelf_left"]["orientation"]["y"], 
                                                                z=constants["pose_shelf_left"]["orientation"]["z"], 
                                                                w=constants["pose_shelf_left"]["orientation"]["w"]))
                        )  

POS_SHELF_RIGHT = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf_right"]["point"]["x"], y=constants["pose_shelf_right"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf_right"]["orientation"]["x"], 
                                                                y=constants["pose_shelf_right"]["orientation"]["y"], 
                                                                z=constants["pose_shelf_right"]["orientation"]["z"], 
                                                                w=constants["pose_shelf_right"]["orientation"]["w"]))
                        )  



POINT_PLACE = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            point=Point(x=constants["point_place"]["x"], y=constants["point_place"]["y"], z=constants["point_place"]["z"]))
# if need to specify left and right point of the table for placing
# POINT_SHELF_LEFT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                 point=Point(x=constants["point_shelf_left"]["x"], y=constants["point_shelf_left"]["y"], z=0.0))
# POINT_SHELF_RIGHT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                     point=Point(x=constants["point_shelf_right"]["x"], y=constants["point_shelf_right"]["y"], z=0.0))
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]

ARM_POS_SCAN1 = [x / 180 * math.pi for x in constants["arm_pos_scan1"]]
ARM_POS_SCAN2 = [x / 180 * math.pi for x in constants["arm_pos_scan2"]]
ARM_POS_SCAN3 = [x / 180 * math.pi for x in constants["arm_pos_scan3"]]
# add more if needed
# ARM_POS_SCAN3 = [x / 180 * math.pi for x in constants["arm_pos_scan3"]]
# ARM_POS_SCAN3 = [x / 180 * math.pi for x in constants["arm_pos_scan3"]]

ARM_POS_DROP = [x / 180 * math.pi for x in constants["arm_pos_drop"]]
ARM_POS_PLACING = [x / 180 * math.pi for x in constants["arm_pos_placing"]]
N_LAYERS = constants["n_layers"]

GRASP_POSE_DUMMY = Pose(position=Point(x=15.226666017642406, y=-0.4662523345900057, z=0.0),
                        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0))

drugs = constants["drugs"]

##############################################################################   KEYS   ####################################################################################
################################   NAV   ################################
KEY_POS_SHELF1 = "pos_shelf1"
KEY_POS_SHELF2 = "pos_shelf2"
KEY_POS_SHELF3 = "pos_shelf3"
KEY_POS_SHELF_LEFT = "pos_shelf_left"
KEY_POS_SHELF_RIGHT = "pos_shelf_right"

KEY_POS_TABLE = "pos_table"

KEY_POS_COMMAND = "pos_command" #position to receive command
KEY_POINT_TABLE_LEFT = "point_table_left"
KEY_POINT_TABLE_RIGHT = "point_table_right"
################################   ARM   ################################
KEY_ARM_SCAN1 = "arm_scan1"
KEY_ARM_SCAN2 = "arm_scan2"
KEY_ARM_SCAN3 = "arm_scan3"

KEY_ARM_NAVIGATING = "arm_navigating"

KEY_ARM_PLACING = "arm_placing"

KEY_ARM_DROP = "arm_drop"
KEY_GRASP_POSE_DUMMY = "grasp_pose_dummy"
KEY_GRASP_POSE = "grasp_pose"
################################   VISION   ################################
KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"
KEY_POINT_PLACE_DUMMY = "point_place_dummy"
KEY_ENV_POINTS = "env_points"
KEY_TARGET_FRAME = "target_frame"

KEY_TARGET_GRID = "target_grid"
################################   AUDIO   ################################
KEY_PROMPT = "prompt"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"

################################   OTHERS  ################################
KEY_COMMAND = "command"
KEY_CHECK_GRID = "check_grid"
KEY_GUEST_DRUGS = "drugs"

KEY_DOOR_STATUS = "door_status"

KEY_PLACE_REASON = "place_reason"

KEY_SCAN_RESULT = "scan_result"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_action"
grid_service_name = "check_grid_service"
point_target_frame = "base_link"

##############################################################################   KEYS   ####################################################################################

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter", memory=True)
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    
    # if not DEBUG_NO_GOTO:
    #     root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))

    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_TTSCN(name="Announce entering arena", bb_source=None, message="我要出发咯"))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_CHECK_GRID)))
    root.add_child(parallel_enter_arena)

    root.add_child(BtNode_WriteGrid(name="Check target's grid", bb_key_dest=KEY_TARGET_GRID))

    return root

def createLeaveArena():
    root = py_trees.composites.Sequence(name="Leave", memory=True)
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    
    # if not DEBUG_NO_GOTO:
    #     root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))

    parallel_enter_arena = py_trees.composites.Parallel("Leave arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="离开中"))
    # parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", service_name="move_base", target_pose=POS_TABLE, target_frame=point_target_frame)))
    
    # root.add_child(BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Leaving to starting point", key=KEY_POS_COMMAND), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createListenToGuest(bb_dest_key:str, word_list: List[str]):
    root = py_trees.composibacktes.Selector(name="Listen to guest", memory=True)
    root.add_child(BtNode_PhraseExtraction(name="Listen to guest", bb_dest_key=bb_dest_key, wordlist=word_list, timeout=7.0))
    root.add_child(BtNode_GraspRequest(name="Request Grasp from Guest", object_names=word_list, object_codes=word_list, bb_dest_key=bb_dest_key))
    return py_trees.decorators.Retry(name="retry", child=root, num_failures=10)
    
def createGetInfo(storage_key:str):
    root = py_trees.composites.Sequence(name=f"Get drugs", memory=True)
    loop = py_trees.composites.Sequence(name=f"get and confirm drugs", memory=True)
    loop.add_child(BtNode_TTSCN(name=f"Prompt for drugs", bb_source=None, message=f"请告诉我需要帮您拿什么药物"))
    loop.add_child(createListenToGuest(bb_dest_key=storage_key, word_list=drugs))
    loop.add_child(BtNode_Confirm(name=f"Confirm drugs prompt", key_confirmed=storage_key, type="drugs"))
    loop.add_child(BtNode_GetConfirmation(name=f"Get drugs confirmation", timeout=5.0))
    root.add_child(py_trees.decorators.Retry(name="retry", child=loop, num_failures=10))
    return root

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN1, object=ARM_POS_SCAN1))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN2, object=ARM_POS_SCAN2))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN3, object=ARM_POS_SCAN3))

    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing", bb_namespace="", bb_source=None, bb_key=KEY_ARM_PLACING, object=ARM_POS_PLACING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm drop", bb_namespace="", bb_source=None, bb_key=KEY_ARM_DROP, object=ARM_POS_DROP))
    
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF1, object=POS_SHELF1))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF2, object=POS_SHELF2))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF3, object=POS_SHELF3))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF_LEFT, object=POS_SHELF_LEFT))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF_RIGHT, object=POS_SHELF_RIGHT))

    root.add_child(BtNode_WriteToBlackboard("Write Check Grid Position", bb_namespace="", bb_source=None, bb_key=KEY_CHECK_GRID, object=POS_CHECK_GRID))


    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE, object=POS_TABLE))
    # root.add_child(BtNode_WriteToBlackboard("Write Position Table2", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE2, object=POS_TABLE2))
    # root.add_child(BtNode_WriteToBlackboard("Write Position Table3", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE3, object=POS_TABLE3))
    root.add_child(BtNode_WriteToBlackboard("Write Target Frame", bb_namespace="", bb_source=None, bb_key=KEY_TARGET_FRAME, object=point_target_frame))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list))
    root.add_child(BtNode_WriteToBlackboard("Write Grasp Pose", bb_namespace="", bb_source=None, bb_key=KEY_GRASP_POSE_DUMMY, object=GRASP_POSE_DUMMY))
    root.add_child(BtNode_WriteToBlackboard("Write Point Place", bb_namespace="", bb_source=None, bb_key=KEY_POINT_PLACE, object=POINT_PLACE))
    # root.add_child(BtNode_WriteToBlackboard("Write Point Table Left", bb_namespace="", bb_source=None, bb_key=KEY_POINT_TABLE_LEFT, object=POINT_SHELF_LEFT))
    # root.add_child(BtNode_WriteToBlackboard("Write Point Table Right", bb_namespace="", bb_source=None, bb_key=KEY_POINT_TABLE_RIGHT, object=POINT_SHELF_RIGHT))
    return root

def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=20.0))
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    parallel_move_arm = py_trees.composites.Parallel("Move arm to find object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_move_arm.add_child(BtNode_Announce(name="Announce moving arm", bb_source=None, message="Moving arm to find object"))
    parallel_move_arm.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    root.add_child(parallel_move_arm)

    find_and_grasp = py_trees.composites.Sequence(name="find and grasp", memory=True)
    # find object on table
    find_and_grasp.add_child(BtNode_FindObjTable("Find object on table", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, action_name=grasp_service_name))
    find_and_grasp.add_child(parallel_grasp)

    root.add_child(find_and_grasp)
    root.add_child(py_trees.decorators.Retry('retry', BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), 5))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createPlaceOnTable(bb_key_place:str=KEY_POINT_PLACE):
    root = py_trees.composites.Sequence(name="Place object", memory=True)
    # move arm to navigating position
    root.add_child(BtNode_MoveArmSingle("Move arm to navigating for easier scanning", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=25.0))
    # move arm to scan position
    scan_parallel = py_trees.composites.Parallel("Scan object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    scan_parallel.add_child(BtNode_Announce(name="Announce scanning", bb_source=None, message="Scanning table to determine where to place object"))
    scan_parallel.add_child(BtNode_CategorizeGrocery("Categorize object", n_layers=N_LAYERS, bb_key_prompt=KEY_PROMPT, 
                                            bb_key_image=KEY_TABLE_IMG, bb_key_segment=KEY_OBJ_SEG, 
                                            bb_target_frame=KEY_TARGET_FRAME, bb_key_result_point=bb_key_place, 
                                            bb_key_env_points=KEY_ENV_POINTS, bb_key_reason=KEY_PLACE_REASON,
                                            bb_key_shelf_left=KEY_POINT_TABLE_LEFT, bb_key_shelf_right=KEY_POINT_TABLE_RIGHT))
    root.add_child(scan_parallel)
    if DO_PLACE:
        root.add_child(BtNode_MoveArmSingle("Move arm to scan", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    # announce placing on table
    place_parallel = py_trees.composites.Parallel("Place object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    place_parallel.add_child(BtNode_Announce(name="Announce placing on table", bb_source=KEY_PLACE_REASON))
    # place_parallel.add_child(BtNode_Announce(name="Announce placing on table", bb_source=None, message="Attempting to place on table"))
    
    if DO_PLACE:
        place_sequence = py_trees.composites.Selector("Sequence of place", memory=True)
        place_sequence.add_child(BtNode_Place(name="Place object on table", 
                                            bb_key_point=bb_key_place,
                                            bb_key_pose=KEY_GRASP_POSE_DUMMY if USE_GRASP_DUMMY else KEY_GRASP_POSE, 
                                            bb_key_env_points=KEY_ENV_POINTS,
                                            action_name=place_service_name))
        if TRY_TWICE:
            place_sequence.add_child(BtNode_Place(name="Place object on table", 
                                                bb_key_point=bb_key_place, 
                                                bb_key_pose=KEY_GRASP_POSE_DUMMY, 
                                                bb_key_env_points=KEY_ENV_POINTS, 
                                                action_name=place_service_name))
            place_parallel.add_child(py_trees.decorators.Retry(name="retry 2 times", child=place_sequence, num_failures=2))
        else:
            place_parallel.add_child(place_sequence)
    
    root.add_child(place_parallel)
    # IF BtNode_Place fails
    # root.add_child(py_trees.decorators.Retry("retry", 
    #                                          BtNode_MoveArmSingle("Move arm to drop", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_DROP, add_octomap=False),
    #                                          3))
    # root.add_child(py_trees.decorators.Retry("retry",
    #                                          BtNode_GripperAction("open gripper", True),
    #                                          3))
    # root.add_child(BtNode_Announce(name="Announce placing complete", bb_source=None, message="Placing on table complete"))
    # root.add_child(py_trees.decorators.Retry("retry",
    #                                          BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING),
    #                                          3))
    # root.add_child(BtNode_TurnPanTilt(name="turn pantilt", x=0.0, y=20.0))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createGoToShelf(bb_key_shelf: str):
    root = py_trees.composites.Sequence(name="Go to shelf", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to shelf", bb_source=None, message="Going to shelf"))
    if DO_NAV:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to shelf", bb_key_shelf), num_failures=10))
    return root

def createGoToTable(bb_key_table: str = KEY_POS_TABLE):
    root = py_trees.composites.Sequence(name="Go to table", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to table", bb_source=None, message="Going to table"))
    if DO_NAV:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to table", bb_key_table), num_failures=10))
    return root

def createStoreOnce(bb_key_target: str, trial: int):
    root = py_trees.composites.Sequence(name="Store drugs once", memory=True)
    target_grid = py_trees.blackboard.Blackboard.get(KEY_TARGET_GRID)

    root.add_child(createGoToShelf(f"pos_shelf{target_grid[trial][0]}"))
    root.add_child(createGraspOnce(f"arm_scan{target_grid[trial][1]}"))
    root.add_child(createGoToTable())
    root.add_child(createPlaceOnTable())
    return root

def createYanglaozhucan():
    root = py_trees.composites.Sequence(name="Taking drugs", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(name="Announce starting storing drugs", bb_source=None, message="Starting taking drugs"))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    #root.add_child(createGetInfo(KEY_GUEST_DRUGS))
    root.add_child(createEnterArena()) # return which grid to go to

    trial = 0
    retry_store = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(bb_key_target=KEY_TARGET_GRID, trial=trial), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 1 times", child=retry_store, num_success=1))
    # if more than 1 item, repeat storing process, give different table positions or different arm positions for each item 
    # trail = 1
    # retry_store2 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(bb_key_target=KEY_TARGET_GRID, trial=trial), num_failures=2)
    # root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store2, num_success=2))
    # trial = 2
    # retry_store3 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(bb_key_target=KEY_TARGET_GRID, trial=trial), num_failures=2)
    # root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store3, num_success=2))

    root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Picking drugs task complete"))
    root.add_child(createLeaveArena())
    return root
