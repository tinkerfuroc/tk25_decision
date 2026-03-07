import py_trees
from typing import List

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_ConvertGraspPose
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
prompt_drugs = ["aspirin box", "ibuprofen box", "acetaminophen box", "tray"]# depends on the drugs given, last one must be "tray"

prompt_list = prompt_drugs 

USE_GRASP_DUMMY = False

TRY_TWICE = False
DO_PLACE = True
DO_NAV = True


POS_CHECK_GRID = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["ending_point"]["point"]["x"], y=constants["ending_point"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["ending_point"]["orientation"]["x"], 
                                                            y=constants["ending_point"]["orientation"]["y"], 
                                                            z=constants["ending_point"]["orientation"]["z"], 
                                                            w=constants["ending_point"]["orientation"]["w"]))
                        )

POS_TABLE1 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_table1"]["point"]["x"], y=constants["pose_table1"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_table1"]["orientation"]["x"], 
                                                            y=constants["pose_table1"]["orientation"]["y"], 
                                                            z=constants["pose_table1"]["orientation"]["z"], 
                                                            w=constants["pose_table1"]["orientation"]["w"]))
                        )
# if needed to go to different tables or different position of a table
POS_TABLE2 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_table2"]["point"]["x"], y=constants["pose_table2"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_table2"]["orientation"]["x"], 
                                                                y=constants["pose_table2"]["orientation"]["y"], 
                                                                z=constants["pose_table2"]["orientation"]["z"], 
                                                                w=constants["pose_table2"]["orientation"]["w"]))
                            )
POS_TABLE3 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_table3"]["point"]["x"], y=constants["pose_table3"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_table3"]["orientation"]["x"], 
                                                                y=constants["pose_table3"]["orientation"]["y"], 
                                                                z=constants["pose_table3"]["orientation"]["z"], 
                                                                w=constants["pose_table3"]["orientation"]["w"]))
                        )
POS_SHELF1 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf1"]["point"]["x"], y=constants["pose_shelf1"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf1"]["orientation"]["x"], 
                                                                y=constants["pose_shelf1"]["orientation"]["y"], 
                                                                z=constants["pose_shelf1"]["orientation"]["z"], 
                                                                w=constants["pose_shelf1"]["orientation"]["w"]))
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
ARM_POS_SCAN4 = [x / 180 * math.pi for x in constants["arm_pos_scan4"]]
ARM_POS_SCAN5 = [x / 180 * math.pi for x in constants["arm_pos_scan5"]]
ARM_POS_SCAN6 = [x / 180 * math.pi for x in constants["arm_pos_scan6"]]
ARM_POS_SCAN7 = [x / 180 * math.pi for x in constants["arm_pos_scan7"]]
ARM_POS_SCAN8 = [x / 180 * math.pi for x in constants["arm_pos_scan8"]]
ARM_POS_SCAN9 = [x / 180 * math.pi for x in constants["arm_pos_scan9"]]

ARM_POS_DROP = [x / 180 * math.pi for x in constants["arm_pos_drop"]]
ARM_POS_PLACING = [x / 180 * math.pi for x in constants["arm_pos_placing"]]
N_LAYERS = constants["n_layers"]

GRASP_POSE_DUMMY = Pose(position=Point(x=15.226666017642406, y=-0.4662523345900057, z=0.0),
                        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0))

drugs = constants["drugs"]

##############################################################################   KEYS   ####################################################################################
################################   NAV   ################################
KEY_POS_SHELF1 = "pos_shelf1"
KEY_POS_SHELF_LEFT = "pos_shelf_left"
KEY_POS_SHELF_RIGHT = "pos_shelf_right"

KEY_POS_TABLE1 = "pos_table1"
KEY_POS_TABLE2 = "pos_table2"
KEY_POS_TABLE3 = "pos_table3"

KEY_POS_COMMAND = "pos_command" #position to receive command
KEY_POINT_TABLE_LEFT = "point_table_left"
KEY_POINT_TABLE_RIGHT = "point_table_right"
################################   ARM   ################################
KEY_ARM_SCAN1 = "arm_pos_scan1"
KEY_ARM_SCAN2 = "arm_pos_scan2"
KEY_ARM_SCAN3 = "arm_pos_scan3"
KEY_ARM_SCAN4 = "arm_pos_scan4"
KEY_ARM_SCAN5 = "arm_pos_scan5"
KEY_ARM_SCAN6 = "arm_pos_scan6"
KEY_ARM_SCAN7 = "arm_pos_scan7"
KEY_ARM_SCAN8 = "arm_pos_scan8"
KEY_ARM_SCAN9 = "arm_pos_scan9"

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
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to shelf", key=KEY_POS_SHELF1), num_failures=5))
    root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达橱柜"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table1", key=KEY_POS_TABLE1), num_failures=5))
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达桌子1"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to shelf", key=KEY_POS_SHELF1), num_failures=5))
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达橱柜"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table2", key=KEY_POS_TABLE2), num_failures=5))
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达桌子2"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to shelf", key=KEY_POS_SHELF1), num_failures=5))
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达橱柜"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table3", key=KEY_POS_TABLE3), num_failures=5))
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达桌子3"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to shelf", key=KEY_POS_SHELF1), num_failures=5)) 
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达橱柜"))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to endpoint", key=KEY_CHECK_GRID), num_failures=5))
    # root.add_child(BtNode_TTSCN(name="Announce leaving arena", bb_source=None, message="我已抵达终点"))
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
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN4, object=ARM_POS_SCAN4))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN5, object=ARM_POS_SCAN5))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN6, object=ARM_POS_SCAN6))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN7, object=ARM_POS_SCAN7))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN8, object=ARM_POS_SCAN8))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN9, object=ARM_POS_SCAN9))

    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing", bb_namespace="", bb_source=None, bb_key=KEY_ARM_PLACING, object=ARM_POS_PLACING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm drop", bb_namespace="", bb_source=None, bb_key=KEY_ARM_DROP, object=ARM_POS_DROP))
    
    ##########################################################   中关村 NAV KEYS   ###################################################################
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF1, object=POS_SHELF1))
    
    root.add_child(BtNode_WriteToBlackboard("Write Check Grid Position", bb_namespace="", bb_source=None, bb_key=KEY_CHECK_GRID, object=POS_CHECK_GRID))


    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE1, object=POS_TABLE1))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table2", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE2, object=POS_TABLE2))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table3", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE3, object=POS_TABLE3))
    ##########################################################   中关村 NAV KEYS   ###################################################################

    root.add_child(BtNode_WriteToBlackboard("Write Target Frame", bb_namespace="", bb_source=None, bb_key=KEY_TARGET_FRAME, object=point_target_frame))
    root.add_child(BtNode_WriteToBlackboard("Write Grasp Pose", bb_namespace="", bb_source=None, bb_key=KEY_GRASP_POSE_DUMMY, object=GRASP_POSE_DUMMY))
    root.add_child(BtNode_WriteToBlackboard("Write Point Place", bb_namespace="", bb_source=None, bb_key=KEY_POINT_PLACE, object=POINT_PLACE))
    # root.add_child(BtNode_WriteToBlackboard("Write Point Table Left", bb_namespace="", bb_source=None, bb_key=KEY_POINT_TABLE_LEFT, object=POINT_SHELF_LEFT))
    # root.add_child(BtNode_WriteToBlackboard("Write Point Table Right", bb_namespace="", bb_source=None, bb_key=KEY_POINT_TABLE_RIGHT, object=POINT_SHELF_RIGHT))
    return root

def createGraspOnce(bb_key_shelf: str):
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    # root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=20.0))
    # root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    parallel_move_arm = py_trees.composites.Parallel("Move arm to find object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_move_arm.add_child(BtNode_Announce(name="Announce moving arm", bb_source=None, message="Moving arm to find object"))
    parallel_move_arm.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=bb_key_shelf, add_octomap=True))
    root.add_child(parallel_move_arm)

    find = py_trees.composites.Selector(name="Find object", memory=True)
    find1 = py_trees.composites.Sequence(name="Find object1 sequence", memory=True)
    find1.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list[0]))
    find1.add_child(BtNode_FindObjTable("Find object on shelf", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    find.add_child(find1)
    find2 = py_trees.composites.Sequence(name="Find object2 sequence", memory=True)
    find2.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list[1]))
    find2.add_child(BtNode_FindObjTable("Find object on shelf", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    find.add_child(find2)
    find3 = py_trees.composites.Sequence(name="Find object3 sequence", memory=True)
    find3.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list[2]))
    find3.add_child(BtNode_FindObjTable("Find object on shelf", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    find.add_child(find3)
    root.add_child(py_trees.decorators.Retry('retry', find, num_failures=3))

    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, action_name=grasp_service_name))
    root.add_child(parallel_grasp)

    root.add_child(py_trees.decorators.Retry('retry', BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), 5))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createPlaceOnTable():
    root = py_trees.composites.Sequence(name="Place object", memory=True)
    # move arm to navigating position
    root.add_child(BtNode_MoveArmSingle("Move arm to navigating for easier scanning", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=25.0))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list[-1]))
    root.add_child(BtNode_FindObjTable("Find object on shelf", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    
    # if DO_PLACE:
    #     root.add_child(BtNode_MoveArmSingle("Move arm to scan", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    # announce placing on table
    place_parallel = py_trees.composites.Parallel("Place object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    place_parallel.add_child(BtNode_Announce(name="Announce placing on table", bb_source=KEY_PLACE_REASON))
    place_selector = py_trees.composites.Selector(name="Try dummy if place fail", memory=True)
    # place_parallel.add_child(BtNode_Announce(name="Announce placing on table", bb_source=None, message="Attempting to place on table"))
    
    if DO_PLACE:
        place_sequence = py_trees.composites.Sequence("Sequence of place", memory=True)
        place_sequence.add_child(BtNode_Place(name="Place object on table", 
                                            bb_key_point=KEY_OBJECT,
                                            bb_key_pose=KEY_GRASP_POSE_DUMMY if USE_GRASP_DUMMY else KEY_GRASP_POSE, 
                                            bb_key_env_points=KEY_ENV_POINTS,
                                            action_name=place_service_name))
        if TRY_TWICE:
            place_sequence.add_child(BtNode_Place(name="Place object on table", 
                                                bb_key_point=KEY_OBJECT, 
                                                bb_key_pose=KEY_GRASP_POSE_DUMMY, 
                                                bb_key_env_points=KEY_ENV_POINTS, 
                                                action_name=place_service_name))
            place_selector.add_child(py_trees.decorators.Retry(name="retry 2 times", child=place_sequence, num_failures=2))
        else:
            place_selector.add_child(place_sequence)
    
    # IF BtNode_Place fails
    dummy_place_sequence = py_trees.composites.Sequence("Dummy place sequence", memory=True)
    dummy_place_sequence.add_child(BtNode_Announce(name="Announce placing on table"))
    dummy_place_sequence.add_child(py_trees.decorators.Retry("retry", 
                                             BtNode_MoveArmSingle("Move arm to drop", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_DROP, add_octomap=False),
                                             3))
    dummy_place_sequence.add_child(py_trees.decorators.Retry("retry",
                                             BtNode_GripperAction("open gripper", True),
                                             3))
    dummy_place_sequence.add_child(BtNode_Announce(name="Announce placing complete", bb_source=None, message="Placing on table complete"))
    dummy_place_sequence.add_child(py_trees.decorators.Retry("retry",
                                             BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING),
                                             3))

    place_selector.add_child(dummy_place_sequence)
    place_parallel.add_child(place_selector)

    root.add_child(place_parallel)

    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createGoToShelf(bb_key_shelf: str):
    root = py_trees.composites.Sequence(name="Go to shelf", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to shelf", bb_source=None, message="Going to shelf"))
    if DO_NAV:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to shelf", bb_key_shelf), num_failures=10))
    return root

def createGoToTable(bb_key_table: str = KEY_POS_TABLE1):
    root = py_trees.composites.Sequence(name="Go to tray observation point", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to table", bb_key_table), num_failures=10))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list[-1]))
    root.add_child(BtNode_FindObjTable("Find tray on table", KEY_PROMPT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    root.add_child(BtNode_ConvertGraspPose("convert grasp pose", KEY_POSE_WAVING_PERSON, KEY_POSE_GRASP_POSE))

    BtNode_ConvertGraspPose
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
    return

def createTryArm():
    root = py_trees.composites.Sequence(name="Try Arm", memory=True)
    root.add_child(BtNode_Announce(name="Announce trying arm", bb_source=None, message="Trying arm"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle("Move arm to try", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), num_failures=3))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN1))
    # root.add_child(py_trees.behaviours.(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN2))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN3))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN4))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN5))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN6))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN7))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN8))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN9))
    # root.add_child(py_trees.behaviours.sleep.Sleep(name="Sleep after try arm", duration=1.0))
    
    return root

def createYanglaozhucan():
    root = py_trees.composites.Sequence(name="Taking drugs", memory=True)
    root.add_child(createConstantWriter())
    #root.add_child(BtNode_Announce(name="Announce starting storing drugs", bb_source=None, message="Starting taking drugs"))
    #root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    #root.add_child(createGetInfo(KEY_GUEST_DRUGS))
    root.add_child(createEnterArena()) # return which grid to go to
    root.add_child(createTryArm())
    root.add_child(py_trees.behaviours.Running(name="Running"))

    #trial = 0
    #retry_store = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(bb_key_targe  =KEY_TARGET_GRID, trial=trial), num_failures=2)
    #root.add_child(py_trees.decorators.Repeat(name="repeat 1 times", child=retry_store, num_success=1))
    # if more than 1 item, repeat storing process, give different table positions or different arm positions for each item 
    # trail = 1
    # retry_store2 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(bb_key_target=KEY_TARGET_GRID, trial=trial), num_failures=2)
    # root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store2, num_success=2))
    # trial = 2
    # retry_store3 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(bb_key_target=KEY_TARGET_GRID, trial=trial), num_failures=2)
    # root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store3, num_success=2))#

    #root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Picking drugs task complete"))
    #root.add_child(createLeaveArena())
    return root
