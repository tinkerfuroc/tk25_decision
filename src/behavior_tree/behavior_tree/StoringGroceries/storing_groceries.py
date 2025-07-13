import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_DoorDetection, BtNode_TurnPanTilt, BtNode_ScanFor
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Place, BtNode_GripperAction
from .customNodes import BtNode_CategorizeGrocery, BtNode_FindObjTable, BtNode_GraspWithPose

import math, time
import json

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/StoringGroceries/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

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
            "cereal bowl": "blue bowl"
            }

prompt_drinks = "green sprite bottle . black cola bottle . orange fanta bottle . clear water bottle . white milk box"
prompt_food = "pink oreo box . yellow chips can . red chips can . white bread"
prompt_utilities = "blue dishsoap bottle . white handwash bottle . blue shampoo bottle . blue bowl"

prompt_list = prompt_drinks + " . " + prompt_food + " . " + prompt_utilities
prompt_list = "bottle . white box . yellow box"
USE_GRASP_DUMMY = False

TRY_TWICE = False
DO_PLACE = True
DO_NAV = True

#14.949769937531444
POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_table1"]["point"]["x"], y=constants["pose_table1"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_table1"]["orientation"]["x"], 
                                                            y=constants["pose_table1"]["orientation"]["y"], 
                                                            z=constants["pose_table1"]["orientation"]["z"], 
                                                            w=constants["pose_table1"]["orientation"]["w"]))
                            )
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
POS_SHELF = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_shelf"]["point"]["x"], y=constants["pose_shelf"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_shelf"]["orientation"]["x"], 
                                                                y=constants["pose_shelf"]["orientation"]["y"], 
                                                                z=constants["pose_shelf"]["orientation"]["z"], 
                                                                w=constants["pose_shelf"]["orientation"]["w"]))
                            )
POS_TABLE3 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=10.8109273910, y=3.56819748, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.546470351326438, w=0.8374784505413614))
                        )
POS_SHELF = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.4658832550048828, y=-1.2834669351577759, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.47666905902949136, w=0.8790828221299397))
                        )
POINT_PLACE = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            point=Point(x=constants["point_place"]["x"], y=constants["point_place"]["y"], z=constants["point_place"]["z"]))
POINT_SHELF_LEFT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                point=Point(x=constants["point_shelf_left"]["x"], y=constants["point_shelf_left"]["y"], z=0.0))
POINT_SHELF_RIGHT = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                    point=Point(x=constants["point_shelf_right"]["x"], y=constants["point_shelf_right"]["y"], z=0.0))
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SCAN = [x / 180 * math.pi for x in constants["arm_pos_scan"]]
ARM_POS_DROP = [x / 180 * math.pi for x in constants["arm_pos_drop"]]
ARM_POS_PLACING = [x / 180 * math.pi for x in constants["arm_pos_placing"]]
N_LAYERS = constants["n_layers"]

GRASP_POSE_DUMMY = Pose(position=Point(x=15.226666017642406, y=-0.4662523345900057, z=0.0),
                        orientation=Quaternion(x=0.707106781, y=0.0, z=0.707106781, w=0.0))

KEY_POS_SHELF = "pos_shelf"
KEY_POS_TABLE = "pos_table"
KEY_POS_TABLE2 = "pos_table2"
KEY_POS_TABLE3 = "pos_table3"
KEY_POINT_SHELF_LEFT = "point_shelf_left"
KEY_POINT_SHELF_RIGHT = "point_shelf_right"

KEY_ARM_SCAN = "arm_scan"
#
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_PLACING = "arm_placing"
KEY_ARM_DROP = "arm_drop"

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

KEY_SCAN_RESULT = "scan_result"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_action"
point_target_frame = "base_link"

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter arena", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle("move arm to navigating", arm_service_name, KEY_ARM_NAVIGATING), num_failures=5))
    if DO_NAV:
        # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))
        pass
    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering arena"))
    parallel_enter_arena.add_child(BtNode_TurnPanTilt(name="Turn pan tile", x=0.0, y=20.0, speed=0.0))
    # parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", service_name="move_base", target_pose=POS_TABLE, target_frame=point_target_frame)))
    if DO_NAV:
        parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_POS_TABLE), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Placing", bb_namespace="", bb_source=None, bb_key=KEY_ARM_PLACING, object=ARM_POS_PLACING))
    root.add_child(BtNode_WriteToBlackboard("Write Arm drop", bb_namespace="", bb_source=None, bb_key=KEY_ARM_DROP, object=ARM_POS_DROP))
    root.add_child(BtNode_WriteToBlackboard("Write Position Shelf", bb_namespace="", bb_source=None, bb_key=KEY_POS_SHELF, object=POS_SHELF))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE, object=POS_TABLE))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE2, object=POS_TABLE2))
    root.add_child(BtNode_WriteToBlackboard("Write Position Table", bb_namespace="", bb_source=None, bb_key=KEY_POS_TABLE3, object=POS_TABLE3))
    root.add_child(BtNode_WriteToBlackboard("Write Target Frame", bb_namespace="", bb_source=None, bb_key=KEY_TARGET_FRAME, object=point_target_frame))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object=prompt_list))
    root.add_child(BtNode_WriteToBlackboard("Write Grasp Pose", bb_namespace="", bb_source=None, bb_key=KEY_GRASP_POSE_DUMMY, object=GRASP_POSE_DUMMY))
    root.add_child(BtNode_WriteToBlackboard("Write Point Place", bb_namespace="", bb_source=None, bb_key=KEY_POINT_PLACE, object=POINT_PLACE))
    root.add_child(BtNode_WriteToBlackboard("Write Point Shelf Left", bb_namespace="", bb_source=None, bb_key=KEY_POINT_SHELF_LEFT, object=POINT_SHELF_LEFT))
    root.add_child(BtNode_WriteToBlackboard("Write Point Shelf Right", bb_namespace="", bb_source=None, bb_key=KEY_POINT_SHELF_RIGHT, object=POINT_SHELF_RIGHT))
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

def createPlaceOnShelf():
    root = py_trees.composites.Sequence(name="Place object", memory=True)
    # move arm to navigating position
    root.add_child(BtNode_MoveArmSingle("Move arm to navigating for easier scanning", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=25.0))
    # move arm to scan position
    scan_parallel = py_trees.composites.Parallel("Scan object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    scan_parallel.add_child(BtNode_Announce(name="Announce scanning", bb_source=None, message="Scanning shelf to determine where to place object"))
    scan_parallel.add_child(BtNode_CategorizeGrocery("Categorize object", n_layers=N_LAYERS, bb_key_prompt=KEY_PROMPT, 
                                            bb_key_image=KEY_TABLE_IMG, bb_key_segment=KEY_OBJ_SEG, 
                                            bb_target_frame=KEY_TARGET_FRAME, bb_key_result_point=KEY_POINT_PLACE, 
                                            bb_key_env_points=KEY_ENV_POINTS, bb_key_reason=KEY_PLACE_REASON,
                                            bb_key_shelf_left=KEY_POINT_SHELF_LEFT, bb_key_shelf_right=KEY_POINT_SHELF_RIGHT))
    root.add_child(scan_parallel)
    if DO_PLACE:
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
                                            action_name=place_service_name))
        if TRY_TWICE:
            place_sequence.add_child(BtNode_Place(name="Place object on shelf", 
                                                bb_key_point=KEY_POINT_PLACE, 
                                                bb_key_pose=KEY_GRASP_POSE_DUMMY, 
                                                bb_key_env_points=KEY_ENV_POINTS, 
                                                action_name=place_service_name))
            place_parallel.add_child(py_trees.decorators.Retry(name="retry 2 times", child=place_sequence, num_failures=2))
        else:
            place_parallel.add_child(place_sequence)
    
    root.add_child(place_parallel)
    # Not sure if this works
    root.add_child(py_trees.decorators.Retry("retry", 
                                             BtNode_MoveArmSingle("Move arm to drop", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_DROP, add_octomap=False),
                                             3))
    root.add_child(py_trees.decorators.Retry("retry",
                                             BtNode_GripperAction("open gripper", True),
                                             3))
    root.add_child(BtNode_Announce(name="Announce placing complete", bb_source=None, message="Placing on shelf complete"))
    root.add_child(py_trees.decorators.Retry("retry",
                                             BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING),
                                             3))
    root.add_child(BtNode_TurnPanTilt(name="turn pantilt", x=0.0, y=20.0))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createGoToShelf():
    root = py_trees.composites.Sequence(name="Go to shelf", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to shelf", bb_source=None, message="Going to shelf"))
    if DO_NAV:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to shelf", KEY_POS_SHELF), num_failures=10))
    return root

def createGoToTable(KEY_TABLE_POS):
    root = py_trees.composites.Sequence(name="Go to table", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to table", bb_source=None, message="Going to table"))
    if DO_NAV:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to table", KEY_TABLE_POS), num_failures=10))
    return root

def createStoreOnce(KEY_TABLE_POS):
    root = py_trees.composites.Sequence(name="Store groceries once", memory=True)
    root.add_child(createGoToTable(KEY_TABLE_POS))
    root.add_child(createGraspOnce())
    root.add_child(createGoToShelf())
    #time.sleep(30)
    root.add_child(createPlaceOnShelf())
    return root

def createStoreGroceries():
    root = py_trees.composites.Sequence(name="Store groceries", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_Announce(name="Announce starting storing groceries", bb_source=None, message="Starting storing groceries"))
    # root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(createEnterArena())
    # retry_store = py_trees.decorators.Retry(name=f"retry 5 times", child=createStoreOnce(), num_failures=5)
    # root.add_child(py_trees.decorators.Repeat(name="repeat 5 times", child=retry_store, num_success=5))
    # # root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Storing groceries task complete"))
    # root.add_child(BtNode_ScanFor("scan and categorize", None, KEY_SCAN_RESULT, object=prompt_drinks, category="drinks"))
    # root.add_child(BtNode_ScanFor("scan and categorize", None, KEY_SCAN_RESULT, object=prompt_food, category="food"))
    # root.add_child(BtNode_ScanFor("scan and categorize", None, KEY_SCAN_RESULT, object=prompt_utilities, category="utilities"))
    retry_store = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(KEY_POS_TABLE), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 1 times", child=retry_store, num_success=1))
    retry_store2 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(KEY_POS_TABLE2), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store2, num_success=2))
    retry_store3 = py_trees.decorators.Retry(name=f"retry 2 times", child=createStoreOnce(KEY_POS_TABLE3), num_failures=2)
    root.add_child(py_trees.decorators.Repeat(name="repeat 2 times", child=retry_store3, num_success=2))
    root.add_child(BtNode_Announce(name="Announce complete", bb_source=None, message="Storing groceries task complete"))
    return root
