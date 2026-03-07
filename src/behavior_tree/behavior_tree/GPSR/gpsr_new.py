import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard, BtNode_WaitTicks
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation, BtNode_Listen
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection, BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_GripperAction
from behavior_tree.StoringGroceries.customNodes import BtNode_FindObjTable, BtNode_GraspWithPose

from .node_test import DecideNextAction, CheckAndWriteAction, WriteActionSuccessful
from .custom_nodes import BtNode_ScanForWavingPerson #BtNode_QA, 
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import py_trees_ros
import rclpy
from behavior_tree.visualization import create_post_tick_visualizer

import json
import math

ANNOUNCE_REASONING = True

def parsePoseStamped(json_dict: dict):
    point = json_dict["point"]
    orientation = json_dict["orientation"]
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
        pose=Pose(position=Point(x=point["x"], y=point["y"], z=0.0),
        orientation=Quaternion(x=orientation['x'], y=orientation['y'], z=orientation['z'], w=orientation['w']))                
    )

# TODO: read from json file, fill in arms poses, and poses and object dictionary,
try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

possible_poses = constants["possible_poses"]
for key, value in possible_poses.items():
    possible_poses[key] = parsePoseStamped(value)
possible_objects = constants["possible_objects"]
pose_command = parsePoseStamped(constants["pose_command"])

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SCAN = [x / 180 * math.pi for x in constants["arm_pos_scan"]]

KEY_INSTRUCTION = "instruction"
KEY_TASK_STATUS = "task_status"
KEY_PAST_ACTIONS = "past_actions"
KEY_NEXT_ACTION = "next_action"
KEY_NEXT_ACTION_PARAMS = "next_action_parameters"
KEY_NEXT_ACTION_REASONING = "next_action_reasoning"
KEY_TARGET_POSE = "pose_target"
KEY_TARGET_OBJECT = "target_object"
KEY_ANNOUNCE_TEXT = "announce_text"
KEY_TARGET_OBJECT_NAME = "target_object_name"

KEY_TABLE_IMG = "table_img"
KEY_OBJ_SEG = "object_segmentation"
KEY_OBJECT = "object"
KEY_POINT_PLACE = "point_place"
KEY_POINT_PLACE_DUMMY = "point_place_dummy"
KEY_ENV_POINTS = "env_points"
KEY_GRASP_ANNOUNCEMENT = "grasp_announcement"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN = "arm_scan"
KEY_GRASP_POSE = "grasp_pose"
KEY_DOOR_STATUS = "door_status"
KEY_POSE_COMMAND = "pose_command"

KEY_QA_ANSWER = "qa_answer"
KEY_POSE_WAVING_PERSON = "waving_person"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
point_target_frame = "base_link"

categories = {
            "kuat": "green cola can",
            "fanta": "orange fanta can",
            "milk": "blue and white milk box",
            "orange juice": "orange juice bottle",
            "coke": "red cola can",
            "coffee": "brown and black box",
            "lime": "lime fruit",
            "tangerine": "tangerine fruit",
            "pear": "pear fruit",
            "lemon": "lemon fruit",
            "apple": "apple fruit",
            "snack bag": "snack bag",
            "pringles": "chips can",
            "chocolate": "chocolate bar",
            "ketchup": "red bottle with yellow cap",
            "oats": "whte cereal box",
            "mayo": "white bottle with black cap",
            "tuna": "blue tuna can"
            }

drink_items = ["kuat", "fanta", "milk", "orange juice", "coke", "coffee"]
fruit_items = ["lime", "tangerine", "pear", "lemon", "apple"]
snack_items = ["snack bag", "pringles", "chocolate"]
food_items = ["ketchup", "oats", "mayo", "tuna"]

def build_prompt(items):
    return " . ".join(categories[item] for item in items)

prompt_drinks = build_prompt(drink_items)
prompt_foods = build_prompt(food_items)
prompt_snacks = build_prompt(snack_items)
prompt_fruits = build_prompt(fruit_items)

# 拼接全部
prompt_list = " . ".join([prompt_drinks, prompt_foods, prompt_snacks, prompt_fruits])

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter arena", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle("move arm to navigating", arm_service_name, KEY_ARM_NAVIGATING), num_failures=5))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))
    root.add_child(BtNode_WaitTicks("wait for 10 ticks", 10))
    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering arena"))
    parallel_enter_arena.add_child(BtNode_TurnPanTilt(name="Turn pan tile", x=0.0, y=20.0, speed=0.0))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_POSE_COMMAND), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Pose Command", bb_namespace="", bb_source=None, bb_key=KEY_POSE_COMMAND, object=pose_command))
    return root

def createGoto():
    return py_trees.decorators.Retry(
        name="retry",
        child=BtNode_GotoAction("go to", KEY_TARGET_POSE),
        num_failures=5
    )

def createGotoWaving():
    root = py_trees.composites.Sequence("root of goto_waving", True)
    root.add_child(BtNode_ScanForWavingPerson("find waving person", KEY_POSE_WAVING_PERSON, use_orbbec=True, target_frame="base_link"))
    root.add_child(BtNode_Announce("announce found waving person", None, message="Found waving person. Dear person, could you move a little so I can reach you?"))
    root.add_child(py_trees.decorators.Retry("retry", BtNode_GotoAction("goto instruction point", KEY_POSE_WAVING_PERSON), 5))
    return root

def createGrasp():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_TurnPanTilt(name='turn pantilt', x=0.0, y=20.0))
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    parallel_move_arm = py_trees.composites.Parallel("Move arm to find object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_move_arm.add_child(BtNode_Announce(name="Announce moving arm", bb_source=None, message="Moving arm to find object"))
    parallel_move_arm.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    root.add_child(parallel_move_arm)

    find_and_grasp = py_trees.composites.Sequence(name="find and grasp", memory=True)
    # find object on table
    find_and_grasp.add_child(BtNode_FindObjTable("Find object on table", KEY_TARGET_OBJECT, KEY_TABLE_IMG, KEY_OBJ_SEG, KEY_OBJECT, KEY_GRASP_ANNOUNCEMENT))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, action_name=grasp_service_name))
    find_and_grasp.add_child(parallel_grasp)

    root.add_child(find_and_grasp)
    root.add_child(py_trees.decorators.Retry('retry', BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), 5))
    grasp_root = py_trees.decorators.Retry(name=f"retry 3 times", child=root, num_failures=3)

    ex_machina_grasp = py_trees.composites.Sequence("grasp ex machina", True)
    ex_machina_grasp.add_child(py_trees.decorators.Retry('retry', BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), 5))
    ex_machina_grasp.add_child(BtNode_GripperAction("open gripper", True))
    ex_machina_grasp.add_child(BtNode_Announce("announce help grasp", bb_source=KEY_TARGET_OBJECT_NAME, message="Dear referee, please help me grasp the "))
    ex_machina_grasp.add_child(BtNode_Announce("announce help grasp2", bb_source=None, message="Put it in my gripper please. Thank you"))
    ex_machina_grasp.add_child(BtNode_WaitTicks("wait ticks", 8)) # modify this if needed
    ex_machina_grasp.add_child(BtNode_Announce("announce", None, message="Thank you"))
    ex_machina_grasp.add_child(BtNode_GripperAction("close gripper", False))
    
    total_grasp = py_trees.composites.Selector(
        "grasp selector",
        True,
        [grasp_root, ex_machina_grasp]
        )
    return total_grasp

def createAnnounce():
    return BtNode_Announce(name="announce message", bb_source=KEY_ANNOUNCE_TEXT)

def createQA():
    root = py_trees.composites.Sequence(name="QA root", memory=True)
    # root.add_child(BtNode_QA("QA node", KEY_QA_ANSWER))
    root.add_child(BtNode_Announce("announce answer", KEY_QA_ANSWER))
    return root


def createExecuteInstruction():
    execute_one_step = py_trees.composites.Sequence("Root", True)

    decide_node = DecideNextAction()

    action_selector = py_trees.composites.Selector("ActionSelector", True)

    # Goto
    goto_branch = py_trees.composites.Sequence("Goto", True)
    goto_branch.add_children([
        CheckAndWriteAction("goto"),
        createGoto(),
        WriteActionSuccessful("goto")
    ])

    # Goto waving
    waving_branch = py_trees.composites.Sequence("GotoWaving", True)
    waving_branch.add_children([
        CheckAndWriteAction("goto_waving"),
        createGotoWaving(),
        WriteActionSuccessful("goto_waving")
    ])

    # Grasp
    grasp_branch = py_trees.composites.Sequence("Grasp", True)
    grasp_branch.add_children([
        CheckAndWriteAction("grasp"),
        createGrasp(),
        WriteActionSuccessful("grasp")
    ])

    # Announce
    announce_branch = py_trees.composites.Sequence("Announce", True)
    announce_branch.add_children([
        CheckAndWriteAction("announce"),
        createAnnounce(),
        WriteActionSuccessful("announce")
    ])

    # QA
    qa_branch = py_trees.composites.Sequence("QA", True)
    qa_branch.add_children([
        CheckAndWriteAction("qa"),
        createQA(),
        WriteActionSuccessful("qa")
    ])

    action_selector.add_children([
        goto_branch,
        waving_branch,
        grasp_branch,
        announce_branch,
        qa_branch
    ])

    finished_guard = CheckAndWriteAction("finished", name="check if finished")
    execute_action = py_trees.composites.Selector(
        "execute action selector", 
        True, 
        [py_trees.decorators.SuccessIsFailure("S is F", action_selector), finished_guard]
        )

    execute_one_step.add_child(BtNode_Announce("announce planning", bb_source=None, message="Planning next action"))
    execute_one_step.add_child(decide_node)
    if ANNOUNCE_REASONING:
        execute_one_step.add_child(BtNode_Announce("announce reasoning", bb_source=KEY_NEXT_ACTION_REASONING))
    execute_one_step.add_child(execute_action)

    execute_action = py_trees.decorators.Retry("Retry until success", execute_one_step, num_failures=10)
    return execute_action

def createCompleteOneCommand():
    root = py_trees.composites.Sequence("complete one command", True)

    # get_command = py_trees.composites.Sequence(name=f"get and confirm {type}", memory=True)
    # get_command.add_child(BtNode_Announce(name=f"Prompt for getting command", bb_source=None, message=f"Please speak to me after the beep sound. Tell me your command."))
    
    # get_command.add_child(BtNode_Listen(name="Listen to guest", bb_dest_key=KEY_INSTRUCTION, timeout=10.0))
    # get_command.add_child(BtNode_Announce(name=f"ask to confirm command", bb_source=KEY_INSTRUCTION, message=f"Am I correct, you command is "))
    # get_command.add_child(BtNode_GetConfirmation("confirm instruction"))
    # get_command.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="Starting execution."))
    root.add_child(BtNode_WriteToBlackboard("Write Pose Command", bb_namespace="", bb_source=None, bb_key=KEY_INSTRUCTION, object="go to waving person"))
    # root.add_child(py_trees.decorators.Retry("retry", get_command, 100))

    root.add_child(createExecuteInstruction())

    return root

def createGPSR():
    root = py_trees.composites.Sequence("GPSR", True)
    root.add_child(createConstantWriter())
    # root.add_child(createEnterArena())

    return_to_instruction_point = py_trees.composites.Parallel("return to instruction point", py_trees.common.ParallelPolicy.SuccessOnAll())
    return_to_instruction_point.add_child(BtNode_Announce("announce returning", bb_source=None, message="Returning to instruction point"))
    return_to_instruction_point.add_child(
        py_trees.decorators.Retry("retry", BtNode_GotoAction("goto instruction point", KEY_POSE_COMMAND), 5)
    )
    complete_instruction = py_trees.composites.Sequence(
        "complete on instruction and return",
        True,
        [createCompleteOneCommand(), return_to_instruction_point]
    )
    
    safe_guarded = py_trees.composites.Selector(
        "complete and return safeguard",
        True,
        [complete_instruction,
         py_trees.decorators.Retry("retry", BtNode_GotoAction("goto instruction point", KEY_POSE_COMMAND), 5)
         ]
    )
    root.add_child(
        py_trees.decorators.Repeat("repeat", safe_guarded, 5)
    )

    return root

def main():
    rclpy.init()
    # Setup blackboard values

    root = py_trees.composites.Sequence("Root", True)
    gpsr = createGPSR()
    root.add_children([
        gpsr,
        BtNode_Announce("announce finished", bb_source=None, message="GPSR accomplished"),
        py_trees.behaviours.Running("running")
    ])

    print("=== Running Behavior Tree ===")
    # Wrap the tree in a ROS-friendly interface
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
    )

    # Setup and spin
    tree.setup(timeout=15, node_name="root_node")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="GPSR",
    )
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    finally:
        shutdown_visualizer()
        rclpy.shutdown()
