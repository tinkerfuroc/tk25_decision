import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard, BtNode_WaitTicks
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation, BtNode_Listen
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection, BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_GripperAction
# from behavior_tree.StoringGroceries.customNodes import BtNode_FindObjTable, BtNode_GraspWithPose

# from .node_test import DecideNextAction, CheckAndWriteAction, WriteActionSuccessful
from .custom_nodes import BtNode_ScanForWavingPerson #BtNode_QA, 
from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import py_trees_ros
import rclpy

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
pose_shelf = possible_poses["shelf"]
pose_desk_lamp = possible_poses["desk_lamp"]
pose_kitchen = possible_poses["kitchen"]
pose_bathroom = possible_poses["bathroom"]
pose_QA_point = possible_poses["QA_point"]

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
KEY_SHELF_POSE = "shelf_pose"

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

def createGPSR():
    command1 = "tell me what is the smallest object on the shelf"
    command2 = "take the waving person from the desk lamp to the kitchen"
    command3 = "look for a person raising their right arm in the bathroom and answer a question"
    root = py_trees.composites.Sequence("GPSR", True)
    root.add_child(createConstantWriter())
    
    # arrive at instruction point
    root.add_child(createEnterArena())
    
    # ask for the three instructions
    root.add_child(BtNode_Announce(name="Prompt for instruction", bb_source=None, message="Dear person, please give me your first command after the beep sound."))
    root.add_child(py_trees.timers.Timer("wait before listening", duration=5.0))
    root.add_child(BtNode_Announce(name=f"ask to confirm instruction", bb_source=KEY_INSTRUCTION, message=f"Am I correct, your first command is {command1}"))
    root.add_child(BtNode_GetConfirmation("confirm instruction1"))

    root.add_child(BtNode_Announce(name="Prompt for instruction", bb_source=None, message="Dear person, please give me your second command after the beep sound."))
    root.add_child(py_trees.timers.Timer("wait before listening", duration=5.0))
    root.add_child(BtNode_Announce(name=f"ask to confirm instruction", bb_source=KEY_INSTRUCTION, message=f"Am I correct, your second command is {command2}"))
    root.add_child(BtNode_GetConfirmation("confirm instruction2"))

    root.add_child(BtNode_Announce(name="Prompt for instruction", bb_source=None, message="Dear person, please give me your third command after the beep sound."))
    root.add_child(py_trees.timers.Timer("wait before listening", duration=5.0))
    root.add_child(BtNode_Announce(name=f"ask to confirm instruction", bb_source=KEY_INSTRUCTION, message=f"Am I correct, your third command is {command3}"))
    root.add_child(BtNode_GetConfirmation("confirm instruction3"))

    # execution
    execution1 = py_trees.composites.Sequence("execute first command", True)
    execution1.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="The execution procedure for this command is go to the shelf, scan for the objects, find the smallest object, and announce the smallest object. Starting execution."))
    execution1.add_child(BtNode_GotoAction("go to shelf", KEY_SHELF_POSE))
    execution1.add_child(BtNode_Announce(name="announce scanning", bb_source=None, message="Scanning for objects on the shelf."))
    execution1.add_child(py_trees.timers.Timer("wait for scan", duration=1.0))
    execution1.add_child(BtNode_Announce(name="announce smallest object", bb_source=KEY_SMALLEST_OBJECT, message="The smallest object on the shelf is on the left, it is a cola can."))
    root.add_child(execution1)

    execution2 = py_trees.composites.Sequence("execute second command", True)
    execution2.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="The execution procedure for this command is go to the desk lamp, scan for the waving person, announce follow me, go to the kitchen. Starting execution."))
    execution2.add_child(BtNode_GotoAction("go to desk lamp", pose_desk_lamp))
    execution2.add_child(BtNode_Announce(name="announce scanning", bb_source=None, message="Scanning for the waving person."))
    execution2.add_child(py_trees.timers.Timer("wait for scan", duration=1.0))
    # we should have two person here in the scene, one is waving, the other is not
    execution2.add_child(BtNode_Announce(name="announce found waving person", bb_source=None, message="I have found the waving person. You are the person on the right, please follow me."))
    execution2.add_child(BtNode_GotoAction("go to kitchen", pose_kitchen))
    execution2.add_child(BtNode_Announce(name="announce arrived kitchen", bb_source=None, message="We have arrived at the kitchen. You can stop following me now."))
    root.add_child(execution2)

    execution3 = py_trees.composites.Sequence("execute third command", True)
    execution3.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="The execution procedure for this command is to go to the bathroom, scan for the raising person, go to the raising person, announce ask the question, announce the answer. Starting execution."))
    execution3.add_child(BtNode_GotoAction("go to bathroom", pose_bathroom))
    execution3.add_child(BtNode_Announce(name="announce scanning", bb_source=None, message="Scanning for the raising person."))
    execution3.add_child(py_trees.timers.Timer("wait for scan", duration=1.0))
    # we should have two person here in the scene, one is raising hand, the other is not
    execution3.add_child(BtNode_Announce(name="announce found raising person", bb_source=None, message="I have found the raising person. Approaching you now."))
    execution3.add_child(BtNode_GotoAction("approach raising person", pose_QA_point))
    execution3.add_child(BtNode_Announce(name="announce question", bb_source=None, message="Dear person, please ask your question after the beep sound."))
    execution3.add_child(py_trees.timers.Timer("wait before listening", duration=5.0))
    # the question is "what is 17+25"
    execution3.add_child(BtNode_Announce(name="announce answer", bb_source=KEY_QA_ANSWER, message="The answer to your question is 42."))
    root.add_child(execution3)

    root.add_child(BtNode_Announce(name="announce all commands finished", bb_source=None, message="I have completed all your commands. Thank you for your time. Goodbye!"))

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
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        # print(py_trees.display.unicode_blackboard())
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    rclpy.spin(tree.node)

    rclpy.shutdown()