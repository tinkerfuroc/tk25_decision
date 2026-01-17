import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation, BtNode_Listen
from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_GripperAction
# from behavior_tree.StoringGroceries.customNodes import BtNode_FindObjTable, BtNode_GraspWithPose

# from .node_test import DecideNextAction, CheckAndWriteAction, WriteActionSuccessful
# from .custom_nodes import BtNode_ScanForWavingPerson #BtNode_QA, 
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

ARM_POS_NAVIGATING = [x / 180.0 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_SCAN = [x / 180.0 * math.pi for x in constants["arm_pos_scan"]]

KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN = "arm_scan"

KEY_INSTRUCTION = "instruction"
KEY_ANNOUNCE_TEXT = "announce_text"

KEY_QA_ANSWER = "qa_answer"
KEY_POSE_WAVING_PERSON = "waving_person"

KEY_POSE_SHELF = "shelf"
KEY_POSE_LAMP = "desk_lamp"
KEY_POSE_KITCHEN = "kitchen"
KEY_POSE_BATHROOM = "bathroom"
KEY_POSE_QA_POINT = "QA_point"
KEY_POSE_COMMAND = "pose_command"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
point_target_frame = "base_link"

def createConstantWriter():
    root = py_trees.composites.Parallel("Write Constants", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write command pose", bb_namespace="", bb_source=None, bb_key=KEY_POSE_COMMAND, object=pose_command))
    root.add_child(BtNode_WriteToBlackboard("Write shelf pose", bb_namespace="", bb_source=None, bb_key=KEY_POSE_SHELF, object=pose_shelf))
    root.add_child(BtNode_WriteToBlackboard("Write lamp pose", bb_namespace="", bb_source=None, bb_key=KEY_POSE_LAMP, object=pose_desk_lamp))
    root.add_child(BtNode_WriteToBlackboard("Write kitchen pose", bb_namespace="", bb_source=None, bb_key=KEY_POSE_KITCHEN, object=pose_kitchen))
    root.add_child(BtNode_WriteToBlackboard("Write bathroom pose", bb_namespace="", bb_source=None, bb_key=KEY_POSE_BATHROOM, object=pose_bathroom))
    root.add_child(BtNode_WriteToBlackboard("Write QA point pose", bb_namespace="", bb_source=None, bb_key=KEY_POSE_QA_POINT, object=pose_QA_point))
    return root

def createEnterArena():
    root = py_trees.composites.Sequence("Enter Arena", True)
    root.add_child(BtNode_Announce(name="announce entering arena", bb_source=None, message="Starting GPSR. Entering the arena now."))
    root.add_child(BtNode_MoveArmSingle(name="move arm to navigating position", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    root.add_child(BtNode_GripperAction(name="open gripper before navigation", open_gripper=True))
    root.add_child(BtNode_TurnPanTilt(name="turn pan tilt up", y=45.0))
    root.add_child(py_trees.timers.Timer("go to command point", duration=17.0))
    return root

def createGPSR():
    command1 = "tell me what is the smallest object on the sink"
    command2 = "take the waving person from the podium to the office"
    command3 = "look for a person raising their right arm in the living room and answer a question"
    root = py_trees.composites.Sequence("GPSR", True)
    root.add_child(createConstantWriter())
    
    # arrive at instruction point
    root.add_child(createEnterArena())
    
    # ask for the three instructions
    root.add_child(BtNode_Announce(name="Prompt for instruction", bb_source=None, message="Dear person, please give me your first command after the beep sound."))
    root.add_child(BtNode_Listen("listen for command", bb_dest_key=KEY_INSTRUCTION, timeout=5.0))
    root.add_child(BtNode_Announce(name=f"ask to confirm instruction", bb_source=None, message=f"Am I correct, your first command is {command1}"))
    root.add_child(BtNode_GetConfirmation("confirm instruction1"))
    root.add_child(BtNode_Announce("say thank you", bb_source=None, message="Thank you, I will remember that."))

    root.add_child(BtNode_Announce(name="Prompt for instruction", bb_source=None, message="Please give me your second command after the beep sound."))
    root.add_child(BtNode_Listen("listen for command", bb_dest_key=KEY_INSTRUCTION, timeout=5.0))
    root.add_child(BtNode_Announce(name=f"ask to confirm instruction", bb_source=None, message=f"Am I correct, your second command is {command2}"))
    root.add_child(BtNode_GetConfirmation("confirm instruction2"))
    root.add_child(BtNode_Announce("say thank you", bb_source=None, message="Thank you, I will remember that."))

    root.add_child(BtNode_Announce(name="Prompt for instruction", bb_source=None, message="Please give me your third command after the beep sound."))
    root.add_child(BtNode_Listen("listen for command", bb_dest_key=KEY_INSTRUCTION, timeout=5.0))
    root.add_child(BtNode_Announce(name=f"ask to confirm instruction", bb_source=None, message=f"Am I correct, your third command is {command3}"))
    root.add_child(BtNode_GetConfirmation("confirm instruction3"))
    root.add_child(BtNode_Announce("say thank you", bb_source=None, message="Thank you, I will remember that."))

    # execution
    execution1 = py_trees.composites.Sequence("execute first command", True)
    execution1.add_child(BtNode_Announce(name="announce starting first command", bb_source=None, message="Processing first command."))
    execution1.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="I shall go to the sink. Scan for objects. Find the smallest object. Go to command point. Announce the smallest object"))
    execution1.add_child(BtNode_Announce("starting execution", bb_source=None, message="Starting execution now."))
    execution1.add_child(BtNode_Announce(name="announce going to sink", bb_source=None, message="Going to the sink."))
    execution1.add_child(py_trees.timers.Timer("dummy for going to sink", duration=40.0))
    execution1.add_child(BtNode_TurnPanTilt("turn head down for scanning", y=20.0))
    execution1.add_child(BtNode_Announce(name="announce scanning", bb_source=None, message="Scanning objects."))
    execution1.add_child(py_trees.timers.Timer("wait for scan", duration=3.0))
    execution1.add_child(BtNode_Announce(name="announce found objects", bb_source=None, message="Found smallest object, returning."))
    execution1.add_child(BtNode_TurnPanTilt("turn head up", y=45.0))
    execution1.add_child(BtNode_Announce(name="announce going to sink", bb_source=None, message="Going to the command point."))
    execution1.add_child(py_trees.timers.Timer("dummy for returning from sink", duration=40.0))
    execution1.add_child(BtNode_Announce(name="announce smallest object", bb_source=None, message="Dear person, the smallest object on the sink is a bottle cola."))
    root.add_child(execution1)

    execution2 = py_trees.composites.Sequence("execute second command", True)
    execution2.add_child(BtNode_Announce(name="announce starting second command", bb_source=None, message="Processing second command."))
    execution2.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="I shall go to the podium. Scan for waving person. Announce follow me. Go to the office."))
    execution2.add_child(BtNode_Announce("starting execution", bb_source=None, message="Starting execution now."))
    execution2.add_child(BtNode_Announce(name="announce going to podium", bb_source=None, message="Going to the podium."))
    execution2.add_child(py_trees.timers.Timer("dummy for going to podium", duration=30.0))
    execution2.add_child(BtNode_Announce(name="announce scanning", bb_source=None, message="Scanning for the waving person."))
    execution2.add_child(py_trees.timers.Timer("wait for scan", duration=3.0))
    # we should have two person here in the scene, one is waving, the other is not
    execution2.add_child(BtNode_Announce(name="announce found waving person", bb_source=None, message="Found waving person, approaching"))
    execution2.add_child(py_trees.timers.Timer("dummy for going to waving person", duration=20.0))
    execution2.add_child(BtNode_Announce(name="announce follow me", bb_source=None, message="Please follow me to the office."))
    execution2.add_child(py_trees.timers.Timer("dummy for going to office", duration=50.0))
    execution2.add_child(BtNode_Announce(name="announce arrived office", bb_source=None, message="We have arrived at the office. You can stop following me now."))
    root.add_child(execution2)

    execution3 = py_trees.composites.Sequence("execute third command", True)
    execution3.add_child(BtNode_Announce(name="announce starting third command", bb_source=None, message="Processing third command."))
    execution3.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message="I shall go to the living room. Scan for person raising their right arm. Answer a question."))
    execution3.add_child(BtNode_Announce("starting execution", bb_source=None, message="Starting execution now."))
    execution3.add_child(BtNode_Announce(name="announce going to living room", bb_source=None, message="Going to the living room."))
    execution3.add_child(py_trees.timers.Timer("dummy for going to living room", duration=35.0))
    execution3.add_child(BtNode_Announce(name="announce scanning", bb_source=None, message="Scanning for person raising their right arm."))
    execution3.add_child(py_trees.timers.Timer("wait for scan", duration=2.0))
    # we should have two person here in the scene, one is raising hand, the other is not
    execution3.add_child(BtNode_Announce(name="announce found raising person", bb_source=None, message="Found person raising their right arm, approaching."))
    execution3.add_child(py_trees.timers.Timer("dummy for going to raising person", duration=20.0))
    execution3.add_child(BtNode_Announce(name="announce question", bb_source=None, message="Please ask your question after the beep sound."))
    execution3.add_child(BtNode_Listen("listen for command", bb_dest_key=KEY_INSTRUCTION, timeout=5.0))
    # the question is "what is 17+25"
    execution3.add_child(BtNode_Announce(name="announce received question", bb_source=None, message="I heard your question. Let me think about it."))
    execution3.add_child(BtNode_Announce(name="announce answer", bb_source=None, message="The answer to your question is 42."))
    root.add_child(execution3)

    root.add_child(BtNode_Announce(name="announce all commands finished", bb_source=None, message="I have completed all three commands."))

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
    tree.tick_tock(period_ms=250.0,post_tick_handler=print_tree)

    rclpy.spin(tree.node)

    rclpy.shutdown()