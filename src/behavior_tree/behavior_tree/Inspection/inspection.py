import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_Listen
from behavior_tree.TemplateNodes.Manipulation import BtNode_PointTo, BtNode_MoveArmSingle


from .customNodes import BtNode_PressEnterToSucceed

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random
import math
import json
import select
import sys

# read from `constant.json` in the same directory
# load file
try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/Inspection/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

pose_inspection = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_inspection"]["point"]["x"], y=constants["pose_inspection"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_inspection"]["orientation"]["x"], 
                                                            y=constants["pose_inspection"]["orientation"]["y"], 
                                                            z=constants["pose_inspection"]["orientation"]["z"], 
                                                            w=constants["pose_inspection"]["orientation"]["w"]))
                            )
pose_exit = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_exit"]["point"]["x"], y=constants["pose_exit"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_exit"]["orientation"]["x"], 
                                                            y=constants["pose_exit"]["orientation"]["y"], 
                                                            z=constants["pose_exit"]["orientation"]["z"], 
                                                            w=constants["pose_exit"]["orientation"]["w"]))
                            )

tinker_description = constants["tinker_description"]

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]



KEY_INSPECTION_POSE = "inspection_pose"
KEY_EXIT_POSE = "exit_pose"
KEY_LISTEN_RESULT = "listen_result"
KEY_ARM_NAVIGATING = "arm_navigating"


arm_service_name = "arm_joint_service"

def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Write inspection location", bb_namespace="", bb_source=None, bb_key=KEY_INSPECTION_POSE, object=pose_inspection))
    root.add_child(BtNode_WriteToBlackboard(name="Write exit location", bb_namespace="", bb_source=None, bb_key=KEY_EXIT_POSE, object=pose_exit))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    return root

def createToIspection():
    root = py_trees.composites.Sequence(name="Go to inspection point", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("Go to inspection point", KEY_INSPECTION_POSE), num_failures=10))
    return root

def createToExit():
    root = py_trees.composites.Sequence(name="Go to exit", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("Go to exit", KEY_EXIT_POSE), num_failures=10))
    return root

def createQandA():
    # TODO (for audio)
    # audio package should create a new node for general Q & A (just answering questions)
    # the node should accept a question (string) and an optional background_info (string)
    root = py_trees.composites.Sequence(name="Q & A", memory=True)
    root.add_child(BtNode_Listen(name="listen", bb_dest_key=KEY_LISTEN_RESULT))
    # TODO: replace these announce nodes with a call to the audio node for Q & A (listen result as question, tinker_description as background_info)
    # announce the results with the call to audio instead
    root.add_child(BtNode_Announce(name="announce I heard you", bb_source=None, message="I hear you, you said"))
    root.add_child(BtNode_Announce(name="repeat result of listen", bb_source=KEY_LISTEN_RESULT))
    return root

def createInspection():
    root = py_trees.composites.Sequence(name="Inspection Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    root.add_child(createToIspection())

    root.add_child(BtNode_Announce(name="Announce: I am ready to inspect", bb_source=None, message="I am Tinker, I am ready for inspection. I will briefly introduce myself."))
    root.add_child(BtNode_Announce(name="inrtoduce self", bb_source=None, message=tinker_description))
    
    # answer three questions (currently just repeats the question without answering)
    # root.add_child(py_trees.decorators.Repeat(name="repeat 3 times", child=createQandA(), num_success=3))
    root.add_child(BtNode_PressEnterToSucceed())

    root.add_child(BtNode_Announce(name="announce leaving", bb_source=None, message="Heading to the exit."))
    root.add_child(createToExit())
    return root
