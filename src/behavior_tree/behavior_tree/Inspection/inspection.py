import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce

from .customNodes import BtNode_PressEnterToSucceed

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random
import math
import json
import select
import sys

# POINT_TO_PERSON = False
TURN_PAN_TILT = True

MAX_SCAN_DISTANCE = 4.5

DEBUG_NO_GOTO = False

DISABLE_FEATURE_MATCH = False
DISABLE_FOLLOW_HEAD = False

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

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]



KEY_INSPECTION_POSE = "inspection_pose"
KEY_EXIT_POSE = "exit_pose"

arm_service_name = "arm_joint_service"

def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Write inspection location", bb_namespace="", bb_source=None, bb_key=KEY_INSPECTION_POSE, object=pose_inspection))
    root.add_child(BtNode_WriteToBlackboard(name="Write exit location", bb_namespace="", bb_source=None, bb_key=KEY_EXIT_POSE, object=pose_exit))
    return root

def createToIspection():
    root = py_trees.composites.Sequence(name="Go to inspection point", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("Go to inspection point", KEY_INSPECTION_POSE), num_failures=10))
    return root

def createToExit():
    root = py_trees.composites.Sequence(name="Go to exit", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("Go to exit", KEY_EXIT_POSE), num_failures=10))
    return root

def createReceptionist():
    root = py_trees.composites.Sequence(name="Inspection Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())
    root.add_child(createToIspection)
    root.add_child(BtNode_Announce("Announce: I am ready to inspect", "I am ready to inspect"))
    root.add_child(BtNode_PressEnterToSucceed(name="Press Enter to start inspection"))
    root.add_child(createToExit)
    return root
