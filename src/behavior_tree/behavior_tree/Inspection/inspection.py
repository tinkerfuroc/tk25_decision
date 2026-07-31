import py_trees
from behavior_tree.core.resources import read_json

from behavior_tree.nodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.nodes.Navigation import BtNode_GotoAction
from behavior_tree.nodes.Audio import BtNode_Announce
from behavior_tree.nodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.nodes.Vision import  BtNode_DoorDetection, BtNode_TurnPanTilt

from .customNodes import BtNode_PressEnterToSucceed

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random
import math
import select
import sys

constants = read_json("behavior_tree.Inspection")

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
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_DOOR_STATUS = "door_status"


arm_action_name = "joint_move_action"

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

def createInspection():
    root = py_trees.composites.Sequence(name="Inspection Root", memory=True)

    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    # tuck the arm into the navigating pose
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", action_name=arm_action_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))

    # announce readiness and aim the pan-tilt at the referees, in parallel,
    # before waiting on the door
    ready = py_trees.composites.Parallel(name="Announce ready + aim pan-tilt", policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False))
    ready.add_child(BtNode_Announce(name="Announce ready for inspection", bb_source=None, message="I am ready for inspection, please open the door"))
    ready.add_child(BtNode_TurnPanTilt(name="Aim pan-tilt for inspection", x=0.0, y=45.0))
    root.add_child(ready)

    # wait until the door is detected open (Retry keeps polling on closed/error)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))

    # announce as soon as the door is open
    root.add_child(BtNode_Announce(name="Announce door open", bb_source=None, message="door open"))

    # navigate to the inspection point
    root.add_child(createToIspection())

    # brief self-introduction for the referees
    root.add_child(BtNode_Announce(name="introduce self", bb_source=None, message="Dear referees, I am Tinker."))

    # wait for the operator to press Enter, then head out
    root.add_child(BtNode_PressEnterToSucceed())

    root.add_child(BtNode_Announce(name="announce leaving", bb_source=None, message="Heading to the exit."))
    root.add_child(createToExit())
    return root
