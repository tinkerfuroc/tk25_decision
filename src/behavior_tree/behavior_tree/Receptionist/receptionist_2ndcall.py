import py_trees
from typing import List

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction, BtNode_GetConfirmation, BtNode_Listen, BtNode_CompareInterest
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_SeatRecommend, BtNode_FeatureMatching, BtNode_TurnPanTilt, BtNode_DoorDetection, BtNode_TurnTo
from behavior_tree.TemplateNodes.Manipulation import BtNode_PointTo, BtNode_MoveArmSingle, BtNode_GripperAction
import py_trees_ros

from .customNodes import BtNode_CombinePerson, BtNode_Introduce, BtNode_Confirm, BtNode_HeadTracking, BtNode_HeadTrackingAction

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy
import warnings

import random
import math
import json

PRINT_DEBUG = True
PRINT_BLACKBOARD = False

# POINT_TO_PERSON = False
TURN_PAN_TILT = True

MAX_SCAN_DISTANCE = 4.5

DEBUG_NO_GOTO = False

DISABLE_FEATURE_MATCH = False
DISABLE_FOLLOW_HEAD = True

# read from `constant.json` in the same directory
# load file
try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/Receptionist/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

def pose_reader(pose_dict):
    return PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=pose_dict["point"]["x"], y=pose_dict["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=pose_dict["orientation"]["x"], 
                                                            y=pose_dict["orientation"]["y"], 
                                                            z=pose_dict["orientation"]["z"], 
                                                            w=pose_dict["orientation"]["w"]))
                        )

def arm_pose_reader(arm_pose_list):
    return [x / 180 * math.pi for x in arm_pose_list]

pose_door = pose_reader(constants["pose_door"])
pose_sofa = pose_reader(constants["pose_sofa"])
ARM_POS_NAVIGATING = arm_pose_reader(constants["arm_pos_navigating"])
ARM_POS_POINT_TO = arm_pose_reader(constants["arm_pos_point_to"])
ARM_POS_DROP = arm_pose_reader(constants["arm_pos_drop"])

host_name = constants["host_name"]
host_drink = constants["host_drink"]
drinks = constants["drinks"]
names = constants["names"]

KEY_ARM_INIT_POSE = "arm_init_pose"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_POINT_TO = "arm_point_to"
KEY_ARM_DROP_BAG_POSE = "arm_pos_drop" # new key for bag dropping pose

KEY_DOOR_POSE = "door_pose"
KEY_SOFA_POSE = "sofa_pose"

KEY_HOST_NAME = "host_name"
KEY_HOST_DRINK = "host_drink"
KEY_HOST_FEATURES = "host_features"

KEY_GUEST_NAME = "guest_name"
KEY_GUEST_DRINK = "guest_drink"
KEY_GUEST_FEATURES = "guest_features"

KEY_PERSONS = "persons"
KEY_PERSON_CENTROIDS = "centroids"

KEY_SEAT_RECOMMENDATION = "seat_recommendation"

KEY_DOOR_STATUS = "door_status"

arm_service_name = "arm_joint_service"

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter", memory=True)
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))

    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering"))
    
    if not DEBUG_NO_GOTO:
        parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to Sofa", key=KEY_SOFA_POSE), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Write door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    root.add_child(BtNode_WriteToBlackboard(name="Write sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Write host name", bb_namespace="", bb_source=None, bb_key=KEY_HOST_NAME, object=host_name))
    root.add_child(BtNode_WriteToBlackboard(name="Write host drink", bb_namespace="", bb_source=None, bb_key=KEY_HOST_DRINK, object=host_drink))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_PERSONS, object=[]))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm init pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_INIT_POSE, object=ARM_POS_POINT_TO))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm navigating pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    # new addition for bag grasping pose
    root.add_child(BtNode_WriteToBlackboard(name="Write grasp bag pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_POINT_TO, object=ARM_POS_POINT_TO))
    # new addition for bag dropping pose
    root.add_child(BtNode_WriteToBlackboard(name="Write drop bag pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_DROP_BAG_POSE, object=ARM_POS_DROP))
    return root

def createListenToGuest(bb_dest_key:str, word_list: List[str]):
    root = py_trees.composites.Selector(name="Listen to guest", memory=True)
    root.add_child(BtNode_PhraseExtraction(name="Listen to guest", bb_dest_key=bb_dest_key, wordlist=word_list, timeout=7.0))
    root.add_child(py_trees.decorators.SuccessIsFailure(name="success is failure", child=BtNode_Announce(name="Listen Failed, ask for repeat", bb_source=None, message="I'm sorry. Could you please repeat that louder and closer?")))
    return py_trees.decorators.Retry(name="retry", child=root, num_failures=10)

def createGetInfo(type:str, storage_key:str):
    root = py_trees.composites.Sequence(name=f"Get {type}", memory=True)
    loop = py_trees.composites.Sequence(name=f"get and confirm {type}", memory=True)
    loop.add_child(BtNode_Announce(name=f"Prompt for {type}", bb_source=None, message=f"Speak loudly and tell me your {type}"))
    loop.add_child(createListenToGuest(bb_dest_key=storage_key, word_list=names if type == "name" else drinks))
    loop.add_child(BtNode_Confirm(name=f"Confirm {type} prompt", key_confirmed=storage_key, type=type))
    loop.add_child(BtNode_GetConfirmation(name=f"Get {type} confirmation", timeout=5.0))
    root.add_child(py_trees.decorators.Retry(name="retry", child=loop, num_failures=10))
    return root

def createGetName():
    root = py_trees.composites.Sequence(name="Get correct name and drink", memory=True)
    root.add_child(BtNode_Announce(name="Reminder of beep", bb_source=None, message="Hi I am Tinker, please speak to me after the beep sound."))
    root.add_child(createGetInfo("name", KEY_GUEST_NAME))
    return root

# warnings.warn("drink can no longer be asked during entry in Robocup 2025", DeprecationWarning)
def createGetNameAndDrink():
    root = py_trees.composites.Sequence(name="Get correct name and drink", memory=True)
    root.add_child(BtNode_Announce(name="Reminder of beep", bb_source=None, message="Hi I am Tinker, please speak to me after the beep sound."))
    # root.add_child(createGetInfo("name", KEY_GUEST_NAME))
    root.add_child(createGetName())
    root.add_child(createGetInfo("favorite drink", KEY_GUEST_DRINK))
    return root

def createRegisterFeature():
    root = py_trees.composites.Sequence(name="Register features of person in front", memory=True)
    root.add_child(BtNode_Announce(name="Ask to stand in front", bb_source=None, message="Stand one meter in front of me. Thank you"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_GUEST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine person's info", key_dest=KEY_PERSONS, key_name=KEY_GUEST_NAME, key_drink=KEY_GUEST_DRINK, key_features=KEY_GUEST_FEATURES))
    root.add_child(BtNode_Announce(name="Indicate follow", bb_source=None, message="Follow me"))
    return root

def createRecommendSeat():
    find_and_recommend_seat = py_trees.composites.Sequence(name="find and recommend seat", memory=True)
    find_and_recommend_seat.add_child(BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS))
    find_and_recommend_seat.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))
    return find_and_recommend_seat

def createFirstIntroductionsSimple():
    find_and_recommend_seat = py_trees.composites.Sequence(name="find and recommend seat", memory=True)
    find_and_recommend_seat.add_child(BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS))
    find_and_recommend_seat.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))
    
    look_at_guest1 = py_trees.composites.Sequence(name="look at guest 1", memory=True)
    look_at_guest1.add_child(BtNode_TurnPanTilt(name="Turn head to the right", x=90.0, y=45.0, speed=0.0))
    head_tracking = py_trees.behaviours.Running("dummy head track")
    if not DISABLE_FOLLOW_HEAD:
        head_tracking = BtNode_HeadTrackingAction(name="Follow guest head action", actionName="follow_head_action")
    look_at_guest1.add_child(head_tracking)

    root = py_trees.composites.Parallel(
        name="first seat recommendation", 
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([find_and_recommend_seat]), 
        children=[find_and_recommend_seat, look_at_guest1]
    )
    return root

def createSecondIntroductionsSimple():
    root = py_trees.composites.Sequence(name="second introductions", memory=True)
    find_and_recommend_seat = py_trees.composites.Sequence(name="find and recommend seat", memory=True)
    find_and_recommend_seat.add_child(BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS))
    
    introductions = py_trees.composites.Sequence(
        name="introduce first guest to second guest",
        memory=True
        )

    introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the right", x=90.0, y=45.0, speed=0.0))

    head_tracking = py_trees.behaviours.Running("dummy head track")
    point_to = py_trees.behaviours.Running("dummy head track")
    if not DISABLE_FOLLOW_HEAD:
        head_tracking = BtNode_HeadTrackingAction(
            name="Follow guest head action", 
            actionName="follow_head_action"
            )
        # head_tracking = py_trees.decorators.Repeat(name="repeat head tracking", child=py_trees.decorators.FailureIsSuccess("f is s", BtNode_HeadTracking(name="Follow guest2 head", service_name="follow_head_service")), num_success = -1)
    if not DISABLE_FEATURE_MATCH:
        # point to guest1
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_PointTo(
            name="Point to guest1", 
            service_name=arm_service_name, 
            bb_key_persons=KEY_PERSONS, 
            bb_key_points=KEY_PERSON_CENTROIDS, 
            bb_key_init_pose=KEY_ARM_INIT_POSE, 
            target_id=1
            ), num_failures=3)
        point_to = py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco)
    introduce = BtNode_Introduce(name="introduce first guest to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=1, describe_introduced=False)
    turn_head_arm3 = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([introduce]), children=[head_tracking, point_to, introduce])
    introductions.add_child(turn_head_arm3)
    
    # introduce second guest to first guest
    introduce2 = py_trees.composites.Sequence(name="sequence", memory=True)
    introduce2.add_child(BtNode_TurnTo(name="Turn to guest1", bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, target_id=1))
    introduce2.add_child(BtNode_Introduce(name="introduce second guest to first guest", key_person=KEY_PERSONS, target_id=1, introduced_id=2))
    introduce_w_followhead2 = py_trees.composites.Parallel(name="Introduce second to first", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    introduce_w_followhead2.add_child(introduce2)
    if not DISABLE_FEATURE_MATCH:
        # point to guest
        # first_introductions.add_child(BtNode_PointTo(name="Point to guest", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=1))
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle(name="Move arm to right", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_INIT_POSE, add_octomap=False), num_failures=3)
        introduce_w_followhead2.add_child(py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco))
    introductions.add_child(introduce_w_followhead2)
    root.add_child(introductions)

    root.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))
    return root

def createToDoor():
    root = py_trees.composites.Sequence(name="Go to door", memory=True)
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    if not DEBUG_NO_GOTO:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    return root

def createToSofa():
    root = py_trees.composites.Parallel(name="Go to sofa", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    navigation_seq = py_trees.composites.Sequence(name="Go to sofa", memory=True)
    navigation_seq.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    navigation_seq.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    if not DEBUG_NO_GOTO:
        navigation_seq.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))
    root.add_child(navigation_seq)
    return root

def createAnnounceAndScanSofa():
    root = py_trees.composites.Sequence(name="Announce while feature matching", memory=True)
    # Turn head down a bit for better feature matching
    root.add_child(BtNode_TurnPanTilt(name="Turn head down", x=0.0, y=20.0, speed=0.0))
    if not DISABLE_FEATURE_MATCH:
        parallel_matching = py_trees.composites.Parallel(name="Feature matching", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
        parallel_matching.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_FeatureMatching(name="Feature matching", bb_dest_key=KEY_PERSON_CENTROIDS, bb_persons_key=KEY_PERSONS, max_distance=MAX_SCAN_DISTANCE), num_failures=5))
        parallel_matching.add_child(BtNode_Announce(name="Announce feature matching", bb_source=None, message="Scanning seated personnels"))
        root.add_child(parallel_matching)
    root.add_child(BtNode_Announce(name="Tell guest to stand on right", bb_source=None, message="Stand on my right"))
    return root

def createGreetGuest():
    root = py_trees.composites.Sequence(name="Greet guest", memory=True)
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    root.add_child(createToDoor())
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    root.add_child(createGetNameAndDrink())
    root.add_child(createRegisterFeature())
    return root

def createScanHostFeatures():
    root = py_trees.composites.Sequence(name="Scan host features", memory=True)
    root.add_child(BtNode_TurnPanTilt(name="Turn head down", x=0.0, y=20.0, speed=0.0))
    if not DEBUG_NO_GOTO:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))    
    root.add_child(BtNode_Announce(name="announce scanning host features", bb_source=None, message="Scanning host"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_HOST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine host's info", key_dest=KEY_PERSONS, key_name=KEY_HOST_NAME, key_drink=KEY_HOST_DRINK, key_features=KEY_HOST_FEATURES))
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    return root

def createGraspBag():
    root = py_trees.composites.Sequence(name="Grasp bag", memory=True)
    root.add_child(py_trees.decorators.Retry(
        name="retry", 
        child=BtNode_MoveArmSingle(
            name="Move arm to bag position", 
            service_name=arm_service_name, 
            arm_pose_bb_key=KEY_ARM_POINT_TO,
            add_octomap=False
        ), 
        num_failures=3
    ))
    root.add_child(BtNode_GripperAction(name="Open gripper", open_gripper=True))

    root.add_child(BtNode_Announce(name="Ask guest to fetch his bag", bb_source=None, message="Could you please hand me your bag by placing the handle in my gripper?"))
    
    root.add_child(py_trees.timers.Timer(name="Wait for bag placement", duration=2.0))
    root.add_child(BtNode_GripperAction(name="Close gripper", open_gripper=False))

    # move arm back to navigating pose
    root.add_child(py_trees.decorators.Retry(
        name="retry", 
        child=BtNode_MoveArmSingle(
            name="Move arm to navigation pose", 
            service_name=arm_service_name, 
            arm_pose_bb_key=KEY_ARM_NAVIGATING, 
            add_octomap=False
        ), 
        num_failures=3
    ))
    return root

def createFollowPerson():
    root = py_trees.composites.Sequence(name="Follow person", memory=True)
    root.add_child(BtNode_Announce(name="Announce follow", bb_source=None, message="I shall follow you."))
    root.add_child(py_trees.timers.Timer(name="Dummy wait for follow", duration=10.0))
    root.add_child(BtNode_Announce(name="Announce follow end", bb_source=None, message="I sensed you have arrived."))
    return root

def createDropBag():
    root = py_trees.composites.Sequence(name="Drop the bag", memory=True)
    root.add_child(BtNode_Announce(name="Ask host where to drop the bag", bb_source=None, message="Where should I drop the bag?"))
    root.add_child(py_trees.timers.Timer(name="Wait for host response", duration=5.0))

    root.add_child(py_trees.decorators.Retry(
        name="retry", 
        child=BtNode_MoveArmSingle(
            name="Move arm to drop position", 
            service_name=arm_service_name, 
            arm_pose_bb_key=KEY_ARM_DROP_BAG_POSE,
            add_octomap=False
        ), 
        num_failures=3
    ))

    root.add_child(BtNode_GripperAction(name="Open gripper to drop", open_gripper=True))
    root.add_child(py_trees.timers.Timer(name="Wait for bag drop", duration=2.0))
    # root.add_child(py_trees.decorators.Retry(
    #     name="retry", 
    #     child=BtNode_MoveArmSingle(
    #         name="Move arm to navigation pose after drop", 
    #         service_name=arm_service_name, 
    #         arm_pose_bb_key=KEY_ARM_NAVIGATING, 
    #         add_octomap=False
    #     ), 
    #     num_failures=3
    # ))
    return root

def createReceptionist():
    root = py_trees.composites.Sequence(name="Receptionist Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    # announce start and scan host features
    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting receptionist."))
    root.add_child(createEnterArena())
    root.add_child(createScanHostFeatures())

    ############## greeting 1st guest ######################
    
    # go to door to greet first guest
    root.add_child(BtNode_Announce(name="announce going to greet 1st guest", bb_source=None, message="Greeting guest"))
    root.add_child(createGreetGuest())

    # go to sofa now
    root.add_child(createToSofa())
    root.add_child(createRecommendSeat())

    ############ first guest completed, now for second guest ###########

    root.add_child(BtNode_Announce(name="announce going to greet 2nd guest", bb_source=None, message="Greeting guest"))   
    root.add_child(createGreetGuest())
    root.add_child(createGraspBag())

    # go to sofa now
    root.add_child(createToSofa())
    root.add_child(createAnnounceAndScanSofa())
    # root.add_child(createFirstIntroductionsSimple())
    root.add_child(createSecondIntroductionsSimple())
    root.add_child(createFollowPerson())
    root.add_child(createDropBag())

    root.add_child(BtNode_Announce(name="Task accomplished", bb_source=None, message="HRI task accomplished."))
    root.add_child(py_trees.behaviours.Running(name="end"))

    return root

def main():
    rclpy.init(args=None)

    root = createReceptionist()
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)

    # make it a ros tree
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="root_node", timeout=15)

    # function for display the tree to standard output
    def print_tree(tree):
        print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
        if PRINT_BLACKBOARD:
            print(py_trees.display.unicode_blackboard())

    if PRINT_DEBUG:
        py_trees.logging.level = py_trees.logging.Level.DEBUG
    
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()