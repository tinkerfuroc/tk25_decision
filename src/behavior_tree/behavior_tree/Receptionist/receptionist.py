import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction, BtNode_GetConfirmation, BtNode_Listen, BtNode_CompareInterest
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_SeatRecommend, BtNode_FeatureMatching, BtNode_TurnPanTilt, BtNode_DoorDetection, BtNode_TurnTo
from behavior_tree.TemplateNodes.Manipulation import BtNode_PointTo, BtNode_MoveArmSingle

from .customNodes import BtNode_CombinePerson, BtNode_Introduce, BtNode_Confirm, BtNode_HeadTracking

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random
import math
import json

# POINT_TO_PERSON = False
TURN_PAN_TILT = True

MAX_SCAN_DISTANCE = 4.5

DEBUG_NO_GOTO = False

DISABLE_FEATURE_MATCH = False
DISABLE_FOLLOW_HEAD = False

# read from `constant.json` in the same directory
# load file
try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/Receptionist/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

pose_door = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_door"]["point"]["x"], y=constants["pose_door"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_door"]["orientation"]["x"], 
                                                            y=constants["pose_door"]["orientation"]["y"], 
                                                            z=constants["pose_door"]["orientation"]["z"], 
                                                            w=constants["pose_door"]["orientation"]["w"]))
                            )
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_sofa"]["point"]["x"], y=constants["pose_sofa"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_sofa"]["orientation"]["x"], 
                                                            y=constants["pose_sofa"]["orientation"]["y"], 
                                                            z=constants["pose_sofa"]["orientation"]["z"], 
                                                            w=constants["pose_sofa"]["orientation"]["w"]))
                            )
pose_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_table"]["point"]["x"], y=constants["pose_table"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_table"]["orientation"]["x"], 
                                                            y=constants["pose_table"]["orientation"]["y"], 
                                                            z=constants["pose_table"]["orientation"]["z"], 
                                                            w=constants["pose_table"]["orientation"]["w"]))
                            )
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in constants["arm_pos_navigating"]]
ARM_POS_POINT_TO = [x / 180 * math.pi for x in constants["arm_pos_point_to"]]

host_name = constants["host_name"]
host_drink = constants["host_drink"]
drinks = constants["drinks"]
names = constants["names"]

KEY_ARM_INIT_POSE = "arm_init_pose"
KEY_ARM_NAVIGATING = "arm_navigating"

KEY_DOOR_POSE = "door_pose"
KEY_SOFA_POSE = "sofa_pose"
KEY_DOOR_POSE_TURNED = "door_pose_turned"
KEY_SOFA_POSE_TURNED = "sofa_pose_turned"

KEY_TABLE_POSE = "table_pose"

KEY_HOST_NAME = "host_name"
KEY_HOST_DRINK = "host_drink"
KEY_HOST_FEATURES = "host_features"

KEY_GUEST_NAME = "guest_name"
KEY_GUEST_DRINK = "guest_drink"
KEY_GUEST_FEATURES = "guest_features"
KEY_GUEST1_INTEREST = "guest1_interest"
KEY_GUEST2_INTEREST = "guest2_interest"

KEY_PERSONS = "persons"
KEY_PERSON_CENTROIDS = "centroids"

KEY_SEAT_RECOMMENDATION = "seat_recommendation"

KEY_DOOR_STATUS = "door_status"

arm_service_name = "arm_joint_service"

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter", memory=True)
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    
    if not DEBUG_NO_GOTO:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))

    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering"))
    # parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", service_name="move_base", target_pose=POS_TABLE, target_frame=point_target_frame)))
    
    # root.add_child(BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False))
    if not DEBUG_NO_GOTO:
        parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_SOFA_POSE), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Wirte door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    root.add_child(BtNode_WriteToBlackboard(name="Wirte sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Write door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    root.add_child(BtNode_WriteToBlackboard(name="Write sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Write host name", bb_namespace="", bb_source=None, bb_key=KEY_HOST_NAME, object=host_name))
    root.add_child(BtNode_WriteToBlackboard(name="Write host drink", bb_namespace="", bb_source=None, bb_key=KEY_HOST_DRINK, object=host_drink))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_PERSONS, object=[]))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_ARM_INIT_POSE, object=ARM_POS_POINT_TO))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard(name="Write table location", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_POSE, object=pose_table))

    return root

def createListenToGuest(bb_dest_key:str, word_list: list[str]):
    root = py_trees.composites.Selector(name="Listen to guest", memory=True)
    root.add_child(BtNode_PhraseExtraction(name="Listen to guest", bb_dest_key=bb_dest_key, wordlist=word_list, timeout=7.0))
    root.add_child(py_trees.decorators.SuccessIsFailure(name="success is failure", child=BtNode_Announce(name="Listen Failed, ask for repeat", bb_source="", message="I'm sorry. Could you please repeat that louder and closer?")))
    return py_trees.decorators.Retry(name="retry", child=root, num_failures=10)

def createGetInfo(type:str, storage_key:str):
    root = py_trees.composites.Sequence(name=f"Get {type}", memory=True)
    loop = py_trees.composites.Sequence(name=f"get and confirm {type}", memory=True)
    loop.add_child(BtNode_Announce(name=f"Prompt for {type}", bb_source="", message=f"Speak loudly and tell me your {type}"))
    loop.add_child(createListenToGuest(bb_dest_key=storage_key, word_list=names if type == "name" else drinks))
    loop.add_child(BtNode_Confirm(name=f"Confirm {type} prompt", key_confirmed=storage_key, type=type))
    loop.add_child(BtNode_GetConfirmation(name=f"Get {type} confirmation", timeout=5.0))
    root.add_child(py_trees.decorators.Retry(name="retry", child=loop, num_failures=10))
    return root

def createGetNameAndDrink():
    root = py_trees.composites.Sequence(name="Get correct name and drink", memory=True)
    root.add_child(createGetInfo("name", KEY_GUEST_NAME))
    root.add_child(createGetInfo("favorite drink", KEY_GUEST_DRINK))
    return root

def createRegisterFeature():
    root = py_trees.composites.Sequence(name="Register features of person in front", memory=True)

    root.add_child(BtNode_Announce(name="Ask to stand in front", bb_source=None, message="Stand one meter in front of me. Thank you"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_GUEST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine person's info", key_dest=KEY_PERSONS, key_name=KEY_GUEST_NAME, key_drink=KEY_GUEST_DRINK, key_features=KEY_GUEST_FEATURES))

    root.add_child(BtNode_Announce(name="Indicate follow", bb_source=None, message="Follow me"))

    return root

def createFindFavoriteDrink(bb_key_fav_drink : str):
    root = py_trees.composites.Sequence(name="Find favorite drink", memory=True)
    # Go to table after greeting first guest
    if not DEBUG_NO_GOTO:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_TABLE_POSE), num_failures=10))
    parallel_announce_scan = py_trees.composites.Parallel(name="Announce and scan", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_announce_scan.add_child(BtNode_Announce(name="Announce scanning at table", bb_source=None, message="Scanning at the table."))

    # TODO: Add scanning node here
    root.add_child(BtNode_Announce(name="Missing find drink module", bb_source=None, message="Missing find drink module"))

    root.add_child(parallel_announce_scan)
    # TODO: Add announce found drink here

    return root

def createFirstIntroductions():
    first_introductions = py_trees.composites.Sequence(name="First introductions", memory=True)
    first_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the right", x=90.0, y=45.0, speed=0.0))
    
    # introduce host to first guest
    # follow guest1 head
    if not DISABLE_FOLLOW_HEAD:
        head_tracking = py_trees.decorators.Repeat(name="repeat head tracking", child=py_trees.decorators.FailureIsSuccess("f is s", BtNode_HeadTracking(name="Follow guest1 head", service_name="follow_head_service")), num_success = -1)
    if not DISABLE_FEATURE_MATCH:
        # point to guest1
        # first_introductions.add_child(BtNode_PointTo(name="Point to host", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=0))
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_PointTo(name="Point to host", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=0), num_failures=3)
        point_to = py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco)
    
    introduce = BtNode_Introduce(name="introduce host to guest", key_person=KEY_PERSONS, target_id=1, introduced_id=0)
    
    turn_head_arm = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([introduce]), children=[head_tracking, point_to, introduce])
    first_introductions.add_child(turn_head_arm)

    turn_head_arm2 = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    # introduce first guest to host
    introduce_sequence2 = py_trees.composites.Sequence(name="sequence", memory=True)
    # introduce_sequence2.add_child(BtNode_TurnPanTilt(name="Turn head to the front", x=0.0, y=20.0, speed=0.0))
    introduce_sequence2.add_child(BtNode_TurnTo(name="Turn to host", key_person=KEY_PERSONS, target_id=0))
    
    turn_head_arm2.add_child(introduce_sequence2)

    if not DISABLE_FEATURE_MATCH:
        # point to guest1
        # first_introductions.add_child(BtNode_PointTo(name="Point to guest", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=1))
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_INIT_POSE, add_octomap=False), num_failures=3)
        turn_head_arm2.add_child(py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco))
    first_introductions.add_child(turn_head_arm2)
    introduce_sequence2.add_child(BtNode_Introduce(name="introduce guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=1))
    return first_introductions

def createSecondIntroductions():
    second_introductions = py_trees.composites.Sequence(name="Second introductions", memory=True)
    second_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the right", x=90.0, y=45.0, speed=0.0))
    
    # introduce host to second guest
    # follow guest1 head
    if not DISABLE_FOLLOW_HEAD:
        head_tracking = py_trees.decorators.Repeat(name="repeat head tracking", child=py_trees.decorators.FailureIsSuccess("f is s", BtNode_HeadTracking(name="Follow guest2 head", service_name="follow_head_service")), num_success = -1)
    if not DISABLE_FEATURE_MATCH:
        # point to host
        # second_introductions.add_child(BtNode_PointTo(name="Point to host", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=0))
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_PointTo(name="Point to host", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=0), num_failures=3)
        point_to = py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco)

    introduce = BtNode_Introduce(name="introduce host to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=0)
    turn_head_arm1 = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([introduce]), children=[head_tracking, point_to, introduce])
    second_introductions.add_child(turn_head_arm1)
    
    # introduce second guest to host
    turn_head_arm2 = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    introduce_sequence2 = py_trees.composites.Sequence(name="sequence", memory=True)
    introduce_sequence2.add_child(BtNode_TurnTo(name="Turn to host", key_person=KEY_PERSONS, target_id=0))
    
    turn_head_arm2.add_child(introduce_sequence2)
    if not DISABLE_FEATURE_MATCH:
        # point to guest2
        # first_introductions.add_child(BtNode_PointTo(name="Point to guest", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=1))
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle(name="Move arm to nav, point to guest2", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_INIT_POSE, add_octomap=False), num_failures=3)
        turn_head_arm2.add_child(py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco))
    introduce_sequence2.add_child(BtNode_Introduce(name="introduce second guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=2))
    second_introductions.add_child(turn_head_arm2)

    # introduce first guest to second guest
    second_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the right", x=90.0, y=45.0, speed=0.0))
    
    if not DISABLE_FOLLOW_HEAD:
        head_tracking = py_trees.decorators.Repeat(name="repeat head tracking", child=py_trees.decorators.FailureIsSuccess("f is s", BtNode_HeadTracking(name="Follow guest2 head", service_name="follow_head_service")), num_success = -1)
    if not DISABLE_FEATURE_MATCH:
        # point to guest1
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_PointTo(name="Point to guest1", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=1), num_failures=3)
        point_to = py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco)
    
    introduce = BtNode_Introduce(name="introduce first guest to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=1, describe_introduced=True)
    turn_head_arm3 = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnSelected([introduce]), children=[head_tracking, point_to, introduce])
    second_introductions.add_child(turn_head_arm3)
    
    # introduce second guest to first guest
    turn_head_arm4 = py_trees.composites.Parallel(name="Turn head and arm", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    introduce_sequence4 = py_trees.composites.Sequence(name="sequence", memory=True)
    introduce_sequence4.add_child(BtNode_TurnTo(name="Turn to guest1", key_person=KEY_PERSONS, target_id=1))
    
    turn_head_arm4.add_child(introduce_sequence4)
    if not DISABLE_FEATURE_MATCH:
        # point to guest
        # first_introductions.add_child(BtNode_PointTo(name="Point to guest", service_name=arm_service_name, bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_init_pose=KEY_ARM_INIT_POSE, target_id=1))
        deco = py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_INIT_POSE, add_octomap=False), num_failures=3)
        turn_head_arm4.add_child(py_trees.decorators.FailureIsSuccess(name="failure is success", child=deco))
    second_introductions.add_child(turn_head_arm4)
    introduce_sequence4.add_child(BtNode_Introduce(name="introduce second guest to first guest", key_person=KEY_PERSONS, target_id=1, introduced_id=2))

    return second_introductions

def createToDoor():
    root = py_trees.composites.Sequence(name="Go to door", memory=True)
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    if not DEBUG_NO_GOTO:
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to turn position", KEY_SOFA_POSE_TURNED), num_failures=10))
        root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    return root

def createToSofa(interest_key : str):
    root = py_trees.composites.Parallel(name="Go to sofa while chatting", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    navigation_seq = py_trees.composites.Sequence(name="Go to sofa", memory=True)
    navigation_seq.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    navigation_seq.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))
    if not DEBUG_NO_GOTO:
        navigation_seq.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to turn position", KEY_DOOR_POSE_TURNED), num_failures=10))
        navigation_seq.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))
    
    get_interest_seq = py_trees.composites.Sequence(name="Get interest", memory=True)
    get_interest_seq.add_child(BtNode_Announce(name="Ask for interest", bb_source=None, message="What are you interested in?"))
    get_interest_seq.add_child(BtNode_Listen(name="Listen to guest", bb_dest_key=interest_key, timeout=5.0))
    get_interest_seq.add_child(BtNode_Announce(name="Repeat interest", bb_source=interest_key))

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
    # TODO: add turn pan tilt to face guest
    root.add_child(BtNode_Announce(name="Tell guest to stand on right", bb_source=None, message="Stand on my right"))
    # TODO: add point to guest being introduced
    # if not DISABLE_FEATURE_MATCH:
    #     # point to guest
    #     root.add_child(BtNode_PointTo(name="Point to host", bb_key_persons=KEY_PERSONS, bb_key_points=KEY_PERSON_CENTROIDS, bb_key_description=KEY_HOST_FEATURES))
    return root

def createGreetGuest():
    root = py_trees.composites.Sequence(name="Greet guest", memory=True)
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    root.add_child(createToDoor())
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

def createReceptionist():
    root = py_trees.composites.Sequence(name="Receptionist Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    # announce start and scan host features
    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting receptionist"))
    root.add_child(createEnterArena())
    root.add_child(createScanHostFeatures())

    # go to door to greet first guest
    root.add_child(BtNode_Announce(name="announce going to greet 1st guest", bb_source=None, message="Greeting guest"))
    root.add_child(createGreetGuest())

    # go to living room for introductions
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to living room", KEY_SOFA_POSE), num_failures=10))
    root.add_child(createFindFavoriteDrink(KEY_GUEST_DRINK))
    root.add_child(createToSofa(KEY_GUEST1_INTEREST))
    root.add_child(createAnnounceAndScanSofa())

    # introduce first guest and host to each other, then recommend a seat
    first_introductions = createFirstIntroductions()
    find_seat_recommendation1 = BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS)
    root.add_child(py_trees.composites.Parallel(name="Get recommendation 1", 
                                                policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[first_introductions, find_seat_recommendation1]))
    root.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))

    # go to door to greet second guest
    root.add_child(BtNode_Announce(name="announce going to greet 2nd guest", bb_source=None, message="Greeting guest"))
    root.add_child(createGreetGuest())

    # go to living room for introductions
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to living room", KEY_SOFA_POSE), num_failures=10))
    root.add_child(createFindFavoriteDrink(KEY_GUEST_DRINK))
    root.add_child(createToSofa(KEY_GUEST2_INTEREST))
    root.add_child(createAnnounceAndScanSofa())

    # introduce second guest
    second_introductions = createSecondIntroductions()
    # TODO: add turn pan tilt back
    find_seat_recommendation2 = BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS)
    root.add_child(py_trees.composites.Parallel(name="Get recommendation 2", 
                                                policy=py_trees.common.ParallelPolicy.SuccessOnAll(), 
                                                children=[second_introductions, find_seat_recommendation2]
                                                )
                   )
    root.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))

    root.add_child(BtNode_Announce(name="Task accomplished", bb_source=None, message="Receptionist task accomplished."))
    root.add_child(py_trees.behaviours.Running(name="end"))

    root.add_child(BtNode_CompareInterest(name="Compare interest", bb_source_key1=KEY_GUEST1_INTEREST, bb_source_key2=KEY_GUEST2_INTEREST))
    return root


