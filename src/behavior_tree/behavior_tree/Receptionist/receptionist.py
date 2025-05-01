import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_SeatRecommend, BtNode_FeatureMatching, BtNode_TurnPanTilt, BtNode_DoorDetection
from behavior_tree.TemplateNodes.Manipulation import BtNode_PointTo

from .customNodes import BtNode_CombinePerson, BtNode_Introduce, BtNode_Confirm

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random

POINT_TO_PERSON = False
TURN_PAN_TILT = True

# pose_door = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
#                         pose=Pose(position=Point(x=-0.023241120846270065, y=2.5612594602417316, z=0.0), 
#                                   orientation=Quaternion(x=0.0, y=0.0, z=0.719766525996756, w=0.6942162113164466))
#                                   )
pose_door = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=1.974, y=0.292, z=0.0), 
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.980803242, w=-0.195))
                                  )
# pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                         pose=Pose(position=Point(x=0.34294540817019464, y=1.2331769456468695, z=0.0), 
#                                   orientation=Quaternion(x=0.0, y=0.0, z=-0.6351394196542072, w=0.7723975126845741))
#                                   )
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=4.692, y=1.531, z=0.0), 
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.888436267, w=0.459))
                                  )
pose_door_turned = pose_door
pose_sofa_turned = pose_sofa
# pose_door_turned = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0), 
#                                          orientation=Quaternion(x=0.0, y=0.0, z=-0.673597350035888, w=0.7390985117185863))
#                                  )
# pose_sofa_turned = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                pose=Pose(position=Point(x=4.9390, y=14.5222, z=0.0),
#                                          orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                                 )

host_name = "host John"
host_drink = "pepsi"

names = ["Alex", "Joe", "Cassandra", "Steven", "Ryan"]
drinks = ["tea", "coffee", "Mountain Dew", "Cola", "Hot chocolate"]

MAX_SCAN_DISTANCE = 2.0

KEY_DOOR_POSE = "door_pose"
KEY_SOFA_POSE = "sofa_pose"
KEY_DOOR_POSE_TURNED = "door_pose_turned"
KEY_SOFA_POSE_TURNED = "sofa_pose_turned"

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

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter arena", memory=True)
    
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))
    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering arena"))
    # parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", service_name="move_base", target_pose=POS_TABLE, target_frame=point_target_frame)))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_SOFA_POSE), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    # root.add_child(BtNode_WriteToBlackboard(name="Wirte door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    # root.add_child(BtNode_WriteToBlackboard(name="Wirte sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Write door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    root.add_child(BtNode_WriteToBlackboard(name="Write sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Write door turned location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE_TURNED, object=pose_door_turned))
    root.add_child(BtNode_WriteToBlackboard(name="Write sofa turned location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE_TURNED, object=pose_sofa_turned))
    root.add_child(BtNode_WriteToBlackboard(name="Write host name", bb_namespace="", bb_source=None, bb_key=KEY_HOST_NAME, object=host_name))
    root.add_child(BtNode_WriteToBlackboard(name="Write host drink", bb_namespace="", bb_source=None, bb_key=KEY_HOST_DRINK, object=host_drink))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_PERSONS, object=[]))

    return root

def createListenToGuest(bb_dest_key:str, word_list: list[str]):
    root = py_trees.composites.Selector(name="Listen to guest", memory=True)
    root.add_child(BtNode_PhraseExtraction(name="Listen to guest", bb_dest_key=bb_dest_key, wordlist=word_list, timeout=7.0))
    root.add_child(py_trees.decorators.SuccessIsFailure(name="success is failure", child=BtNode_Announce(name="Listen Failed, ask for repeat", bb_source="", message="I'm sorry. Could you please repeat that closer to my mic?")))
    return py_trees.decorators.Retry(name="retry", child=root, num_failures=10)

def createGetInfo(type:str, storage_key:str):
    root = py_trees.composites.Sequence(name=f"Get {type}", memory=True)
    loop = py_trees.composites.Sequence(name=f"get and confirm {type}", memory=True)
    loop.add_child(BtNode_Announce(name=f"Prompt for {type}", bb_source="", message=f"Please speak loudly into my microphone and tell me your {type}"))
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

    root.add_child(BtNode_Announce(name="Ask to stand in front", bb_source=None, message="Please one meter in front of me so I can remember you."))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_GUEST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine person's info", key_dest=KEY_PERSONS, key_name=KEY_GUEST_NAME, key_drink=KEY_GUEST_DRINK, key_features=KEY_GUEST_FEATURES))

    root.add_child(BtNode_Announce(name="Indicate follow", bb_source=None, message="Please follow me"))

    return root

def createFirstIntroductions():
    first_introductions = py_trees.composites.Sequence(name="First introductions", memory=True)
    first_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the left", x=-90.0, y=45.0, speed=0.0))
    first_introductions.add_child(BtNode_Introduce(name="introduce host to guest", key_person=KEY_PERSONS, target_id=1, introduced_id=0))
    first_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the front", x=0.0, y=20.0, speed=0.0))
    first_introductions.add_child(BtNode_Introduce(name="introduce guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=1))
    return first_introductions

def createSecondIntroductions():
    second_introductions = py_trees.composites.Sequence(name="Second introductions", memory=True)
    # introduce second guest to host
    second_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the left", x=-90.0, y=45.0, speed=0.0))
    second_introductions.add_child(BtNode_Introduce(name="introduce host to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=0))
    second_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the front", x=0.0, y=20.0, speed=0.0))
    second_introductions.add_child(BtNode_Introduce(name="introduce second guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=2))
    # introduce second guest to first guest
    second_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the left", x=-90.0, y=45.0, speed=0.0))
    second_introductions.add_child(BtNode_Introduce(name="introduce first guest to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=1, describe_introduced=True))
    second_introductions.add_child(BtNode_TurnPanTilt(name="Turn head to the front", x=0.0, y=20.0, speed=0.0))
    second_introductions.add_child(BtNode_Introduce(name="introduce second guest to first guest", key_person=KEY_PERSONS, target_id=1, introduced_id=2))
    return second_introductions

def createToDoor():
    root = py_trees.composites.Sequence(name="Go to door", memory=True)
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to turn position", KEY_SOFA_POSE_TURNED), num_failures=10))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    return root

def createToSofa():
    root = py_trees.composites.Sequence(name="Go to sofa", memory=True)
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to turn position", KEY_DOOR_POSE_TURNED), num_failures=10))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))
    return root

def createAnnounceAndScanSofa():
    root = py_trees.composites.Parallel(name="Announce while feature matching", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    # Turn head down a bit for better feature matching
    root.add_child(BtNode_TurnPanTilt(name="Turn head down", x=0.0, y=20.0, speed=0.0))
    # parallel_matching = py_trees.composites.Parallel(name="Feature matching", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    # parallel_matching.add_child(BtNode_FeatureMatching(name="Feature matching", bb_dest_key=KEY_PERSON_CENTROIDS, bb_persons_key=KEY_PERSONS, max_distance=MAX_SCAN_DISTANCE))
    # parallel_matching.add_child(BtNode_Announce(name="Announce feature matching", bb_source=None, message="Scanning seated personnels"))
    # root.add_child(parallel_matching)
    # TODO: add turn pan tilt to face guest
    root.add_child(BtNode_Announce(name="Tell guest to stand on left", bb_source=None, message="Please stand on my left side"))
    # TODO: add point to guest being introduced
    # root.add_child(BtNode_PointTo(name="Point to guest", bb_source=KEY_PERSONS, target_id=1, point_to_person=POINT_TO_PERSON))
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
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))    
    root.add_child(BtNode_Announce(name="announce scanning host features", bb_source=None, message="Scanning host features"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_HOST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine host's info", key_dest=KEY_PERSONS, key_name=KEY_HOST_NAME, key_drink=KEY_HOST_DRINK, key_features=KEY_HOST_FEATURES))
    root.add_child(BtNode_TurnPanTilt(name="Turn head up", x=0.0, y=45.0, speed=0.0))
    return root

def createReceptionist():
    root = py_trees.composites.Sequence(name="Receptionist Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    # announce start and scan host features
    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting receptionist"))
    root.add_child(createEnterArena())
    root.add_child(createScanHostFeatures())

    # go to door to greet first guest
    root.add_child(BtNode_Announce(name="announce going to greet 1st guest", bb_source=None, message="Going to greet first guest"))
    root.add_child(createGreetGuest())

    # go to living room for introductions
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to living room", KEY_SOFA_POSE), num_failures=10))
    root.add_child(createToSofa())
    root.add_child(createAnnounceAndScanSofa())

    # introduce first guest and host to each other, then recommend a seat
    first_introductions = createFirstIntroductions()
    # TODO: add turn pan tilt back
    find_seat_recommendation1 = BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS)
    root.add_child(py_trees.composites.Parallel(name="Get recommendation 1", 
                                                policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[first_introductions, find_seat_recommendation1]))
    root.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))

    # go to door to greet second guest
    root.add_child(BtNode_Announce(name="announce going to greet 2nd guest", bb_source=None, message="Going to greet second guest"))
    root.add_child(createGreetGuest())

    # go to living room for introductions
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to living room", KEY_SOFA_POSE), num_failures=10))
    root.add_child(createToSofa())
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

    return root


