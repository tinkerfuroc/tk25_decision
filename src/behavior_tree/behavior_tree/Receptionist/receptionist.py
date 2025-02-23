import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_SeatRecommend, BtNode_FeatureMatching

from .customNodes import BtNode_CombinePerson, BtNode_Introduce, BtNode_Confirm

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random


pose_door = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'), 
                        pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0), 
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                                  )
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=4.9390, y=14.5222, z=0.0), 
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.673597350035888, w=0.7390985117185863))
                                  )
pose_door_turned = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                               pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0), 
                                         orientation=Quaternion(x=0.0, y=0.0, z=-0.673597350035888, w=0.7390985117185863))
                                 )
pose_sofa_turned = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                               pose=Pose(position=Point(x=4.9390, y=14.5222, z=0.0),
                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                                )

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

def createGetInfo(type:str, storage_key:str):
    root = py_trees.composites.Sequence(name=f"Get {type}", memory=True)
    loop = py_trees.composites.Sequence(name=f"get and confirm {type}", memory=True)
    loop.add_child(BtNode_Announce(name=f"Prompt for {type}", bb_source="", message=f"Please stand clode to my microphone and tell me your {type}"))
    # TODO: add Audio node for listening and extracting name from user response
    # random filler dummy values for now
    value = names[random.randint(0, len(names)-1)] if type == "name" else drinks[random.randint(0, len(drinks)-1)]
    loop.add_child(BtNode_Announce(name=f"Getting {type}", bb_source="", message=f"Missing module for speech recognition, assuming {type} of {value}"))
    loop.add_child(py_trees.behaviours.SetBlackboardVariable(name=f"write {type} to blackboard", variable_name=storage_key, variable_value=value, overwrite=True))
    loop.add_child(BtNode_Confirm(name=f"Confirm {type} prompt", key_confirmed=storage_key, type=type))
    # TODO add confirmation module
    loop.add_child(BtNode_Announce(name=f"Confirm {type}", bb_source="", message="Missing module for name confirmation, assuming confirmed"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=loop, num_failures=10))
    return root

def createGetNameAndDrink():
    root = py_trees.composites.Sequence(name="Get correct name and drink", memory=True)
    root.add_child(createGetInfo("name", KEY_GUEST_NAME))
    root.add_child(createGetInfo("favorite drink", KEY_GUEST_DRINK))

    # get_name = py_trees.composites.Sequence(name="Get name", memory=True)
    # get_name.add_child(BtNode_Announce(name="Initial greeting", bb_source="", message="Hello"))
    # loop_name = py_trees.composites.Sequence(name="get and confirm name", memory=True)
    # # TODO: add Audio node for listening and extracting name from user response
    # loop_name.add_child(BtNode_Announce(name="Prompt for name", bb_source="", message="Please stand clode to my microphone and tell me your name"))
    # name = names[random.randint(0, len(names)-1)]
    # loop_name.add_child(BtNode_Announce(name="Getting name", bb_source="", message=f"Missing module for speech recognition, assuming name of {name}"))
    # loop_name.add_child(py_trees.behaviours.SetBlackboardVariable(name="write name to blackboard", variable_name=KEY_GUEST_NAME, variable_value=name, overwrite=True))
    # loop_name.add_child(BtNode_Announce(name="Confirm name", bb_source="", message="Missing module for name confirmation, assuming confirmed"))
    # get_name.add_child(py_trees.decorators.Retry(name="retry", child=loop_name, num_failures=10))
    # root.add_child(get_name)
    
    # get_drink = py_trees.composites.Sequence(name="Get name", memory=True)
    # loop_drink = py_trees.composites.Sequence(name="get and confirm drink", memory=True)
    # # TODO: add Audio node for listening and extracting name from user response
    # loop_drink.add_child(BtNode_Announce(name="Prompt for drink", bb_source="", message="Please stand clode to my microphone and tell me your favorite drink"))
    # drink = drinks[random.randint(0, len(drinks)-1)]
    # loop_drink.add_child(BtNode_Announce(name="Getting drink", bb_source="", message=f"Missing module for speech recognition, assuming favorite drink of {drink}"))
    # loop_drink.add_child(py_trees.behaviours.SetBlackboardVariable(name="write drink to blackboard", variable_name=KEY_GUEST_DRINK, variable_value=drink, overwrite=True))
    # loop_drink.add_child(BtNode_Announce(name="Confirm drink", bb_source="", message="Missing module for drink confirmation, assuming confirmed"))
    # get_drink.add_child(py_trees.decorators.Retry(name="retry", child=loop_drink, num_failures=10))
    # root.add_child(get_drink)

    return root

def createRegisterFeature():
    root = py_trees.composites.Sequence(name="Register features of person in front", memory=True)

    root.add_child(BtNode_Announce(name="Ask to stand in front", bb_source=None, message="Please stand in a meter in front of me so I can remember you. Thank you"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_GUEST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine person's info", key_dest=KEY_PERSONS, key_name=KEY_GUEST_NAME, key_drink=KEY_GUEST_DRINK, key_features=KEY_GUEST_FEATURES))

    root.add_child(BtNode_Announce(name="Indicate follow", bb_source=None, message="Please follow me"))

    return root

def createFirstIntroductions():
    first_introductions = py_trees.composites.Sequence(name="First introductions", memory=True)
    first_introductions.add_child(BtNode_Introduce(name="introduce host to guest", key_person=KEY_PERSONS, target_id=1, introduced_id=0))
    first_introductions.add_child(BtNode_Introduce(name="introduce guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=1))
    return first_introductions

def createSecondIntroductions():
    second_introductions = py_trees.composites.Sequence(name="Second introductions", memory=True)
    # introduce second guest to host
    second_introductions.add_child(BtNode_Introduce(name="introduce host to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=0))
    second_introductions.add_child(BtNode_Introduce(name="introduce second guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=2))
    # introduce second guest to first guest
    second_introductions.add_child(BtNode_Introduce(name="introduce first guest to second guest", key_person=KEY_PERSONS, target_id=2, introduced_id=1, describe_introduced=True))
    second_introductions.add_child(BtNode_Introduce(name="introduce second guest to first guest", key_person=KEY_PERSONS, target_id=1, introduced_id=2))
    return second_introductions

def createToDoor():
    root = py_trees.composites.Sequence(name="Go to door", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to turn position", KEY_SOFA_POSE_TURNED), num_failures=10))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    return root

def createToSofa():
    root = py_trees.composites.Sequence(name="Go to sofa", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to turn position", KEY_DOOR_POSE_TURNED), num_failures=10))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))
    return root

def createAnnounceAndScanSofa():
    root = py_trees.composites.Parallel(name="Announce while feature matching", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    root.add_child(BtNode_Announce(name="Tell guest to stand on left", bb_source=None, message="Please stand on my left side"))
    root.add_child(BtNode_FeatureMatching(name="Feature matching", bb_dest_key=KEY_PERSON_CENTROIDS, bb_persons_key=KEY_PERSONS, max_distance=MAX_SCAN_DISTANCE))
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
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))    
    root.add_child(BtNode_Announce(name="announce scanning host features", bb_source=None, message="Scanning host features"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_HOST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine host's info", key_dest=KEY_PERSONS, key_name=KEY_HOST_NAME, key_drink=KEY_HOST_DRINK, key_features=KEY_HOST_FEATURES))
    return root

def createReceptionist():
    root = py_trees.composites.Sequence(name="Receptionist Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    # announce start and scan host features
    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting, going to scan host features"))
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


