import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureExtraction, BtNode_SeatRecommend

from .customNodes import BtNode_CombinePerson, BtNode_Introduce

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

import random


pose_door = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715)))
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.9390, y=14.5222, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.673597350035888, w=0.7390985117185863)))


host_name = "host John"
host_drink = "pepsi"

names = ["Alex", "Joe", "Cassandra", "Steven", "Ryan"]
drinks = ["tea", "coffee", "Mountain Dew", "Cola", "Hot chocolate"]

KEY_DOOR_POSE = "door_pose"
KEY_SOFA_POSE = "sofa_pose"

KEY_HOST_NAME = "host_name"
KEY_HOST_DRINK = "host_drink"
KEY_HOST_FEATURES = "host_features"

KEY_GUEST_NAME = "guest_name"
KEY_GUEST_DRINK = "guest_drink"
KEY_GUEST_FEATURES = "guest_features"

KEY_PERSONS = "persons"

KEY_SEAT_RECOMMENDATION = "seat_recommendation"

def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    # root.add_child(BtNode_WriteToBlackboard(name="Wirte door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    # root.add_child(BtNode_WriteToBlackboard(name="Wirte sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Wirte door location", bb_namespace="", bb_source=None, bb_key=KEY_DOOR_POSE, object=pose_door))
    root.add_child(BtNode_WriteToBlackboard(name="Wirte sofa location", bb_namespace="", bb_source=None, bb_key=KEY_SOFA_POSE, object=pose_sofa))
    root.add_child(BtNode_WriteToBlackboard(name="Wirte host name", bb_namespace="", bb_source=None, bb_key=KEY_HOST_NAME, object=host_name))
    root.add_child(BtNode_WriteToBlackboard(name="Wirte host drink", bb_namespace="", bb_source=None, bb_key=KEY_HOST_DRINK, object=host_drink))
    root.add_child(BtNode_WriteToBlackboard(name="Initialize persons", bb_namespace="", bb_source=None, bb_key=KEY_PERSONS, object=[]))

    return root

def createGetNameAndDrink():
    root = py_trees.composites.Sequence(name="Get correct name and drink", memory=True)

    get_name = py_trees.composites.Sequence(name="Get name", memory=True)
    get_name.add_child(BtNode_Announce(name="Initial greeting", bb_source="", message="Hello"))
    loop_name = py_trees.composites.Sequence(name="get and confirm name", memory=True)
    # TODO: add Audio node for listening and extracting name from user response
    loop_name.add_child(BtNode_Announce(name="Prompt for name", bb_source="", message="Please stand clode to my microphone and tell me your name"))
    name = names[random.randint(0, len(names)-1)]
    loop_name.add_child(BtNode_Announce(name="Getting name", bb_source="", message=f"Missing module for speech recognition, assuming name of {name}"))
    loop_name.add_child(py_trees.behaviours.SetBlackboardVariable(name="write name to blackboard", variable_name=KEY_GUEST_NAME, variable_value=name, overwrite=True))
    loop_name.add_child(BtNode_Announce(name="Confirm name", bb_source="", message="Missing module for name confirmation, assuming confirmed"))
    get_name.add_child(py_trees.decorators.Retry(name="retry", child=loop_name, num_failures=10))
    root.add_child(get_name)
    
    get_drink = py_trees.composites.Sequence(name="Get name", memory=True)
    loop_drink = py_trees.composites.Sequence(name="get and confirm drink", memory=True)
    # TODO: add Audio node for listening and extracting name from user response
    loop_drink.add_child(BtNode_Announce(name="Prompt for drink", bb_source="", message="Please stand clode to my microphone and tell me your favorite drink"))
    drink = drinks[random.randint(0, len(drinks)-1)]
    loop_drink.add_child(BtNode_Announce(name="Getting drink", bb_source="", message=f"Missing module for speech recognition, assuming favorite drink of {drink}"))
    loop_drink.add_child(py_trees.behaviours.SetBlackboardVariable(name="write drink to blackboard", variable_name=KEY_GUEST_DRINK, variable_value=drink, overwrite=True))
    loop_drink.add_child(BtNode_Announce(name="Confirm drink", bb_source="", message="Missing module for drink confirmation, assuming confirmed"))
    get_drink.add_child(py_trees.decorators.Retry(name="retry", child=loop_drink, num_failures=10))
    root.add_child(get_drink)

    return root

def createRegisterFeature():
    root = py_trees.composites.Sequence(name="Register features of person in front", memory=True)

    root.add_child(BtNode_Announce(name="Ask to stand in front", bb_source=None, message="Please stand in front of me so I can remember you"))
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_GUEST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine person's info", key_dest=KEY_PERSONS, key_name=KEY_GUEST_NAME, key_drink=KEY_GUEST_DRINK, key_features=KEY_GUEST_FEATURES))

    root.add_child(BtNode_Announce(name="Indicate follow", bb_source=None, message="Please follow me to the living room"))

    return root

def createReceptionist():
    root = py_trees.composites.Sequence(name="Receptionist Root", memory=True)
    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Starting, going to scan host features"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to sofa", KEY_SOFA_POSE), num_failures=10))    
    root.add_child(BtNode_FeatureExtraction(name="extract features", bb_dest_key=KEY_HOST_FEATURES))
    root.add_child(BtNode_CombinePerson(name="combine host's info", key_dest=KEY_PERSONS, key_name=KEY_HOST_NAME, key_drink=KEY_HOST_DRINK, key_features=KEY_HOST_FEATURES))

    # go to door to greet guest
    root.add_child(BtNode_Announce(name="announce intention", bb_source=None, message="Going to greet first guest"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to door", KEY_DOOR_POSE), num_failures=10))
    root.add_child(createGetNameAndDrink())
    root.add_child(createRegisterFeature())

    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to living room", KEY_SOFA_POSE), num_failures=10))

    first_introductions = py_trees.composites.Sequence(name="First introductions", memory=True)
    first_introductions.add_child(BtNode_Introduce(name="introduce host to guest", key_person=KEY_PERSONS, target_id=1, introduced_id=0))
    first_introductions.add_child(BtNode_Introduce(name="introduce guest to host", key_person=KEY_PERSONS, target_id=0, introduced_id=1))
    find_seat_recommendation = BtNode_SeatRecommend(name="Get seat recommendation", bb_dest_key=KEY_SEAT_RECOMMENDATION, bb_source_key=KEY_PERSONS)
    root.add_child(py_trees.composites.Parallel(name="Get recommendation", 
                                                policy=py_trees.common.ParallelPolicy.SuccessOnAll(), children=[first_introductions, find_seat_recommendation]))
    root.add_child(BtNode_Announce(name="announce seat recommendation", bb_source=KEY_SEAT_RECOMMENDATION))

    # TODO: add the second part of receptionist in here

    return root


