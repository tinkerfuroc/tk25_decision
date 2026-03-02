import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard, BtNode_WaitTicks
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation, BtNode_Listen
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection, BtNode_TurnPanTilt

from .custom_nodes import BtNode_ScanForWavingPerson, BtNode_ScanForWavingPersonNew
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import py_trees_ros
import rclpy
from behavior_tree.visualization import create_post_tick_visualizer

import json
import math

USE_NEW_SCAN_WAVING = True
TRY_GOTO_GRASP_POSE = True
if TRY_GOTO_GRASP_POSE:
    from behavior_tree.TemplateNodes.Navigation import BtNode_ConvertGraspPose
    KEY_POSE_GRASP_POSE = "grasp_pose"
DEBUG_GOTO = True
THRESHOLD_METERS = 6.0

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

egpsr_poses = {}
for key, value in constants["egpsr_rooms"].items():
    egpsr_poses["pose_" + key] = parsePoseStamped(value)

pose_command = parsePoseStamped(constants["pose_command"])

KEY_POSE_COMMAND = "pose_command"

KEY_POSE_WAVING_PERSON = "waving_person"

KEY_DOOR_STATUS = "door_status"
KEY_ALL_WAVING_PERSONS = "all_waving_persons"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
point_target_frame = "base_link"

def createEnterArena():
    root = py_trees.composites.Sequence(name="Enter arena", memory=True)
    
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))
    root.add_child(BtNode_Announce(name="announce door opened", bb_source=None, message="Door is open."))
    root.add_child(BtNode_WaitTicks("wait for 3 second", 12)) # 2 seconds
    parallel_enter_arena = py_trees.composites.Parallel("Enter arena", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_enter_arena.add_child(BtNode_Announce(name="Announce entering arena", bb_source=None, message="Entering arena"))
    parallel_enter_arena.add_child(BtNode_TurnPanTilt(name="Turn pan tile", x=0.0, y=20.0, speed=0.0))
    parallel_enter_arena.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="Go to table", key=KEY_POSE_COMMAND), num_failures=5))
    root.add_child(parallel_enter_arena)
    return root

def createConstantWriter():
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Pose Command", bb_namespace="", bb_source=None, bb_key=KEY_POSE_COMMAND, object=pose_command))
    
    # 把要检查的pose都写在里面
    for key, value in egpsr_poses.items():
        root.add_child(BtNode_WriteToBlackboard(f"Write {key}", bb_namespace="", bb_source=None, bb_key=key, object=value))

    return root

def createGotoWaving():
    
    root = py_trees.composites.Sequence("root of goto_waving", True)
    root.add_child(BtNode_TurnPanTilt("Turn pan tilt", x=0.0, y=45.0, speed=0.0))
    root.add_child(BtNode_Announce("announce found waving person", None, message="I'm searching for waving person. Keep your hands above your head."))
    root.add_child(BtNode_WaitTicks("wait for 1.5 second", 6))
    root.add_child(BtNode_Announce("announce found waving person", None, message="I'm searching for waving person. Keep your hands above your head."))
    root.add_child(BtNode_WaitTicks("wait for 1.5 second", 6))
    root.add_child(BtNode_Announce("announce found waving person", None, message="I'm searching for waving person. Keep your hands above your head."))
    root.add_child(BtNode_WaitTicks("wait for 1.5 second", 6))


    if USE_NEW_SCAN_WAVING:
        root.add_child(py_trees.decorators.Retry("retry", BtNode_ScanForWavingPersonNew("find waving persons", KEY_ALL_WAVING_PERSONS, KEY_POSE_WAVING_PERSON, THRESHOLD_METERS, target_frame="map"), 5))
    else:
        root.add_child(BtNode_ScanForWavingPerson("find waving person", KEY_POSE_WAVING_PERSON, detect_wavinguse_orbbec=True, target_frame="base_link"))
    root.add_child(BtNode_Announce("announce found waving person", None, message="Found waving person. Dear person, could you move a little so I can reach you?"))
    if TRY_GOTO_GRASP_POSE:
        root.add_child(BtNode_ConvertGraspPose("convert grasp pose", KEY_POSE_WAVING_PERSON, KEY_POSE_GRASP_POSE))
        root.add_child(py_trees.decorators.Retry("retry grasp_pose", BtNode_GotoAction("goto instruction point", KEY_POSE_GRASP_POSE), 5))
    else:
        root.add_child(py_trees.decorators.Retry("retry", BtNode_GotoAction("goto instruction point", KEY_POSE_WAVING_PERSON), 5))
    return root

def createGetCommand(key_dest, confirmed_message="Starting execution"):
    root = py_trees.composites.Sequence("get one command from user", True)
    get_command = py_trees.composites.Sequence(name=f"get and confirm {type}", memory=True)
    get_command.add_child(BtNode_Announce(name=f"Prompt for getting command", bb_source=None, message=f"Please speak to me after the beep sound. Tell me your command."))
    get_command.add_child(BtNode_WaitTicks("wait for 1.5 second", 6))
    get_command.add_child(BtNode_Listen(name="Listen to guest", bb_dest_key=key_dest, timeout=10.0))
    get_command.add_child(BtNode_Announce(name=f"ask to confirm command", bb_source=key_dest, message=f"Am I correct, you command is "))
    get_command.add_child(BtNode_WaitTicks("wait for 2.5 second", 10))
    get_command.add_child(BtNode_GetConfirmation("confirm instruction"))
    get_command.add_child(BtNode_Announce(name="announce confirmed", bb_source=None, message=confirmed_message))

    root.add_child(py_trees.decorators.Retry("retry", get_command, 100))
    return root

def createGoAndFindPerson(key_pose, do_go=True):
    root = py_trees.composites.Sequence("goto and find waving person to get command", True)
    if do_go:
        root.add_child(py_trees.decorators.Retry("retry go_and_find", BtNode_GotoAction("Go to pose", key_pose), 5))
    root.add_child(createGotoWaving())
    root.add_child(createGetCommand("hello", confirmed_message="Command received, I will execute it later."))
    return root

def createEGPSR():
    root = py_trees.composites.Sequence("GPSR", True)
    root.add_child(createConstantWriter())
    root.add_child(BtNode_TurnPanTilt("Turn pan tilt", x=0.0, y=45.0, speed=0.0))
    root.add_child(createEnterArena())

    # scan each room
    for key in egpsr_poses:
        root.add_child(py_trees.decorators.FailureIsSuccess("F is s", createGoAndFindPerson(key, DEBUG_GOTO)))

    return root

def main():
    rclpy.init()
    # Setup blackboard values

    root = py_trees.composites.Sequence("Root", True)
    egpsr = createEGPSR()
    root.add_children([
        egpsr,
        BtNode_Announce("announce finished", bb_source=None, message="EGPSR accomplished"),
        py_trees.behaviours.Running("running")
    ])

    print("=== Running Behavior Tree ===")
    # Wrap the tree in a ROS-friendly interface
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
    )

    # Setup and spin
    tree.setup(timeout=15, node_name="root_node")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="EGPSR",
    )
    tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)

    try:
        rclpy.spin(tree.node)
    finally:
        shutdown_visualizer()
        rclpy.shutdown()
