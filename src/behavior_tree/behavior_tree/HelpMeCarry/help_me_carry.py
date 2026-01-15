import py_trees
import json
import math

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard, BtNode_WaitTicks
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_GoToLuggage
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_GripperAction
from behavior_tree.StoringGroceries.customNodes import BtNode_GraspWithPose
from behavior_tree.HelpMeCarry.customNodes import BtNode_HumanFollowingAction
from .customNodes import BtNode_FindPointedLuggage
from .Track import createFollowPerson
from .Follow import (
    createFollowPersonComplete,
    createFollowPersonUntilStopped,
    BtNode_TrackPersonAction,
    BtNode_ProcessTrackPosition,
    BtNode_CheckTargetStopped,
    BB_KEY_TRACK_POSITION,
    BB_KEY_TRANSFORM_SUCCESS,
    BB_KEY_FOLLOW_GOAL,
    DEFAULT_TARGET_FRAME
)

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

try:
    file = open("/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/HelpMeCarry/constants.json", "r")
    constants = json.load(file)
    file.close()
except FileNotFoundError:
    print("ERROR: constants.json not found!")
    raise FileNotFoundError

POS_START = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=0.0, y=0.0, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                        )

# 定义机械臂姿态
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, 0.0, 0.0, 0.0, 0.0, -45.0, 0.0]]
ARM_POS_SCAN2 = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]
ARM_POS_SCAN3 = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [9999.0, -23.0, 0.0, 0.0, 0.0, -76.0, 1.0]]


KEY_POS_START = "pose_start"
KEY_POS_FINAL = "pose_final"  # 最终位置

pose_final = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=constants["pose_final"]["point"]["x"], y=constants["pose_final"]["point"]["y"], z=0.0),
                                    orientation=Quaternion(x=constants["pose_final"]["orientation"]["x"], 
                                                            y=constants["pose_final"]["orientation"]["y"], 
                                                            z=constants["pose_final"]["orientation"]["z"], 
                                                            w=constants["pose_final"]["orientation"]["w"])))

MASTER_NAME = "master"
FOLLOW_DISTANCE = 2.0  # 跟随距离


# 定义黑板键值
KEY_ARM_SCAN = "arm_scan"
KEY_ARM_SCAN2 = "arm_scan2"
KEY_ARM_SCAN3 = "arm_scan3"
KEY_ARM_NAVIGATING = "arm_navigating"

KEY_LUGGAGE_DETECTION = "luggage_detection"
KEY_PROMPT = "prompt"
KEY_GOAL = "goal"  # 用于导航的目标位置
KEY_GOAL_TMP = "goal_tmp"

KEY_ANNOUNCE_MSG = "announce_msg"
KEY_GRASP_POSE = "grasp_pose"
KEY_OBJECT = "object"

# 定义follow的目标位置键值
KEY_MASTER_NAME = "master_name"
KEY_FOLLOW_DISTANCE = "follow_distance"
KEY_FOLLOW_TARGET_GOAL = "follow_target"
KEY_MASTER_POSITION = "master_position"



# 服务名称
arm_service_name = "arm_joint_service"
grasp_service_name = "grasp"
point_target_frame = "base_link"
follow_action_name = "human_following"

def createConstantWriter():
    """初始化常量"""
    root = py_trees.composites.Sequence("Write Constants", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan 2", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN2, object=ARM_POS_SCAN2))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Scan 3", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN3, object=ARM_POS_SCAN3))
    root.add_child(BtNode_WriteToBlackboard("Write Arm Navigating", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write Prompt", bb_namespace="", bb_source=None, bb_key=KEY_PROMPT, object="green bag"))
    root.add_child(BtNode_WriteToBlackboard("Write starting point", bb_namespace="", bb_source=None, bb_key=KEY_POS_START, object=POS_START))
    root.add_child(BtNode_WriteToBlackboard("Write Master name", bb_namespace="", bb_source=None, bb_key=KEY_MASTER_NAME, object=MASTER_NAME))
    root.add_child(BtNode_WriteToBlackboard("Write Follow Distance", bb_namespace="", bb_source=None, bb_key=KEY_FOLLOW_DISTANCE, object=FOLLOW_DISTANCE))
    return root

def createFollow():
    root = py_trees.composites.Sequence(name="Follow", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_MoveArmSingle("move arm to navigating", arm_service_name, KEY_ARM_NAVIGATING), num_failures=5))
    root.add_child(BtNode_TurnPanTilt(name="Move pan tilt", x=0.0, y=45.0, speed=0.0))
    parallel_follow = py_trees.composites.Parallel("Follow", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    get_master_pos = py_trees.decorators.Retry(name="retry", child=BtNode_HumanFollowingAction(name="human follow", action_name=follow_action_name, bb_dest=KEY_MASTER_POSITION), num_failures=5)
    goto_master = py_trees.decorators.Retry("retry", BtNode_GotoAction(name="Goto action", key=KEY_MASTER_POSITION, action_name="navigate_to_pose", wait_for_server_timeout_sec=-3), 9999)
    
    parallel_follow.add_child(py_trees.decorators.Repeat("repeat", get_master_pos, 9999))
    parallel_follow.add_child(py_trees.decorators.Repeat("repeat", goto_master, 9999))
    
    root.add_child(parallel_follow)
    return root

def createFindPointedLuggage():
    """找到被指向的行李"""
    root = py_trees.composites.Sequence("Find Pointed Luggage", memory=True)

    root.add_child(BtNode_TurnPanTilt(name="Move pan tilt", x=0.0, y=20.0, speed=0.0))

    root.add_child(BtNode_MoveArmSingle("Move arm  back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to starting point", KEY_POS_START), num_failures=10))

    # 宣布开始找行李
    root.add_child(BtNode_Announce(name="Announce finding luggage", bb_source=None, message="Looking for luggage"))
    
    # 使用FindPointedLuggage节点寻找行李 - 不移动机械臂
    root.add_child(BtNode_FindPointedLuggage(
        name="Find pointed luggage",
        bb_namespace="",
        bb_key=KEY_GOAL,
        bb_key_announce_msg=KEY_ANNOUNCE_MSG
    ))
    
    # 宣布找到行李
    root.add_child(BtNode_Announce(name="Announce found luggage", bb_source=KEY_ANNOUNCE_MSG))

    root.add_child(BtNode_MoveArmSingle(name="Move arm to grasp", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))

    root.add_child(BtNode_Announce(name="Announce found luggage", bb_source=None, message="Please hand the luggage to me, thank you!"))
    root.add_child(BtNode_Announce(name="Announce found luggage", bb_source=None, message="Moving arm back"))

    root.add_child(BtNode_MoveArmSingle(name="Move arm to grasp", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=True))

    root.add_child(BtNode_TurnPanTilt(name="Move pan tilt", x=0.0, y=45.0, speed=0.0))
    
    return root

# def createNavigateToLuggage():
#     """导航到行李位置 - 整个机器人底盘移动"""
#     root = py_trees.composites.Sequence("Navigate to Luggage", memory=True)
    
#     # # 宣布开始导航
#     root.add_child(BtNode_Announce(name="Announce navigating", bb_source=None, message="Going to pick up luggage"))
    
#     # 导航到行李位置 - 整个机器人底盘移动
#     # 这里使用KEY_GOAL作为目标位置的键值
#     root.add_child(BtNode_GoToLuggage(
#         name="point frame",
#         bb_src_key=KEY_GOAL_TMP,
#         bb_target_key=KEY_GOAL
#     ))

#     root.add_child(py_trees.decorators.Retry(
#         name="Retry navigation",
#         child=BtNode_GotoAction(name="Go to luggage", key=KEY_GOAL),
#         # child=BtNode_GoToLuggage(name="Go to luggage", bb_key=KEY_GOAL),
#         num_failures=3
#     ))
    
#     # # 宣布到达目标位置
#     # root.add_child(BtNode_Announce(name="Announce arrived", bb_source=None, message="I have arrived at the luggage location"))
    
#     return root

def createGraspLuggage(arm_pose_key):
    """到达位置后再次查找行李 - 使用FindObj而不是ScanFor"""
    root = py_trees.composites.Sequence("Find Luggage at Location", memory=True)
        
    # 先移动机械臂到扫描位置
    move_arm_parallel = py_trees.composites.Parallel("Move Arm for Scanning", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    move_arm_parallel.add_child(BtNode_Announce(name="Announce moving arm", bb_source=None, message="Moving my arm to better see the luggage"))
    move_arm_parallel.add_child(BtNode_MoveArmSingle("Move arm to scan position", service_name=arm_service_name, arm_pose_bb_key=arm_pose_key, add_octomap=True))
    root.add_child(move_arm_parallel)

    # root.add_child(BtNode_FindObj(
    #     name="Find luggage",
    #     bb_source=KEY_PROMPT,  # 从这里获取物体名称
    #     bb_namespace="",
    #     bb_key=KEY_LUGGAGE_DETECTION,  # 结果存储在这里
    #     service_name="object_detection",
    #     object=None,  # 将从bb_source获取物体名称
    #     target_object_cls=None  # 不指定目标类别
    # ))

    root.add_child(BtNode_FindObj(name=f"find green bag", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object="green bag", target_object_cls="green bag"))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source="", message=f"grasping bag"))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp luggage", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, action_name=grasp_service_name))
    
    # 宣布找到行李
    root.add_child(BtNode_Announce(name="Announce found luggage locally", bb_source=None, message="Grasped."))
    
    return root

# def createGraspLuggage():
#     """抓取行李"""
#     root = py_trees.composites.Sequence("Grasp Luggage", memory=True)
    
#     # 现在才移动机械臂到扫描位置
#     root.add_child(BtNode_MoveArmSingle("Move arm to scan position", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    
#     # 打开夹爪
#     root.add_child(BtNode_GripperAction(name="Open gripper", open_gripper=True))
    
#     # 宣布开始抓取
#     root.add_child(BtNode_Announce(name="Announce grasping", bb_source=None, message="I'm going to grasp the luggage now"))
    
#     # 抓取行李
#     root.add_child(py_trees.decorators.Retry(
#         name="Retry grasping",
#         child=BtNode_Grasp(
#             name="Grasp luggage",
#             bb_source=KEY_LUGGAGE_DETECTION,
#             action_name=grasp_service_name
#         ),
#         num_failures=2
#     ))
    
#     # 抓取后将手臂移回导航姿态
#     root.add_child(BtNode_MoveArmSingle("Move arm to navigate position", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    
#     # 宣布抓取成功
#     root.add_child(BtNode_Announce(name="Announce grasp complete", bb_source=None, message="I have successfully grasped the luggage"))
    
#     return root
def createReturnToStart():
    """Return to the starting position"""
    return BtNode_GotoAction(name="ReturnToStart", key=KEY_POS_START, action_name="navigate_to_pose")


def createFollowWithTrackPerson(
    follow_distance: float = 1.5,
    stationary_threshold: float = 0.4,
    stationary_duration: float = 5.0,
    track_action_name: str = "track_person",
    nav_action_name: str = "navigate_to_pose"
):
    """
    Create the follow person behavior using the new TrackPerson action.
    
    This creates a parallel tree that:
    1. Tracks the person continuously using TrackPerson action
    2. Processes position updates and navigates toward the target
    3. Detects when the person has stopped moving
    4. Asks for confirmation when the person stops
    5. Returns SUCCESS when confirmed, otherwise continues following
    
    Args:
        follow_distance: Distance to maintain from the target (meters)
        stationary_threshold: Movement threshold for stationary detection (meters)
        stationary_duration: Time target must be stationary before asking (seconds)
        track_action_name: Name of the TrackPerson action server
        nav_action_name: Name of the navigation action server
    """
    root = py_trees.composites.Sequence(name="Follow With TrackPerson", memory=True)
    
    # Prepare robot for following - move arm to safe position
    root.add_child(py_trees.decorators.Retry(
        name="Retry Move Arm",
        child=BtNode_MoveArmSingle("Move arm to navigating", arm_service_name, KEY_ARM_NAVIGATING),
        num_failures=5
    ))
    
    # Move pan-tilt to look forward/slightly down to see person
    root.add_child(BtNode_TurnPanTilt(name="Move pan tilt for tracking", x=0.0, y=30.0, speed=0.0))
    
    # Announce start
    root.add_child(BtNode_Announce(
        name="Announce Start Following",
        bb_source=None,
        message="I will follow you now. Please walk slowly and let me know when you have reached your destination."
    ))
    
    # Main follow-confirm loop - retry until person confirms they've reached destination
    follow_confirm_loop = py_trees.composites.Sequence(name="Follow-Confirm Loop", memory=True)
    
    # Create the follow-until-stopped behavior
    follow_until_stopped = createFollowPersonUntilStopped(
        follow_distance=follow_distance,
        stationary_threshold=stationary_threshold,
        stationary_duration=stationary_duration,
        action_name=track_action_name,
        nav_action_name=nav_action_name
    )
    follow_confirm_loop.add_child(follow_until_stopped)
    
    # Person has stopped - ask for confirmation
    follow_confirm_loop.add_child(BtNode_Announce(
        name="Ask If Reached",
        bb_source=None,
        message="Have you reached your destination?"
    ))
    
    # Get confirmation - FAILURE means not yet reached, continue following
    follow_confirm_loop.add_child(BtNode_GetConfirmation(name="Get Destination Confirmation"))
    
    # Wrap in retry decorator - keeps looping until confirmed
    follow_retry = py_trees.decorators.Retry(
        name="Retry Until Destination Confirmed",
        child=follow_confirm_loop,
        num_failures=-1  # Infinite retries
    )
    
    root.add_child(follow_retry)
    
    # Announce completed following
    root.add_child(BtNode_Announce(
        name="Announce Arrived",
        bb_source=None,
        message="Great! You have reached your destination. Now please show me which bag to pick up."
    ))
    
    return root


def createPickUpBag():
    """
    Create the bag pickup behavior.
    
    This behavior:
    1. Looks for the pointed luggage
    2. Announces which bag is being pointed to
    3. Moves the arm to receive the bag
    4. Asks the person to hand over the bag
    """
    root = py_trees.composites.Sequence(name="Pick Up Bag", memory=True)
    
    # Move pan-tilt to look at person's hands
    root.add_child(BtNode_TurnPanTilt(name="Move pan tilt to see pointing", x=0.0, y=20.0, speed=0.0))
    
    # Move arm to safe position first
    root.add_child(BtNode_MoveArmSingle(
        "Move arm back",
        service_name=arm_service_name,
        arm_pose_bb_key=KEY_ARM_NAVIGATING
    ))
    
    # Announce looking for luggage
    root.add_child(BtNode_Announce(
        name="Announce finding luggage",
        bb_source=None,
        message="Looking for the bag you are pointing to"
    ))
    
    # Find the pointed luggage with retry
    find_luggage = py_trees.decorators.Retry(
        name="Retry Find Pointed Luggage",
        child=BtNode_FindPointedLuggage(
            name="Find pointed luggage",
            bb_namespace="",
            bb_key=KEY_GOAL,
            bb_key_announce_msg=KEY_ANNOUNCE_MSG
        ),
        num_failures=5
    )
    root.add_child(find_luggage)
    
    # Announce which bag was found
    root.add_child(BtNode_Announce(name="Announce found luggage", bb_source=KEY_ANNOUNCE_MSG))
    
    # Move arm to receive position
    root.add_child(BtNode_MoveArmSingle(
        name="Move arm to receive",
        service_name=arm_service_name,
        arm_pose_bb_key=KEY_ARM_SCAN,
        add_octomap=True
    ))
    
    # Ask person to hand over the bag
    root.add_child(BtNode_Announce(
        name="Ask for bag",
        bb_source=None,
        message="Please hand the luggage to me. Thank you!"
    ))
    
    # Wait for person to place bag
    root.add_child(BtNode_WaitTicks(name="Wait for bag placement", ticks=100))
    
    # Move arm back to safe position
    root.add_child(BtNode_MoveArmSingle(
        name="Move arm back after receive",
        service_name=arm_service_name,
        arm_pose_bb_key=KEY_ARM_NAVIGATING,
        add_octomap=True
    ))
    
    # Confirm received
    root.add_child(BtNode_Announce(
        name="Announce received",
        bb_source=None,
        message="I have received the bag. Moving arm back to safe position."
    ))
    
    # Move pan-tilt back to forward position
    root.add_child(BtNode_TurnPanTilt(name="Move pan tilt forward", x=0.0, y=45.0, speed=0.0))
    
    return root


def createReturnWithBag():
    """
    Create the return to start behavior with the bag.
    """
    root = py_trees.composites.Sequence(name="Return With Bag", memory=True)
    
    root.add_child(BtNode_Announce(
        name="Announce returning",
        bb_source=None,
        message="I will now return to the starting position."
    ))
    
    root.add_child(py_trees.decorators.Retry(
        name="Retry Return",
        child=createReturnToStart(),
        num_failures=10
    ))
    
    root.add_child(BtNode_Announce(
        name="Announce arrived at start",
        bb_source=None,
        message="I have returned to the starting position. Task completed!"
    ))
    
    return root


def createHelpMeCarry():
    """
    Create the complete "Help Me Carry" behavior tree.
    
    The robot will:
    1. Initialize constants and prepare
    2. Follow the person until they reach their destination (using TrackPerson action)
    3. Ask for confirmation when person stops
    4. Find and pick up the pointed bag
    5. Return to the starting position
    """
    root = py_trees.composites.Sequence("Help Me Carry", memory=True)
    
    # Step 1: Initialize constants
    root.add_child(createConstantWriter())
    
    # Step 2: Follow person until destination reached
    follow_behavior = createFollowWithTrackPerson(
        follow_distance=FOLLOW_DISTANCE,
        stationary_threshold=0.4,
        stationary_duration=5.0,
        track_action_name="track_person",
        nav_action_name="navigate_to_pose"
    )
    root.add_child(follow_behavior)
    
    # Step 3: Pick up the pointed bag
    pickup_behavior = createPickUpBag()
    root.add_child(pickup_behavior)
    
    # Step 4: Return to starting position (optional - uncomment if needed)
    # return_behavior = createReturnWithBag()
    # root.add_child(return_behavior)
    
    # Final announcement
    root.add_child(BtNode_Announce(
        name="Announce Task Complete",
        bb_source=None,
        message="Help me carry task completed successfully!"
    ))
    
    return root


def createHelpMeCarrySimple():
    """
    Create a simplified "Help Me Carry" behavior tree for testing.
    
    This version uses the complete follow behavior from Follow.py
    """
    root = py_trees.composites.Sequence("Help Me Carry Simple", memory=True)
    
    # Initialize constants
    root.add_child(createConstantWriter())
    
    # Use the complete follow person behavior from Follow.py
    root.add_child(createFollowPersonComplete(
        follow_distance=FOLLOW_DISTANCE,
        stationary_threshold=0.4,
        stationary_duration=5.0,
        action_name="track_person",
        nav_action_name="navigate_to_pose"
    ))
    
    # Find and pick up luggage
    root.add_child(createFindPointedLuggage())
    
    return root