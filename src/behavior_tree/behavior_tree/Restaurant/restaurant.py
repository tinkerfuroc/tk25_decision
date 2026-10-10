from py_trees.composites import Sequence, Selector, Parallel
from .custom_nodes import (
    BtNode_GetCommand,
    BtNode_ScanForWavingPerson
)
from py_trees.trees import BehaviourTree
from py_trees import decorators
import py_trees

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

# Import additional nodes from TemplateNodes
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction, BtNode_GetConfirmation
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_TrackPerson, BtNode_DoorDetection
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_ComputeGraspPose, BtNode_PoseTransform
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle
from behavior_tree.Receptionist.customNodes import BtNode_Confirm
import math

# ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -44.0, 26.0, 20.0, 30.0, -92.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]

pose_scan_sequence = [
    # +30 deg
    PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='base_link'),
                pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), 
                          orientation=Quaternion(x=0.0, y=0.0, z=0.258542066, w=0.966))
                ),
    # -60 deg
    PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='base_link'),
                pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), 
                          orientation=Quaternion(x=0.0, y=0.0, z=-0.500043998, w=0.866))
                ),
    # +30 deg and move forward
    PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='base_link'),
                pose=Pose(position=Point(x=1.0, y=0.0, z=0.0), 
                          orientation=Quaternion(x=0.0, y=0.0, z=0.258542066, w=0.966))
                ),
]

pose_start = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                         pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), 
                                   orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                         )

N_SCAN_POSE = len(pose_scan_sequence)
# scan_poses = [KEY_POSE_LIVINGROOM, KEY_POSE_KITCHEN, KEY_POSE_DININGROOM, KEY_POSE_BEDROOM]
KEY_SCAN_POSES = [f"scan_pose_seq_{i}" for i in range(N_SCAN_POSE)]

KEY_START_POSE = "start_pose"
KEY_DEST_POSE = "dest_pose"

KEY_ARM_POSE = "arm_pose"
KEY_GRASP_PROMPT = "grasp_prompt"
KEY_OBJECT = "object"
KEY_GRASP_POSE = "go_to_grasp_pose"

KEY_ORDER = "order"
KEY_ARM_NAVIGATING = "arm_navigating"

KEY_WAVING_PERSON = "waving person"

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"

def createConstantWriter():
    root = Sequence("Root", memory=True)
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_ARM_POSE, None, ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_ARM_NAVIGATING, None, ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_START_POSE, None, pose_start))
    # root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_START_POSE, None, pose_start))
    for i in range(N_SCAN_POSE):
        root.add_child(BtNode_WriteToBlackboard("Write to blackboard", "", KEY_SCAN_POSES[i], None, pose_scan_sequence[i]))
    return root

def createScanForWaving(idx):
    root = Sequence("scan for waving", True)

    root.add_child(decorators.Retry("retry", BtNode_PoseTransform(name=f"transform scan pose {idx}",
                                                                  bb_source=KEY_SCAN_POSES[idx],
                                                                  bb_target=KEY_DEST_POSE,
                                                                  target_frame="map"), 3))
    root.add_child(BtNode_Announce("announce", None, message="Going to scanning position"))
    root.add_child(decorators.Retry("retry", BtNode_GotoAction(f"goto scan pos {idx}", KEY_DEST_POSE), 3))
    # Scan for waving person, store to blackboard
    root.add_child(BtNode_Announce("announce", None, message="Scanning for waving person"))
    root.add_child(BtNode_ScanForWavingPerson("scan for waving person", bb_target=KEY_WAVING_PERSON, use_orbbec=True, target_frame="map"))
    root.add_child(BtNode_Announce("announce", None, message="Waving person found."))
    return root


def createFindPerson():
    root = Selector("selector", True)
    for it in range(3):
        for i in range(3):
            root.add_child(createScanForWaving(i))
    root.add_child(decorators.SuccessIsFailure("SisF", BtNode_Announce("announce failed to find person", None, message="Failed to find waving person")))

    return root
    

def createRecvOrder():
    loop = Sequence(name=f"get and confirm order", memory=True)
    loop.add_child(BtNode_Announce(name=f"Prompt for getting order", bb_source="", message=f"Please tell me your order"))
    loop.add_child(BtNode_GetCommand(name="get command", bb_dest_key=KEY_ORDER))
    loop.add_child(BtNode_Confirm(name=f"Confirm command", key_confirmed=KEY_ORDER, type="order"))
    loop.add_child(BtNode_GetConfirmation(name=f"Get confirmation", timeout=5.0))
    loop.add_child(BtNode_Announce("received command", None, message="Order received, will execute later"))

    return decorators.Retry("retry", loop, 3)

def createRestaurant():
    root = Sequence("Restaurant", True)
    root.add_child(createConstantWriter())
    root.add_child(createFindPerson())

    go_to_person_node = Sequence("go to person", memory=True)
    go_to_person_node.add_child(BtNode_ComputeGraspPose("compute grasp pose", bb_src=KEY_WAVING_PERSON, bb_target=KEY_GRASP_POSE))
    go_to_person_node.add_child(py_trees.decorators.Retry('retry', BtNode_GotoAction('go to person', key=KEY_GRASP_POSE), 2))

    root.add_child(decorators.FailureIsSuccess('fiss', decorators.Retry('retry', go_to_person_node, 5)))

    root.add_child(createRecvOrder())

    root.add_child(decorators.Retry("retry", BtNode_GotoAction(f"goto", KEY_START_POSE), 3))

    return root