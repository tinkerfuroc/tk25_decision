import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArm, BtNode_Drop
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction

from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
import rclpy
import math

pose_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715)))
pose_starting = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.9390, y=14.5222, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.673597350035888, w=0.7390985117185863)))


point_drop = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="base_link"), point=Point(x=0.8, y=0.3, z=0.2))

arm_scan = [x / 180 * math.pi for x in []]
arm_navigating = [x / 180 * math.pi for x in []]
# arm_scan = [x / 180 * math.pi for x in []]

KEY_POSE_TABLE = 'pose_table'
KEY_POSE_STARTING = 'pose_sofa'
KEY_DROP_POINTS = "point_drop"

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"

KEY_OBJECT = "object"

KEY_DUMMY = 'dummy'

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"


def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArm("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    root.add_child(BtNode_FindObj(name="find object", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object="bottle of water"))
    root.add_child(BtNode_Grasp("Grasp trash", bb_source=KEY_OBJECT, service_name=grasp_service_name))
    root.add_child(BtNode_MoveArm("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return root

def create_demo():
    root = py_trees.composites.Sequence("Intelligent demo", memory=True)

    root.add_child(BtNode_WriteToBlackboard(name="Write door location", bb_namespace="", bb_source=None, bb_key=KEY_POSE_TABLE, object=pose_table))
    root.add_child(BtNode_WriteToBlackboard(name="Write sofa location", bb_namespace="", bb_source=None, bb_key=KEY_POSE_STARTING, object=pose_starting))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=arm_scan))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm navigating pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=arm_navigating))

    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Hi, I'm Tinker, you can instruct me to do tasks for you"))
    root.add_child(BtNode_PhraseExtraction(name="Get command", wordlist=['table'], bb_dest_key=KEY_DUMMY, timeout=7.0))
    
    root.add_child(BtNode_GotoAction(name="go to table", key=KEY_POSE_TABLE))
    root.add_child(createGraspOnce())

    root.add_child(BtNode_GotoAction(name="go back", key=KEY_POSE_STARTING))
    root.add_child(BtNode_Drop(name="hand bottle over", bb_source="", service_name="start_drop", bin_point=point_drop))

    root.add_child(BtNode_Announce(name="Announce finished", bb_source="", message="Getting water completed"))
    root.add_child(py_trees.behaviours.Running(name="finished"))