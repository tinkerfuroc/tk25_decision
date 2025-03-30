import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Drop
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_PhraseExtraction

from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
import rclpy
import math

pose_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=5.1324, y=13.4106, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.6026580611435992, w=0.7979995371794634)))
pose_table_middle = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=5.1324, y=13.4106, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0825770446495785, w=0.9965846836556046)))
pose_starting = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.0222, y=16.4780, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.9965846836556046, w=0.0825770446495785)))
pose_starting_middle = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.0222, y=16.4780, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.6026580611435992, w=0.7979995371794634)))


point_drop = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="base_link"), point=Point(x=0.7, y=0.0, z=0.3))

arm_navigating = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
arm_scan = [0.0, -0.4187, 0.0, 1.709, 0.0, 1.343, 0.0]
# arm_scan = [x / 180 * math.pi for x in [-19.4, -18.3, -1.7, 97.4, 11.4, 80.8, 0]]
# arm_scan = [x / 180 * math.pi for x in [-12.9, -18.3, -5.4, 82.2, 12.3, 63.7, -30.9]]
# arm_scan = [x / 180 * math.pi for x in [-12.9, -18.0, -2.4, 11.4, 12.3, -56.1, -8.2]]
arm_scan_mid = [x / 180 * math.pi for x in [-87.6, -18.0, 8.3, 42.4, 1.6, -56.1, -20]]
# arm_scan = [x / 180 * math.pi for x in []]

KEY_POSE_TABLE = 'pose_table'
KEY_POSE_STARTING = 'pose_sofa'
KEY_POSE_TABLE_MIDDLE = 'pose_table_middle'
KEY_POSE_STARTING_MIDDLE = 'pose_starting_middle'
KEY_DROP_POINTS = "point_drop"

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN_MIDDLE = "arm_scan_middle"

KEY_OBJECT = "object"

KEY_DUMMY = 'dummy'

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"


def createGraspOnce():
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArmSingle("Move arm to scan middle", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN_MIDDLE))
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    root.add_child(BtNode_FindObj(name="find object", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object="green bottle", target_object_cls='green bottle'))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source="", message="grasping green water bottle"))
    parallel_grasp.add_child(BtNode_Grasp("Grasp trash", bb_source=KEY_OBJECT, service_name=grasp_service_name))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArmSingle("Move arm to scan middle", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN_MIDDLE))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return root

def create_demo():
    root = py_trees.composites.Sequence("Intelligent demo", memory=True)

    parallel_to_bb = py_trees.composites.Parallel("Parallel to BB", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write door location", bb_namespace="", bb_source=None, bb_key=KEY_POSE_TABLE, object=pose_table))
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write sofa location", bb_namespace="", bb_source=None, bb_key=KEY_POSE_STARTING, object=pose_starting))
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=arm_scan))
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write arm navigating pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=arm_navigating))
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write arm scan middle pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN_MIDDLE, object=arm_scan_mid))
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write sofa middle", bb_namespace="", bb_source=None, bb_key=KEY_POSE_STARTING_MIDDLE, object=pose_starting_middle))
    parallel_to_bb.add_child(BtNode_WriteToBlackboard(name="Write table middle", bb_namespace="", bb_source=None, bb_key=KEY_POSE_TABLE_MIDDLE, object=pose_table_middle))
    root.add_child(parallel_to_bb)

    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Hi, I'm Tinker, you can instruct me to do tasks for you"))
    root.add_child(py_trees.decorators.FailureIsSuccess(name="wrapper", child=BtNode_PhraseExtraction(name="Get command", wordlist=['table'], bb_dest_key=KEY_DUMMY, timeout=7.0)))
    
    root.add_child(BtNode_Announce(name="Announce go to table", bb_source=None, message="Going to table"))
    root.add_child(py_trees.decorators.FailureIsSuccess(name="wrapper", child=BtNode_PhraseExtraction(name="Get command", wordlist=['table'], bb_dest_key=KEY_DUMMY, timeout=20.0)))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="go to start middle", key=KEY_POSE_STARTING_MIDDLE), num_failures=5))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="go to table", key=KEY_POSE_TABLE), num_failures=5))
    root.add_child(py_trees.decorators.Retry(name="retry grasp", child=createGraspOnce(), num_failures=5))

    root.add_child(BtNode_Announce(name="Announce go back", bb_source="", message="Going back to hand bottle over"))
    root.add_child(py_trees.decorators.FailureIsSuccess(name="wrapper", child=BtNode_PhraseExtraction(name="Get command", wordlist=['table'], bb_dest_key=KEY_DUMMY, timeout=20.0)))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="turn 90 degrees", key=KEY_POSE_TABLE_MIDDLE), num_failures=5))
    # root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(name="go back", key=KEY_POSE_STARTING), num_failures=5))
    root.add_child(BtNode_Announce(name="Announce hand bottle over", bb_source="", message="Here is your water"))
    root.add_child(BtNode_Drop(name="hand bottle over", bb_source="", service_name="start_drop", bin_point=point_drop))

    root.add_child(BtNode_Announce(name="Announce finished", bb_source="", message="Getting water completed"))
    root.add_child(py_trees.behaviours.Running(name="finished"))

    return root