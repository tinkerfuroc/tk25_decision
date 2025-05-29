import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_GetPointCloud, BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Drop, BtNode_Place, BtNode_GripperAction
from behavior_tree.StoringGroceries.customNodes import BtNode_GraspWithPose

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

POS_KITCHEN = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                        )

# scan positions
N_SCAN_POS = 2
POS_TABLE_SCAN_1 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                              pose=Pose(position=Point(x=11.228998184204102, y=3.896925210952759, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=-0.5613218483610379, w=0.8275975969953924))
                              ),
POS_TABLE_SCAN_2 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                              pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                              )

# 2 drop positions
N_DROP_POS = 2
POS_TABLE_DROP_1 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                              pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                             ),
POS_TABLE_DROP_2 = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                              pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                             )
POINT_TABLE_DROP_1 = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                  point=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0)
                                 )
POINT_TABLE_DROP_2 = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                  point=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0)
                                 )

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]

N_ARM_POS_SCAN = 3
ARM_POS_SCANS = [
    [x / 180 * math.pi for x in [0.0, -43.0, 0.0, 45.0, 0.0, 31.0, 0.0]],
    [x / 180 * math.pi for x in [-20.0, -43.0, 0.0, 45.0, 0.0, 31.0, 0.0]],
    [x / 180 * math.pi for x in [20.0, -43.0, 0.0, 45.0, 0.0, 31.0, 0.0]],
]

# ARM_POS_SCAN_MIDDLE = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 60.0, 30.0, -86.0, 0.0]]

KEY_ARM_SCANS = [f"arm_scan_{i}" for i in range(N_ARM_POS_SCAN)]
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN_MIDDLE = "arm_scan_middle"

prompt_bowl = "bowl"
prompt_spoon = "spoon"
prompt_cereal = "cereal bag"
prompt_milk = "milk"

prompts = [prompt_milk, prompt_cereal, prompt_bowl, prompt_spoon]

MAX_SCAN_DISTANCE = 2.0

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"
point_cloud_service_name = "get_point_cloud_service"
point_target_frame = "base_link"

KEY_KITCHEN_POSE = "door_pose"
KEY_TABLE_SCAN_POSE = [f"table_scan_pose_{i}" for i in range(N_SCAN_POS)]
KEY_TABLE_DROP_POSE = [f"table_drop_pose_{i}" for i in range(N_DROP_POS)]
KEY_TABLE_DROP_POINT = [f"table_drop_point_{i}" for i in range(N_DROP_POS)]
KEY_OBJECT_POINT = "object_point"

KEY_OBJECT = "object"
KEY_OBJECT_CLASS = "object_class"

KEY_GRASP_POSE = "grasp_pose"
KEY_ENV_POINTS = "env_points"


def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Write kitchen pose", bb_namespace="", bb_source=None, bb_key=KEY_KITCHEN_POSE, object=POS_KITCHEN))
    root.add_child(BtNode_WriteToBlackboard(name="Write table scan poses", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_SCAN_POSE[0], object=POS_TABLE_SCAN_1))
    root.add_child(BtNode_WriteToBlackboard(name="Write table scan poses", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_SCAN_POSE[1], object=POS_TABLE_SCAN_2))
    root.add_child(BtNode_WriteToBlackboard(name="Write table drop poses", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_DROP_POSE[0], object=POS_TABLE_DROP_1))
    root.add_child(BtNode_WriteToBlackboard(name="Write table drop poses", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_DROP_POSE[1], object=POS_TABLE_DROP_2))
    root.add_child(BtNode_WriteToBlackboard(name="Write table drop points", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_DROP_POINT[0], object=POINT_TABLE_DROP_1))
    root.add_child(BtNode_WriteToBlackboard(name="Write table drop points", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_DROP_POINT[1], object=POINT_TABLE_DROP_1))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCANS[0], object=ARM_POS_SCANS[0]))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCANS[1], object=ARM_POS_SCANS[1]))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCANS[2], object=ARM_POS_SCANS[2]))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm navigating pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    # root.add_child(BtNode_WriteToBlackboard(name="Write arm scan middle pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN_MIDDLE, object=ARM_POS_SCAN_MIDDLE))

    return root

def createGoPositionAndFind(obj_name, position):
    scan_at_pos = py_trees.composites.Sequence(name="Scan at pos 1", memory=True)
    
    # move to scan position
    scan_at_pos.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=True), 3))
    scan_at_pos.add_child(py_trees.decorators.Retry("retry", BtNode_GotoAction(f"Goto grasp position {position}", key=KEY_TABLE_SCAN_POSE[position]), 5))

    # move arm for scan
    scan_at_pos.add_child(BtNode_Announce(name="Announce moving arm", bb_source="", message=f"Moving arm to find {obj_name}"))
    scan_poses = py_trees.composites.Selector(name="Scan at Poses", memory=True)
    for j in range(N_ARM_POS_SCAN):
        scan_j = py_trees.composites.Sequence(name="Scan", memory=True)
        scan_j.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCANS[j], add_octomap=True), 3))
        scan_j.add_child(BtNode_FindObj(name=f"find {obj_name}", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object=obj_name, target_object_cls=obj_name))
        scan_poses.add_child(scan_j)
    scan_at_pos.add_child(scan_poses)
    return scan_at_pos

def createGoPositionAndPlace(obj_name, drop_position):
    root = py_trees.composites.Sequence(name=f"Place {obj_name}", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(f"go to drop {obj_name}", KEY_TABLE_DROP_POSE[drop_position]), num_failures=5))
    parallel_place = py_trees.composites.Parallel("Parallel Place", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_place.add_child(BtNode_Announce(name="Announce placing", bb_source="", message=f"placing {obj_name}"))
    parallel_place.add_child(BtNode_GetPointCloud("Find environment point cloud", bb_point_cloud_key=KEY_ENV_POINTS, service_name=point_cloud_service_name, camera_name="orbbec"))
    parallel_place.add_child(BtNode_Place(name="Place object on table",
                                          bb_key_point=KEY_TABLE_DROP_POINT[drop_position],
                                          bb_key_pose=KEY_GRASP_POSE,
                                          bb_key_env_points=KEY_ENV_POINTS,
                                          action_name=place_service_name))
    fail_fallback = py_trees.composites.Selector(name="fallback", memory=True)
    fail_fallback.add_child(parallel_place)
    fail_fallback.add_child(py_trees.decorators.Repeat("repeat", BtNode_GripperAction("Open gripper", True), 3))
    root.add_child(fail_fallback)
    return root

def createGraspAndPlace(obj_name="object", drop_position=0):
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(py_trees.decorators.FailureIsSuccess("FisS", BtNode_TurnPanTilt("Turn to scan", 0.0, 20.0, 0.0)))

    # find object on table
    scan_at_poses = py_trees.composites.Selector(name="Scan at Poses", memory=True)
    for i in range(N_SCAN_POS):
        scan_at_poses.add_child(createGoPositionAndFind(obj_name, i))
    
    root.add_child(py_trees.decorators.Retry("retry", scan_at_poses, 2))

    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source="", message=f"grasping {obj_name}"))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, service_name=grasp_service_name))
    root.add_child(parallel_grasp)

    # move arm back
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING), 3))

    # go to drop pose and drop
    drop_at_poses = py_trees.composites.Selector(name="Drop at Poses", memory=True)
    for i in range(N_DROP_POS):
        drop_at_poses.add_child(createGoPositionAndPlace(obj_name, (drop_position + i) % N_DROP_POS))
    
    # root.add_child(py_trees.decorators.Retry("retry", drop_at_poses, 2))
    root.add_child(drop_at_poses)

    return root

def createEnterKitchen():
    root = py_trees.composites.Sequence(name="Enter Kitchen", memory=True)
    root.add_child(BtNode_Announce(name="Announce going to kitchen", bb_source=None, message="Going to kitchen"))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction("go to kitchen", KEY_KITCHEN_POSE), num_failures=10))
    return root

def createServeBreakfast():
    root = py_trees.composites.Sequence(name="Serve Breakfast", memory=True)
    root.add_child(createConstantWriter())
    root.add_child(createEnterKitchen())
    root.add_child(createGraspAndPlace(prompts[0], 0))
    root.add_child(createGraspAndPlace(prompts[1], 1))
    root.add_child(createGraspAndPlace(prompts[2], 0))
    root.add_child(createGraspAndPlace(prompts[3], 1))

    # announce finished, then end on a running node
    root.add_child(BtNode_Announce(name="Announce finished", bb_source=None, message="Breakfast served"))
    root.add_child(py_trees.behaviours.Running(name="Running"))
    # root.add_child(BtNode_GripperAction("Open gripper", True))

    return root


def draw():
    root = createServeBreakfast()
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)