import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Drop

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

POS_KITCHEN = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                            )
POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                            )
POS_BOWL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                            )
POS_SPOON = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                            )
POS_CEREAL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                            )
POS_MILK = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                            pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
                                        orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
                            )

POINT_DROP_BOWL = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                point=Point(x=4.3053, y=15.9896, z=0.0)
                                )
POINT_DROP_SPOON = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                point=Point(x=4.3053, y=15.9896, z=0.0)
                                )
POINT_DROP_CEREAL = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                point=Point(x=4.3053, y=15.9896, z=0.0)
                                )
POINT_DROP_MILK = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                                point=Point(x=4.3053, y=15.9896, z=0.0)
                                )

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.6, -60, -2.6, -3.9, 12.4, -84.9, -8.2]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -0.4187, 0.0, 1.709, 0.0, 1.343, 0.0]]
ARM_POS_SCAN_MIDDLE = [x / 180 * math.pi for x in [-87.6, -18.0, 8.3, 42.4, 1.6, -56.1, -20]]

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN_MIDDLE = "arm_scan_middle"


prompt_bowl = "red bowl"
prompt_spoon = "spoon"
prompt_cereal = "cereal box"
prompt_milk = "white milk carton"

MAX_SCAN_DISTANCE = 2.0

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"

KEY_KITCHEN_POSE = "door_pose"
KEY_TABLE_POSE = "table_pose"
KEY_BOWL_POSE = "bowl_pose"
KEY_SPOON_POSE = "spoon_pose"
KEY_CEREAL_POSE = "cereal_pose"
KEY_MILK_POSE = "milk_pose"
KEY_MILK_POINT = "milk_point"
KEY_BOWL_POINT = "bowl_point"
KEY_SPOON_POINT = "spoon_point"
KEY_CEREAL_POINT = "cereal_point"

KEY_OBJECT = "object"


def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Write kitchen pose", key=KEY_KITCHEN_POSE, value=POS_KITCHEN))
    root.add_child(BtNode_WriteToBlackboard(name="Write table pose", key=KEY_TABLE_POSE, value=POS_TABLE))
    root.add_child(BtNode_WriteToBlackboard(name="Write bowl pose", key=KEY_BOWL_POSE, value=POS_BOWL))
    root.add_child(BtNode_WriteToBlackboard(name="Write spoon pose", key=KEY_SPOON_POSE, value=POS_SPOON))
    root.add_child(BtNode_WriteToBlackboard(name="Write cereal pose", key=KEY_CEREAL_POSE, value=POS_CEREAL))
    root.add_child(BtNode_WriteToBlackboard(name="Write milk pose", key=KEY_MILK_POSE, value=POS_MILK))
    root.add_child(BtNode_WriteToBlackboard(name="Write bowl point", key=KEY_BOWL_POINT, value=POINT_DROP_BOWL))
    root.add_child(BtNode_WriteToBlackboard(name="Write spoon point", key=KEY_SPOON_POINT, value=POINT_DROP_SPOON))
    root.add_child(BtNode_WriteToBlackboard(name="Write cereal point", key=KEY_CEREAL_POINT, value=POINT_DROP_CEREAL))
    root.add_child(BtNode_WriteToBlackboard(name="Write milk point", key=KEY_MILK_POINT, value=POINT_DROP_MILK))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", key=KEY_ARM_SCAN, value=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm navigating pose", key=KEY_ARM_NAVIGATING, value=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan middle pose", key=KEY_ARM_SCAN_MIDDLE, value=ARM_POS_SCAN_MIDDLE))

    return root

def createGraspOnce(obj_name="object"):
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    root.add_child(BtNode_MoveArmSingle("Move arm to scan middle", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN_MIDDLE))
    root.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN))
    root.add_child(BtNode_FindObj(name=f"find {obj_name}", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object=obj_name, target_object_cls=obj_name))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source="", message=f"grasping {obj_name}"))
    parallel_grasp.add_child(BtNode_Grasp(f"Grasp {obj_name}", bb_source=KEY_OBJECT, service_name=grasp_service_name))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArmSingle("Move arm to scan middle", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN_MIDDLE))
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return root

def createFindAndPlace(key_find_location, key_drop_location, obj_name):
    root = py_trees.composites.Sequence(name=f"Find and place {obj_name}", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(f"go to find {obj_name}", key_find_location), num_failures=10))
    root.add_child(createGraspOnce(obj_name=obj_name))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(f"go to drop {obj_name}", KEY_TABLE_POSE), num_failures=10))
    root.add_child(BtNode_Drop(f"Drop {obj_name}", bb_source=key_drop_location, service_name=grasp_service_name))
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
    root.add_child(createFindAndPlace(KEY_BOWL_POSE, KEY_BOWL_POINT, prompt_bowl))
    root.add_child(createFindAndPlace(KEY_SPOON_POSE, KEY_SPOON_POINT, prompt_spoon))
    root.add_child(createFindAndPlace(KEY_CEREAL_POSE, KEY_CEREAL_POINT, prompt_cereal))
    root.add_child(createFindAndPlace(KEY_MILK_POSE, KEY_MILK_POINT, prompt_milk))

    # announce finished, then end on a running node
    root.add_child(BtNode_Announce(name="Announce finished", bb_source=None, message="Breakfast served"))
    root.add_child(py_trees.behaviours.Running(name="Running"))

    return root


def draw():
    root = createServeBreakfast()
    py_trees.display.render_dot_tree(root, with_blackboard_variables=True)