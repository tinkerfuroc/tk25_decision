import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj, BtNode_GetPointCloud
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp, BtNode_MoveArmSingle, BtNode_Drop, BtNode_Place
from behavior_tree.StoringGroceries.customNodes import BtNode_GraspWithPose

import math

from geometry_msgs.msg import PointStamped, PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

# POS_KITCHEN = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                             pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
#                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                             )
# POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                             pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
#                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                             )
# POS_BOWL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                             pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
#                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                             )
# POS_SPOON = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                             pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
#                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                             )
# POS_CEREAL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                             pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
#                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                             )
# POS_MILK = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                             pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0),
#                                         orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715))
#                             )

POS_KITCHEN = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                        )
POS_TABLE = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=8.620, y=8.704, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.594, w=0.80446504))
                        )
POS_BOWL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                        )
POS_SPOON = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                        )
POS_CEREAL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                        )
POS_MILK = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        pose=Pose(position=Point(x=1.9052205940802227, y=0.07986240342385699, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=-0.6162222736893007, w=0.787572288370527))
                        )

# POINT_DROP_BOWL = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                 point=Point(x=4.3053, y=15.9896, z=0.0)
#                                 )
# POINT_DROP_SPOON = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                 point=Point(x=4.3053, y=15.9896, z=0.0)
#                                 )
# POINT_DROP_CEREAL = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                 point=Point(x=4.3053, y=15.9896, z=0.0)
#                                 )
# POINT_DROP_MILK = PointStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
#                                 point=Point(x=4.3053, y=15.9896, z=0.0)
#                                 )

POINT_DROP_BOWL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        # pose=Pose(position=Point(x=-0.2942876962347504, y=0.7816651007796609, z=0.0),
                        pose=Pose(position=Point(x=-0.2942876962347504, y=0.6316651007796609, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.732980291613576, w=0.680249874))
                        )
POINT_DROP_SPOON = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        # pose=Pose(position=Point(x=-0.2942876962347504, y=0.7816651007796609, z=0.0),
                        pose=Pose(position=Point(x=-0.2942876962347504, y=0.6316651007796609, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.732980291613576, w=0.680249874))
                        )
POINT_DROP_CEREAL = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        # pose=Pose(position=Point(x=-0.2942876962347504, y=0.7816651007796609, z=0.0),
                        pose=Pose(position=Point(x=-0.2942876962347504, y=0.6316651007796609, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.732980291613576, w=0.680249874))
                        )
POINT_DROP_MILK = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                        # pose=Pose(position=Point(x=-0.2942876962347504, y=0.7816651007796609, z=0.0),
                        pose=Pose(position=Point(x=-0.2942876962347504, y=0.6316651007796609, z=0.0),
                                  orientation=Quaternion(x=0.0, y=0.0, z=0.732980291613576, w=0.680249874))
                        )

# ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
# ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -0.4187, 0.0, 1.709, 0.0, 1.343, 0.0]]
# ARM_POS_SCAN_MIDDLE = [x / 180 * math.pi for x in [-87.6, -18.0, 8.3, 42.4, 1.6, -56.1, -20]]

ARM_POS_NAVIGATING = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 0.0, 30.0, -86.0, 0.0]]
ARM_POS_SCAN = [x / 180 * math.pi for x in [0.0, -50.0, 0.0, 66.0, 0.0, 55.0, 0.0]]
ARM_POS_SCAN_MIDDLE = [x / 180 * math.pi for x in [-87.0, -40.0, 28.0, 60.0, 30.0, -86.0, 0.0]]

KEY_ARM_SCAN = "arm_scan"
KEY_ARM_NAVIGATING = "arm_navigating"
KEY_ARM_SCAN_MIDDLE = "arm_scan_middle"

prompt_bowl = "bowl"
prompt_spoon = "spoon"
prompt_cereal = "cereal box"
prompt_milk = "white milk carton"

MAX_SCAN_DISTANCE = 2.0

arm_service_name = "arm_joint_service"
grasp_service_name = "start_grasp"
place_service_name = "place_service"
point_cloud_service_name = "get_point_cloud_service"
point_target_frame = "base_link"

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
KEY_OBJECT_CLASS = "object_class"

KEY_GRASP_POSE = "grasp_pose"
KEY_ENV_POINTS = "env_points"


def createConstantWriter():
    root = py_trees.composites.Parallel(name="Write constants to blackboard", policy=py_trees.common.ParallelPolicy.SuccessOnAll())

    root.add_child(BtNode_WriteToBlackboard(name="Write kitchen pose", bb_namespace="", bb_source=None, bb_key=KEY_KITCHEN_POSE, object=POS_KITCHEN))
    root.add_child(BtNode_WriteToBlackboard(name="Write table pose", bb_namespace="", bb_source=None, bb_key=KEY_TABLE_POSE, object=POS_TABLE))
    root.add_child(BtNode_WriteToBlackboard(name="Write bowl pose", bb_namespace="", bb_source=None, bb_key=KEY_BOWL_POSE, object=POS_BOWL))
    root.add_child(BtNode_WriteToBlackboard(name="Write spoon pose", bb_namespace="", bb_source=None, bb_key=KEY_SPOON_POSE, object=POS_SPOON))
    root.add_child(BtNode_WriteToBlackboard(name="Write cereal pose", bb_namespace="", bb_source=None, bb_key=KEY_CEREAL_POSE, object=POS_CEREAL))
    root.add_child(BtNode_WriteToBlackboard(name="Write milk pose", bb_namespace="", bb_source=None, bb_key=KEY_MILK_POSE, object=POS_MILK))
    root.add_child(BtNode_WriteToBlackboard(name="Write bowl point", bb_namespace="", bb_source=None, bb_key=KEY_BOWL_POINT, object=POINT_DROP_BOWL))
    root.add_child(BtNode_WriteToBlackboard(name="Write spoon point", bb_namespace="", bb_source=None, bb_key=KEY_SPOON_POINT, object=POINT_DROP_SPOON))
    root.add_child(BtNode_WriteToBlackboard(name="Write cereal point", bb_namespace="", bb_source=None, bb_key=KEY_CEREAL_POINT, object=POINT_DROP_CEREAL))
    root.add_child(BtNode_WriteToBlackboard(name="Write milk point", bb_namespace="", bb_source=None, bb_key=KEY_MILK_POINT, object=POINT_DROP_MILK))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN, object=ARM_POS_SCAN))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm navigating pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_NAVIGATING, object=ARM_POS_NAVIGATING))
    root.add_child(BtNode_WriteToBlackboard(name="Write arm scan middle pose", bb_namespace="", bb_source=None, bb_key=KEY_ARM_SCAN_MIDDLE, object=ARM_POS_SCAN_MIDDLE))

    return root

def createGraspOnce(obj_name="object"):
    root = py_trees.composites.Sequence(name="Grasp Once", memory=True)
    parallel_move_arm = py_trees.composites.Parallel("Move arm to find object", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_move_arm.add_child(BtNode_Announce(name="Announce moving arm", bb_source="", message="Moving arm to find object"))
    parallel_move_arm.add_child(BtNode_MoveArmSingle("Move arm to find", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_SCAN, add_octomap=True))
    # parallel_move_arm.add_child(BtNode_WriteToBlackboard(name="Write arm scan middle pose", bb_namespace="", bb_source=None, bb_key=KEY_OBJECT_CLASS, object=obj_name))
    root.add_child(parallel_move_arm)
    # find object on table
    root.add_child(BtNode_FindObj(name=f"find {obj_name}", bb_source=None, bb_namespace=None, bb_key=KEY_OBJECT, object=obj_name, target_object_cls=obj_name))
    # add parallel node to grasp and announcing it is grasping
    parallel_grasp = py_trees.composites.Parallel("Parallel Grasp", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_grasp.add_child(BtNode_Announce(name="Announce grasping", bb_source="", message=f"grasping {obj_name}"))
    parallel_grasp.add_child(BtNode_GraspWithPose(f"Grasp object on table", bb_key_vision_res=KEY_OBJECT, bb_key_pose=KEY_GRASP_POSE, service_name=grasp_service_name))
    root.add_child(parallel_grasp)
    root.add_child(BtNode_MoveArmSingle("Move arm back", service_name=arm_service_name, arm_pose_bb_key=KEY_ARM_NAVIGATING))
    return py_trees.decorators.Retry(name="retry 5 times", child=root, num_failures=5)

def createFindAndPlace(key_find_location, key_drop_location, obj_name):
    root = py_trees.composites.Sequence(name=f"Find and place {obj_name}", memory=True)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(f"go to find {obj_name}", key_find_location), num_failures=10))
    root.add_child(createGraspOnce(obj_name=obj_name))
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_GotoAction(f"go to drop {obj_name}", KEY_TABLE_POSE), num_failures=10))
    parallel_place = py_trees.composites.Parallel("Parallel Place", policy=py_trees.common.ParallelPolicy.SuccessOnAll())
    parallel_place.add_child(BtNode_Announce(name="Announce placing", bb_source="", message=f"placing {obj_name}"))
    parallel_place.add_child(BtNode_GetPointCloud("Find environment point cloud", bb_point_cloud_key=KEY_ENV_POINTS, service_name=point_cloud_service_name, camera_name="orbbec"))
    parallel_place.add_child(BtNode_Place(name="Place object on table",
                                          bb_key_point=key_drop_location,
                                          bb_key_pose=KEY_GRASP_POSE,
                                          bb_key_env_points=KEY_ENV_POINTS,
                                          service_name=place_service_name))
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