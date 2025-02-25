import py_trees

from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, 

from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
import rclpy
import math

pose_table = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.3053, y=15.9896, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.8851875996971402, w=0.46523425641542715)))
pose_sofa = PoseStamped(header=Header(stamp=rclpy.time.Time().to_msg(), frame_id='map'),
                      pose=Pose(position=Point(x=4.9390, y=14.5222, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=-0.673597350035888, w=0.7390985117185863)))

arm_poses_d = [

]
arm_poses = [
    [x / 180 * math.pi for x in p] for p in arm_poses_d
]

KEY_POSE_TABLE = 'pose_table'
KEY_POSE_SOFA = 'pose_sofa'

def create_demo():
    root = py_trees.composites.Sequence("Intelligent demo", memory=True)

    root.add_child(BtNode_Announce(name="Announce start", bb_source=None, message="Hi, I'm Tinker, you can instruct me to do tasks for you"))
    root.add_child(BtNode)
