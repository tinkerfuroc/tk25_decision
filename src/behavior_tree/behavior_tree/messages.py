from tinker_vision_msgs.srv import ObjectDetection
# from tinker_decision_msgs.srv import ObjectDetection
from tk_nav_interfaces.srv import Goto, GotoGrasp, RelToAbs
# from tinker_decision_msgs.srv import Goto, GotoGrasp, RelToAbs
# from tinker_decision_msgs.srv import Drop
# from tinker_decision_msgs.srv import Grasp
from tinker_arm_msgs.srv import Grasp, Drop
# from tinker_decision_msgs.srv import Announce, WaitForStart
from tinker_audio_msgs.srv import TextToSpeech, WaitForStart

from compute_grasp_interface.srv import ComputeGrasp
from nav2_msgs.action import NavigateToPose

import rclpy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from tinker_arm_msgs.srv import ArmJointService
from std_msgs.msg import Header
from tinker_vision_msgs.msg import Object