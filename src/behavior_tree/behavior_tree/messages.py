"""
Message imports for behavior_tree package.
Conditionally imports real or mock messages based on availability.
"""
from behavior_tree.config import get_config

# Get configuration
_config = get_config()

# Import based on availability
if _config.has_dependency('tinker_vision_msgs'):
    from tinker_vision_msgs.srv import ObjectDetection, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, FollowHead
    from tinker_vision_msgs.msg import Object, PanTiltCtrl
    from tinker_vision_msgs.action import Categorize, FollowHeadAction, HumanFollowing
else:
    from behavior_tree.mock_messages import ObjectDetection, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, FollowHead
    from behavior_tree.mock_messages import Object, PanTiltCtrl
    from behavior_tree.mock_messages import Categorize, FollowHeadAction, HumanFollowing

if _config.has_dependency('tinker_vision_msgs_26'):
    from tinker_vision_msgs_26.srv import DetectWaving
    from tinker_vision_msgs_26.action import TrackPerson
else:
    from behavior_tree.mock_messages import DetectWaving
    from behavior_tree.mock_messages import TrackPerson

if _config.has_dependency('tinker_arm_msgs'):
    from tinker_arm_msgs.srv import Drop, ArmJointService, PointTo
    from tinker_arm_msgs.action import Place, Grasp, JointMove, CartesianMove
else:
    from behavior_tree.mock_messages import Drop, ArmJointService, PointTo
    from behavior_tree.mock_messages import Place, Grasp, JointMove, CartesianMove

if _config.has_dependency('tinker_audio_msgs'):
    from tinker_audio_msgs.srv import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
    from tinker_audio_msgs.action import GetConfirmation as GetConfirmationAction
    from tinker_audio_msgs.action import Listen as ListenAction
else:
    from behavior_tree.mock_messages import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
    from behavior_tree.mock_messages import GetConfirmation as GetConfirmationAction
    from behavior_tree.mock_messages import Listen as ListenAction

if _config.has_dependency('tinker_nav_msgs'):
    from tinker_nav_msgs.srv import SetLuggagePose, ComputeGrasp
else:
    from behavior_tree.mock_messages import SetLuggagePose, ComputeGrasp

# `Follow` lives only in tk26_ws's tinker_nav_msgs; the tk25_ws package of the
# same name ships srv-only. Import defensively so BT builds even when only
# tk25 is sourced — the mock stub still exercises the tree shape.
try:
    from tinker_nav_msgs.action import Follow
except (ImportError, ModuleNotFoundError):
    from behavior_tree.mock_messages import Follow

if _config.has_dependency('nav2_msgs'):
    from nav2_msgs.action import NavigateToPose
else:
    from behavior_tree.mock_messages import NavigateToPose

if _config.has_dependency('control_msgs'):
    from control_msgs.action import GripperCommand
else:
    from behavior_tree.mock_messages import GripperCommand

if _config.has_dependency('action_msgs'):
    import action_msgs.msg as action_msgs
else:
    from behavior_tree.mock_messages import MockActionMsgsModule as action_msgs

# Standard ROS2 imports (always available)
import rclpy
from geometry_msgs.msg import Point, PointStamped, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

# Print status on import
if _config.is_mock_mode():
    print("\n" + "="*60)
    print("⚠️  Behavior Tree: Running with Mock Messages")
    print("="*60)
    _config.print_status()
    print()