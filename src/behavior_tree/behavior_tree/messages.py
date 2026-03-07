"""
Message imports for behavior_tree package.
Conditionally imports real or mock messages based on availability.
"""
from behavior_tree.config import get_config

# Get configuration
_config = get_config()

# Import based on availability
if _config.has_dependency('tinker_vision_msgs'):
    from tinker_vision_msgs.srv import ObjectDetection, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, FollowHead, DetectWaving
    from tinker_vision_msgs.msg import Object, PanTiltCtrl
    from tinker_vision_msgs.action import Categorize, FollowHeadAction, HumanFollowing
else:
    from behavior_tree.mock_messages import ObjectDetection, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, FollowHead, DetectWaving
    from behavior_tree.mock_messages import Object, PanTiltCtrl
    from behavior_tree.mock_messages import Categorize, FollowHeadAction, HumanFollowing

if _config.has_dependency('tinker_arm_msgs'):
    from tinker_arm_msgs.srv import Drop, ArmJointService, PointTo
    from tinker_arm_msgs.action import Place, Grasp, JointMove, CartesianMove
else:
    from behavior_tree.mock_messages import Drop, ArmJointService, PointTo
    from behavior_tree.mock_messages import Place, Grasp, JointMove, CartesianMove

if _config.has_dependency('tinker_audio_msgs'):
    from tinker_audio_msgs.srv import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
else:
    from behavior_tree.mock_messages import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest

if _config.has_dependency('tinker_nav_msgs'):
    from tinker_nav_msgs.srv import SetLuggagePose, ComputeGrasp
else:
    from behavior_tree.mock_messages import SetLuggagePose, ComputeGrasp

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