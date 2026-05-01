"""
Message imports for behavior_tree package.
Conditionally imports real or mock messages based on availability.
"""
from behavior_tree.config import get_config

# Get configuration
_config = get_config()

# Import based on availability
if _config.has_dependency('tinker_vision_msgs_26'):
    from tinker_vision_msgs_26.srv import ObjectDetection, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, FollowHead, DetectWaving
    from tinker_vision_msgs_26.srv import ObjectDetectionGeneralist, SeatRecommendBbox, PlacingLocation, GetImage
    from tinker_vision_msgs_26.msg import Object, PanTiltCtrl, PanTiltCommand, PanTiltState, BoundingBox
    from tinker_vision_msgs_26.action import Categorize, FollowHeadAction, HumanFollowing, TrackPerson
else:
    from behavior_tree.mock_messages import ObjectDetection, FeatureExtraction, SeatRecommendation, FeatureMatching, GetPointCloud, DoorDetection, FollowHead, DetectWaving
    from behavior_tree.mock_messages import ObjectDetectionGeneralist, SeatRecommendBbox, PlacingLocation
    from behavior_tree.mock_messages import Object, PanTiltCtrl, PanTiltCommand, PanTiltState, BoundingBox
    from behavior_tree.mock_messages import Categorize, FollowHeadAction, HumanFollowing, TrackPerson

if _config.has_dependency('tinker_arm_msgs'):
    from tinker_arm_msgs.srv import Drop, ArmJointService, PointTo
    from tinker_arm_msgs.action import Place, Grasp, JointMove, CartesianMove
    try:
        from tinker_arm_msgs.action import FoldClothing
    except ImportError:
        from behavior_tree.mock_messages import FoldClothing
else:
    from behavior_tree.mock_messages import Drop, ArmJointService, PointTo
    from behavior_tree.mock_messages import Place, Grasp, JointMove, CartesianMove
    from behavior_tree.mock_messages import FoldClothing

if _config.has_dependency('tinker_audio_msgs'):
    from tinker_audio_msgs.srv import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
    from tinker_audio_msgs.action import GetConfirmation as GetConfirmationAction, Listen as ListenAction
    from tinker_audio_msgs.action import PhraseExtraction as PhraseExtractionAction
else:
    from behavior_tree.mock_messages import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
    from behavior_tree.mock_messages import GetConfirmationAction, ListenAction
    from behavior_tree.mock_messages import PhraseExtractionAction

if _config.has_dependency('tinker_nav_msgs'):
    from tinker_nav_msgs.srv import SetLuggagePose, ComputeGrasp
    # from tinker_nav_msgs.srv import FindApproachPose
else:
    from behavior_tree.mock_messages import SetLuggagePose, ComputeGrasp, FindApproachPose

# `Follow` lives only in tk26_ws's tinker_nav_msgs; the tk25_ws package of the
# same name ships srv-only. Import defensively so BT builds even when only
# tk25 is sourced — the mock stub still exercises the tree shape.
try:
    from tinker_nav_msgs.action import Follow
except (ImportError, ModuleNotFoundError):
    from behavior_tree.mock_messages import Follow

if _config.has_dependency('nav2_msgs'):
    from nav2_msgs.action import NavigateToPose
    print("Using real nav2_msgs NavigateToPose")
else:
    from behavior_tree.mock_messages import NavigateToPose
    print("WARNING: nav2_msgs not found, using mock NavigateToPose !!!")

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