"""
Mock implementations for ROS2 service and action interfaces.
These are used when tinker packages are not available.
"""


# Mock action_msgs.msg module (for GoalStatus)
class MockGoalStatus:
    """Mock GoalStatus constants."""
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class MockActionMsgsModule:
    """Mock action_msgs.msg module."""
    GoalStatus = MockGoalStatus
    
    # Add as module attribute for compatibility
    STATUS_UNKNOWN = 0
    STATUS_ACCEPTED = 1
    STATUS_EXECUTING = 2
    STATUS_CANCELING = 3
    STATUS_SUCCEEDED = 4
    STATUS_CANCELED = 5
    STATUS_ABORTED = 6


class MockService:
    """Base class for mock service types."""
    
    class Request:
        """Mock service request."""
        def __init__(self):
            pass
    
    class Response:
        """Mock service response."""
        def __init__(self):
            self.status = 0
            self.error_msg = ""
            self.success = True


class MockAction:
    """Base class for mock action types."""
    
    class Goal:
        """Mock action goal."""
        def __init__(self):
            pass
    
    class Result:
        """Mock action result."""
        def __init__(self):
            self.success = True
            self.error_msg = ""
    
    class Feedback:
        """Mock action feedback."""
        def __init__(self):
            pass


class MockMessage:
    """Base class for mock message types."""
    def __init__(self):
        pass


# Mock Vision Services
class ObjectDetection(MockService):
    """Mock ObjectDetection service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.prompt = ""
            self.flags = ""
            self.category = ""
            self.camera = ""
            self.target_frame = ""
    
    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.objects = []
            self.image_width = 0
            self.image_height = 0


class ObjectDetectionGeneralist(MockService):
    """Mock for tk26 generalist ObjectDetection — boolean flags, detection_source tag."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.camera = ""
            self.prompt = ""
            self.target_frame = ""
            self.sort_closest = False
            self.sort_highest = False
            self.return_rgb_image = False
            self.return_depth_image = False
            self.return_segments = False
            self.force_vlm_sam = False
            self.use_vlm_sam_fallback = False

    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.person_id = 0
            self.objects = []
            self.detection_source = "none"


class FeatureExtraction(MockService):
    """Mock FeatureExtraction service."""
    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.features = []


class SeatRecommendation(MockService):
    """Mock SeatRecommendation service."""
    pass


class SeatRecommendBbox(MockService):
    """Mock SeatRecommendBbox service (recommendation + bbox + 3D centroid)."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.camera = ""
            self.names = []
            self.features = []
            self.target_frame = ""

    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.recommendation = ""
            self.bbox = BoundingBox()
            self.centroid = None


class BoundingBox(MockMessage):
    """Mock BoundingBox message — xmin/ymin/xmax/ymax pixel ints."""
    def __init__(self, xmin: int = 0, ymin: int = 0, xmax: int = 0, ymax: int = 0):
        super().__init__()
        self.xmin = xmin
        self.ymin = ymin
        self.xmax = xmax
        self.ymax = ymax


class FeatureMatching(MockService):
    """Mock FeatureMatching service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.features = []
            self.persons = []
    
    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.matched_ids = []
            self.centroids = []


class GetPointCloud(MockService):
    """Mock GetPointCloud service."""
    pass


class DoorDetection(MockService):
    """Mock DoorDetection service."""
    pass


class FollowHead(MockService):
    """Mock FollowHead service."""
    pass


class DetectWaving(MockService):
    """Mock DetectWaving service."""
    pass


class PlacingLocation(MockService):
    """Mock PlacingLocation service."""
    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.candidate_points = []
            self.candidate_bboxes = []


# Mock Arm Services
class Drop(MockService):
    """Mock Drop service."""
    pass


class ArmJointService(MockService):
    """Mock ArmJointService."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.joint0 = 0.0
            self.joint1 = 0.0
            self.joint2 = 0.0
            self.joint3 = 0.0
            self.joint4 = 0.0
            self.joint5 = 0.0
            self.joint6 = 0.0
            self.add_octomap = False


class PointTo(MockService):
    """Mock PointTo service."""
    pass


# Mock Arm Actions
class Place(MockAction):
    """Mock Place action."""
    pass


class Grasp(MockAction):
    """Mock Grasp action."""
    pass


class JointMove(MockAction):
    """Mock JointMove action."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.joint0 = 0.0
            self.joint1 = 0.0
            self.joint2 = 0.0
            self.joint3 = 0.0
            self.joint4 = 0.0
            self.joint5 = 0.0
            self.joint6 = 0.0
            self.env_points = []


class FoldClothing(MockAction):
    """Mock FoldClothing action."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.target_point = None
            self.object_label = ""
            self.fold_cycles = 0
            self.env_points = []

    class Result(MockAction.Result):
        def __init__(self):
            super().__init__()
            self.success = False
            self.error_msg = ""


class CartesianMove(MockAction):
    """Mock CartesianMove action."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.env_points = []
            self.target_pose = None


# Mock Audio Services
class TTSCnRequest(MockService):
    """Mock TTS Chinese Request service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.input_text = ""


class TextToSpeech(MockService):
    """Mock TextToSpeech service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.text = ""


class WaitForStart(MockService):
    """Mock WaitForStart service."""
    pass


class PhraseExtraction(MockService):
    """Mock PhraseExtraction service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.wordlist = []
            self.timeout = 0.0

    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.result = ""


class PhraseExtractionAction(MockAction):
    """Mock PhraseExtraction action (tk_24_audio `phrase_extraction_action`)."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.timeout = 0.0
            self.wordlist = []

    class Result(MockAction.Result):
        def __init__(self):
            super().__init__()
            self.status = 0
            self.error_message = ""
            self.phrase = ""

    class Feedback(MockAction.Feedback):
        def __init__(self):
            super().__init__()
            self.progress = 0.0
            self.status_message = ""
            self.partial_transcription = ""


class GetConfirmation(MockService):
    """Mock GetConfirmation service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.timeout = 0.0
    
    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.confirmed = True


class Listen(MockService):
    """Mock Listen service."""
    pass


class CompareInterest(MockService):
    """Mock CompareInterest service."""
    pass


class QuestionAnswer(MockService):
    """Mock QuestionAnswer service."""
    pass


class GraspRequest(MockService):
    """Mock GraspRequest service."""
    pass


# Mock Navigation Services
class SetLuggagePose(MockService):
    """Mock SetLuggagePose service."""
    pass


class ComputeGrasp(MockService):
    """Mock ComputeGrasp service."""
    pass


class OrientationAngle(MockService):
    """Mock OrientationAngle service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.max_try = 0
            self.timeout = 0.0

    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.angle = 0.0


class FindApproachPose(MockService):
    """Mock FindApproachPose service."""
    class Request(MockService.Request):
        def __init__(self):
            super().__init__()
            self.target = None
            self.desired_distance = 0.0
            self.min_distance = 0.0
            self.max_distance = 0.0
            self.num_angles = 0
            self.check_reachability = False
            self.preferred_yaw_rad = float('nan')
            self.facing_yaw_offset_rad = 0.0
            self.timeout_sec = 0.0
            self.robot_pose_override = None

    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.status = 0
            self.errormsg = ""
            self.pose = None
            self.chosen_distance = 0.0
            self.score = 0.0
            self.reachable = False
            self.alternates = []
            self.angle_to_target_rad = 0.0


# Mock Vision Messages
class Object(MockMessage):
    """Mock Object message."""
    def __init__(self):
        super().__init__()
        self.class_name = ""
        self.confidence = 0.0
        self.center = None


class PanTiltCtrl(MockMessage):
    """Mock PanTiltCtrl message."""
    def __init__(self):
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.speed = 0.0


class PanTiltCommand(MockMessage):
    """Mock PanTiltCommand message."""
    ABSOLUTE = 0
    RELATIVE = 1
    def __init__(self):
        super().__init__()
        self.mode = 0
        self.pan_rad = 0.0
        self.tilt_rad = 0.0
        self.speed_raw = 0
        self.accel_raw = 0


class PanTiltState(MockMessage):
    """Mock PanTiltState message."""
    def __init__(self):
        super().__init__()
        self.pan_rad = 0.0
        self.tilt_rad = 0.0
        self.connected = False
        self.feedback_ok = False


# Mock Vision Actions
class Categorize(MockAction):
    """Mock Categorize action."""
    pass


class FollowHeadAction(MockAction):
    """Mock FollowHeadAction."""
    pass


class HumanFollowing(MockAction):
    """Mock HumanFollowing action."""
    pass


class GetConfirmationAction(MockAction):
    """Mock GetConfirmation action from tinker_audio_msgs."""
    pass


class ListenAction(MockAction):
    """Mock Listen action from tinker_audio_msgs."""
    pass


class Doorbell(MockAction):
    """Mock Doorbell action from tinker_audio_msgs."""

    class Result(MockAction.Result):
        def __init__(self):
            super().__init__()
            self.status = 0
            self.error_message = ""
            self.transcription = ""

    class Feedback(MockAction.Feedback):
        def __init__(self):
            super().__init__()
            self.elapsed = 0.0
            self.partial_transcription = ""


class TrackPerson(MockAction):
    """Mock TrackPerson action (tk26_vision)."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.target_frame = ""
            self.target_point_topic = ""
            self.return_rgb_img = False
            self.return_depth_img = False
            self.return_segment = False
            self.debug = False

    class Feedback(MockAction.Feedback):
        def __init__(self):
            super().__init__()
            self.target_lost = False
            self.target_track_id = -1
            self.is_transformation_successful = False
            self.target_position = None


class Follow(MockAction):
    """Mock Follow action (tk26 tinker_nav_msgs)."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.timeout = 2.0

    class Result(MockAction.Result):
        def __init__(self):
            super().__init__()
            self.result = 1

    class Feedback(MockAction.Feedback):
        def __init__(self):
            super().__init__()
            self.status = ""
            self.point_header = None
            self.nav_goal_header = None


# Mock Navigation Actions (from nav2)
class NavigateToPose(MockAction):
    """Mock NavigateToPose action."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.pose = None


# Mock Control Actions (from control_msgs)
class GripperCommand(MockAction):
    """Mock GripperCommand action."""
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.command = None
    
    class Command:
        """Mock gripper command."""
        def __init__(self):
            self.position = 0.0
            self.max_effort = 0.0
