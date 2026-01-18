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


class FeatureExtraction(MockService):
    """Mock FeatureExtraction service."""
    class Response(MockService.Response):
        def __init__(self):
            super().__init__()
            self.features = []


class SeatRecommendation(MockService):
    """Mock SeatRecommendation service."""
    pass


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
