"""GPSR small trees.

Each ``create_<action>()`` returns a self-contained ``py_trees.composites.Sequence``
implementing one atomic action that the RoboCup GPSR command generator can ask
the robot to do. Small trees read their parameters from the shared GPSR
blackboard keys (see ``bb_keys`` below) so they can be either dispatched by the
orchestrator or driven standalone from a dev test script that pre-fills the
blackboard.

The factories deliberately stay shallow: they compose primitives from
``TemplateNodes`` (vision / nav / audio / manipulation) and rely on the
orchestrator for retries and re-planning.
"""

import math
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.blackboard import Blackboard
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion
from std_msgs.msg import Header
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard, BtNode_WaitTicks
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_ConvertGraspPose
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
)
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_MoveArmSingle,
    BtNode_GripperAction,
)

from behavior_tree.StoringGroceries.customNodes import (
    BtNode_FindObjTable,
    BtNode_GraspWithPose,
)

from .custom_nodes import (
    BtNode_QA,
    BtNode_ScanForWavingPersonNew,
)


# ---------------------------------------------------------------------------
# Blackboard keys (single source of truth, shared with orchestrator)
# ---------------------------------------------------------------------------

class bb_keys:
    # Orchestrator state
    COMMAND = "gpsr/command"
    PLAN = "gpsr/plan"
    PLAN_INDEX = "gpsr/plan_index"
    CURRENT_ACTION = "gpsr/current_action"
    CURRENT_PARAMS = "gpsr/current_params"
    STATE_LOG = "gpsr/state_log"
    CORRECTION_COUNT = "gpsr/correction_count"
    LAST_FAILURE = "gpsr/last_failure"

    # Per-action working keys (filled by orchestrator just before dispatch)
    TARGET_POSE = "gpsr/target_pose"            # PoseStamped
    TARGET_LOCATION = "gpsr/target_location"    # str
    TARGET_OBJECT_NAME = "gpsr/target_object_name"     # str
    TARGET_OBJECT_PROMPT = "gpsr/target_object_prompt" # str (vision prompt)
    TARGET_OBJECT_DETECTION = "gpsr/target_object_detection"  # vision result
    TARGET_OBJECT = "gpsr/target_object"        # vision Object pick used by grasp
    TARGET_PERSON_PROMPT = "gpsr/target_person_prompt"  # str
    TARGET_PERSON_POSE = "gpsr/target_person_pose"      # PoseStamped/PointStamped
    TARGET_PERSON_DETECTION = "gpsr/target_person_detection"
    ALL_WAVING_PERSONS = "gpsr/all_waving_persons"
    ANNOUNCE_TEXT = "gpsr/announce_text"
    QA_ANSWER = "gpsr/qa_answer"
    COUNT_VALUE = "gpsr/count_value"

    # Manipulation working keys (re-using the conventions from gpsr_new.py)
    TABLE_IMG = "gpsr/table_img"
    OBJ_SEG = "gpsr/object_segmentation"
    GRASP_POSE = "gpsr/grasp_pose"
    GRASP_ANNOUNCEMENT = "gpsr/grasp_announcement"
    ARM_NAVIGATING = "gpsr/arm_navigating"
    ARM_SCAN = "gpsr/arm_scan"


ARM_SERVICE_NAME = "arm_joint_service"
GRASP_SERVICE_NAME = "start_grasp"
WAVING_THRESHOLD_METERS = 6.0


# ---------------------------------------------------------------------------
# Tiny utility behaviours used inside the small trees
# ---------------------------------------------------------------------------

class BtNode_BlackboardSet(Behaviour):
    """Set ``key = value`` on the blackboard each tick. Always SUCCESS."""

    def __init__(self, name: str, key: str, value):
        super().__init__(name)
        self._key = key
        self._value = value
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._key, access=Access.WRITE)

    def update(self):
        self._client.set(self._key, self._value, overwrite=True)
        return Status.SUCCESS


class BtNode_AnnounceFromBB(Behaviour):
    """Speak the contents of a blackboard key by delegating to BtNode_Announce.

    Wraps BtNode_Announce so the small trees can keep parameters on the
    blackboard without each owning their own announce instance with a fixed
    message.
    """

    def __init__(self, name: str, bb_text_key: str, prefix: str = ""):
        super().__init__(name)
        self._bb_text_key = bb_text_key
        self._prefix = prefix
        self._inner = None
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._bb_text_key, access=Access.READ)
        self._client.register_key(bb_keys.ANNOUNCE_TEXT, access=Access.WRITE)
        self._inner = BtNode_Announce(
            name=self.name + "/inner",
            bb_source=bb_keys.ANNOUNCE_TEXT,
        )
        self._inner.setup(**kwargs)

    def initialise(self):
        try:
            text = self._client.get(self._bb_text_key)
        except Exception:
            text = ""
        text = str(text) if text is not None else ""
        self._client.set(bb_keys.ANNOUNCE_TEXT, f"{self._prefix}{text}", overwrite=True)
        self._inner.initialise()

    def update(self):
        return self._inner.update()

    def terminate(self, new_status):
        self._inner.terminate(new_status)


class BtNode_ExtractDetection(Behaviour):
    """Pick the closest object out of a generalist scan result and write it.

    Reads the ``ObjectDetectionGeneralist.Response`` left at ``bb_detection_src``,
    grabs ``objects[0]`` (the generalist already supports ``sort_closest``), and
    writes the picked object + a PointStamped of its centroid for downstream
    nodes. Returns FAILURE if the scan produced no objects.
    """

    def __init__(
        self,
        name: str,
        bb_detection_src: str,
        bb_object_dst: str,
        bb_point_dst: str,
        target_frame: str = "map",
    ):
        super().__init__(name)
        self._src = bb_detection_src
        self._object_dst = bb_object_dst
        self._point_dst = bb_point_dst
        self._target_frame = target_frame
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._object_dst, access=Access.WRITE)
        self._client.register_key(self._point_dst, access=Access.WRITE)

    def update(self):
        try:
            result = self._client.get(self._src)
        except Exception as exc:
            self.feedback_message = f"No detection result in {self._src}: {exc}"
            return Status.FAILURE

        objects = getattr(result, "objects", None) or []
        if not objects:
            self.feedback_message = "Scan returned 0 objects"
            return Status.FAILURE

        picked = objects[0]
        self._client.set(self._object_dst, picked, overwrite=True)

        header = getattr(result, "header", None)
        if header is None:
            header = Header(frame_id=self._target_frame, stamp=rclpy.time.Time().to_msg())
        point_stamped = PointStamped(header=header, point=picked.centroid)
        self._client.set(self._point_dst, point_stamped, overwrite=True)
        self.feedback_message = (
            f"Picked detection at ({picked.centroid.x:.2f}, {picked.centroid.y:.2f})"
        )
        return Status.SUCCESS


class BtNode_PointToPoseStamped(Behaviour):
    """Convert a PointStamped on the blackboard to a PoseStamped facing the point.

    Used so small trees can hand a navigation-ready goal to BtNode_GotoAction
    when vision only gives them a PointStamped.
    """

    def __init__(self, name: str, bb_point_key: str, bb_pose_key: str):
        super().__init__(name)
        self._point_key = bb_point_key
        self._pose_key = bb_pose_key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._point_key, access=Access.READ)
        self._client.register_key(self._pose_key, access=Access.WRITE)

    def update(self):
        try:
            point = self._client.get(self._point_key)
        except Exception as exc:
            self.feedback_message = f"Missing point key: {exc}"
            return Status.FAILURE

        if isinstance(point, PoseStamped):
            self._client.set(self._pose_key, point, overwrite=True)
            return Status.SUCCESS
        if not isinstance(point, PointStamped):
            self.feedback_message = f"Unsupported type for {self._point_key}: {type(point)}"
            return Status.FAILURE

        # Build a PoseStamped at the target location with identity orientation.
        pose = PoseStamped(
            header=Header(stamp=rclpy.time.Time().to_msg(), frame_id=point.header.frame_id or "map"),
            pose=Pose(
                position=Point(x=point.point.x, y=point.point.y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )
        self._client.set(self._pose_key, pose, overwrite=True)
        return Status.SUCCESS


class BtNode_CountDetections(Behaviour):
    """Count objects in a stored detection result and write the integer."""

    def __init__(self, name: str, bb_detection_src: str, bb_count_dst: str):
        super().__init__(name)
        self._src = bb_detection_src
        self._dst = bb_count_dst
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._dst, access=Access.WRITE)

    def update(self):
        try:
            result = self._client.get(self._src)
        except Exception as exc:
            self.feedback_message = f"Missing detection result: {exc}"
            return Status.FAILURE
        count = len(getattr(result, "objects", []) or [])
        self._client.set(self._dst, count, overwrite=True)
        self.feedback_message = f"counted {count} objects"
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Small-tree factories
# ---------------------------------------------------------------------------

def create_goto():
    """Navigate to ``bb_keys.TARGET_POSE`` (filled by orchestrator)."""
    seq = py_trees.composites.Sequence("small/goto", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce going", bb_keys.TARGET_LOCATION, prefix="Going to "
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry goto",
        BtNode_GotoAction("goto target", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    return seq


def create_find_object():
    """Scan for the object named in ``bb_keys.TARGET_OBJECT_PROMPT`` and store the closest hit."""
    seq = py_trees.composites.Sequence("small/find_object", memory=True)
    seq.add_child(BtNode_TurnPanTilt("turn pantilt down", x=0.0, y=20.0))
    seq.add_child(BtNode_ScanForGeneralist(
        name="generalist scan",
        bb_source=bb_keys.TARGET_OBJECT_PROMPT,
        bb_key=bb_keys.TARGET_OBJECT_DETECTION,
        use_orbbec=True,
        transform_to_map=True,
        use_vlm_sam_fallback=True,
        sort_closest=True,
        return_segments=True,
    ))
    seq.add_child(BtNode_ExtractDetection(
        "pick closest object",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_object_dst=bb_keys.TARGET_OBJECT,
        bb_point_dst=bb_keys.TARGET_PERSON_POSE,  # reuse for now, overwritten if person flow runs
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "announce found", bb_keys.TARGET_OBJECT_NAME, prefix="I can see the "
    ))
    return seq


def create_find_person():
    """Scan for a person matching ``bb_keys.TARGET_PERSON_PROMPT`` and store their pose.

    Uses the waving-person service when the prompt mentions "wav" (a common
    GPSR descriptor); otherwise falls back to the generalist with prompt
    "person".
    """
    seq = py_trees.composites.Sequence("small/find_person", memory=True)
    seq.add_child(BtNode_TurnPanTilt("look forward", x=0.0, y=10.0))
    seq.add_child(BtNode_Announce(
        "announce searching", bb_source=None,
        message="Looking for a person, please stay still.",
    ))

    selector = py_trees.composites.Selector("person scan strategies", memory=False)
    # waving-person specialist
    selector.add_child(BtNode_ScanForWavingPersonNew(
        "find waving persons",
        bb_keys.ALL_WAVING_PERSONS,
        bb_keys.TARGET_PERSON_POSE,
        WAVING_THRESHOLD_METERS,
        target_frame="map",
    ))
    # generalist fallback
    generalist_branch = py_trees.composites.Sequence("generalist person scan", memory=True)
    generalist_branch.add_child(BtNode_ScanForGeneralist(
        name="generalist person scan",
        bb_source=None,
        bb_key=bb_keys.TARGET_PERSON_DETECTION,
        object="person",
        use_orbbec=True,
        transform_to_map=True,
        sort_closest=True,
    ))
    generalist_branch.add_child(BtNode_ExtractDetection(
        "pick closest person",
        bb_detection_src=bb_keys.TARGET_PERSON_DETECTION,
        bb_object_dst=bb_keys.TARGET_OBJECT,
        bb_point_dst=bb_keys.TARGET_PERSON_POSE,
    ))
    selector.add_child(generalist_branch)
    seq.add_child(selector)
    seq.add_child(BtNode_Announce(
        "announce found person", bb_source=None,
        message="Found a person.",
    ))
    return seq


def create_follow():
    """Follow the person currently being tracked.

    Uses the action-based BtNode_TrackPersonAction since the production
    HelpMeCarry tree already proves this path. Caller is responsible for
    cancelling/terminating the action when navigation should stop.
    """
    from behavior_tree.TemplateNodes.TrackPersonAction import BtNode_TrackPersonAction
    seq = py_trees.composites.Sequence("small/follow", memory=True)
    seq.add_child(BtNode_Announce(
        "announce follow", bb_source=None,
        message="I will follow you. Please walk slowly.",
    ))
    seq.add_child(BtNode_TrackPersonAction(
        name="track person",
        target_frame="map",
    ))
    return seq


def create_guide():
    """Lead a person to ``bb_keys.TARGET_POSE`` while announcing instructions."""
    seq = py_trees.composites.Sequence("small/guide", memory=True)
    seq.add_child(BtNode_Announce(
        "announce guide", bb_source=None,
        message="Please follow me. I will guide you to your destination.",
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry guide goto",
        BtNode_GotoAction("guide to target", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "arrived announce", bb_keys.TARGET_LOCATION, prefix="We have arrived at "
    ))
    return seq


def create_greet():
    """Locate a person (closest), then speak a greeting that names them."""
    seq = py_trees.composites.Sequence("small/greet", memory=True)
    seq.add_child(create_find_person())
    seq.add_child(BtNode_AnnounceFromBB(
        "greet announce", bb_keys.TARGET_PERSON_PROMPT, prefix="Hello "
    ))
    return seq


def create_grasp():
    """Pick up the object referenced by the vision flow.

    Mirrors gpsr_new.py:createGrasp() but reads BB keys from the GPSR namespace
    rather than the legacy keys, and falls back to an ex-machina ask-the-referee
    branch on hardware failure.
    """
    primary = py_trees.composites.Sequence("grasp/primary", memory=True)
    primary.add_child(BtNode_TurnPanTilt("turn pantilt down", x=0.0, y=20.0))
    primary.add_child(BtNode_MoveArmSingle(
        "arm to navigating",
        service_name=ARM_SERVICE_NAME,
        arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        add_octomap=False,
    ))
    par_scan = py_trees.composites.Parallel(
        "arm to scan + announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par_scan.add_child(BtNode_Announce(
        "say move arm", bb_source=None,
        message="Moving my arm to look for the object",
    ))
    par_scan.add_child(BtNode_MoveArmSingle(
        "arm to scan",
        service_name=ARM_SERVICE_NAME,
        arm_pose_bb_key=bb_keys.ARM_SCAN,
        add_octomap=True,
    ))
    primary.add_child(par_scan)

    find_and_grasp = py_trees.composites.Sequence("find and grasp", memory=True)
    find_and_grasp.add_child(BtNode_FindObjTable(
        "find object on table",
        bb_keys.TARGET_OBJECT_NAME,
        bb_keys.TABLE_IMG,
        bb_keys.OBJ_SEG,
        bb_keys.TARGET_OBJECT,
        bb_keys.GRASP_ANNOUNCEMENT,
    ))
    par_grasp = py_trees.composites.Parallel(
        "grasp+announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par_grasp.add_child(BtNode_Announce("say grasp", bb_source=bb_keys.GRASP_ANNOUNCEMENT))
    par_grasp.add_child(BtNode_GraspWithPose(
        "grasp object",
        bb_key_vision_res=bb_keys.TARGET_OBJECT,
        bb_key_pose=bb_keys.GRASP_POSE,
        action_name=GRASP_SERVICE_NAME,
    ))
    find_and_grasp.add_child(par_grasp)
    primary.add_child(find_and_grasp)
    primary.add_child(py_trees.decorators.Retry(
        "retry arm back",
        BtNode_MoveArmSingle(
            "arm back to navigating",
            service_name=ARM_SERVICE_NAME,
            arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        ),
        num_failures=5,
    ))

    primary_with_retry = py_trees.decorators.Retry(
        "retry primary 3x", primary, num_failures=3,
    )

    ex_machina = py_trees.composites.Sequence("grasp/ex_machina", memory=True)
    ex_machina.add_child(py_trees.decorators.Retry(
        "retry arm back",
        BtNode_MoveArmSingle(
            "arm to navigating",
            service_name=ARM_SERVICE_NAME,
            arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        ),
        num_failures=5,
    ))
    ex_machina.add_child(BtNode_GripperAction("open gripper", True))
    ex_machina.add_child(BtNode_AnnounceFromBB(
        "ask referee", bb_keys.TARGET_OBJECT_NAME, prefix="Dear referee, please help me grasp the "
    ))
    ex_machina.add_child(BtNode_Announce(
        "say put in gripper", bb_source=None,
        message="Put it in my gripper please. Thank you.",
    ))
    ex_machina.add_child(BtNode_WaitTicks("wait", 8))
    ex_machina.add_child(BtNode_GripperAction("close gripper", False))

    return py_trees.composites.Selector(
        "small/grasp",
        memory=True,
        children=[primary_with_retry, ex_machina],
    )


def create_place():
    """Place the currently-held object at ``bb_keys.TARGET_POSE`` / handover.

    The minimum-viable implementation: navigate to the place location, then
    open the gripper while announcing the action. Real placement-by-vision is
    out of scope for the small-tree layer.
    """
    seq = py_trees.composites.Sequence("small/place", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce placing", bb_keys.TARGET_LOCATION, prefix="Placing the item at "
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry place goto",
        BtNode_GotoAction("goto place pose", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    seq.add_child(BtNode_Announce(
        "say releasing", bb_source=None, message="Releasing object now."
    ))
    seq.add_child(BtNode_GripperAction("open gripper", True))
    seq.add_child(BtNode_WaitTicks("wait", 4))
    seq.add_child(BtNode_GripperAction("close gripper", False))
    return seq


def create_deliver():
    """Bring the currently-held object to a person at ``bb_keys.TARGET_POSE``."""
    seq = py_trees.composites.Sequence("small/deliver", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce delivering", bb_keys.TARGET_OBJECT_NAME, prefix="Delivering the "
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry deliver goto",
        BtNode_GotoAction("goto recipient", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    seq.add_child(BtNode_Announce(
        "say take it", bb_source=None,
        message="Here you go, please take the object from my gripper.",
    ))
    seq.add_child(BtNode_WaitTicks("wait for take", 8))
    seq.add_child(BtNode_GripperAction("open gripper", True))
    seq.add_child(BtNode_WaitTicks("wait", 4))
    seq.add_child(BtNode_GripperAction("close gripper", False))
    return seq


def create_count():
    """Count objects matching ``bb_keys.TARGET_OBJECT_PROMPT`` and announce the number."""
    seq = py_trees.composites.Sequence("small/count", memory=True)
    seq.add_child(BtNode_TurnPanTilt("turn pantilt", x=0.0, y=10.0))
    seq.add_child(BtNode_Announce(
        "say scanning", bb_source=None, message="I am counting now."
    ))
    seq.add_child(BtNode_ScanForGeneralist(
        name="scan to count",
        bb_source=bb_keys.TARGET_OBJECT_PROMPT,
        bb_key=bb_keys.TARGET_OBJECT_DETECTION,
        use_orbbec=True,
        transform_to_map=False,
        use_vlm_sam_fallback=True,
        sort_closest=True,
        return_segments=False,
    ))
    seq.add_child(BtNode_CountDetections(
        "count detections",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "say count", bb_keys.COUNT_VALUE, prefix="I counted "
    ))
    return seq


def create_answer_question():
    """Listen to a question and answer via the question_answer_service."""
    seq = py_trees.composites.Sequence("small/answer_question", memory=True)
    seq.add_child(BtNode_Announce(
        "say ask", bb_source=None,
        message="Please ask me a question after the beep.",
    ))
    seq.add_child(BtNode_WaitTicks("beep wait", 6))
    seq.add_child(BtNode_QA("qa", bb_key_dest=bb_keys.QA_ANSWER, timeout=10.0))
    seq.add_child(BtNode_AnnounceFromBB(
        "say answer", bb_keys.QA_ANSWER, prefix=""
    ))
    return seq


def create_tell_info():
    """Speak the text already placed in ``bb_keys.ANNOUNCE_TEXT`` by the orchestrator.

    For ``tell`` commands (time, day, team info, person info), the orchestrator
    resolves the topic into a sentence string and writes it to the announce
    blackboard before dispatching this tree.
    """
    seq = py_trees.composites.Sequence("small/tell_info", memory=True)
    seq.add_child(BtNode_Announce("say tell", bb_source=bb_keys.ANNOUNCE_TEXT))
    return seq


def create_say():
    """Generic announcement using ``bb_keys.ANNOUNCE_TEXT``."""
    seq = py_trees.composites.Sequence("small/say", memory=True)
    seq.add_child(BtNode_Announce("announce", bb_source=bb_keys.ANNOUNCE_TEXT))
    return seq


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

ACTION_FACTORIES = {
    "goto": create_goto,
    "find_object": create_find_object,
    "find_person": create_find_person,
    "follow": create_follow,
    "guide": create_guide,
    "greet": create_greet,
    "grasp": create_grasp,
    "place": create_place,
    "deliver": create_deliver,
    "count": create_count,
    "answer_question": create_answer_question,
    "tell_info": create_tell_info,
    "say": create_say,
}
