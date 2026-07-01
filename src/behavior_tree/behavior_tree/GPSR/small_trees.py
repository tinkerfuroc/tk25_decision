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
from behavior_tree.TemplateNodes.Navigation import (
    BtNode_GotoAction,
    BtNode_ConvertGraspPose,
    BtNode_CaptureCurrentPose,
)
from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
    BtNode_FeatureExtraction,
)
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_MoveArmSingle,
    BtNode_GripperAction,
)

from behavior_tree.StoringGroceries.customNodes import (
    BtNode_FindObjTable,
    BtNode_GraspWithPose,
)
from behavior_tree.PickAndPlace.custom_nodes import BtNode_GetImage

from .custom_nodes import (
    BtNode_ScanForWavingPersonNew,
    BtNode_VLMQuery,
    BtNode_LLMQuery,
)


# ---------------------------------------------------------------------------
# Blackboard keys (single source of truth, shared with orchestrator)
# ---------------------------------------------------------------------------

class bb_keys:
    # Orchestrator state
    COMMAND = "gpsr/command"
    START_POSE = "gpsr/start_pose"      # PoseStamped — where the command was received
    DYNAMIC_LOCATIONS = "gpsr/dynamic_locations"  # dict[str -> PoseStamped] (runtime-recorded labels)
    LAST_CAPTURE = "gpsr/last_capture"            # PoseStamped scratch for the most recent capture
    CURRENT_DYNLABEL = "gpsr/current_dynlabel"    # str — label for the record_position step being run
    PLAN = "gpsr/plan"
    PLAN_INDEX = "gpsr/plan_index"
    CURRENT_ACTION = "gpsr/current_action"
    CURRENT_PARAMS = "gpsr/current_params"
    STATE_LOG = "gpsr/state_log"
    CORRECTION_COUNT = "gpsr/correction_count"
    LAST_FAILURE = "gpsr/last_failure"
    PLAN_SPEECH = "gpsr/plan_speech"    # str — spoken rehearsal of the plan steps

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
    PERSON_NAV_POSE = "gpsr/person_nav_pose"            # PoseStamped (approach goal)
    PERSON_VISION_PROMPT = "gpsr/person_vision_prompt"  # str (descriptor -> vision prompt)
    ALL_WAVING_PERSONS = "gpsr/all_waving_persons"
    ANNOUNCE_TEXT = "gpsr/announce_text"
    QA_QUESTION = "gpsr/qa_question"       # str — the question heard for answer_question
    QA_ANSWER = "gpsr/qa_answer"
    COUNT_VALUE = "gpsr/count_value"
    VLM_ANSWER = "gpsr/vlm_answer"
    DESCRIBE_FEATURES = "gpsr/describe_features"   # str — textual person description
    DESCRIBE_IMAGE = "gpsr/describe_image"         # sensor_msgs/Image comparison crop
    ASK_QUESTION = "gpsr/ask_question"     # str — the question to speak to a person
    PERSON_ANSWER = "gpsr/person_answer"   # str — the person's captured spoken answer
    VLM_QUESTION = "gpsr/vlm_question"     # str — question for the generic VLM fallback
    VLM_IMAGE = "gpsr/vlm_image"           # sensor_msgs/Image grabbed for the VLM fallback
    LLM_QUESTION = "gpsr/llm_question"     # str — question for the generic LLM fallback
    LLM_ANSWER = "gpsr/llm_answer"         # str — text LLM fallback answer
    REPORT_INFO = "gpsr/report_info"       # str — latest gathered result to report
                                           #       (count/describe/ask/vlm write it;
                                           #       a later text-less announce speaks it)

    # Manipulation working keys (re-using the conventions from gpsr_new.py)
    TABLE_IMG = "gpsr/table_img"
    OBJ_SEG = "gpsr/object_segmentation"
    GRASP_POSE = "gpsr/grasp_pose"
    GRASP_ANNOUNCEMENT = "gpsr/grasp_announcement"
    ARM_NAVIGATING = "gpsr/arm_navigating"
    ARM_SCAN = "gpsr/arm_scan"
    LAST_NAV_LOCATION = "gpsr/last_nav_location"  # str — location the robot most
                                           #   recently navigated to (goto /
                                           #   search_object); lets a later grasp
                                           #   infer a shelf grasp.
    GRASP_FROM_SHELF = "gpsr/grasp_from_shelf"  # bool — the object is on a shelf:
                                           #   skip the real grasp (unsafe /
                                           #   might damage the shelf) and go
                                           #   straight to the ask-referee branch.


ARM_ACTION_NAME = "joint_move_action"
GRASP_SERVICE_NAME = "start_grasp"
WAVING_THRESHOLD_METERS = 6.0

# Fixed capacity for the room sweep (create_search_object). The dispatcher
# builds every small tree once, before any command, so the sweep cannot size
# itself to the runtime location's spot list — it is built with this many
# branches and the orchestrator fills only as many SEARCH_POSE_<i> keys as the
# location actually has (the rest stay unset and are guarded out). Bump this if
# a room ever needs more than this many recorded search spots.
MAX_SEARCH_SPOTS = 6
SEARCH_POSE_KEYS = [f"gpsr/search_pose_{i}" for i in range(MAX_SEARCH_SPOTS)]


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


class BtNode_SetReportInfo(Behaviour):
    """Persist a gathered result into the shared ``REPORT_INFO`` buffer.

    The generalized "remember what to tell the operator" step: a perception
    action (count / describe_person / ask_person / vlm_fallback) writes its
    human-readable result here, and a later text-less ``announce`` speaks it
    back at the operator. Replaces the per-result report_* small trees — those
    were just announces of a stored value, which is exactly this buffer.
    """

    def __init__(self, name: str, bb_source: str, prefix: str = ""):
        super().__init__(name)
        self._src = bb_source
        self._prefix = prefix
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(bb_keys.REPORT_INFO, access=Access.WRITE)

    def update(self):
        try:
            value = self._client.get(self._src)
        except Exception:
            value = None
        text = f"{self._prefix}{value}" if value is not None else self._prefix
        self._client.set(bb_keys.REPORT_INFO, text.strip(), overwrite=True)
        self.feedback_message = f"buffered: {text[:60]}"
        return Status.SUCCESS


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
        from behavior_tree.config import is_subsystem_mocked
        if is_subsystem_mocked("vision"):
            # Mocked scan -> no real object. Write a placeholder point at the
            # origin so the (also-mocked) grasp can proceed; the real object pose
            # is irrelevant when manipulation is mocked too.
            self._client.set(self._object_dst, None, overwrite=True)
            self._client.set(
                self._point_dst,
                PointStamped(
                    header=Header(frame_id=self._target_frame,
                                  stamp=rclpy.time.Time().to_msg()),
                    point=Point(x=0.0, y=0.0, z=0.0),
                ),
                overwrite=True,
            )
            self.feedback_message = "MOCK(vision): synthetic detection at origin"
            return Status.SUCCESS
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


class BtNode_CheckBBContains(Behaviour):
    """SUCCESS iff the string at ``key`` contains ``substring`` (case-insensitive).

    Used as a guard so a Selector branch only runs when the descriptor on the
    blackboard actually calls for it (e.g. waving-person specialist only when
    the descriptor mentions waving).
    """

    def __init__(self, name: str, key: str, substring: str):
        super().__init__(name)
        self._key = key
        self._substring = substring.lower()
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._key, access=Access.READ)

    def update(self):
        try:
            value = self._client.get(self._key)
        except Exception:
            self.feedback_message = f"{self._key} not set"
            return Status.FAILURE
        if self._substring in str(value or "").lower():
            return Status.SUCCESS
        self.feedback_message = f"{self._key} does not mention {self._substring!r}"
        return Status.FAILURE


class BtNode_CheckBBKeySet(Behaviour):
    """SUCCESS iff ``key`` exists on the blackboard and is not None.

    Guards each branch of the room sweep (create_search_object): a SEARCH_POSE
    slot the orchestrator did not fill (the location has fewer spots than the
    sweep's capacity) reads back as unset/None, so the branch fails fast and the
    Selector moves on instead of navigating to a stale/empty goal.
    """

    def __init__(self, name: str, key: str):
        super().__init__(name)
        self._key = key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._key, access=Access.READ)

    def update(self):
        try:
            value = self._client.get(self._key)
        except Exception:
            self.feedback_message = f"{self._key} not set"
            return Status.FAILURE
        if value is None:
            self.feedback_message = f"{self._key} is None"
            return Status.FAILURE
        return Status.SUCCESS


class BtNode_CheckNotFromShelf(Behaviour):
    """SUCCESS unless ``GRASP_FROM_SHELF`` is truthy.

    Guards the primary grasp so a shelf grasp is skipped: when the object is on
    a shelf the robot cannot grasp it safely (and risks damaging the shelf), so
    this fails, the grasp Selector falls straight through to the ask-referee
    (deus-ex-machina) branch, and no arm scan / grasp is ever attempted.
    """

    def __init__(self, name: str = "not from shelf?"):
        super().__init__(name)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.GRASP_FROM_SHELF, access=Access.READ)

    def update(self):
        try:
            from_shelf = bool(self._client.get(bb_keys.GRASP_FROM_SHELF))
        except Exception:
            from_shelf = False
        if from_shelf:
            self.feedback_message = "object is on the shelf — skipping to ask-referee"
            return Status.FAILURE
        return Status.SUCCESS


class BtNode_BuildPersonPrompt(Behaviour):
    """Turn the planner's person descriptor into a usable vision prompt.

    Bare names ("Liam") cannot be detected visually, so they collapse to
    "person". Attribute descriptors ("person pointing to the left",
    "waving person") pass through so the generalist VLM can use them.
    """

    def __init__(self, name: str, bb_descriptor_key: str, bb_prompt_key: str):
        super().__init__(name)
        self._src = bb_descriptor_key
        self._dst = bb_prompt_key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._dst, access=Access.WRITE)

    def update(self):
        try:
            descriptor = str(self._client.get(self._src) or "").strip()
        except Exception:
            descriptor = ""
        low = descriptor.lower()
        if not low:
            prompt = "person"
        elif "person" in low:
            prompt = low
        elif len(low.split()) == 1:
            prompt = "person"  # bare name — not visually detectable
        else:
            prompt = f"person {low}"
        self._client.set(self._dst, prompt, overwrite=True)
        self.feedback_message = f"vision prompt: {prompt!r}"
        return Status.SUCCESS


class BtNode_RegisterLabeledPose(Behaviour):
    """Store the last-captured pose into the dynamic-location registry.

    Reads the PoseStamped at ``src_key`` (written by BtNode_CaptureCurrentPose)
    and the label string at ``label_key``, then inserts ``registry[label] =
    pose`` into the dict at ``registry_key`` so a later ``goto(location=label)``
    can navigate there. Each label is independent; recording a new one never
    clobbers the others.
    """

    def __init__(self, name: str, src_key: str, label_key: str, registry_key: str):
        super().__init__(name)
        self._src = src_key
        self._label_key = label_key
        self._registry_key = registry_key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._label_key, access=Access.READ)
        self._client.register_key(self._registry_key, access=Access.WRITE)

    def update(self):
        try:
            pose = self._client.get(self._src)
        except Exception as exc:
            self.feedback_message = f"no captured pose to register: {exc}"
            return Status.FAILURE
        try:
            label = self._client.get(self._label_key)
        except Exception:
            label = None
        label = (str(label) if label is not None else "").strip().lower() or "start_position"
        try:
            registry = self._client.get(self._registry_key)
        except Exception:
            registry = None
        if not isinstance(registry, dict):
            registry = {}
        registry[label] = pose
        self._client.set(self._registry_key, registry, overwrite=True)
        try:
            self.feedback_message = (
                f"registered '{label}' at "
                f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})"
            )
        except Exception:
            self.feedback_message = f"registered '{label}'"
        return Status.SUCCESS


class BtNode_CountDetections(Behaviour):
    """Count objects in a stored detection result and write the integer.

    With ``fail_if_zero=True`` a zero count returns FAILURE so a Selector
    parent can hand over to a fallback strategy (the detector finding nothing
    is more often a detection miss than a true zero).
    """

    def __init__(self, name: str, bb_detection_src: str, bb_count_dst: str,
                 fail_if_zero: bool = False):
        super().__init__(name)
        self._src = bb_detection_src
        self._dst = bb_count_dst
        self._fail_if_zero = fail_if_zero
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._dst, access=Access.WRITE)

    def update(self):
        from behavior_tree.config import is_subsystem_mocked
        if is_subsystem_mocked("vision"):
            # The detector was mocked (auto-completed without writing a real
            # result), so a zero count is a mock artifact, not a true miss.
            # Report a synthetic hit so the find/count tree succeeds and the plan
            # continues instead of failing into endless fallback / replan.
            self._client.set(self._dst, 1, overwrite=True)
            self.feedback_message = "MOCK(vision): synthetic count=1"
            return Status.SUCCESS
        try:
            result = self._client.get(self._src)
        except Exception as exc:
            self.feedback_message = f"Missing detection result: {exc}"
            return Status.FAILURE
        count = len(getattr(result, "objects", []) or [])
        self._client.set(self._dst, count, overwrite=True)
        if self._fail_if_zero and count == 0:
            self.feedback_message = "counted 0 objects — deferring to fallback"
            return Status.FAILURE
        self.feedback_message = f"counted {count} objects"
        return Status.SUCCESS


# ---------------------------------------------------------------------------
# Small-tree factories
# ---------------------------------------------------------------------------

def _tuck_arm_for_nav(label: str = "tuck arm for nav"):
    """Move the arm to the ``base_moving`` (navigating) pose before driving.

    The arm must be folded back to ``arm_pos_navigating`` so it does not block
    the lidar / occupy the robot's footprint during base motion. Every small
    tree that issues a base move (goto / follow / guide / approach / deliver /
    place) tucks first. The pose is read from ``bb_keys.ARM_NAVIGATING``, seeded
    once at startup by the orchestrator entry point (``_arm_constants_to_bb``).
    Retry-wrapped and propagates failure: if the arm cannot tuck, we do NOT
    drive with the arm sticking out — the orchestrator self-correction handles it.
    """
    return py_trees.decorators.Retry(
        f"retry {label}",
        BtNode_MoveArmSingle(
            label,
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
            add_octomap=False,
        ),
        num_failures=3,
    )


def create_goto():
    """Navigate to ``bb_keys.TARGET_POSE`` (filled by orchestrator)."""
    seq = py_trees.composites.Sequence("small/goto", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce going", bb_keys.TARGET_LOCATION, prefix="Going to "
    ))
    seq.add_child(_tuck_arm_for_nav("tuck arm before goto"))
    seq.add_child(py_trees.decorators.Retry(
        "retry goto",
        BtNode_GotoAction("goto target", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    return seq


def create_find_object():
    """Scan for the object named in ``bb_keys.TARGET_OBJECT_PROMPT``.

    Locate-only: stores the full detection result and verifies at least one
    match exists. It deliberately does NOT pick a single instance — choosing
    the object to manipulate is the grasp tree's job (``BtNode_FindObjTable``
    re-detects on the table surface with the arm camera).
    """
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
    seq.add_child(BtNode_CountDetections(
        "verify objects found",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE,
        fail_if_zero=True,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "announce found", bb_keys.TARGET_OBJECT_NAME, prefix="I can see the "
    ))
    return seq


def create_search_object():
    """Sweep a room's search spots until the target object is located.

    The finder for FETCH / GRASP tasks. For "fetch a coke from the living room"
    where the exact in-room spot is unknown, visit each pose in the location's
    search-spot list (materialised by the orchestrator into SEARCH_POSE_0..N),
    tucking the arm and driving to each, then scanning; stop at the FIRST spot
    where the object is seen (the robot is then parked there with the object in
    view for grasp). SUCCESS = found; FAILURE = swept every spot, none found.

    Built at fixed capacity (MAX_SEARCH_SPOTS) because the dispatcher constructs
    each small tree once. The orchestrator fills only the SEARCH_POSE_i keys the
    location has; unfilled slots are guarded out by BtNode_CheckBBKeySet so they
    neither navigate nor count as "found". A memory Selector returns SUCCESS on
    the first branch that succeeds, FAILURE only if all branches fail.
    """
    sweep = py_trees.composites.Selector("small/search_object", memory=True)
    for i, pose_key in enumerate(SEARCH_POSE_KEYS):
        branch = py_trees.composites.Sequence(f"search spot {i}", memory=True)
        branch.add_child(BtNode_CheckBBKeySet(f"spot {i} set?", pose_key))
        branch.add_child(_tuck_arm_for_nav(f"tuck arm before spot {i}"))
        branch.add_child(py_trees.decorators.Retry(
            f"retry goto spot {i}",
            BtNode_GotoAction(f"goto spot {i}", key=pose_key),
            num_failures=3,
        ))
        branch.add_child(create_find_object())
        sweep.add_child(branch)
    return sweep


def create_approach_person():
    """Navigate to the person pose stored by the most recent ``find_person``.

    Atomic action — the LLM plans ``find_person`` then ``approach_person``
    when an interaction (describe / follow / guide / handover) needs the robot
    standing next to the person rather than across the room. Converts
    ``bb_keys.TARGET_PERSON_POSE`` (PointStamped from vision) into a nav goal
    and drives there.
    """
    seq = py_trees.composites.Sequence("small/approach_person", memory=True)
    seq.add_child(BtNode_PointToPoseStamped(
        "person point to nav pose",
        bb_point_key=bb_keys.TARGET_PERSON_POSE,
        bb_pose_key=bb_keys.PERSON_NAV_POSE,
    ))
    seq.add_child(_tuck_arm_for_nav("tuck arm before approach"))
    seq.add_child(py_trees.decorators.Retry(
        "retry approach",
        BtNode_GotoAction("goto person", key=bb_keys.PERSON_NAV_POSE),
        num_failures=3,
    ))
    return seq


def create_find_person():
    """Scan for a person matching ``bb_keys.TARGET_PERSON_PROMPT`` and store
    their pose. Locate-only — it does NOT move the robot.

    The waving-person specialist only runs when the descriptor actually
    mentions waving; otherwise the generalist scans with a vision prompt built
    from the descriptor (bare names collapse to "person"). To stand next to
    the person, the planner emits ``approach_person`` as a separate step.
    """
    seq = py_trees.composites.Sequence("small/find_person", memory=True)
    # Look FORWARD/up to see a standing person's body+face, not down at the floor.
    seq.add_child(BtNode_TurnPanTilt("turn pantilt forward", x=0.0, y=45.0))
    seq.add_child(BtNode_BuildPersonPrompt(
        "descriptor to vision prompt",
        bb_descriptor_key=bb_keys.TARGET_PERSON_PROMPT,
        bb_prompt_key=bb_keys.PERSON_VISION_PROMPT,
    ))
    seq.add_child(BtNode_Announce(
        "announce searching", bb_source=None,
        message="Looking for a person, please stay still.",
    ))

    selector = py_trees.composites.Selector("person scan strategies", memory=False)
    # waving-person specialist — only when the descriptor calls for it
    waving_branch = py_trees.composites.Sequence("waving person branch", memory=True)
    waving_branch.add_child(BtNode_CheckBBContains(
        "descriptor mentions waving?", bb_keys.TARGET_PERSON_PROMPT, "wav",
    ))
    waving_branch.add_child(BtNode_ScanForWavingPersonNew(
        "find waving persons",
        bb_keys.ALL_WAVING_PERSONS,
        bb_keys.TARGET_PERSON_POSE,
        WAVING_THRESHOLD_METERS,
        target_frame="map",
    ))
    selector.add_child(waving_branch)
    # generalist fallback — prompt carries the descriptor where possible
    generalist_branch = py_trees.composites.Sequence("generalist person scan", memory=True)
    generalist_branch.add_child(BtNode_ScanForGeneralist(
        name="generalist person scan",
        bb_source=bb_keys.PERSON_VISION_PROMPT,
        bb_key=bb_keys.TARGET_PERSON_DETECTION,
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


def create_describe_person():
    """Look at the person in view and speak a description of them.

    Closes the "tell me the name / pose / gesture of the person" gap: the
    vision feature-extraction service returns a human-readable description
    string (the same ``feature`` text HRI speaks when introducing a guest —
    see HRI/hri.py BtNode_FeatureExtraction + BtNode_Introduce). The planner
    runs ``find_person`` first to locate + approach the person, then this
    tree frames the face, extracts the description, and announces it.
    """
    seq = py_trees.composites.Sequence("small/describe_person", memory=True)
    seq.add_child(BtNode_TurnPanTilt("turn pantilt up", x=0.0, y=45.0))
    seq.add_child(BtNode_Announce(
        "announce describing", bb_source=None,
        message="Let me take a look at this person.",
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry feature extraction",
        BtNode_FeatureExtraction(
            "extract person description",
            bb_dest_key=bb_keys.DESCRIBE_FEATURES,
            bb_image_key=bb_keys.DESCRIBE_IMAGE,
        ),
        num_failures=3,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "announce description", bb_keys.DESCRIBE_FEATURES,
        prefix="Here is what I can tell about the person. ",
    ))
    seq.add_child(BtNode_SetReportInfo(
        "buffer description", bb_keys.DESCRIBE_FEATURES,
        prefix="Here is what the person looks like. ",
    ))
    return seq


def create_ask_person():
    """Ask the person in front a spoken question and capture their answer.

    For information you can ONLY get by ASKING — the person's name, age,
    favourite drink, where they are from. This is the audio counterpart to
    describe_person (which reports VISIBLE traits only and cannot obtain a
    name). Mirrors the HRI intake pattern (HRI/hri.py ``_create_get_info``):
    speak the question, prompt to answer after the beep, listen for the free
    answer, then read it back. The orchestrator writes the literal question to
    ``bb_keys.ASK_QUESTION`` from the plan's ``question`` param; the captured
    answer lands in ``bb_keys.PERSON_ANSWER`` for a later ``report_answer``.
    """
    seq = py_trees.composites.Sequence("small/ask_person", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "ask question", bb_keys.ASK_QUESTION, prefix="",
    ))
    seq.add_child(BtNode_Announce(
        "announce answer after beep", bb_source=None,
        message="Please answer after the beep.",
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry listen answer",
        BtNode_ListenAction(
            "listen to answer",
            bb_dest_key=bb_keys.PERSON_ANSWER,
            timeout=10.0,
        ),
        num_failures=2,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "repeat answer", bb_keys.PERSON_ANSWER, prefix="Thank you. I heard ",
    ))
    seq.add_child(BtNode_SetReportInfo(
        "buffer answer", bb_keys.PERSON_ANSWER, prefix="The person told me. ",
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
    seq.add_child(_tuck_arm_for_nav("tuck arm before follow"))
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
    seq.add_child(_tuck_arm_for_nav("tuck arm before guide"))
    seq.add_child(py_trees.decorators.Retry(
        "retry guide goto",
        BtNode_GotoAction("guide to target", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "announce arrived", bb_keys.TARGET_LOCATION, prefix="We have arrived at "
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
        service_name=ARM_ACTION_NAME,
        arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        add_octomap=False,
    ))
    par_scan = py_trees.composites.Parallel(
        "arm to scan + announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par_scan.add_child(BtNode_Announce(
        "announce move arm", bb_source=None,
        message="Moving my arm to look for the object",
    ))
    par_scan.add_child(BtNode_MoveArmSingle(
        "arm to scan",
        service_name=ARM_ACTION_NAME,
        arm_pose_bb_key=bb_keys.ARM_SCAN,
        add_octomap=True,
    ))
    primary.add_child(par_scan)

    find_and_grasp = py_trees.composites.Sequence("find and grasp", memory=True)
    # Let the RealSense settle on the table after the arm reaches table_grasp
    # (first frames after the arm stops can be motion-blurred / unsynced), THEN
    # detect — and RETRY the detection in place. Without this, one fast "no
    # detection" makes the whole grasp fail and the arm jumps back to
    # base_moving before vision really had a look. Retry holds the arm at
    # table_grasp across several detection attempts instead.
    find_and_grasp.add_child(BtNode_WaitTicks("settle before scan", 6))
    # Detect on the table with the GENERALIST detector on the arm RealSense
    # (open-vocab: YOLO-World / Gemini-VLM + SAM fallback), NOT the stock-COCO
    # yolo_seg — so competition labels like "coke" are actually found (COCO only
    # knows generic classes like "bottle"). It returns rgb+depth+segments, so the
    # grasp server gets everything it needs, and FAILS (status!=0) when nothing
    # matches so the Retry re-looks. The full response lands in TARGET_OBJECT,
    # which the grasp node reads as vision_result.
    find_and_grasp.add_child(py_trees.decorators.Retry(
        "retry detect on table",
        BtNode_ScanForGeneralist(
            name="realsense generalist detect",
            bb_source=bb_keys.TARGET_OBJECT_PROMPT,
            bb_key=bb_keys.TARGET_OBJECT,
            use_orbbec=False,            # arm RealSense at table_grasp
            transform_to_map=False,      # keep camera frame for the grasp server
            use_vlm_sam_fallback=True,   # open-vocab for coke / fanta / ...
            sort_closest=True,
            return_rgb_image=True,
            return_depth_image=True,
            return_segments=True,
        ),
        num_failures=5,
    ))
    par_grasp = py_trees.composites.Parallel(
        "grasp+announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par_grasp.add_child(BtNode_AnnounceFromBB(
        "announce grasp", bb_keys.TARGET_OBJECT_NAME, prefix="Grasping the ",
    ))
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
            service_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        ),
        num_failures=5,
    ))

    primary_with_retry = py_trees.decorators.Retry(
        "retry primary 3x", primary, num_failures=3,
    )

    # Only attempt the real grasp when the object is NOT on a shelf. A shelf
    # grasp fails this guard immediately (no arm scan, no grasp motion) so the
    # Selector drops through to the ask-referee branch below — the robot cannot
    # safely reach into a shelf and could damage it.
    guarded_primary = py_trees.composites.Sequence("grasp/try_unless_shelf", memory=True)
    guarded_primary.add_child(BtNode_CheckNotFromShelf("skip primary if from shelf"))
    guarded_primary.add_child(primary_with_retry)

    ex_machina = py_trees.composites.Sequence("grasp/ex_machina", memory=True)
    ex_machina.add_child(py_trees.decorators.Retry(
        "retry arm back",
        BtNode_MoveArmSingle(
            "arm to navigating",
            service_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_NAVIGATING,
        ),
        num_failures=5,
    ))
    ex_machina.add_child(BtNode_GripperAction("open gripper", True))
    ex_machina.add_child(BtNode_AnnounceFromBB(
        "ask referee", bb_keys.TARGET_OBJECT_NAME, prefix="Dear referee, please help me grasp the "
    ))
    ex_machina.add_child(BtNode_Announce(
        "announce put in gripper", bb_source=None,
        message="Put it in my gripper please. Thank you.",
    ))
    ex_machina.add_child(BtNode_WaitTicks("wait", 8))
    ex_machina.add_child(BtNode_GripperAction("close gripper", False))

    return py_trees.composites.Selector(
        "small/grasp",
        memory=True,
        children=[guarded_primary, ex_machina],
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
    seq.add_child(_tuck_arm_for_nav("tuck arm before place goto"))
    seq.add_child(py_trees.decorators.Retry(
        "retry place goto",
        BtNode_GotoAction("goto place pose", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    seq.add_child(BtNode_Announce(
        "announce releasing", bb_source=None, message="Releasing object now."
    ))
    seq.add_child(BtNode_GripperAction("open gripper", True))
    seq.add_child(BtNode_WaitTicks("wait", 4))
    seq.add_child(BtNode_GripperAction("close gripper", False))
    return seq


def create_deliver():
    """Bring the currently-held object to a person.

    Navigates to the recipient's room (``bb_keys.TARGET_POSE``, materialised
    from ``recipient_location``), then visually detects and approaches the
    recipient before opening the gripper — handing over at a room waypoint
    with nobody in front of the gripper is the failure mode this guards
    against.
    """
    seq = py_trees.composites.Sequence("small/deliver", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce delivering", bb_keys.TARGET_OBJECT_NAME, prefix="Delivering the "
    ))
    seq.add_child(_tuck_arm_for_nav("tuck arm before deliver goto"))
    seq.add_child(py_trees.decorators.Retry(
        "retry deliver goto",
        BtNode_GotoAction("goto recipient room", key=bb_keys.TARGET_POSE),
        num_failures=5,
    ))
    seq.add_child(create_find_person())       # detect the recipient
    seq.add_child(create_approach_person())   # walk up to them
    seq.add_child(BtNode_Announce(
        "announce take it", bb_source=None,
        message="Here you go, please take the object from my gripper.",
    ))
    seq.add_child(BtNode_WaitTicks("wait for take", 8))
    seq.add_child(BtNode_GripperAction("open gripper", True))
    seq.add_child(BtNode_WaitTicks("wait", 4))
    seq.add_child(BtNode_GripperAction("close gripper", False))
    return seq


def create_count():
    """Count objects matching ``bb_keys.TARGET_OBJECT_PROMPT`` and announce the number.

    Counting is NOT a first-class capability of the vision stack — the
    detector enumerates what it happens to segment, which is unreliable for
    category counting. Structure: try detection-based counting first (and
    keep the RGB frame); if the detector fails or returns zero, fall back to
    sending the captured image plus the counting question to the multimodal
    LLM and speaking its answer.
    """
    primary = py_trees.composites.Sequence("count/by_detector", memory=True)
    primary.add_child(BtNode_TurnPanTilt("turn pantilt", x=0.0, y=10.0))
    primary.add_child(BtNode_Announce(
        "announce scanning", bb_source=None, message="I am counting now."
    ))
    primary.add_child(BtNode_ScanForGeneralist(
        name="scan to count",
        bb_source=bb_keys.TARGET_OBJECT_PROMPT,
        bb_key=bb_keys.TARGET_OBJECT_DETECTION,
        use_orbbec=True,
        transform_to_map=False,
        use_vlm_sam_fallback=True,
        sort_closest=True,
        return_segments=False,
        return_rgb_image=True,  # keep the frame for the VLM fallback
    ))
    primary.add_child(BtNode_CountDetections(
        "count detections",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE,
        fail_if_zero=True,  # zero hits -> let the VLM double-check the frame
    ))
    primary.add_child(BtNode_AnnounceFromBB(
        "announce count", bb_keys.COUNT_VALUE, prefix="I counted "
    ))
    primary.add_child(BtNode_SetReportInfo(
        "buffer count", bb_keys.COUNT_VALUE, prefix="I counted ",
    ))

    vlm_fallback = py_trees.composites.Sequence("count/vlm_fallback", memory=True)
    vlm_fallback.add_child(BtNode_Announce(
        "announce vlm fallback", bb_source=None,
        message="My detector could not count them. Let me look at the picture myself.",
    ))
    vlm_fallback.add_child(BtNode_VLMQuery(
        "vlm count",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template=(
            "How many {value} are visible in this image? Answer in one short "
            "spoken sentence that starts with the number."
        ),
        bb_fill_key=bb_keys.TARGET_OBJECT_PROMPT,
    ))
    vlm_fallback.add_child(BtNode_AnnounceFromBB(
        "announce vlm count", bb_keys.VLM_ANSWER, prefix=""
    ))
    vlm_fallback.add_child(BtNode_SetReportInfo(
        "buffer vlm count", bb_keys.VLM_ANSWER, prefix="",
    ))

    return py_trees.composites.Selector(
        "small/count", memory=True,
        children=[primary, vlm_fallback],
    )


def create_answer_question():
    """Listen to a spoken question and answer it with the text LLM.

    Rebuilt from listen + LLM + announce — no separate ``question_answer_service``
    node is needed (that server only did listen -> answer -> speak internally,
    which these BT nodes already provide). Prompt, listen the question into
    ``QA_QUESTION``, answer it with gpt-4.1 (``BtNode_LLMQuery``, current
    date/time injected, honest "cannot access that" for live-data questions),
    then speak the answer. Same building blocks as ``create_llm_fallback``,
    but the question arrives by voice instead of from the plan.
    """
    seq = py_trees.composites.Sequence("small/answer_question", memory=True)
    seq.add_child(BtNode_Announce(
        "announce ask", bb_source=None,
        message="Please ask me a question after the beep.",
    ))
    seq.add_child(py_trees.decorators.Retry(
        "retry listen question",
        BtNode_ListenAction(
            "listen to question",
            bb_dest_key=bb_keys.QA_QUESTION,
            timeout=10.0,
        ),
        num_failures=2,
    ))
    seq.add_child(BtNode_LLMQuery(
        "answer question",
        bb_question_key=bb_keys.QA_QUESTION,
        bb_answer_dst=bb_keys.QA_ANSWER,
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "announce answer", bb_keys.QA_ANSWER, prefix="",
    ))
    return seq


def create_announce():
    """Speak the text in ``bb_keys.ANNOUNCE_TEXT`` (filled from the plan's
    ``text`` param by the orchestrator).

    Single spoken-output action — replaces the former ``say`` / ``tell_info``
    twins, which were structurally identical. Used for telling the time/day,
    team info, reporting results, and explaining refusals.
    """
    seq = py_trees.composites.Sequence("small/announce", memory=True)
    seq.add_child(BtNode_Announce("announce text", bb_source=bb_keys.ANNOUNCE_TEXT))
    return seq


def create_record_position():
    """Capture the robot's current map pose and register it under a label.

    The orchestrator writes the label (from the plan's ``label`` param) to
    ``bb_keys.CURRENT_DYNLABEL`` before dispatch, so this tree can store the
    pose into the ``DYNAMIC_LOCATIONS`` registry under that name. A later
    ``goto(location=<label>)`` then resolves to it. With no label it defaults
    to ``start_position``. The single auto start-pose snapshot at command
    start is captured separately by the orchestrator (straight to START_POSE).
    """
    seq = py_trees.composites.Sequence("small/record_position", memory=True)
    seq.add_child(BtNode_CaptureCurrentPose(
        "capture current pose", bb_key=bb_keys.LAST_CAPTURE,
    ))
    seq.add_child(BtNode_RegisterLabeledPose(
        "register labeled pose",
        src_key=bb_keys.LAST_CAPTURE,
        label_key=bb_keys.CURRENT_DYNLABEL,
        registry_key=bb_keys.DYNAMIC_LOCATIONS,
    ))
    return seq


def create_vlm_fallback():
    """Look at the scene and answer a visual question with the gpt-4.1 VLM.

    Generic escape hatch for VISUAL tasks no specific small tree covers — "what
    colour is the X", "is the door open", "what is on the table", "what is the
    person holding". Grabs a fresh camera frame, sends it plus the question
    (written to ``bb_keys.VLM_QUESTION`` by the orchestrator) to gpt-4.1, and
    speaks the answer. The planner navigates to the right spot first if needed.
    """
    seq = py_trees.composites.Sequence("small/vlm_fallback", memory=True)
    seq.add_child(BtNode_TurnPanTilt("turn pantilt forward", x=0.0, y=15.0))
    seq.add_child(BtNode_Announce(
        "announce looking", bb_source=None, message="Let me take a look.",
    ))
    seq.add_child(BtNode_GetImage(
        "grab frame", camera="orbbec", bb_key_rgb_image=bb_keys.VLM_IMAGE,
    ))
    seq.add_child(BtNode_VLMQuery(
        "vlm answer",
        bb_detection_src=bb_keys.VLM_IMAGE,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template="{value}",
        bb_fill_key=bb_keys.VLM_QUESTION,
    ))
    seq.add_child(BtNode_AnnounceFromBB("announce vlm answer", bb_keys.VLM_ANSWER))
    seq.add_child(BtNode_SetReportInfo(
        "buffer view", bb_keys.VLM_ANSWER, prefix="Here is what I saw. ",
    ))
    return seq


def create_llm_fallback():
    """Answer a general (non-visual) question with the gpt-4.1 text LLM.

    Generic escape hatch for general-knowledge / contextual questions the robot
    has no dedicated action for — "what is the date / day / time", simple facts
    or maths. The question (written to ``bb_keys.LLM_QUESTION`` by the
    orchestrator) goes to gpt-4.1 with the current date/time injected; the
    spoken answer is announced. Questions needing live data the model cannot
    have (e.g. current weather) get an honest "I cannot access that" reply.
    """
    seq = py_trees.composites.Sequence("small/llm_fallback", memory=True)
    seq.add_child(BtNode_LLMQuery(
        "llm answer",
        bb_question_key=bb_keys.LLM_QUESTION,
        bb_answer_dst=bb_keys.LLM_ANSWER,
    ))
    seq.add_child(BtNode_AnnounceFromBB("announce llm answer", bb_keys.LLM_ANSWER))
    return seq


# ---------------------------------------------------------------------------
# Registry
# ---------------------------------------------------------------------------

ACTION_FACTORIES = {
    "goto": create_goto,
    "find_object": create_find_object,
    "search_object": create_search_object,
    "find_person": create_find_person,
    "approach_person": create_approach_person,
    "describe_person": create_describe_person,
    "ask_person": create_ask_person,
    "follow": create_follow,
    "guide": create_guide,
    "grasp": create_grasp,
    "place": create_place,
    "deliver": create_deliver,
    "count": create_count,
    "answer_question": create_answer_question,
    "announce": create_announce,
    "record_position": create_record_position,
    "vlm_fallback": create_vlm_fallback,
    "llm_fallback": create_llm_fallback,
}
