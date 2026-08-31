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

import json
import math
import os
import time
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Access, Status
from py_trees.blackboard import Blackboard
from geometry_msgs.msg import PoseStamped, PointStamped, Pose, Point, Quaternion
from std_msgs.msg import Header, String
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy

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
    BtNode_DoorDetection,
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
    BtNode_ParseCountFromAnswer,
    BtNode_LLMQuery,
)
from .telemetry import get_default_telemetry


# ---------------------------------------------------------------------------
# Blackboard keys (single source of truth, shared with orchestrator)
# ---------------------------------------------------------------------------

class bb_keys:
    # Orchestrator state
    RUN_ID = "gpsr/run_id"                    # str — mission/trajectory id
    TASK_ID = "gpsr/task_id"                  # str — stable task id, not command text
    PLAN_REVISION = "gpsr/plan_revision"      # int — initial plan=1, replans increment
    TREE_REVISION = "gpsr/tree_revision"      # int/string — active generated tree revision
    TASK_OUTCOME = "gpsr/task_outcome"        # dict — explicit terminal classification
    SUPERVISOR_STEP_DISPOSITION = "gpsr/supervisor_step_disposition"
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
    STEP_METHOD = "gpsr/step_method"    # str — optional method claim a step's
                                         #   own branch sets before finishing
                                         #   (e.g. grasp's "autonomous" /
                                         #   "referee_fallback"). Generic: any
                                         #   step can claim one; only grasp
                                         #   does today. Read once by
                                         #   BtNode_LogStepResult into
                                         #   step.finished's "method" field,
                                         #   then CLEARED so a later step
                                         #   without a claim never inherits it.
    MISSION_UNRECOVERABLE = "gpsr/mission_unrecoverable"  # bool — latched
                                         #   True once PersistentFailureCap
                                         #   gives up on the tuck-arm goal
                                         #   (V-2 fix, task-K review): every
                                         #   create_goto() instance fails
                                         #   fast on this flag from then on,
                                         #   so ALL of its side-effectful
                                         #   nodes (not just the tuck retry)
                                         #   go quiet on later root restarts.
                                         #   Never reset once set.

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
    PERSON_VISION_PROMPT_GENERIC = "gpsr/person_vision_prompt_generic"  # str — always
                                           #   bare "person" (N2, round-5
                                           #   rerun fix); a SEPARATE key from
                                           #   PERSON_VISION_PROMPT so the
                                           #   generic-scan branch's own
                                           #   BtNode_ScanForGeneralist never
                                           #   clobbers the descriptor-derived
                                           #   prompt other branches re-read
                                           #   on restarts.
    ALL_WAVING_PERSONS = "gpsr/all_waving_persons"
    PERSON_PROVENANCE = "gpsr/person_provenance"  # str — how TARGET_PERSON_POSE
                                           #   was obtained (e.g.
                                           #   "relaxed_generic" — see
                                           #   BtNode_ExtractDetection's
                                           #   relaxed mode, L2/L3 round-4
                                           #   battery fix); the waving
                                           #   specialist's own provenance
                                           #   ("waving_specialist") is
                                           #   synthesized in
                                           #   orchestrator._target_gate_evidence
                                           #   from ALL_WAVING_PERSONS instead.
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
    ARM_ORBBEC_LOOK = "gpsr/arm_orbbec_look"  # arm pose that clears the orbbec
                                           #   head camera's view; moved to
                                           #   before any orbbec scan (find_object
                                           #   / count / vlm / find_person /
                                           #   describe_person) AND used as the
                                           #   stow pose for navigation (replaces
                                           #   arm_pos_navigating). NOT the arm
                                           #   RealSense grasp scan (uses ARM_SCAN).
    PENDING_NAV_LOCATION = "gpsr/pending_nav_location"  # str — location a self-navigating step's params NAME, written at materialisation time (before the step has run). J4 (round-3 adversarial review, M10): the at_robot() gate must never trust this -- read only by consumers that need the intended destination early (grasp's no-grasp/shelf inference), never by fact verification.
    LAST_NAV_LOCATION = "gpsr/last_nav_location"  # str — location the robot most
                                           #   recently SUCCESSFULLY navigated to
                                           #   (goto / any self-navigating action);
                                           #   written by the step-finished path
                                           #   (BtNode_LogStepResult), never at
                                           #   materialisation -- this is the
                                           #   at_robot() gate's evidence (J4).
    GRASP_ASK_REFEREE = "gpsr/grasp_ask_referee"  # bool — the object is on
                                           #   furniture the robot must NOT grasp
                                           #   from (shelf / cabinet / coat_rack):
                                           #   skip the real grasp (unsafe / might
                                           #   damage it) and go straight to the
                                           #   ask-referee (deus-ex-machina) branch.
    GRASP_REFEREE_LOCATION = "gpsr/grasp_referee_location"  # str — the no-grasp
                                           #   furniture name the ask-referee
                                           #   branch drives to first.
    GRASP_REFEREE_POSE = "gpsr/grasp_referee_pose"  # PoseStamped — its resolved
                                           #   pose (None if unresolvable; the
                                           #   goto is then a best-effort no-op).
    GRASP_REFEREE_IS_APPLIANCE = "gpsr/grasp_referee_is_appliance"  # bool — the
                                           #   no-grasp furniture is a closed
                                           #   appliance (refrigerator / fridge /
                                           #   washing machine / dishwasher) the
                                           #   robot cannot open: the ask-referee
                                           #   branch MUST first ask the referee to
                                           #   open it, deterministically.
    APPLIANCE_OPENED = "gpsr/appliance_opened"  # bool — the referee has already
                                           #   been asked to open the appliance
                                           #   (by an explicit open() step or the
                                           #   grasp branch's own open-ask), so the
                                           #   grasp branch does not ask twice.
                                           #   Reset False per task.

    # Arena entry (GPSR starts OUTSIDE the arena, in front of the door)
    DOOR_STATUS = "gpsr/door_status"       # int — 1 open / 0 closed (BtNode_DoorDetection)
    ARENA_ENTERED = "gpsr/arena_entered"   # truthy once the robot has crossed the
                                           #   threshold; latched so the door step
                                           #   runs exactly once per mission.

    # Batch command intake: collect N commands + their plans UP FRONT (at the
    # command point), then execute them one by one. Each task's plan/command is
    # stashed under an indexed slot key (prefix + <i>).
    SAVED_PLAN_PREFIX = "gpsr/saved_plan_"        # + <i> -> list[dict] plan for task i
    SAVED_COMMAND_PREFIX = "gpsr/saved_command_"  # + <i> -> str command for task i

    # --- two-layer planning (top-layer split + lower-layer per-target plans) ---
    TARGETS = "gpsr/targets"                     # list[str] top-layer target descriptions (current command)
    NUM_TARGETS = "gpsr/num_targets"             # int — how many targets the top layer split out
    TARGET_INDEX = "gpsr/target_index"           # int — current target being executed (DynamicExecutor writes)
    CURRENT_TARGET = "gpsr/current_target"       # str — the active target's NL description
    CURRENT_TARGET_PLAN = "gpsr/current_target_plan"  # list[dict] flattened aggregate (logging/plan_judge compat)
    SAVED_TARGETS_PREFIX = "gpsr/saved_targets_"      # + <slot> -> list[str] targets for command slot
    SAVED_TARGET_PLAN_PREFIX = "gpsr/saved_target_plan_"  # + <slot>_<i> -> list[dict] action-plan for target i of slot
    # Replan request channel (extension point; trigger logic to be announced).
    REPLAN_REQUEST = "gpsr/replan_request"       # dict {level: "target"|"command", index: int, reason: str}
    TARGET_REPLAN_COUNT = "gpsr/target_replan_count"  # int — per-target replan budget, reset when a target advances
    DEFERRED_PRECONDITIONS = "gpsr/deferred_preconditions"  # list[str] — canonical preconditions the active target's own plan self-establishes; verified by the postcondition gate instead of at entry
    GATE_COMPLETED_STEPS = "gpsr/gate_completed_steps"  # list[dict] — the active target's own plan steps whose established facts the postcondition gate just committed (J3): fed to the next replan as completed_steps so it is not repeated
    FACTS = "gpsr/facts"
    # list[str] — canonical facts established by successful target postcondition gates for the active command slot.


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

# Pan-tilt search sweep (degrees). When looking for a target the robot tries
# every (pan, tilt) combination in turn, stopping at the first one where the
# target is detected (see _pantilt_sweep). Higher tilt looks up, lower looks
# down (matching the existing nodes: 45 = up at a standing person, 20 = down at
# a table, 0 = level). Each list is swept in order, pan-inner / tilt-outer, so
# the first tilt is the primary look.
PAN_SWEEP_DEG = [0.0, 45.0, -45.0]   # centre first, then left, then right
HUMAN_TILT_DEG = [45.0, 20.0]        # up at a standing person, then lower
OBJECT_TILT_DEG = [0.0, 20.0]        # level, then angled down at a surface


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


class BtNode_BlackboardCopy(Behaviour):
    """Copy ``src`` -> ``dst`` on the blackboard each tick.

    SUCCESS after copying; FAILURE if ``src`` is unset/None. Unlike
    ``BtNode_WriteToBlackboard(bb_source=...)`` it re-reads ``src`` on EVERY
    tick (no cached value), so it is safe inside loops / re-entrant trees — used
    by the batch-command flow to stash each task's plan/command into a slot and
    restore it before execution.
    """

    def __init__(self, name: str, src: str, dst: str):
        super().__init__(name)
        self._src = src
        self._dst = dst
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._dst, access=Access.WRITE)

    def update(self):
        try:
            value = self._client.get(self._src)
        except Exception:
            self.feedback_message = f"{self._src} not set"
            return Status.FAILURE
        if value is None:
            self.feedback_message = f"{self._src} is None"
            return Status.FAILURE
        self._client.set(self._dst, value, overwrite=True)
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

    L2a (round-4 battery fix, runs 008/011): ``relaxed=True`` is an opt-in
    mode used by ``_person_scan_strategies``' generalist fallback (a strict-
    then-relaxed Selector around this node) after the STRICT pass has
    already failed to descriptor-match. Instead of blindly taking
    ``objects[0]``, it picks the first detection whose label token-matches
    the generic person-class labels (shared with
    ``validators._SIM_PERSON_CLASS_LABELS``, not duplicated) and, on a hit,
    also writes ``bb_provenance_dst`` (if given) so the postcondition gate's
    relaxed branch can require it (see ``validators._verify``'s
    ``person_found`` branch).

    W-4 (round-4 review fix, doc-only): with ``GPSR_SIM_IDENTITY_RELAXED``
    unset, this path is dead (always FAILURE) in a REAL-vision run -- see
    the flag check in ``update()``, right after the mocked-vision
    short-circuit above it. That mocked-vision short-circuit runs FIRST and
    returns SUCCESS unconditionally (irrespective of ``relaxed``/the flag),
    so with vision mocked this node is not literally always FAILURE either
    -- it is simply unreachable there for a DIFFERENT reason: the identical
    short-circuit in the STRICT extract earlier in the same Selector
    (``_person_scan_strategies``'s ``generalist_branch``) already succeeds
    first, so the Selector never reaches this relaxed branch when vision is
    mocked.
    """

    def __init__(
        self,
        name: str,
        bb_detection_src: str,
        bb_object_dst: str,
        bb_point_dst: str,
        target_frame: str = "map",
        relaxed: bool = False,
        bb_provenance_dst: str = None,
    ):
        super().__init__(name)
        self._src = bb_detection_src
        self._object_dst = bb_object_dst
        self._point_dst = bb_point_dst
        self._target_frame = target_frame
        self._relaxed = relaxed
        self._provenance_dst = bb_provenance_dst
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._object_dst, access=Access.WRITE)
        self._client.register_key(self._point_dst, access=Access.WRITE)
        if self._provenance_dst:
            self._client.register_key(self._provenance_dst, access=Access.WRITE)

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

        if self._relaxed:
            from .validators import _sim_identity_relaxed_enabled
            if not _sim_identity_relaxed_enabled():
                self.feedback_message = (
                    "relaxed extraction disabled (GPSR_SIM_IDENTITY_RELAXED != 1)"
                )
                return Status.FAILURE

        try:
            result = self._client.get(self._src)
        except Exception as exc:
            self.feedback_message = f"No detection result in {self._src}: {exc}"
            return Status.FAILURE

        objects = getattr(result, "objects", None) or []
        if not objects:
            self.feedback_message = "Scan returned 0 objects"
            return Status.FAILURE

        if self._relaxed:
            from .validators import _SIM_PERSON_CLASS_LABELS, _item_label, _label_tokens
            picked = None
            for candidate in objects:
                label = _item_label(candidate)
                if label and any(tok in _SIM_PERSON_CLASS_LABELS for tok in _label_tokens(label)):
                    picked = candidate
                    break
            if picked is None:
                self.feedback_message = "relaxed extraction: no generic person-class detection"
                return Status.FAILURE
        else:
            picked = objects[0]

        self._client.set(self._object_dst, picked, overwrite=True)

        header = getattr(result, "header", None)
        if header is None:
            header = Header(frame_id=self._target_frame, stamp=rclpy.time.Time().to_msg())
        point_stamped = PointStamped(header=header, point=picked.centroid)
        self._client.set(self._point_dst, point_stamped, overwrite=True)
        if self._relaxed and self._provenance_dst:
            self._client.set(self._provenance_dst, "relaxed_generic", overwrite=True)
        prefix = "Relaxed-picked" if self._relaxed else "Picked"
        self.feedback_message = (
            f"{prefix} detection at ({picked.centroid.x:.2f}, {picked.centroid.y:.2f})"
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
        if point is None:
            # L2c (round-4 battery fix, runs 008/011): a relaxed person-found
            # gate can VALIDATE before the scan actually materialized a pose
            # (see validators._verify's person_found branch) -- without this
            # check the raw None fell into the generic "Unsupported type"
            # branch below, logging a useless "...: NoneType". Name the real
            # cause instead: the scan never wrote a pose.
            self.feedback_message = (
                f"no {self._point_key} recorded — the person scan must "
                "succeed first"
            )
            return Status.FAILURE
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


class BtNode_CheckBBTrue(Behaviour):
    """SUCCESS iff the value at ``key`` is truthy (FAILURE if falsy/unset).

    A generic boolean guard — used to gate the ask-referee-to-open step on
    ``GRASP_REFEREE_IS_APPLIANCE`` (only closed appliances) and, via an
    ``Inverter``, to skip it when ``APPLIANCE_OPENED`` is already set.

    N4 (round-5 rerun fix, bench log flood): ``quiet_on_falsy`` -- default
    False, preserving today's feedback for every other call site of this
    SHARED class -- suppresses the ``"{key} is not truthy"`` feedback on the
    FAILURE branch when set. ``create_goto()``'s fail-fast guard (ticked
    EVERY tick of EVERY goto, essentially the whole mission, and almost
    always FAILURE -- the normal, expected case) passes this: sim run 016
    logged ``gpsr/mission_unrecoverable is not truthy`` 6805x, and bench's
    run.json picked it as the failure DETAIL, hiding the true failure
    reason. FAILURE here is routine; only a SUCCESS (the flag actually
    latched True) is the exceptional, worth-logging case.
    """

    def __init__(self, name: str, key: str, *, quiet_on_falsy: bool = False):
        super().__init__(name)
        self._key = key
        self._quiet_on_falsy = quiet_on_falsy
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._key, access=Access.READ)

    def update(self):
        try:
            value = self._client.get(self._key)
        except Exception:
            value = None
        if bool(value):
            return Status.SUCCESS
        self.feedback_message = (
            "" if self._quiet_on_falsy else f"{self._key} is not truthy"
        )
        return Status.FAILURE


class BtNode_CheckGraspAllowed(Behaviour):
    """SUCCESS unless ``GRASP_ASK_REFEREE`` is truthy.

    Guards the primary grasp so it is skipped when the object sits on furniture
    the robot must not grasp from (shelf / cabinet / coat_rack). In that case
    this fails, the grasp Selector falls straight through to the ask-referee
    (deus-ex-machina) branch, and no arm scan / grasp is ever attempted.
    """

    def __init__(self, name: str = "grasp allowed?"):
        super().__init__(name)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.GRASP_ASK_REFEREE, access=Access.READ)

    def update(self):
        try:
            ask_referee = bool(self._client.get(bb_keys.GRASP_ASK_REFEREE))
        except Exception:
            ask_referee = False
        if ask_referee:
            self.feedback_message = "object is on no-grasp furniture (shelf/cabinet/coat_rack) — skipping to ask-referee"
            return Status.FAILURE
        return Status.SUCCESS


class BtNode_RefereeHandObject(Behaviour):
    """Ask the sim referee to teleport the target object into the gripper.

    P (task-P): SIM f0dff4c added referee actuation -- publish the target
    object's SEMANTIC name on ``/sim/referee/hand_object`` (std_msgs/String,
    reliable QoS); the sim resolves it to the spawned entity, teleports it
    to the TCP, and acks on ``/sim/referee/hand_object_result`` with
    ``{"ok": true|false, "entity": ..., "xyz"|"error": ...}``. This makes
    the ex_machina referee-fallback grasp physically real in sim, so the
    delivered/placed gates verify honestly instead of trusting a
    deterministic "close on air" SUCCESS (bench semantics: referee_assisted
    is a legitimate PASS path).

    Gated by ``GPSR_SIM_REFEREE_HANDOFF`` (truthy idiom matches
    ``_sim_identity_relaxed_enabled()``: ``os.environ.get(...) == "1"``),
    read once at construction -- never per-tick. Flag off (default): SUCCESS
    immediately, no publisher/subscription ever created -- this node is
    completely dead and the ex_machina branch is byte-identical to before
    this task.

    Flag on: publishes ``TARGET_OBJECT_NAME`` on entry (unset/empty -> log +
    immediate SUCCESS, never publishes, never blocks) and waits (RUNNING)
    for the ack up to ``GPSR_SIM_REFEREE_HANDOFF_TIMEOUT_S`` (default 3.0s).
    This node NEVER FAILs the branch -- an ok:true ack, an ok:false nack, a
    malformed/unparseable ack, and a timeout all resolve to SUCCESS (only
    the feedback message differs), so a bad handoff degrades gracefully:
    the flow proceeds to close-on-air and fails honestly at the
    delivered/placed gates instead of hanging here. The ack's entity/name is
    logged, never validated against what was asked for.

    Restart discipline: ``setup()`` creates the publisher/subscription (and
    reads the flag/timeout) at most once per instance -- re-entry after a
    root restart re-publishes (the sim teleport is idempotent) but never
    accumulates ROS entities.
    """

    HAND_OBJECT_TOPIC = "/sim/referee/hand_object"
    HAND_OBJECT_RESULT_TOPIC = "/sim/referee/hand_object_result"
    DEFAULT_TIMEOUT_S = 3.0

    def __init__(self, name: str = "sim referee hand-object"):
        super().__init__(name)
        self._enabled = os.environ.get("GPSR_SIM_REFEREE_HANDOFF") == "1"
        try:
            self._timeout_s = float(
                os.environ.get("GPSR_SIM_REFEREE_HANDOFF_TIMEOUT_S", self.DEFAULT_TIMEOUT_S)
            )
        except (TypeError, ValueError):
            self._timeout_s = self.DEFAULT_TIMEOUT_S
        self._client = None
        self._publisher = None
        self._subscription = None
        self._deadline = None
        self._ack_raw = None
        self._skip = False

    def setup(self, **kwargs):
        if not self._enabled:
            return
        if self._publisher is not None:
            # Already set up on an earlier call -- never accumulate a second
            # publisher/subscription across re-setups (K-round restart
            # discipline lesson).
            return
        try:
            node = kwargs['node']
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}]".format(self.qualified_name)
            raise KeyError(error_message) from e
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_OBJECT_NAME, access=Access.READ)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = node.create_publisher(String, self.HAND_OBJECT_TOPIC, qos)
        self._subscription = node.create_subscription(
            String, self.HAND_OBJECT_RESULT_TOPIC, self._on_ack, qos,
        )

    def _on_ack(self, msg) -> None:
        self._ack_raw = msg.data

    def initialise(self) -> None:
        self._ack_raw = None
        self._deadline = None
        self._skip = False
        if not self._enabled:
            return
        try:
            name = str(self._client.get(bb_keys.TARGET_OBJECT_NAME) or "").strip()
        except Exception:
            name = ""
        if not name:
            self._skip = True
            self.feedback_message = "TARGET_OBJECT_NAME unset -- skipping referee handoff"
            return
        msg = String()
        msg.data = name
        self._publisher.publish(msg)
        self._deadline = time.monotonic() + self._timeout_s
        self.feedback_message = f"waiting for referee handoff ack ({name!r})"

    def update(self) -> Status:
        if not self._enabled:
            self.feedback_message = "GPSR_SIM_REFEREE_HANDOFF != 1"
            return Status.SUCCESS
        if self._skip:
            return Status.SUCCESS
        if self._ack_raw is None:
            if time.monotonic() >= self._deadline:
                self.feedback_message = "no referee ack"
                return Status.SUCCESS
            return Status.RUNNING
        return self._resolve_ack(self._ack_raw)

    def _resolve_ack(self, raw: str) -> Status:
        try:
            payload = json.loads(raw)
        except Exception:
            payload = None
        if not isinstance(payload, dict):
            self.feedback_message = "referee handoff nack: malformed ack"
            return Status.SUCCESS
        if payload.get("ok"):
            entity = payload.get("entity")
            xyz = payload.get("xyz")
            self.feedback_message = f"referee handoff ok: entity={entity} xyz={xyz}"
        else:
            error = payload.get("error")
            self.feedback_message = f"referee handoff nack: {error}" if error else "referee handoff nack"
        return Status.SUCCESS


class BtNode_CheckSimIdentityRelaxed(Behaviour):
    """SUCCESS iff ``GPSR_SIM_IDENTITY_RELAXED=1``, else FAILURE.

    N2 (round-5 rerun fix): guards the generic-person scan branch's OWN
    ``BtNode_ScanForGeneralist`` call in ``_person_scan_strategies`` — unlike
    the existing relaxed_branch (a pure re-parse of an already-run scan, so
    it costs nothing extra when the flag is off), the generic branch issues
    a fresh VLM call. Placing this check in FRONT of that call — the same
    ``_sim_identity_relaxed_enabled()`` used by ``BtNode_ExtractDetection``'s
    own internal check — keeps the whole branch structurally dead (no scan
    ever fires) when the flag is off, instead of only failing after paying
    for the call.
    """

    def __init__(self, name: str = "sim identity relaxed?"):
        super().__init__(name)

    def update(self):
        from .validators import _sim_identity_relaxed_enabled

        if _sim_identity_relaxed_enabled():
            return Status.SUCCESS
        self.feedback_message = "GPSR_SIM_IDENTITY_RELAXED != 1"
        return Status.FAILURE


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


class PersistentFailureCap(py_trees.decorators.Decorator):
    """Caps CONSECUTIVE child FAILUREs at ``max_failures``, with a counter
    that SURVIVES root-restart re-entries.

    K2 (task-K, live-manipulation sim findings, F1): the orchestrator root is
    a memory Sequence that invalidates every child on ANY sibling FAILURE and
    restarts from child 0 on the next tick. An ordinary ``Retry`` decorator's
    ``initialise()`` re-zeroes its own attempt counter on every one of those
    restarts, so a persistently broken action (e.g. the arm controller
    rejecting every goal) never actually gives up -- a real run produced
    3000+ rejected arm goals and 8601 announce repeats over 1500s. This
    decorator wraps a (possibly already ``Retry``-wrapped) child and
    deliberately does NOT reset its failure counter in ``initialise()`` --
    failures accumulate ACROSS re-entries, not per-entry. A child SUCCESS
    resets the counter to 0.

    While under the cap this is a transparent pass-through of the child's
    status (RUNNING stays RUNNING, etc). On reaching the cap, ``on_exhausted``
    (if given) fires exactly once, and from then on this returns FAILURE
    immediately WITHOUT ticking the child again -- no further goals / per-
    attempt side effects, ever.

    A pure single-child ``py_trees.decorators.Decorator`` (task-K review,
    V-3): an earlier revision appended a second ``announce_child`` to
    ``self.children`` to deliver a one-time speech reaction on exhaustion,
    which relied on ``Decorator.stop()``/``tip()`` only ever touching
    ``self.decorated`` (``children[0]``) -- an implementation detail of the
    installed py_trees version, not a documented contract. The one-time
    reaction now lives in ``create_goto()`` instead, as a standard sibling
    Selector branch gated on a blackboard flag ``on_exhausted`` sets (see
    ``bb_keys.MISSION_UNRECOVERABLE`` / V-2) -- this class stays a plain,
    generic single-child cap with no assumptions about what ``on_exhausted``
    does.
    """

    def __init__(self, name: str, child: py_trees.behaviour.Behaviour,
                 max_failures: int, on_exhausted=None):
        super().__init__(name=name, child=child)
        self.max_failures = max_failures
        self.on_exhausted = on_exhausted
        self.consecutive_failures = 0
        self.exhausted = False
        self._fired = False

    def initialise(self) -> None:
        # Deliberately does NOT reset consecutive_failures / exhausted /
        # _fired -- see class docstring; that persistence across re-entries
        # is the entire point of this decorator.
        pass

    def tick(self):
        self.logger.debug(f"{self.__class__.__name__}.tick()")
        if self.status != Status.RUNNING:
            self.initialise()

        if self.exhausted:
            # Cap already reached in a previous entry: do NOT tick the child
            # again (no more goals / side effects). V-4 (task-K review):
            # only transition (call stop()) once, not on every tick spent in
            # this terminal state -- normal py_trees usage calls stop() once
            # per status CHANGE, and a future terminate() override must not
            # fire on every tick forever.
            if self.status != Status.FAILURE:
                self.stop(Status.FAILURE)
            self.status = Status.FAILURE
            yield self
            return

        yield from self.decorated.tick()
        new_status = self.decorated.status

        if new_status == Status.SUCCESS:
            self.consecutive_failures = 0
        elif new_status == Status.FAILURE:
            self.consecutive_failures += 1
            if self.consecutive_failures >= self.max_failures:
                self.exhausted = True
                if not self._fired:
                    self._fired = True
                    if self.on_exhausted is not None:
                        # V-5 (task-K review): a callback's own bug must
                        # never break the tree's tick -- this decorator is a
                        # generic, reusable cap, not just the one call site
                        # whose current callback happens to already guard
                        # its own body.
                        try:
                            self.on_exhausted()
                        except Exception as exc:  # noqa: BLE001
                            self.logger.error(
                                f"{self.__class__.__name__} '{self.name}': "
                                f"on_exhausted() raised: {exc!r}"
                            )

        if new_status != Status.RUNNING:
            self.stop(new_status)
        self.status = new_status
        yield self

    def update(self) -> Status:  # pragma: no cover - custom tick owns status
        return self.status


# ---------------------------------------------------------------------------
# Small-tree factories
# ---------------------------------------------------------------------------

def _tuck_arm_for_nav(label: str = "tuck arm for nav",
                      pose_key: str = None):
    """Move the arm to a stow pose before driving.

    The arm must be folded back so it does not block the lidar / occupy the
    robot's footprint during base motion. Every small tree that issues a base
    move (goto / follow / guide / approach / deliver / place) tucks first.

    ``pose_key`` selects which pose to stow to (defaults to
    ``bb_keys.ARM_ORBBEC_LOOK``, the general stow/orbbec-look pose). The
    ``goto`` tree passes ``bb_keys.ARM_NAVIGATING`` so a pure navigation always
    parks the arm in the dedicated lidar-clearing navigating pose before moving.
    Retry-wrapped and propagates failure: if the arm cannot tuck, we do NOT
    drive with the arm sticking out — the orchestrator self-correction handles it.
    """
    return py_trees.decorators.Retry(
        f"retry {label}",
        BtNode_MoveArmSingle(
            label,
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=pose_key or bb_keys.ARM_ORBBEC_LOOK,
            add_octomap=False,
        ),
        num_failures=3,
    )


def _arm_to_orbbec_look(label: str = "arm to orbbec look"):
    """Move the arm to ``arm_pos_orbbec_look`` so it clears the ORBBEC head
    camera's field of view before an orbbec scan (find_object / count / vlm /
    find_person / describe_person). This is NOT the arm-mounted RealSense grasp
    scan, which needs the arm at ``ARM_SCAN`` — that path is left untouched.

    Best-effort: Retry-wrapped and FailureIsSuccess so a MoveIt hiccup (or an
    un-seeded ARM_ORBBEC_LOOK key) never fails the scan. The pose is read from
    ``bb_keys.ARM_ORBBEC_LOOK``, seeded once at startup alongside the nav/scan
    poses; if it is unset the move simply no-ops and the scan proceeds.
    """
    return py_trees.decorators.FailureIsSuccess(
        f"{label} (best effort)",
        py_trees.decorators.Retry(
            f"retry {label}",
            BtNode_MoveArmSingle(
                label,
                action_name=ARM_ACTION_NAME,
                arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
                add_octomap=False,
            ),
            num_failures=3,
        ),
    )


def _pantilt_sweep(label: str, tilts, make_detect, pan_deg=None):
    """Sweep the pan-tilt across every (pan, tilt) combination until the
    detection subtree ``make_detect()`` SUCCEEDS.

    Iterates ``tilts`` (outer) × ``pan_deg`` (inner, default ``PAN_SWEEP_DEG``),
    so the robot does a full left/centre/right pan at the first tilt, then
    repeats at the next tilt, stopping the instant a combination detects the
    target. ``make_detect`` is a zero-arg factory that must return a FRESH
    detection behaviour on each call — a py_trees node can only occupy one slot
    in the tree, so every branch needs its own instance. ``tilts`` picks the
    look range: ``HUMAN_TILT_DEG`` (up at a standing person) or
    ``OBJECT_TILT_DEG`` (level/down at a surface). ``pan_deg`` overrides the pan
    sweep when a caller (e.g. a ``pan-tilt-sweep`` modification) needs a
    different range.

    Returns a memory Selector: SUCCESS the moment a combination detects the
    target, FAILURE only after every (pan, tilt) has been tried with no hit.
    """
    pans = PAN_SWEEP_DEG if pan_deg is None else [float(p) for p in pan_deg]
    sweep = py_trees.composites.Selector(f"{label} pantilt sweep", memory=True)
    for tilt in tilts:
        for pan in pans:
            branch = py_trees.composites.Sequence(
                f"{label} pan={pan:+.0f} tilt={tilt:+.0f}", memory=True)
            branch.add_child(BtNode_TurnPanTilt(
                f"pan {pan:+.0f} tilt {tilt:+.0f}", x=pan, y=tilt,
            ))
            branch.add_child(make_detect())
            sweep.add_child(branch)
    return sweep


def _one_shot(
    child: py_trees.behaviour.Behaviour,
    policy: py_trees.common.OneShotPolicy = py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
) -> py_trees.decorators.OneShot:
    """Wrap ``child`` so it runs through to completion only ONCE.

    K1 (task-K, live-manipulation sim findings, F1): the orchestrator root is
    a ``py_trees.composites.Sequence(memory=True)`` ticked forever. A memory
    Sequence resumes only from a RUNNING child -- on FAILURE every child is
    invalidated and the next tick restarts from child 0. When a LATER sibling
    keeps failing (e.g. the arm action server rejecting every goal), an
    ALREADY-SUCCEEDED earlier child (like the entry-arena subtree) would
    otherwise be re-ticked in full on every one of those restarts -- re-
    announcing, re-detecting, etc. ``OneShot(ON_SUCCESSFUL_COMPLETION)``
    fixes that: once ``child`` SUCCEEDS, later ticks bounce back the
    memorised SUCCESS without ever ticking ``child`` again.

    Named after ``child`` (not a fixed literal) so wrapping a subtree at its
    call site does not change what shows up in name-based lookups (tests,
    ``tree_serialization``, the tick visualizer) -- the wrapper is invisible
    by name, only its behaviour differs.

    ``policy`` defaults to ``ON_SUCCESSFUL_COMPLETION`` (only a SUCCESS
    latches; a FAILURE keeps retrying on the next entry -- what entry-arena
    needs). K2 passes ``ON_COMPLETION`` for the exhaustion announcement,
    where a single ATTEMPT (success or failure) must be enough -- it must
    never re-announce even if the TTS call itself happens to fail.
    """
    return py_trees.decorators.OneShot(child.name, child, policy)


def create_enter_arena():
    """Enter the arena ONCE, at mission start.

    GPSR starts the robot OUTSIDE the arena, standing in front of the (usually
    closed) entrance door. This KEEPS SENSING the door until it opens:
    ``BtNode_DoorDetection`` returns FAILURE while the door is closed, and the
    ``FailureIsRunning`` decorator maps that to RUNNING, so the node is re-ticked
    (re-sensed) every tick and the tree simply waits — it never gives up and
    never proceeds through a closed door. Once the door reads open the node
    succeeds, we announce, and latch ``ARENA_ENTERED`` so the whole step is
    skipped on every later command round. The robot only crosses the threshold
    once; subsequent rounds return to the command point, never back to the door.

    In mock mode the detection auto-succeeds immediately.
    """
    detect = py_trees.composites.Sequence("detect door + enter", memory=True)
    # Greet + request the door ONCE, before sensing. The memory Sequence runs
    # this to SUCCESS then advances to the door watch and stays there while the
    # door is closed, so the announcement plays exactly once (not every tick).
    detect.add_child(BtNode_Announce(
        "announce ready for gpsr", bb_source=None,
        message="Hi, I am Tinker. I am ready for GPSR. Please open the door.",
    ))
    detect.add_child(py_trees.decorators.FailureIsRunning(
        "keep sensing until door opens",
        BtNode_DoorDetection(
            "detect open door", bb_door_state_key=bb_keys.DOOR_STATUS,
        ),
    ))
    detect.add_child(BtNode_Announce(
        "announce entering arena", bb_source=None,
        message="The door is open. Entering the arena.",
    ))
    detect.add_child(BtNode_BlackboardSet(
        "latch arena entered", bb_keys.ARENA_ENTERED, True,
    ))
    # Run detect at most once: after the first success ARENA_ENTERED is set, so
    # the guard short-circuits the Selector on every later round.
    once = py_trees.composites.Selector("enter arena (once)", memory=False)
    once.add_child(BtNode_CheckBBKeySet("already entered arena?", bb_keys.ARENA_ENTERED))
    once.add_child(detect)
    return once


# Reassurance lines the robot cycles through WHILE navigating. A slow route plan
# would otherwise leave the robot silent, and a referee may shut the task down if
# it looks like it hung — so it keeps talking until the drive lands.
GOTO_KEEPALIVE_LINES = [
    "I am trying to go to the destination.",
    "I am planning, please wait for a while.",
    "Please give me a moment to arrive at the destination.",
]
# Silence between keep-alive lines. The loop MUST NOT announce back to back: a long
# or abnormal navigation (recovery, replanning, blocked path) drags on far longer
# than a normal trip, and un-paced announces flood the TTS service until it stops
# producing audio — going silent EXACTLY when the heartbeat matters most (the robot
# looks hung and a referee may stop the task). A fixed wall-clock gap paces it so it
# keeps talking for the whole drive, however long recovery takes.
GOTO_KEEPALIVE_INTERVAL_SEC = 5.0


def _goto_keepalive_announcer() -> py_trees.behaviour.Behaviour:
    """A never-terminating announcer that loops ``GOTO_KEEPALIVE_LINES`` with a
    fixed ~``GOTO_KEEPALIVE_INTERVAL_SEC`` gap between lines.

    Each line is spoken (blocking), then a wall-clock ``Timer`` waits so the next
    line comes ~5 s later, never back to back — this is what keeps a long/abnormal
    navigation from flooding TTS into silence. Each announce is wrapped in
    ``FailureIsSuccess`` so a TTS hiccup neither skips its pacing gap nor aborts the
    parallel; the ``Timer`` therefore ALWAYS runs. The memory Sequence never returns
    FAILURE (announces are best-effort, timers only SUCCEED), and ``SuccessIsRunning``
    turns its completion back into RUNNING so it re-enters from the first line — an
    endless, paced loop that is always RUNNING. The parallel is thus driven purely by
    the goto child; this just fills the silence for the ENTIRE drive, recovery
    included (the drive stays RUNNING through Nav2 recovery, so the parallel — and
    this announcer — keep ticking).
    """
    lines = py_trees.composites.Sequence("keep-alive lines", memory=True)
    for i, msg in enumerate(GOTO_KEEPALIVE_LINES):
        lines.add_child(py_trees.decorators.FailureIsSuccess(
            f"say keepalive {i} (best-effort)",
            BtNode_Announce(f"nav keepalive {i}", bb_source=None, message=msg),
        ))
        lines.add_child(py_trees.timers.Timer(
            f"keepalive gap {i}", duration=GOTO_KEEPALIVE_INTERVAL_SEC,
        ))
    return py_trees.decorators.SuccessIsRunning("loop nav keepalive", lines)


def _tuck_arm_before_goto_exhausted() -> None:
    """K2 ``on_exhausted`` for the goto tuck-arm ``PersistentFailureCap``.

    Fires once the tuck-arm node has FAILED ``max_failures`` (5) times in a
    row -- at 3 retries/attempt that is at most 15 rejected arm goals total,
    instead of the 3000+ a broken arm controller produced in a real run
    (F1). Emits telemetry AND latches ``bb_keys.MISSION_UNRECOVERABLE`` (V-2,
    task-K review): ``create_goto()``'s own fail-fast guard and its
    "announce unrecoverable" branch both key off this flag, so the ENTIRE
    goto subtree -- not just the tuck-arm retry -- goes quiet on every root
    restart from here on, and the exhaustion speech is delivered exactly
    once via the tree's own tick loop (see ``create_goto()``) rather than
    from this synchronous callback, which cannot drive a multi-tick ROS
    speech call to completion on its own.
    """
    telemetry = get_default_telemetry()
    if telemetry is not None:
        try:
            telemetry.emit(
                "mission.unrecoverable",
                {
                    "reason": "arm goal rejected/aborted repeatedly",
                    "node": "tuck arm before goto",
                },
                phase="execution",
            )
        except Exception:
            pass
    # A plain module-level function, not a Behaviour -- no blackboard client
    # of its own to register a key on. The static Blackboard.set() writes
    # straight to storage (bypassing client access-control), which is fine
    # here: this key is WRITE-once-ever, read back only via BtNode_CheckBBTrue
    # (its own registered client) inside create_goto()'s guard/announce branch.
    py_trees.blackboard.Blackboard.set(bb_keys.MISSION_UNRECOVERABLE, True)


def create_goto():
    """Navigate to ``bb_keys.TARGET_POSE`` (filled by orchestrator).

    While the drive is in progress the robot loops ``GOTO_KEEPALIVE_LINES`` in
    parallel with the nav, so a slow route plan never leaves it silent.
    """
    seq = py_trees.composites.Sequence("small/goto", memory=True)
    # V-2 (task-K review): fail fast, as the FIRST child, once the tuck-arm
    # cap below has given up for good (bb_keys.MISSION_UNRECOVERABLE latched
    # True) -- silences "announce going" and every other side-effectful node
    # in this subtree on every LATER root restart, not just the tuck retry.
    # Tick interplay on the exhaustion restart itself: the flag is only set
    # mid-tick, inside the cap below, so THIS guard has already passed for
    # that one tick; the memory Sequence then resumes directly at the
    # (possibly RUNNING) announce branch until it completes. Only from the
    # NEXT restart does this guard actually block anything -- the intended
    # quiet loop.
    seq.add_child(py_trees.decorators.Inverter(
        "mission still recoverable? (fail-fast guard)",
        BtNode_CheckBBTrue(
            "mission unrecoverable?", bb_keys.MISSION_UNRECOVERABLE,
            quiet_on_falsy=True,
        ),
    ))
    seq.add_child(BtNode_AnnounceFromBB(
        "announce going", bb_keys.TARGET_LOCATION, prefix="Going to "
    ))
    # goto is pure navigation: always park the arm in the dedicated navigating
    # (lidar-clearing) pose before driving, not the orbbec-look stow pose.
    # K2: cap the Retry(num_failures=3) tuck-arm node at 5 CONSECUTIVE
    # failures that survive root restarts (F1: an ordinary decorator's
    # initialise() would re-zero the Retry's own counter every ~500ms while
    # the root keeps failing/restarting, so it never actually gives up).
    # V-2 (task-K review): the cap alone only stops arm goals -- it does NOT
    # by itself silence the rest of this subtree (that is the fail-fast
    # guard above's job, once the flag is latched). This Selector's second
    # branch delivers the one-time exhaustion speech exactly once, gated on
    # the SAME flag the cap's on_exhausted sets, then keeps the branch (and
    # so this whole goto) FAILED so the tuck never counts as satisfied.
    seq.add_child(py_trees.composites.Selector(
        "tuck arm or abort",
        memory=True,
        children=[
            PersistentFailureCap(
                "tuck arm before goto (failure cap)",
                _tuck_arm_for_nav("tuck arm before goto", pose_key=bb_keys.ARM_NAVIGATING),
                max_failures=5,
                on_exhausted=_tuck_arm_before_goto_exhausted,
            ),
            py_trees.composites.Sequence(
                "announce unrecoverable",
                memory=True,
                children=[
                    BtNode_CheckBBTrue(
                        "mission unrecoverable? (announce gate)",
                        bb_keys.MISSION_UNRECOVERABLE,
                    ),
                    _one_shot(
                        BtNode_Announce(
                            "announce arm unrecoverable", bb_source=None,
                            message=(
                                "I cannot move my arm and cannot continue. "
                                "Please check the arm controller."
                            ),
                        ),
                        policy=py_trees.common.OneShotPolicy.ON_COMPLETION,
                    ),
                    py_trees.behaviours.Failure("stay failed"),
                ],
            ),
        ],
    ))
    drive = py_trees.decorators.Retry(
        "retry goto",
        BtNode_GotoAction("goto target", key=bb_keys.TARGET_POSE),
        num_failures=5,
    )
    # Drive + keep-alive chatter run together; the parallel finishes EXACTLY when
    # the drive does (SuccessOnSelected targets the drive child). The announcer
    # never succeeds or fails, so it neither ends the parallel early nor aborts it.
    seq.add_child(py_trees.composites.Parallel(
        "goto + keep talking",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected(
            children=[drive], synchronise=False,
        ),
        children=[drive, _goto_keepalive_announcer()],
    ))
    return seq


def _object_scan_and_verify():
    """Fresh object-detection subtree for one pan/tilt: generalist scan on the
    orbbec, then verify at least one match (FAILURE if none, so the sweep moves
    on). Returns new node instances each call so it drops into every sweep
    branch. Locate-only — it does NOT pick a single instance; choosing the
    object to grasp is the grasp tree's job (arm-camera re-detect)."""
    seq = py_trees.composites.Sequence("object scan+verify", memory=True)
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
    return seq


class BtNode_ReduceObjectQuery(Behaviour):
    """One-shot fallback: reduce an unknown, attribute-laden object query to
    its known-object token subset (e.g. "red bowl" -> "bowl") after the
    strict pantilt sweep has exhausted every angle with no match.

    L1b (round-4 battery fix, run 016): sim battery run 016 planned
    ``object: "red bowl"`` for a command that only ever said "bowl" — four
    full pan/tilt sweeps for "red bowl" never matched anything in the scene
    (the spawned YCB bowl is white). ``create_find_object`` puts a fresh
    instance of this node ahead of a SECOND pantilt sweep, both as the
    second branch of a Selector alongside the original (strict) sweep — see
    that function.

    K-round lesson (``PersistentFailureCap``, task-K): the orchestrator's
    memory-Sequence root re-ticks already-terminal subtrees on ANY later
    sibling's FAILURE. This node's "already reduced" flag is a plain
    instance attribute, deliberately never reset in ``initialise()``
    (mirroring ``PersistentFailureCap``) — a root restart replays the
    sweep, but a SECOND reduction is never re-triggered: this can SUCCEED
    at most once per ``create_find_object`` subtree instance ("per sweep").
    """

    def __init__(self, name: str = "reduce object query"):
        super().__init__(name)
        self._client = None
        self._reduced = False

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_OBJECT_PROMPT, access=Access.WRITE)

    def update(self):
        if self._reduced:
            self.feedback_message = "reduced query already attempted this sweep"
            return Status.FAILURE
        from .orchestrator import reduce_unknown_object_query  # lazy: avoid import cycle
        try:
            prompt = self._client.get(bb_keys.TARGET_OBJECT_PROMPT)
        except Exception as exc:
            self.feedback_message = f"no {bb_keys.TARGET_OBJECT_PROMPT} to reduce: {exc}"
            return Status.FAILURE
        reduced = reduce_unknown_object_query(prompt)
        if reduced is None:
            self.feedback_message = f'no known-object reduction for "{prompt}"'
            return Status.FAILURE
        self._reduced = True
        self._client.set(bb_keys.TARGET_OBJECT_PROMPT, reduced, overwrite=True)
        self.feedback_message = f'reduced query "{prompt}" -> "{reduced}"'
        return Status.SUCCESS


def create_find_object(pan_deg=None, tilt_deg=None):
    """Scan for the object named in ``bb_keys.TARGET_OBJECT_PROMPT``.

    Sweeps the pan-tilt across ``PAN_SWEEP_DEG`` × ``OBJECT_TILT_DEG`` (level,
    then angled down at a surface), scanning + verifying at each angle and
    stopping at the first (pan, tilt) where the object is seen — so an object
    off to the side or on a higher/lower surface is still found without moving
    the base. Locate-only: it does NOT pick a single instance. ``pan_deg`` /
    ``tilt_deg`` override the sweep ranges (used by ``pan-tilt-sweep``
    modifications).

    L1b (round-4 battery fix, run 016): when the full sweep exhausts every
    (pan, tilt) with no match AND the query is an unknown, attribute-laden
    one that token-subset-reduces to a known object name (e.g. "red bowl" ->
    "bowl"), a second Selector branch retries the WHOLE sweep exactly once
    with the reduced query (``BtNode_ReduceObjectQuery``) before this
    ultimately fails. No reduction possible -> behaves exactly as before.
    """
    seq = py_trees.composites.Sequence("small/find_object", memory=True)
    seq.add_child(_arm_to_orbbec_look())  # clear the arm from the orbbec's view
    tilts = OBJECT_TILT_DEG if tilt_deg is None else [float(t) for t in tilt_deg]
    sweep_with_reduction = py_trees.composites.Selector(
        "find_object sweep with query reduction", memory=True,
    )
    sweep_with_reduction.add_child(_pantilt_sweep(
        "find_object", tilts, _object_scan_and_verify, pan_deg=pan_deg,
    ))
    reduced_retry = py_trees.composites.Sequence(
        "find_object reduced-query retry", memory=True,
    )
    reduced_retry.add_child(BtNode_ReduceObjectQuery())
    reduced_retry.add_child(_pantilt_sweep(
        "find_object", tilts, _object_scan_and_verify, pan_deg=pan_deg,
    ))
    sweep_with_reduction.add_child(reduced_retry)
    seq.add_child(sweep_with_reduction)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce found", bb_keys.TARGET_OBJECT_NAME, prefix="I can see the "
    ))
    return seq


class SearchObjectSelector(py_trees.composites.Selector):
    """F2 (round-2 review, fix round 2): ``small/search_object``'s root.

    A stock memory Selector (SUCCESS on the first spot with a match, FAILURE
    only once every filled spot has been swept) EXCEPT: on terminating
    FAILURE it replaces its own ``feedback_message`` with a one-line summary
    of how many spots were actually swept, instead of leaving whatever the
    tip's internal guard message was.

    Without this, ``orchestrator._last_child_feedback`` -- which calls
    ``node.tip()`` and reports ITS ``feedback_message`` -- surfaces the tip
    of the LAST branch tried, which for any room with fewer search spots
    than ``MAX_SEARCH_SPOTS`` is an unfilled slot's ``BtNode_CheckBBKeySet``
    guard: ``"gpsr/search_pose_5 is None"``. That is an internal blackboard
    key, meaningless to an operator or a replanning LLM. ``tip()`` is
    overridden the same way: once this node has failed and set its own
    message, IT becomes the reported tip instead of delegating further down.
    Selector semantics (class name keeps "Selector" so
    ``tree_serialization._semantics`` still classifies it as one; node ids
    stay purely path-based so ``_classify_node_roles``'s
    ``small/search_object/root`` -> ``search_object_sweep`` role is
    unaffected) are otherwise untouched -- structural only, no behaviour
    change on SUCCESS.

    H1 (round-2 review, fix round 2): the summary MUST be computed
    synchronously inside the tick, before this node yields itself, not
    after ``super().tick()``'s generator returns. Under the production
    dispatcher this node sits as the second child of a memory
    ``Sequence`` (``create_dispatcher`` -> ``branch:search_object``).
    ``Sequence.tick()`` closes the child's generator (``GeneratorExit`` at
    its last ``yield``) the instant the child yields itself with a
    non-SUCCESS status -- code written to run "after the loop" in a
    ``tick()`` override never executes on that path, only when something
    (e.g. a test) exhausts the generator directly. ``Selector.tick()``,
    however, calls ``self.stop(FAILURE)`` -- which calls
    ``self.terminate(FAILURE)`` -- BEFORE it yields itself, so computing
    the summary in ``terminate()`` guarantees it is set before any parent
    ever observes this node's terminal yield.

    H2: the guard's status is not reliable to inspect *after* the sweep
    finishes -- a memory Selector invalidates every child before
    ``current_child`` at the START of each tick, so once branch N goes
    RUNNING (a multi-tick goto/scan), branch 0..N-1's guards flip to
    INVALID before the sweep concludes. Track which guards succeeded
    incrementally instead: a tick() override (kept ONLY for this
    observation, not for the summary itself) records branch index ``i``
    into ``self._swept`` the moment ``self.children[i].children[0]`` is
    yielded with status SUCCESS -- capturing it before any later
    invalidation can hide it.
    """

    def __init__(self, name: str, capacity: int):
        super().__init__(name, memory=True)
        self._capacity = capacity
        self._client = None
        self._swept: set[int] = set()

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)

    def initialise(self):
        super().initialise()
        # L2 (round-2 review): clear stale state from a prior sweep so a
        # FAILURE -> SUCCESS re-tick doesn't leave the old "swept ... nothing
        # found" message behind.
        self._swept = set()
        self.feedback_message = ""

    def tick(self):
        # H2: observation-only. The actual summary is computed in
        # terminate(), which py_trees guarantees runs before this node's
        # own terminal yield (see class docstring).
        for node in super().tick():
            if node.status == Status.SUCCESS:
                for i, branch in enumerate(self.children):
                    if branch.children and node is branch.children[0]:
                        self._swept.add(i)
                        break
            yield node

    def terminate(self, new_status):
        super().terminate(new_status)
        if new_status == Status.FAILURE:
            self._summarise_sweep()

    def tip(self):
        if self.status == Status.FAILURE and self.feedback_message:
            return self
        return super().tip()

    def _summarise_sweep(self) -> None:
        try:
            location = self._client.get(bb_keys.TARGET_LOCATION)
        except Exception:
            # L2 (round-2 review): setup() was never called (e.g. a
            # serialisation/role-audit path that never runs the tree), so
            # _client is None. Fall back to the static blackboard accessor
            # rather than reporting "at None".
            try:
                location = Blackboard.get(bb_keys.TARGET_LOCATION)
            except Exception:
                location = None
        self.feedback_message = (
            f"search_object: swept {len(self._swept)} of {self._capacity} spots "
            f"at {location}, nothing found"
        )


def create_search_object(capacity: int = MAX_SEARCH_SPOTS):
    """Sweep a room's search spots until the target object is located.

    The finder for FETCH / GRASP tasks. For "fetch a coke from the living room"
    where the exact in-room spot is unknown, visit each pose in the location's
    search-spot list (materialised by the orchestrator into SEARCH_POSE_0..N),
    tucking the arm and driving to each, then scanning; stop at the FIRST spot
    where the object is seen (the robot is then parked there with the object in
    view for grasp). SUCCESS = found; FAILURE = swept every spot, none found.

    Built at fixed capacity (``MAX_SEARCH_SPOTS``; ``capacity`` overrides it,
    e.g. via the ``search-spots`` modification) because the dispatcher
    constructs each small tree once. The orchestrator fills only the
    SEARCH_POSE_i keys the location has; unfilled slots are guarded out by
    BtNode_CheckBBKeySet so they neither navigate nor count as "found". A
    memory Selector returns SUCCESS on the first branch that succeeds, FAILURE
    only if all branches fail.
    """
    cap = max(1, int(capacity))
    pose_keys = [f"gpsr/search_pose_{i}" for i in range(cap)]
    sweep = SearchObjectSelector("small/search_object", capacity=cap)
    for i, pose_key in enumerate(pose_keys):
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


def _person_scan_strategies(extra_specialist=None):
    """Fresh person-detection Selector for one pan angle.

    Waving-person specialist first (only when the descriptor mentions waving),
    then an optional attribute-specialist branch (only when the descriptor
    mentions ``extra_specialist["gate"]``, scanning with the pinned prompt), and
    finally the generalist vision scan with a prompt built from the descriptor.
    Returns NEW node instances each call so it can be dropped into every branch
    of the pan-tilt sweep (see ``_pantilt_sweep``).
    """
    # F3 (round-2 review): memory=True. Every branch here is a memory
    # Sequence whose tail is a multi-tick async ServiceHandler (waving scan /
    # attribute-specialist scan / generalist scan). A non-memory Selector
    # re-enters child 0 on EVERY tick regardless of which branch is actually
    # in flight; whichever higher-priority branch resolves fast (a guard
    # that fails, or a specialist scan that resolves within the same tick)
    # gets fully re-run from scratch each cycle while a lower-priority
    # branch's in-flight async call never gets a chance to be revisited
    # long enough to complete — a livelock (sim run 005: stuck at pan/tilt
    # pose #1 for 116 cycles, ~8.6 minutes, never advancing). memory=True
    # makes the Selector resume directly at whichever branch last went
    # RUNNING, matching every other async-leaf Selector in this file (e.g.
    # the pantilt sweep itself, `_pantilt_sweep`, is already memory=True).
    selector = py_trees.composites.Selector("person scan strategies", memory=True)
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
    if extra_specialist:
        gate = str(extra_specialist.get("gate") or "").strip()
        prompt = str(extra_specialist.get("prompt") or "").strip()
        if gate:
            selector.add_child(_attribute_person_specialist_branch(gate, prompt))
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
    # L2a/L3 (round-4 battery fix, runs 008/011/019): under
    # GPSR_SIM_IDENTITY_RELAXED=1, when the strict descriptor-specific
    # extract above fails, accept the best GENERIC person-class detection
    # from the SAME scan response (BtNode_ScanForGeneralist already wrote it
    # to TARGET_PERSON_DETECTION before returning FAILURE — see that node's
    # docstring — so no second scan is needed here) and materialize a real
    # pose + provenance for it. Applies to every descriptor including waving
    # ones (L3): the waving specialist branch above still runs FIRST and
    # wins whenever it CAN detect waving; this is only reached once BOTH the
    # specialist and the strict generalist match have failed. Dead
    # (FAILURE) whenever the env flag is off — BtNode_ExtractDetection's
    # relaxed mode checks it itself.
    relaxed_branch = py_trees.composites.Sequence(
        "generalist relaxed person scan", memory=True,
    )
    relaxed_branch.add_child(BtNode_ExtractDetection(
        "pick relaxed generic person",
        bb_detection_src=bb_keys.TARGET_PERSON_DETECTION,
        bb_object_dst=bb_keys.TARGET_OBJECT,
        bb_point_dst=bb_keys.TARGET_PERSON_POSE,
        relaxed=True,
        bb_provenance_dst=bb_keys.PERSON_PROVENANCE,
    ))
    selector.add_child(relaxed_branch)
    # N2 (round-5 rerun fix, sim run 019): the re-parse above only has
    # something to relax when the STRICT scan above actually returned
    # objects (just none descriptor-matching); a zero-match scan
    # ("Scan returned 0 objects" x5 in 019, relaxed_generic never appearing)
    # leaves it nothing to work with. Last resort before the whole selector
    # fails: run one more, bare-"person" scan of our own (own bb key so we
    # never clobber PERSON_VISION_PROMPT, which other branches re-read on
    # restarts) and relax-extract THAT response instead. Placed after
    # relaxed_branch on purpose (guarded FIRST by BtNode_CheckSimIdentityRelaxed,
    # dead/no extra VLM call when the flag is off) so a person the strict
    # query already found costs no extra call — this is only reached once
    # generalist_branch AND the cheap re-parse have both failed.
    generic_scan_branch = py_trees.composites.Sequence(
        "generic person scan", memory=True,
    )
    generic_scan_branch.add_child(BtNode_CheckSimIdentityRelaxed())
    generic_scan_branch.add_child(BtNode_BlackboardSet(
        "pin generic person prompt",
        bb_keys.PERSON_VISION_PROMPT_GENERIC,
        "person",
    ))
    generic_scan_branch.add_child(BtNode_ScanForGeneralist(
        name="generic person scan",
        bb_source=bb_keys.PERSON_VISION_PROMPT_GENERIC,
        bb_key=bb_keys.TARGET_PERSON_DETECTION,
        use_orbbec=True,
        transform_to_map=True,
        sort_closest=True,
    ))
    generic_scan_branch.add_child(BtNode_ExtractDetection(
        "pick relaxed generic person (fresh scan)",
        bb_detection_src=bb_keys.TARGET_PERSON_DETECTION,
        bb_object_dst=bb_keys.TARGET_OBJECT,
        bb_point_dst=bb_keys.TARGET_PERSON_POSE,
        relaxed=True,
        bb_provenance_dst=bb_keys.PERSON_PROVENANCE,
    ))
    selector.add_child(generic_scan_branch)
    return selector


def _attribute_person_specialist_branch(gate: str, prompt: str = ""):
    """Fresh descriptor-gated attribute branch for the person scan strategies.

    Gated on the descriptor mentioning ``gate`` (e.g. "red jacket"); when it
    fires it pins ``PERSON_VISION_PROMPT`` to ``prompt`` (default "person
    <gate>") so the attribute reliably reaches the generalist VLM, scans, and
    extracts the detection. Returns NEW node instances each call so a caller
    (the ``attribute-person-specialist`` modification) can insert a fresh copy
    into every pan-tilt strategy selector.
    """
    gate = gate.strip()
    prompt = (prompt or f"person {gate}").strip()
    branch = py_trees.composites.Sequence(f"{gate} person branch", memory=True)
    branch.add_child(BtNode_CheckBBContains(
        f"descriptor mentions {gate}?", bb_keys.TARGET_PERSON_PROMPT, gate,
    ))
    branch.add_child(BtNode_BlackboardSet(
        f"pin {gate} prompt", bb_keys.PERSON_VISION_PROMPT, prompt,
    ))
    branch.add_child(BtNode_ScanForGeneralist(
        name=f"{gate} specialist scan",
        bb_source=bb_keys.PERSON_VISION_PROMPT,
        bb_key=bb_keys.TARGET_PERSON_DETECTION,
        use_orbbec=True,
        transform_to_map=True,
        sort_closest=True,
    ))
    branch.add_child(BtNode_ExtractDetection(
        f"pick {gate} person",
        bb_detection_src=bb_keys.TARGET_PERSON_DETECTION,
        bb_object_dst=bb_keys.TARGET_OBJECT,
        bb_point_dst=bb_keys.TARGET_PERSON_POSE,
    ))
    return branch


def create_find_person(extra_specialist=None, pan_deg=None, tilt_deg=None):
    """Scan for a person matching ``bb_keys.TARGET_PERSON_PROMPT`` and store
    their pose. Locate-only — it does NOT move the robot.

    Sweeps the pan-tilt across every (pan, tilt) in ``PAN_SWEEP_DEG`` ×
    ``HUMAN_TILT_DEG`` (up at a standing person, then lower), running the
    person-scan strategies at each angle and stopping at the first combination
    where a person is seen — so a person off to the side or a different height
    is still found without moving the base. The waving-person specialist only
    runs when the descriptor mentions waving; an optional ``extra_specialist``
    (``{"gate": ..., "prompt": ...}``, from the ``attribute-person-specialist``
    modification) adds a descriptor-gated attribute branch; otherwise the
    generalist scans with a prompt built from the descriptor. To stand next to
    the person, the planner emits ``approach_person`` separately.
    """
    seq = py_trees.composites.Sequence("small/find_person", memory=True)
    seq.add_child(BtNode_BuildPersonPrompt(
        "descriptor to vision prompt",
        bb_descriptor_key=bb_keys.TARGET_PERSON_PROMPT,
        bb_prompt_key=bb_keys.PERSON_VISION_PROMPT,
    ))
    seq.add_child(BtNode_Announce(
        "announce searching", bb_source=None,
        message="Looking for a person, please stay still.",
    ))
    seq.add_child(_arm_to_orbbec_look())  # clear the arm from the orbbec's view
    seq.add_child(_pantilt_sweep(
        "find_person",
        HUMAN_TILT_DEG if tilt_deg is None else [float(t) for t in tilt_deg],
        lambda: _person_scan_strategies(extra_specialist),
        pan_deg=pan_deg,
    ))
    seq.add_child(BtNode_Announce(
        "announce found person", bb_source=None,
        message="Found a person.",
    ))
    return seq


def create_describe_person(pan_deg=None, tilt_deg=None):
    """Look at the person in view and speak a description of them.

    Closes the "tell me the name / pose / gesture of the person" gap: the
    vision feature-extraction service returns a human-readable description
    string (the same ``feature`` text HRI speaks when introducing a guest —
    see HRI/hri.py BtNode_FeatureExtraction + BtNode_Introduce). The planner
    runs ``find_person`` first to locate + approach the person, then this
    tree frames the face, extracts the description, and announces it.
    ``pan_deg`` / ``tilt_deg`` override the sweep ranges (``pan-tilt-sweep``
    modifications).
    """
    seq = py_trees.composites.Sequence("small/describe_person", memory=True)
    seq.add_child(BtNode_Announce(
        "announce describing", bb_source=None,
        message="Let me take a look at this person.",
    ))
    seq.add_child(_arm_to_orbbec_look())  # clear the arm from the orbbec's view
    # Sweep pan 0/+45/-45 across HUMAN_TILT_DEG (up at a standing person, then
    # lower); extract the description at each angle and stop at the first that
    # succeeds, so a person not squarely in front / a different height is framed.
    seq.add_child(_pantilt_sweep(
        "describe_person",
        HUMAN_TILT_DEG if tilt_deg is None else [float(t) for t in tilt_deg],
        lambda: BtNode_FeatureExtraction(
            "extract person description",
            bb_dest_key=bb_keys.DESCRIBE_FEATURES,
            bb_image_key=bb_keys.DESCRIBE_IMAGE,
        ),
        pan_deg=pan_deg,
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
        arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
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
            arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
        ),
        num_failures=5,
    ))

    # Grasp is EXPENSIVE (full arm scan → detect → pick → arm-back cycle), so cap
    # the whole-cycle retry at a single re-attempt (num_failures=2 = one try + one
    # retry). On the second failure the Selector drops straight to the ask-referee
    # ex_machina branch rather than burning more time on a third heavy cycle.
    primary_with_retry = py_trees.decorators.Retry(
        "retry primary once", primary, num_failures=2,
    )

    # Only attempt the real grasp when the object is NOT on no-grasp furniture
    # (shelf / cabinet / coat_rack). Such a grasp fails this guard immediately
    # (no arm scan, no grasp motion) so the Selector drops through to the
    # ask-referee branch below — the robot cannot safely reach into those and
    # could damage them.
    guarded_primary = py_trees.composites.Sequence("grasp/try_unless_no_grasp", memory=True)
    guarded_primary.add_child(BtNode_CheckGraspAllowed("skip primary at no-grasp furniture"))
    guarded_primary.add_child(primary_with_retry)
    # K3 (task-K, live-manipulation sim findings, F2): claim the "autonomous"
    # method on the SUCCESS path only -- last child of a memory Sequence, so
    # it only runs once everything above it (the whole primary grasp cycle)
    # has actually succeeded.
    guarded_primary.add_child(BtNode_BlackboardSet(
        "mark grasp autonomous", bb_keys.STEP_METHOD, "autonomous",
    ))

    ex_machina = py_trees.composites.Sequence("grasp/ex_machina", memory=True)
    # K3: claim the "referee_fallback" method as the FIRST action -- the
    # Selector only reaches ex_machina once guarded_primary has already
    # failed, so entering this branch at all means the operator is about to
    # ask a human referee for help; set it before the open-gripper.
    ex_machina.add_child(BtNode_BlackboardSet(
        "mark grasp referee fallback", bb_keys.STEP_METHOD, "referee_fallback",
    ))
    # Tuck the arm first so it does not block the lidar during the drive.
    ex_machina.add_child(py_trees.decorators.Retry(
        "retry arm back",
        BtNode_MoveArmSingle(
            "arm to navigating",
            service_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
        ),
        num_failures=5,
    ))
    # Drive to the no-grasp furniture BEFORE asking the referee, so the robot is
    # standing at the shelf/cabinet/coat_rack (where the object and referee are)
    # when it presents its gripper. Guarded + best-effort: if the orchestrator
    # could not resolve a pose (GRASP_REFEREE_POSE unset/None) the whole goto is
    # skipped and we ask from the current spot instead of sending a null goal.
    goto_referee = py_trees.composites.Sequence("goto no-grasp furniture", memory=True)
    goto_referee.add_child(BtNode_CheckBBKeySet(
        "no-grasp furniture pose set?", bb_keys.GRASP_REFEREE_POSE,
    ))
    goto_referee.add_child(BtNode_AnnounceFromBB(
        "announce approaching no-grasp furniture",
        bb_keys.GRASP_REFEREE_LOCATION, prefix="I cannot grasp safely there, going to the ",
    ))
    goto_referee.add_child(py_trees.decorators.Retry(
        "retry goto no-grasp furniture",
        BtNode_GotoAction("goto no-grasp furniture", key=bb_keys.GRASP_REFEREE_POSE),
        num_failures=5,
    ))
    ex_machina.add_child(py_trees.decorators.FailureIsSuccess(
        "goto no-grasp furniture (best effort)", goto_referee,
    ))
    # MUST-DO for closed appliances (refrigerator / fridge / washing machine /
    # dishwasher): before the handover, ask the referee to OPEN it — the robot
    # cannot. This is deterministic (gated on GRASP_REFEREE_IS_APPLIANCE, set by
    # the orchestrator), so it fires even when the planner did not emit a separate
    # open() step. Skipped (via APPLIANCE_OPENED) when an explicit open() step
    # already asked, so we never double-ask. Best-effort: a non-appliance or an
    # already-opened appliance just falls through to the handover.
    open_appliance = py_trees.composites.Sequence("ask referee to open appliance", memory=True)
    open_appliance.add_child(BtNode_CheckBBTrue(
        "furniture is a closed appliance?", bb_keys.GRASP_REFEREE_IS_APPLIANCE))
    open_appliance.add_child(py_trees.decorators.Inverter(
        "not already opened?",
        BtNode_CheckBBTrue("already opened?", bb_keys.APPLIANCE_OPENED)))
    open_appliance.add_child(BtNode_AnnounceFromBB(
        "ask referee to open", bb_keys.GRASP_REFEREE_LOCATION,
        prefix="Dear referee, I cannot open it myself. Please open the "))
    open_appliance.add_child(BtNode_Announce(
        "announce leave open", bb_source=None,
        message="Thank you. Please leave it open for me."))
    open_appliance.add_child(BtNode_WaitTicks("wait for referee to open", 8))
    open_appliance.add_child(BtNode_BlackboardSet(
        "mark appliance opened", bb_keys.APPLIANCE_OPENED, True))
    ex_machina.add_child(py_trees.decorators.FailureIsSuccess(
        "open appliance if needed (best effort)", open_appliance,
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
    # P (task-P, flag-gated): make the referee handoff physically real in
    # sim -- teleport the object to the TCP before closing on it -- when
    # GPSR_SIM_REFEREE_HANDOFF=1. Flag off (default): SUCCESS instantly, no
    # ROS entities created, byte-identical to before this node existed.
    ex_machina.add_child(BtNode_RefereeHandObject("sim referee hand-object"))
    ex_machina.add_child(BtNode_GripperAction("close gripper", False))

    return py_trees.composites.Selector(
        "small/grasp",
        memory=True,
        children=[guarded_primary, ex_machina],
    )


def create_open():
    """Ask a human referee to open a container/appliance the robot cannot open
    itself — a refrigerator/fridge, washing machine, or dishwasher (and cabinets).

    The robot has already driven to it via the preceding ``goto``; here it just
    requests help and waits, then leaves the door open for the following scan /
    ask-referee grasp. Always SUCCESS-ish (announce + wait) so it never blocks the
    rest of the plan — the object retrieval itself is handled by the ex-machina
    grasp, which asks the referee to hand the item over.
    """
    seq = py_trees.composites.Sequence("small/open", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "ask referee to open", bb_keys.TARGET_LOCATION,
        prefix="Dear referee, I cannot open it myself. Please open the "))
    seq.add_child(BtNode_Announce(
        "announce leave open", bb_source=None,
        message="Thank you. Please leave it open for me.",
    ))
    seq.add_child(BtNode_WaitTicks("wait for referee to open", 8))
    # Mark it opened so a following grasp's ask-referee branch does not repeat the
    # open request (the grasp branch opens deterministically when this step was
    # skipped, e.g. the planner omitted open() for a dishwasher).
    seq.add_child(BtNode_BlackboardSet("mark appliance opened", bb_keys.APPLIANCE_OPENED, True))
    return seq


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
    primary.add_child(_arm_to_orbbec_look())  # clear the arm from the orbbec's view
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
    vlm_fallback.add_child(BtNode_ParseCountFromAnswer(
        # The target postcondition gate verifies counted(X) against the
        # COUNT_VALUE artifact; without this the VLM path answers correctly
        # and then fails its gate with "counted(...) (UNKNOWN)", replanning
        # the whole target in a loop until the run times out.
        "parse vlm count",
        bb_answer_src=bb_keys.VLM_ANSWER,
        bb_count_dst=bb_keys.COUNT_VALUE,
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
    seq.add_child(_arm_to_orbbec_look())  # clear the arm from the orbbec's view
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
    "open": create_open,
    "place": create_place,
    "deliver": create_deliver,
    "count": create_count,
    "answer_question": create_answer_question,
    "announce": create_announce,
    "record_position": create_record_position,
    "vlm_fallback": create_vlm_fallback,
    "llm_fallback": create_llm_fallback,
}


# ---------------------------------------------------------------------------
# Modifiable-node audit registry (SMALL_TREE_ROLES)
# ---------------------------------------------------------------------------
# Every behavior-specific specialization point inside the small trees. A
# ``role`` labels a node the lower-layer planner may target with a typed
# ``modifiable_nodes`` template. Roles are stable strings shared with
# ``modifiable_nodes.TemplateSpec.applies_to``; the node ids that carry each
# role are COMPUTED by walking each factory's serialized tree (see
# ``compute_small_tree_roles``), so a structural edit that renumbers a child
# index never breaks the registry. Structural only — this registry does not
# alter behaviour.

def _classify_node_roles(action: str, node: dict) -> list[str]:
    """Return the roles a serialized node of ``action``'s tree carries."""
    name = str(node.get("name", ""))
    node_type = str(node.get("type", ""))
    node_id = str(node.get("node_id", ""))
    roles: list[str] = []
    if "pantilt sweep" in name:
        if action == "find_person":
            roles.append("find_person_sweep")
        elif action == "find_object":
            roles.append("find_object_sweep")
        elif action == "describe_person":
            roles.append("describe_person_sweep")
    if action == "search_object" and node_id == f"small/{action}/root":
        roles.append("search_object_sweep")
    if node_type == "BtNode_VLMQuery":
        roles.append("vlm_query")
    if action == "count" and node_type == "BtNode_VLMQuery":
        roles.append("count_vlm_branch")
    if node_type == "BtNode_Announce":
        # Only a literal-message announce (given_msg set at build) is a valid
        # announce-text target — a bb_source announce reads its text from the
        # blackboard at runtime and cannot be pinned here.
        roles.append("announce_leaf")
    if action == "grasp":
        if node_id == f"small/{action}/root/0":
            roles.append("grasp_primary")
        elif node_id == f"small/{action}/root/1":
            roles.append("grasp_ex_machina")
    return roles


_ROLES_CACHE: dict[str, tuple[str, ...]] | None = None


def compute_small_tree_roles() -> dict[str, tuple[str, ...]]:
    """Build and cache ``role -> (serialized node ids)`` by walking factories.

    Every factory is constructed and serialized once; the resulting mapping is
    cached on the module so repeated lookups (planner + checker) are free.
    Building a small tree has no hardware/mock side effects — nodes only read
    the blackboard once set up and ticked, never at construction.
    """
    from .tree_serialization import serialize_tree

    global _ROLES_CACHE
    if _ROLES_CACHE is not None:
        return _ROLES_CACHE
    by_role: dict[str, list[str]] = {}
    for action, factory in ACTION_FACTORIES.items():
        try:
            tree = factory()
        except Exception:  # noqa: BLE001 — a role audit must never break import
            continue
        try:
            document = serialize_tree(tree, kind=f"small/{action}")
        except Exception:  # noqa: BLE001
            continue
        for node in document["nodes"]:
            for role in _classify_node_roles(action, node):
                by_role.setdefault(role, []).append(node["node_id"])
    result = {role: tuple(dict.fromkeys(ids)) for role, ids in by_role.items()}
    _ROLES_CACHE = result
    return result


def get_small_tree_roles() -> dict[str, tuple[str, ...]]:
    """Cached view of :func:`compute_small_tree_roles`."""
    return compute_small_tree_roles()


SMALL_TREE_ROLES: dict[str, tuple[str, ...]] = {}  # populated on first compute
