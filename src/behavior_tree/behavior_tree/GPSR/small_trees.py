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
    BtNode_Approach,
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
    BtNode_GetPointCloud,
)
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_MoveArmSingle,
    BtNode_GripperAction,
    BtNode_Place,
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
    # Navigation scratch goal.  ``TARGET_POSE`` remains the semantic map
    # waypoint used by contracts/gates; create_goto writes a furniture-safe
    # standoff here before sending the Nav2 action.
    NAV_GOAL_POSE = "gpsr/nav_goal_pose"        # PoseStamped
    TARGET_LOCATION = "gpsr/target_location"    # str
    TARGET_OBJECT_NAME = "gpsr/target_object_name"     # str
    TARGET_OBJECT_PROMPT = "gpsr/target_object_prompt" # str (vision prompt)
    TARGET_OBJECT_DETECTION = "gpsr/target_object_detection"  # vision result
    TARGET_OBJECT = "gpsr/target_object"        # vision Object pick used by grasp
    TARGET_PERSON_PROMPT = "gpsr/target_person_prompt"  # str
    TARGET_PERSON_POSE = "gpsr/target_person_pose"      # PoseStamped/PointStamped
    RESOLVED_PERSON_QUERY = "gpsr/resolved_person_query"  # dict[str, str]
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
    # Counting (sim battery 2026-09-26, run s70718-001: one tilt-10 look
    # asking only for the category members' enriched prompts counted 0 on
    # a table holding bleach, mustard and a bowl -- and the gate accepted it).
    COUNT_MEMBER_PROMPTS = "gpsr/count_member_prompts"  # list[str] | None --
                                           #   a category count asks the
                                           #   detector for EVERY known object
                                           #   (TARGET_OBJECT_PROMPT) and
                                           #   BtNode_CountDetections keeps only
                                           #   detections matching one of these
                                           #   member prompts (key-token match,
                                           #   colour-tolerant). None/empty ->
                                           #   count every detection.
    COUNT_VLM_SUBJECT = "gpsr/count_vlm_subject"  # str -- what the VLM fallback
                                           #   is asked to count ("kitchen items
                                           #   (red bowl, ...)"), so it knows
                                           #   which classes count as members.
    COUNT_RECOUNTED = "gpsr/count_recounted"  # bool -- the full recount sweep
                                           #   ran before a zero was accepted;
                                           #   the counted() gate rejects a
                                           #   zero count without it.
    COUNT_VIEW_VALUE = "gpsr/count_view_value"  # int -- member count of the
                                           #   view just scanned (scratch).
    COUNT_VIEW_COUNTS = "gpsr/count_view_counts"  # list[int] -- member counts
                                           #   from this sweep's individual
                                           #   pan/tilt views.
    COUNT_BEST_PAN_RAD = "gpsr/count_best_pan_rad"  # float -- pan of that view
    COUNT_ATTR_DETECTION = "gpsr/count_attr_detection"  # whole-body re-look frame
    COUNT_ATTR_VLM_VALUE = "gpsr/count_attr_vlm_value"  # int -- VLM count on it
    COUNT_ROOM_POSE = "gpsr/count_room_pose"  # PoseStamped | None -- optional
                                           # room search view for person counts;
                                           # room anchors can be too far from a
                                           # pose-specific actor (notably the
                                           # rcw2026 floor-lying kitchen actor).
    COUNT_BEST_DETECTION = "gpsr/count_best_detection"  # the sweep view's
                                           #   response with the highest count
                                           #   (frame for the VLM fallback).
    COUNT_SWEEP_DONE = "gpsr/count_sweep_done"  # bool -- early-stop latch: a
                                           #   view already counted >= 1 and the
                                           #   next view added nothing.
    # Sim battery 2026-09-30 fix5 (countPrsInRoom): YOLO-World prompted with
    # "persons pointing to the right" boxed a pillow, a toy whale and the
    # robot's own arm at conf 0.05-0.10 -> "I counted 3". A person count now
    # detects plain COCO "person" and asks the VLM about the descriptor.
    COUNT_ATTRIBUTE_QUESTION = "gpsr/count_attribute_question"  # str | None --
                                           #   the descriptor of a person count
                                           #   ("pointing to the right"); set ->
                                           #   the VLM refines the detector's N
                                           #   on the best frame, final = min.
    COUNT_VLM_VALUE = "gpsr/count_vlm_value"  # int -- scratch: a VLM count
                                           #   parsed by a count refinement.
    # fix5 countObjOnPlcmt: a detection labelled with the whole ' . '-joined
    # multi-class prompt cannot be classified against a category's members.
    COUNT_VIEW_UNCLASSIFIED = "gpsr/count_view_unclassified"  # int -- scratch:
                                           #   unclassifiable detections in the
                                           #   view just scanned.
    COUNT_UNCLASSIFIED = "gpsr/count_unclassified"  # int -- max unclassified
                                           #   detections over the sweep's views.
    COUNT_UNCLASSIFIED_DETECTION = "gpsr/count_unclassified_detection"  # the
                                           #   view response holding them (the
                                           #   frame the VLM counts instead).
    # fix5 goToLoc/grasp: the wrist detection re-look (create_grasp).
    GRASP_START_POSE = "gpsr/grasp_start_pose"    # PoseStamped (map) -- heading
                                           #   before the re-look rotated the base.
    GRASP_VIEW_GOAL = "gpsr/grasp_view_goal"      # PoseStamped (map) -- the
                                           #   re-look's rotate-in-place goal.
    GRASP_HEAD_DETECTION = "gpsr/grasp_head_detection"  # orbbec detection
                                           #   (map frame) used to re-aim the base.
    GRASP_HEAD_OBJECT = "gpsr/grasp_head_object"  # closest object of it (scratch)
    GRASP_HEAD_POINT = "gpsr/grasp_head_point"    # PointStamped (map) -- its
                                           #   centroid; the drive-up target.
    REFEREE_REQUEST_TEXT = "gpsr/referee_request_text"  # str -- the composed
                                           #   "Dear referee, ..." request the
                                           #   grasp ex_machina branch speaks.
    REFEREE_SEEN_COUNT = "gpsr/referee_seen_count"  # int -- scratch for the
                                           #   ex_machina look-first sweep.
    REFEREE_HANDOFF_ENTITY = "gpsr/referee_handoff_entity"  # str -- known
                                           #   object KEY the sim referee should
                                           #   hand over (resolved from the
                                           #   label the look-first sweep saw;
                                           #   a category name like "food"
                                           #   resolves to no sim entity).
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

    # Real placement (sim battery 2026-09-26 run s70718-003: the old place
    # opened the gripper from the tucked pose and dropped the banana on the
    # floor). Filled by create_place's own scan, read by BtNode_Place.
    PLACE_ENV_POINTS = "gpsr/place_env_points"      # PointCloud2 — wrist RealSense
                                           #   cloud of the destination surface
                                           #   (octomap input for place_action)
    PLACE_TARGET_POINT = "gpsr/place_target_point"  # PointStamped (base_link) —
                                           #   TCP target above a free spot on
                                           #   the detected surface plane
    PLACE_ORIENTATION = "gpsr/place_orientation"    # Pose — zero quaternion, so
                                           #   place_action reuses the stored
                                           #   grasp orientation (or horizontal)
    # 360 deg room survey (create_survey_room / BtNode_RotateInPlace): the
    # current map pose and the same pose yawed by the survey step.
    SURVEY_CAPTURE_POSE = "gpsr/survey_capture_pose"  # PoseStamped (map)
    SURVEY_GOAL_POSE = "gpsr/survey_goal_pose"        # PoseStamped (map)

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
    SUPERVISOR_REPLAN_COUNT = "gpsr/supervisor_replan_count"  # int — Q3 (task-Q, round-6): supervisor-initiated ("level":"supervisor") replans for the CURRENT target only; same reset lifecycle as TARGET_REPLAN_COUNT (a genuine target completion), but NEVER reset by the supervisor branch itself -- past GPSR_SUPERVISION_MAX_SUPERVISOR_REPLANS it caps the "fresh start" semantics so target.failed stays reachable
    DEFERRED_PRECONDITIONS = "gpsr/deferred_preconditions"  # list[str] — canonical preconditions the active target's own plan self-establishes; verified by the postcondition gate instead of at entry
    GATE_COMPLETED_STEPS = "gpsr/gate_completed_steps"  # list[dict] — the active target's own plan steps whose established facts the postcondition gate just committed (J3): fed to the next replan as completed_steps so it is not repeated
    FACTS = "gpsr/facts"
    # list[str] — canonical facts established by successful target postcondition gates for the active command slot.


ARM_ACTION_NAME = "joint_move_action"
GRASP_SERVICE_NAME = "start_grasp"
WAVING_THRESHOLD_METERS = 6.0

# Nav2's footprint inflation can make a semantic furniture waypoint (and a
# room anchor whose first search spot is furniture) sit inside the obstacle
# halo.  Back away along the waypoint's facing direction before sending the
# goal.  This is deliberately below the arm's normal working distance and is
# applied only to static furniture/room locations, never command_point or a
# runtime alias.
FURNITURE_STANDOFF_DISTANCE_M = 0.35

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
# target is detected (see _pantilt_sweep). Positive tilt looks DOWN (the
# tk26_vision pan_tilt.yaml convention, home_tilt_deg 30; the sim's
# level-forward aim correction matches it): 0 = level, 20 = a table a metre
# or more away, 45 = a surface right in front of the robot or a person's
# body from the head camera. Each list is swept in order, pan-inner /
# tilt-outer, so the first tilt is the primary look.
# Sim battery 2026-09-26 run s70718-003: the bedroom search spot stands
# 0.3-0.5 m from side_table_02 with the objects ~0.7 m below the camera,
# i.e. 55-65 deg down. With only 0/20 the whole sweep looked over the arena
# walls (vision_log snapshots: wall tops and sky) and 36 scans saw nothing;
# the one "hit" in the pre-fix battery was a VLM false positive on that
# view. 45 deg puts a close surface inside the 58.7 deg vertical FOV.
PAN_SWEEP_DEG = [0.0, 45.0, -45.0]   # centre first, then left, then right
HUMAN_TILT_DEG = [45.0, 20.0]        # down at a person's body, then higher
# People stand in the open floor of a room, which from a search spot (the
# robot faces the furniture) is often well off to the side: in the rcw2026
# arena three of the four room persons sit 77-89 deg off the spot's heading
# (sim placements 2026-09-30), outside a +-45 deg pan. Person searches and
# person counts therefore also look +-90 deg (pan range is +-120).
HUMAN_PAN_SWEEP_DEG = [0.0, 45.0, -45.0, 90.0, -90.0]
OBJECT_TILT_DEG = [0.0, 20.0, 45.0]  # level, a far surface, then a close one
# Counting looks progressively further DOWN: sim battery 2026-09-26 run
# s70718-001 counted from a single tilt-10 look while the side table stood
# 0.6 m below the camera, 0.5-1 m away -- out of frame. 60 is the pan-tilt's
# tilt limit (modifiable_nodes.TILT_RANGE_DEG).
COUNT_TILT_DEG = [20.0, 45.0, 60.0]
# Head tilt for the person-attribute re-look (whole body incl. raised arms).
PERSON_ATTRIBUTE_TILT_DEG = 10.0
# Sim battery 2026-09-30 fix5: YOLO-World's 0.05-0.10 boxes (pillow, toy
# whale, the robot's own arm) were counted as persons. Detections that carry a
# real confidence below this are dropped from counts; the VLM path reports 1.0.
COUNT_MIN_CONF = 0.25
# Objects get a lower floor: sim battery 2026-10-01 fix8 run s70718-001 saw
# the mustard bottle on the side table only at 0.11-0.14 and the 0.25 floor
# turned a count of 3 into 1. The person false positives above were all
# <= 0.10, so persons keep 0.25.
COUNT_MIN_CONF_OBJECT = 0.10
# Self-occlusion: a person box that reaches into the bottom SELF_OCCLUSION_
# BOTTOM_FRAC of the head-camera image AND spans more than SELF_OCCLUSION_
# WIDTH_FRAC of its width is the robot's own arm / gripper.
SELF_OCCLUSION_BOTTOM_FRAC = 0.25
SELF_OCCLUSION_WIDTH_FRAC = 0.40
# Grasp detection re-look (create_grasp): base yaw offsets (deg, relative to
# the heading the grasp started from) for the last wrist views.
GRASP_RELOOK_YAW_DEG = [15.0, -15.0]
GRASP_HEAD_TILT_DEG = 45.0
# Sim battery 2026-10-01 fix8 run s70718-004: from the kitchen_table nav pose
# (chairs around the round table) the food stood ~1 m away -- out of the
# wrist camera's view and beyond the arm's reach -- and all four wrist views
# saw nothing. After the head camera finds the object, drive up to it first
# (go_to_approach, best effort) to about this distance.
#
# Keep the furniture approach just outside the inflated east edge. For
# side_table_02 the bowl centroid is x~=0.43 and the pose audit's free east
# approach is x=1.172, so 0.75 m avoids the x~=1.03 candidate produced by
# 0.60 m while remaining within the wrist camera and arm working range.
GRASP_APPROACH_DISTANCE_M = 0.75
# Sim rerun59: the confirmed living-room person centroid drifted ~1.4 m from
# the simulator truth and the deployed go_to_approach ring ([0.45, 1.00] m)
# had no free candidate around that stale point. Delivery can search a wider
# ring so a clear stand pose near the actual person remains reachable.
DELIVERY_PERSON_MAX_DISTANCE_M = 1.60
# The simulator's referee hold releases on the first open-gripper command, but
# the object still needs a few physics steps to fall clear of the hand.  Four
# ticks was shorter than the truth probe's still-in-hand settling window in
# rerun81, so keep the handoff open long enough for the release to be observed.
DELIVERY_RELEASE_SETTLE_TICKS = 20


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
        status = self._inner.update()
        # Sim battery 2026-09-26 (runs s70718-000/-004): the ex_machina
        # "ask referee" announce DID speak "Dear referee, please help me
        # grasp the soup" (the node went RUNNING -> SUCCESS, i.e. the TTS
        # service call completed), but this wrapper never copied the inner
        # node's feedback -- the tree dump showed "ask referee [✓]" with an
        # empty message, so the bench's announcements.txt extractor (which
        # scans orchestrator.log for "Finished announcing ...") never saw
        # it and the referee request looked silent. Mirror the inner
        # feedback so every AnnounceFromBB is auditable like a plain
        # BtNode_Announce.
        self.feedback_message = self._inner.feedback_message
        return status

    def terminate(self, new_status):
        self._inner.terminate(new_status)
        self.feedback_message = self._inner.feedback_message


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
        # Room-qualified person scans need the semantic room anchor to reject
        # a detector centroid pulled onto the furniture in front of the camera.
        # Keep this read optional at runtime so old standalone trees remain
        # valid when they do not materialise a location.
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)
        self._client.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.READ)
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
        # Sim battery 2026-10-01 fix9 run s70718-000: sort_closest put the
        # robot's OWN ARM (a "person" box across the bottom of the head image)
        # first, and the person approach drove to the robot's own position.
        # Same filter as the count path; needs the scan's segments for boxes.
        width, height = BtNode_CountDetections._image_size(result)
        segments = list(getattr(result, "segments", None) or [])
        own_arm = [
            i for i, obj in enumerate(objects)
            if BtNode_CountDetections.is_self_occlusion(
                BtNode_CountDetections._raw_label(obj),
                BtNode_CountDetections._bbox(obj, segments[i] if i < len(segments) else None),
                width, height)
        ]
        if own_arm:
            objects = [obj for i, obj in enumerate(objects) if i not in own_arm]
            if not objects:
                self.feedback_message = "Scan saw only the robot's own arm"
                return Status.FAILURE

        # A pointing/raised-arm segmentation can put the 3-D centroid on the
        # extended limb. In a room-qualified scan the first (closest) result
        # is often the sofa/table anchor rather than the person's body. If a
        # second person result is materially farther from that anchor, prefer
        # it. This is deliberately restricted to gesture-qualified prompts so
        # ordinary person scans keep their detector ordering.
        try:
            descriptor = str(self._client.get(bb_keys.TARGET_PERSON_PROMPT) or "")
        except Exception:
            descriptor = ""
        try:
            location = str(self._client.get(bb_keys.TARGET_LOCATION) or "")
        except Exception:
            location = ""
        objects = self._prefer_body_candidate(objects, descriptor, location)

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

    @staticmethod
    def _prefer_body_candidate(objects, descriptor: str = "", location: str = ""):
        """Return gesture-person detections with a furniture-anchor hit last.

        The generalist requests ``sort_closest``. For room searches this can
        make an extended pointing arm / sofa edge win over the person's body.
        The semantic room pose is only a rejection anchor; no simulator truth
        is consulted, and with no usable anchor the detector order is kept.
        """
        objects = list(objects or [])
        if len(objects) < 2:
            return objects
        text = f"{descriptor} {' '.join(str(getattr(o, 'cls', '')) for o in objects)}".lower()
        if not any(token in text for token in ("point", "rais", "wav")):
            return objects
        key = str(location or "").strip().lower().replace(" ", "_")
        if not key:
            return objects
        try:
            from .orchestrator import KNOWN_LOCATIONS, ROOM_SEARCH_SPOTS
            if key not in ROOM_SEARCH_SPOTS:
                return objects
            anchor = KNOWN_LOCATIONS.get(key)
            if anchor is None:
                for spot in ROOM_SEARCH_SPOTS.get(key, ()):
                    anchor = KNOWN_LOCATIONS.get(spot)
                    if anchor is not None:
                        break
            if anchor is None:
                return objects
            pose = getattr(anchor, "pose", anchor)
            point = getattr(pose, "position", pose)
            ax, ay = float(point.x), float(point.y)
        except (AttributeError, TypeError, ValueError, ImportError):
            return objects

        def dist(item):
            try:
                c = item.centroid
                return math.hypot(float(c.x) - ax, float(c.y) - ay)
            except (AttributeError, TypeError, ValueError):
                return None

        first_distance = dist(objects[0])
        if first_distance is None or first_distance > 0.65:
            return objects
        ranked = [(dist(item), index, item) for index, item in enumerate(objects)]
        usable = [entry for entry in ranked if entry[0] is not None]
        if not usable:
            return objects
        farthest_distance, farthest_index, _ = max(usable, key=lambda entry: entry[0])
        if farthest_index == 0 or farthest_distance < first_distance + 0.40:
            return objects
        return [objects[farthest_index], *objects[:farthest_index], *objects[farthest_index + 1:]]


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


class BtNode_CheckOperatorDelivery(Behaviour):
    """Guard the direct handoff path used by ``deliver(..., me)``.

    ``materialise_params`` resolves ``recipient_location`` into
    ``TARGET_LOCATION`` before the delivery tree runs.  The operator aliases
    all resolve to the captured command-point pose, so a visual person search
    is unnecessary and can move the robot away from the handoff pose.
    """

    _ALIASES = {
        "start_position", "instruction_point", "start", "operator",
        "me", "the_user", "the_operator",
    }

    def __init__(self, name: str = "operator delivery?"):
        super().__init__(name)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)

    def update(self):
        try:
            raw = self._client.get(bb_keys.TARGET_LOCATION)
        except Exception:
            raw = None
        key = str(raw or "").strip().lower().replace(" ", "_")
        if key in self._ALIASES:
            return Status.SUCCESS
        self.feedback_message = f"recipient {raw!r} is not the operator"
        return Status.FAILURE


class BtNode_CheckLastNavMatchesTarget(Behaviour):
    """Guard reuse of a successful navigation already at this step's target.

    Self-navigating actions (notably ``deliver``) traditionally include an
    internal room ``goto``.  A planner may also emit an explicit ``goto``
    immediately before the action when the command requires a return leg,
    e.g. ``goto(start_position)`` followed by ``deliver(...,
    recipient_location=start_position)``.  ``LAST_NAV_LOCATION`` is written
    only after a navigation succeeds, so matching it against the materialised
    target is safe: the action can reuse that pose without driving there a
    second time, while a stale or failed navigation falls through to the
    normal retry path.
    """

    def __init__(self, name: str = "already at target location?"):
        super().__init__(name)
        self._client = None

    @staticmethod
    def _normalise(value):
        return " ".join(str(value or "").casefold().replace("_", " ").split())

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.LAST_NAV_LOCATION, access=Access.READ)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)

    def update(self):
        try:
            last = self._client.get(bb_keys.LAST_NAV_LOCATION)
            target = self._client.get(bb_keys.TARGET_LOCATION)
        except Exception:
            self.feedback_message = "navigation evidence or target is unavailable"
            return Status.FAILURE
        last_norm = self._normalise(last)
        target_norm = self._normalise(target)
        if last_norm and target_norm and last_norm == target_norm:
            return Status.SUCCESS
        self.feedback_message = (
            f"last successful navigation {last!r} does not match target {target!r}"
        )
        return Status.FAILURE


class BtNode_CheckResolvedDeliveryTarget(Behaviour):
    """Guard reuse of a matching person target already resolved in this target.

    A planner may split ``goto(room) -> find_person -> approach_person`` from
    the final ``deliver`` step. In that shape ``deliver`` has no
    ``recipient_location`` of its own; navigating ``TARGET_POSE`` again would
    send the robot back to the room anchor and discard the person pose it just
    approached. Reuse requires the saved successful-search descriptor to
    match the delivery recipient, and an explicit delivery location to match
    the location used by that search.
    """

    def __init__(self, name: str = "resolved delivery target?", *, require_nav: bool = True):
        super().__init__(name)
        self._require_nav = bool(require_nav)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.CURRENT_PARAMS, access=Access.READ)
        self._client.register_key(bb_keys.TARGET_PERSON_POSE, access=Access.READ)
        self._client.register_key(bb_keys.RESOLVED_PERSON_QUERY, access=Access.READ)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)
        if self._require_nav:
            self._client.register_key(bb_keys.PERSON_NAV_POSE, access=Access.READ)

    def update(self):
        try:
            params = self._client.get(bb_keys.CURRENT_PARAMS) or {}
        except Exception:
            params = {}
        if not isinstance(params, dict):
            params = {}
        recipient = next((params.get(key) for key in ("recipient", "person", "descriptor")
                          if str(params.get(key) or "").strip()), None)
        if recipient is None:
            self.feedback_message = "delivery has no recipient descriptor"
            return Status.FAILURE
        try:
            query = self._client.get(bb_keys.RESOLVED_PERSON_QUERY)
        except Exception:
            query = None
        if not isinstance(query, dict):
            self.feedback_message = "no confirmed person search context"
            return Status.FAILURE

        def normalized(value):
            return " ".join(str(value or "").casefold().replace("_", " ").split()).strip(" .,;:!?")

        if not normalized(query.get("descriptor")) or normalized(query.get("descriptor")) != normalized(recipient):
            self.feedback_message = "confirmed person search does not match delivery recipient"
            return Status.FAILURE
        delivery_location = params.get("recipient_location") or params.get("location")
        if delivery_location and normalized(query.get("location")) != normalized(delivery_location):
            self.feedback_message = "confirmed person search does not match delivery location"
            return Status.FAILURE
        try:
            person_pose = self._client.get(bb_keys.TARGET_PERSON_POSE)
        except Exception:
            person_pose = None
        if person_pose is None:
            self.feedback_message = "no resolved recipient person pose"
            return Status.FAILURE
        if self._require_nav:
            try:
                nav_pose = self._client.get(bb_keys.PERSON_NAV_POSE)
            except Exception:
                nav_pose = None
            if nav_pose is None:
                self.feedback_message = "recipient pose has not been approached"
                return Status.FAILURE
        self.feedback_message = "reusing resolved recipient pose"
        return Status.SUCCESS


class BtNode_RecordResolvedPersonQuery(Behaviour):
    """Associate a confirmed person pose with the query that produced it."""

    def __init__(self, name: str = "record resolved person query"):
        super().__init__(name)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_PERSON_PROMPT, access=Access.READ)
        self._client.register_key(bb_keys.TARGET_PERSON_POSE, access=Access.READ)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)
        self._client.register_key(bb_keys.RESOLVED_PERSON_QUERY, access=Access.WRITE)

    def update(self):
        try:
            descriptor = str(self._client.get(bb_keys.TARGET_PERSON_PROMPT) or "").strip()
            location = str(self._client.get(bb_keys.TARGET_LOCATION) or "").strip()
            person_pose = self._client.get(bb_keys.TARGET_PERSON_POSE)
        except Exception:
            descriptor, location, person_pose = "", "", None
        query = {"descriptor": descriptor, "location": location} if descriptor and person_pose is not None else None
        self._client.set(bb_keys.RESOLVED_PERSON_QUERY, query, overwrite=True)
        self.feedback_message = "recorded confirmed person search" if query else "person search has no reusable pose"
        return Status.SUCCESS


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


class BtNode_CheckDetectionUsable(Behaviour):
    """SUCCESS only when a detection response contains a usable frame.

    A generalist scan can return a response object with a captured RGB frame
    even when it failed to match anything. Image consumers must not treat
    that failed response as attribute evidence: doing so turns a VLM zero on
    an empty re-look into a confirmed zero and discards a valid detector hit.
    """

    def __init__(self, name: str, bb_detection_key: str):
        super().__init__(name)
        self._key = bb_detection_key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._key, access=Access.READ)

    @staticmethod
    def is_usable(response) -> bool:
        if response is None or getattr(response, "status", None) != 0:
            return False
        return bool(getattr(response, "objects", None) or [])

    def update(self):
        try:
            response = self._client.get(self._key)
        except Exception:
            response = None
        if self.is_usable(response):
            return Status.SUCCESS
        status = getattr(response, "status", None)
        objects = len(getattr(response, "objects", None) or []) if response is not None else 0
        self.feedback_message = f"no usable detection frame (status={status}, objects={objects})"
        return Status.FAILURE


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


class BtNode_CheckEnvFlag(Behaviour):
    """Gate simulator-only recovery paths without touching the blackboard."""

    def __init__(self, name: str, variable: str, expected: str = "1"):
        super().__init__(name)
        self._variable = variable
        self._expected = expected

    def update(self) -> Status:
        value = os.environ.get(self._variable, "")
        if value == self._expected:
            self.feedback_message = f"{self._variable}={self._expected}"
            return Status.SUCCESS
        self.feedback_message = f"{self._variable} is not enabled"
        return Status.FAILURE


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
        self._client.register_key(bb_keys.REFEREE_HANDOFF_ENTITY, access=Access.READ)
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
        # Prefer the concrete object key the compose node resolved from what
        # the look-first sweep saw (REFEREE_HANDOFF_ENTITY); the raw target
        # name is the fallback (a category like "food" is no sim entity).
        name = ""
        for key in (bb_keys.REFEREE_HANDOFF_ENTITY, bb_keys.TARGET_OBJECT_NAME):
            try:
                name = str(self._client.get(key) or "").strip()
            except Exception:
                name = ""
            if name:
                break
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

    ``bb_member_prompts_key`` (sim battery 2026-09-26, run s70718-001): a
    category count now asks the detector for EVERY known object and filters
    afterwards -- when the list at this key is non-empty only detections whose
    label matches one of those member prompts are counted
    (``validators.count_member_label_matches``: the member's object-key
    tokens, colour-tolerant, so a "bowl" / "white bowl" detection counts for
    the member "red bowl"). Unset / None / empty -> every detection counts
    (specific-object and person counts).
    """

    def __init__(self, name: str, bb_detection_src: str, bb_count_dst: str,
                 fail_if_zero: bool = False, bb_member_prompts_key: str = None,
                 bb_unclassified_dst: str = None):
        super().__init__(name)
        self._src = bb_detection_src
        self._dst = bb_count_dst
        self._fail_if_zero = fail_if_zero
        self._member_key = bb_member_prompts_key
        self._unclassified_dst = bb_unclassified_dst
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src, access=Access.READ)
        self._client.register_key(self._dst, access=Access.WRITE)
        if self._member_key:
            self._client.register_key(self._member_key, access=Access.READ)
        if self._unclassified_dst:
            self._client.register_key(self._unclassified_dst, access=Access.WRITE)

    def _member_prompts(self) -> list:
        if not self._member_key:
            return []
        try:
            value = self._client.get(self._member_key)
        except Exception:
            return []
        if isinstance(value, str):
            value = [value]
        try:
            return [str(v) for v in (value or []) if str(v).strip()]
        except TypeError:
            return []

    # Two detections closer than this (3D centroid, metres) or overlapping
    # more than DUPLICATE_IOU in the image are ONE thing seen twice. Sim
    # battery 2026-09-27 fix4b run 002: the single spawned person came back
    # as two 'persons pointing to the right' boxes -> "I counted 2".
    DUPLICATE_CENTROID_M = 0.30
    # Objects: two distinct objects cannot have centroids this close (the sim
    # spawner keeps >= ~0.12 m centre to centre; one object's centroid jitters
    # a few cm between masks). Sim battery 2026-10-01 fix8 run s70718-001:
    # bowl / bleach / mustard stood 0.17-0.26 m apart on side_table and the
    # 0.30 m person radius merged them into one.
    DUPLICATE_CENTROID_OBJECT_M = 0.10
    DUPLICATE_IOU = 0.5

    @staticmethod
    def _is_person(item) -> bool:
        label = BtNode_CountDetections._raw_label(item)
        return label is not None and "person" in str(label).lower()

    @staticmethod
    def _is_duplicate(a, b, segment_a=None, segment_b=None) -> bool:
        ca, cb = getattr(a, "centroid", None), getattr(b, "centroid", None)
        radius = (BtNode_CountDetections.DUPLICATE_CENTROID_M
                  if BtNode_CountDetections._is_person(a) or BtNode_CountDetections._is_person(b)
                  else BtNode_CountDetections.DUPLICATE_CENTROID_OBJECT_M)
        try:
            if ca is not None and cb is not None and any(
                    abs(float(getattr(ca, k, 0.0))) > 1e-6 for k in ("x", "y", "z")):
                d = math.sqrt(sum((float(getattr(ca, k, 0.0)) - float(getattr(cb, k, 0.0))) ** 2
                                  for k in ("x", "y", "z")))
                if d < radius:
                    return True
        except (TypeError, ValueError):
            pass
        box_a = BtNode_CountDetections._bbox(a, segment_a)
        box_b = BtNode_CountDetections._bbox(b, segment_b)
        if box_a is None or box_b is None:
            return False
        ax0, ay0, ax1, ay1 = box_a
        bx0, by0, bx1, by1 = box_b
        inter = max(0, min(ax1, bx1) - max(ax0, bx0)) * max(0, min(ay1, by1) - max(ay0, by0))
        if inter <= 0:
            return False
        union = (ax1 - ax0) * (ay1 - ay0) + (bx1 - bx0) * (by1 - by0) - inter
        return union > 0 and inter / union > BtNode_CountDetections.DUPLICATE_IOU

    @staticmethod
    def dedupe(objects, segments=None) -> list:
        """Drop repeated detections using 3D centroids and aligned mask boxes."""
        segments = list(segments or [])
        kept = []
        kept_segments = []
        for index, item in enumerate(objects):
            segment = segments[index] if index < len(segments) else None
            if not any(BtNode_CountDetections._is_duplicate(item, prior, segment, prior_segment)
                       for prior, prior_segment in zip(kept, kept_segments)):
                kept.append(item)
                kept_segments.append(segment)
        return kept

    # ------------------------------------------------------------------
    # Sim battery 2026-09-30 fix5 filters (confidence floor, self-occlusion,
    # unclassifiable multi-class labels).
    # ------------------------------------------------------------------
    @staticmethod
    def _raw_label(item):
        """The detection's label as the detector wrote it (NOT normalised --
        ``validators._item_label`` turns " . " into "_._")."""
        for field in ("cls", "class", "name", "label"):
            value = item.get(field) if isinstance(item, dict) else getattr(item, field, None)
            if isinstance(value, str) and value.strip():
                return value
        return None

    @staticmethod
    def is_unclassified_label(label) -> bool:
        """A label that is the whole ' . '-joined multi-class prompt (the
        detector did not say WHICH class it matched)."""
        return label is not None and " . " in str(label)

    @staticmethod
    def _conf(item):
        value = item.get("conf") if isinstance(item, dict) else getattr(item, "conf", None)
        try:
            return None if value is None else float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def passes_conf(item) -> bool:
        """Keep a detection unless it carries a real confidence below
        ``COUNT_MIN_CONF``. No conf / conf <= 0 (unset msg default) keeps it;
        VLM detections report 1.0 and always pass."""
        conf = BtNode_CountDetections._conf(item)
        if conf is None or conf <= 0.0:
            return True
        floor = COUNT_MIN_CONF if BtNode_CountDetections._is_person(item) else COUNT_MIN_CONF_OBJECT
        return conf >= floor

    @staticmethod
    def _bbox(item, segment=None):
        """``(xmin, ymin, xmax, ymax)`` in pixels: from the object's own box
        fields when it has them, else from its segmentation mask (the
        generalist response carries no boxes, only index-aligned masks)."""
        try:
            return tuple(int(getattr(item, k)) for k in ("xmin", "ymin", "xmax", "ymax"))
        except (AttributeError, TypeError, ValueError):
            pass
        if segment is None:
            return None
        try:
            import numpy as np
            h, w = int(segment.height), int(segment.width)
            if h <= 0 or w <= 0:
                return None
            step = int(getattr(segment, "step", 0) or w)
            mask = np.frombuffer(bytes(segment.data), dtype=np.uint8).reshape(h, step)[:, :w]
            ys, xs = np.nonzero(mask)
            if xs.size == 0:
                return None
            return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())
        except Exception:  # noqa: BLE001 -- a malformed mask just means "no box"
            return None

    @staticmethod
    def is_self_occlusion(label, bbox, width, height) -> bool:
        """A PERSON box reaching into the bottom ``SELF_OCCLUSION_BOTTOM_FRAC``
        of the image and wider than ``SELF_OCCLUSION_WIDTH_FRAC`` of it is the
        robot's own arm / gripper in the head camera (fix5 overlay 095858)."""
        if bbox is None or not width or not height:
            return False
        text = str(label or "").lower()
        if "person" not in text and "people" not in text:
            return False
        x0, y0, x1, y1 = bbox
        return (y1 >= height * (1.0 - SELF_OCCLUSION_BOTTOM_FRAC)
                and (x1 - x0) > width * SELF_OCCLUSION_WIDTH_FRAC)

    @staticmethod
    def _image_size(result):
        """(width, height) of the response's frame, 0s when unknown."""
        for img in [getattr(result, "rgb_image", None)] + list(getattr(result, "segments", None) or [])[:1]:
            try:
                w, h = int(getattr(img, "width")), int(getattr(img, "height"))
            except (AttributeError, TypeError, ValueError):
                continue
            if w > 0 and h > 0:
                return w, h
        return 0, 0

    @staticmethod
    def filter_detections(objects, segments=None, width=0, height=0, *, return_segments=False):
        """``(kept, n_low_conf, n_self)``: drop low-confidence detections and
        person boxes that are the robot's own arm (see above)."""
        segments = list(segments or [])
        kept, kept_segments, low, own = [], [], 0, 0
        for i, item in enumerate(list(objects or [])):
            if not BtNode_CountDetections.passes_conf(item):
                low += 1
                continue
            seg = segments[i] if i < len(segments) else None
            bbox = BtNode_CountDetections._bbox(item, seg)
            if BtNode_CountDetections.is_self_occlusion(
                    BtNode_CountDetections._raw_label(item), bbox, width, height):
                own += 1
                continue
            kept.append(item)
            kept_segments.append(seg)
        if return_segments:
            return kept, low, own, kept_segments
        return kept, low, own

    @staticmethod
    def classify_members(objects, member_prompts, segments=None) -> tuple:
        """``(members, unclassified)`` over distinct detections.

        No member filter -> every detection is a member (an unclassifiable
        multi-class label included). With a filter, a detection whose label
        is the whole ' . '-joined prompt cannot be matched to a member: it is
        reported as unclassified (the count tree hands that frame to the VLM)
        instead of silently vanishing."""
        objects = BtNode_CountDetections.dedupe(list(objects or []), segments)
        if not member_prompts:
            return len(objects), 0
        from .validators import _item_label, count_member_label_matches
        n = unclassified = 0
        for item in objects:
            if BtNode_CountDetections.is_unclassified_label(BtNode_CountDetections._raw_label(item)):
                unclassified += 1
                continue
            label = _item_label(item)
            if label is None:
                continue
            if any(count_member_label_matches(label, m) for m in member_prompts):
                n += 1
        return n, unclassified

    @staticmethod
    def count_members(objects, member_prompts) -> int:
        """Distinct detections whose label matches any member prompt (all,
        when the member list is empty); duplicates of one thing count once."""
        return BtNode_CountDetections.classify_members(objects, member_prompts)[0]

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
        raw_objects = list(getattr(result, "objects", []) or [])
        width, height = self._image_size(result)
        objects, n_low, n_own, segments = self.filter_detections(
            raw_objects, getattr(result, "segments", None), width, height, return_segments=True)
        members = self._member_prompts()
        count, unclassified = self.classify_members(objects, members, segments)
        self._client.set(self._dst, count, overwrite=True)
        if self._unclassified_dst:
            self._client.set(self._unclassified_dst, unclassified, overwrite=True)
        detail = f"{count} objects"
        if members:
            detail = f"{count} of {len(raw_objects)} detections matching {len(members)} member prompts"
            if unclassified:
                detail += f", {unclassified} unclassified (multi-class label)"
        if n_low:
            detail += f", dropped {n_low} below conf floor"
        if n_own:
            detail += f", dropped {n_own} self-occlusion (own arm)"
        if self._fail_if_zero and count == 0:
            self.feedback_message = f"counted {detail} — deferring to fallback"
            return Status.FAILURE
        self.feedback_message = f"counted {detail}"
        return Status.SUCCESS


class BtNode_CountKeepMax(Behaviour):
    """Fold one sweep view's count into the running maximum.

    Reads the view's member count at ``bb_view_key`` and the view's detection
    response at ``bb_detection_key``; keeps ``bb_count_dst`` = max over views
    and ``bb_best_dst`` = the response of the best view so far (highest
    member count, ties broken by raw detections so the VLM fallback gets the
    busiest frame even when every view counted zero). Latches
    ``bb_done_key`` True when a view already counted >= 1, at least three
    views have been sampled, and THIS view added nothing. The minimum sample
    count prevents one high detector view from ending the sweep before a
    second view can corroborate or reject it. When two views have the same
    member count and raw detection count, retain the view with the larger
    image footprint. This matters for person attributes: a first detection
    can contain only a torso while a later pan/tilt view contains the raised
    arm needed to answer a gesture question. Always SUCCESS.
    """

    def __init__(self, name: str, bb_view_key: str, bb_detection_key: str,
                 bb_count_dst: str, bb_best_dst: str, bb_done_key: str,
                 view_pan_deg: float = None, bb_best_pan_dst: str = None,
                 bb_history_dst: str = None):
        super().__init__(name)
        # Optional: which pan this view looked at, written (radians) to
        # bb_best_pan_dst whenever this view becomes the best -- the person
        # attribute re-look turns back there (fix9 run s70718-002).
        self._view_pan_deg = view_pan_deg
        self._best_pan_dst = bb_best_pan_dst
        self._view_key = bb_view_key
        self._detection_key = bb_detection_key
        self._count_dst = bb_count_dst
        self._best_dst = bb_best_dst
        self._done_key = bb_done_key
        self._history_dst = bb_history_dst
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._view_key, access=Access.READ)
        self._client.register_key(self._detection_key, access=Access.READ)
        self._client.register_key(self._count_dst, access=Access.WRITE)
        self._client.register_key(self._best_dst, access=Access.WRITE)
        self._client.register_key(self._done_key, access=Access.WRITE)
        if self._history_dst:
            self._client.register_key(self._history_dst, access=Access.WRITE)
        if self._best_pan_dst:
            self._client.register_key(self._best_pan_dst, access=Access.WRITE)

    def _get(self, key, default=None):
        try:
            value = self._client.get(key)
        except Exception:
            return default
        return default if value is None else value

    @staticmethod
    def _frame_area(detection) -> int:
        """Return the visible pixel area of detections in ``detection``.

        Generalist responses expose index-aligned segmentation masks rather
        than bounding boxes. Keep this helper deliberately tolerant of test
        doubles and malformed ROS messages: a missing mask falls back to the
        object's bounding-box area, and an unusable item contributes zero.
        """
        if detection is None:
            return 0
        objects = list(getattr(detection, "objects", []) or [])
        segments = list(getattr(detection, "segments", []) or [])
        area = 0
        for index, item in enumerate(objects):
            segment = segments[index] if index < len(segments) else None
            measured = 0
            if segment is not None:
                try:
                    import numpy as np
                    height = int(getattr(segment, "height", 0) or 0)
                    width = int(getattr(segment, "width", 0) or 0)
                    step = int(getattr(segment, "step", 0) or width)
                    if height > 0 and width > 0 and step >= width:
                        mask = np.frombuffer(bytes(segment.data), dtype=np.uint8)
                        mask = mask.reshape(height, step)[:, :width]
                        measured = int(np.count_nonzero(mask))
                except Exception:
                    measured = 0
            if measured <= 0:
                bbox = BtNode_CountDetections._bbox(item, segment)
                if bbox is not None:
                    x0, y0, x1, y1 = bbox
                    measured = max(0, x1 - x0) * max(0, y1 - y0)
            area += measured
        return area

    def update(self):
        try:
            view = int(self._get(self._view_key, 0) or 0)
        except (TypeError, ValueError):
            view = 0
        try:
            prev = int(self._get(self._count_dst, 0) or 0)
        except (TypeError, ValueError):
            prev = 0
        detection = self._get(self._detection_key)
        raw = len(getattr(detection, "objects", []) or [])
        best = self._get(self._best_dst)
        best_raw = len(getattr(best, "objects", []) or []) if best is not None else -1
        frame_area = self._frame_area(detection)
        best_area = self._frame_area(best)
        if best is None or view > prev or (
                view == prev and (raw, frame_area) > (best_raw, best_area)):
            self._client.set(self._best_dst, detection, overwrite=True)
            if self._best_pan_dst and self._view_pan_deg is not None:
                self._client.set(self._best_pan_dst, math.radians(self._view_pan_deg), overwrite=True)
        new_max = max(prev, view)
        self._client.set(self._count_dst, new_max, overwrite=True)
        history = []
        if self._history_dst:
            history = list(self._get(self._history_dst, []) or [])
            history.append(view)
            self._client.set(self._history_dst, history, overwrite=True)
        enough_views = not self._history_dst or len(history) >= 3
        done = enough_views and prev >= 1 and view <= prev
        self._client.set(self._done_key, done, overwrite=True)
        self.feedback_message = (
            f"view counted {view}, max so far {new_max}"
            + (" — sweep saturated, stopping early" if done else "")
        )
        return Status.SUCCESS


class BtNode_CheckCountPositive(Behaviour):
    """SUCCESS iff the integer at ``key`` is >= 1; FAILURE on zero/unset so a
    Selector parent can hand over to the next counting strategy."""

    def __init__(self, name: str, key: str):
        super().__init__(name)
        self._key = key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._key, access=Access.READ)

    def update(self):
        try:
            value = int(self._client.get(self._key) or 0)
        except Exception:
            value = 0
        if value >= 1:
            self.feedback_message = f"count is {value}"
            return Status.SUCCESS
        self.feedback_message = "count is zero — trying the next strategy"
        return Status.FAILURE


class BtNode_CountWavingPersons(Behaviour):
    """Materialise the count returned by the dedicated waving detector.

    ``DetectWaving`` writes a list of ``PointStamped`` detections to
    ``ALL_WAVING_PERSONS``.  Counting that typed result avoids asking a VLM to
    infer a gesture from a small/cropped RGB frame.  A missing or empty list
    returns FAILURE so the caller can fall back to the ordinary VLM path.
    """

    def __init__(self, name: str = "count waving-person detections"):
        super().__init__(name)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.ALL_WAVING_PERSONS, access=Access.READ)
        self._client.register_key(bb_keys.COUNT_VALUE, access=Access.WRITE)

    def update(self):
        try:
            detections = self._client.get(bb_keys.ALL_WAVING_PERSONS)
        except Exception:
            detections = None
        count = len(detections or []) if isinstance(detections, (list, tuple)) else 0
        if count <= 0:
            self.feedback_message = "dedicated waving detector returned no people"
            return Status.FAILURE
        self._client.set(bb_keys.COUNT_VALUE, count, overwrite=True)
        self.feedback_message = f"counted {count} waving person(s) from dedicated detector"
        return Status.SUCCESS


class BtNode_BestEffortTurnPanTilt(BtNode_TurnPanTilt):
    """Pan/tilt command that can be omitted by standalone tree fixtures.

    Runtime trees provide the ROS node required by ``BtNode_TurnPanTilt``.
    Small-tree unit tests and offline planners intentionally build trees
    without a ROS node; in that context a missing controller is a best-effort
    failure and the caller should continue with its existing camera pose.
    """

    def setup(self, **kwargs):
        self._missing_ros_node = False
        try:
            super().setup(**kwargs)
        except KeyError:
            self._missing_ros_node = True

    def initialise(self):
        if self._missing_ros_node or self.mock_mode:
            return
        return super().initialise()

    def update(self):
        if self._missing_ros_node:
            self.feedback_message = "pan/tilt controller unavailable; keeping current view"
            return Status.FAILURE
        if self.mock_mode:
            self.feedback_message = "MOCK: keeping current pan/tilt view"
            return Status.SUCCESS
        return super().update()


class BtNode_BestEffortScanForWavingPerson(BtNode_ScanForWavingPersonNew):
    """Dedicated waving scan that degrades cleanly without a ROS node."""

    def setup(self, **kwargs):
        self._unavailable = False
        try:
            super().setup(**kwargs)
        except Exception:
            # ActionHandler's legacy missing-node diagnostic itself raises an
            # IndexError while formatting its KeyError; keep this optional
            # specialist from preventing offline tree construction.
            self._unavailable = True

    def initialise(self):
        if self._unavailable:
            return
        return super().initialise()

    def update(self):
        if self._unavailable:
            self.feedback_message = "waving detector unavailable; using VLM fallback"
            return Status.FAILURE
        return super().update()


class BtNode_CountKeepUnclassified(Behaviour):
    """Fold one view's unclassified-detection count into the sweep maximum.

    Reads ``bb_view_key`` (this view's unclassifiable multi-class-label
    detections, written by ``BtNode_CountDetections``) and keeps
    ``bb_max_dst`` = max over views plus ``bb_frame_dst`` = the response of
    that view (the frame the VLM counts the category on). Always SUCCESS.
    """

    def __init__(self, name: str, bb_view_key: str, bb_detection_key: str,
                 bb_max_dst: str, bb_frame_dst: str):
        super().__init__(name)
        self._view_key = bb_view_key
        self._detection_key = bb_detection_key
        self._max_dst = bb_max_dst
        self._frame_dst = bb_frame_dst
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._view_key, access=Access.READ)
        self._client.register_key(self._detection_key, access=Access.READ)
        self._client.register_key(self._max_dst, access=Access.WRITE)
        self._client.register_key(self._frame_dst, access=Access.WRITE)

    def _int(self, key) -> int:
        try:
            return int(self._client.get(key) or 0)
        except Exception:
            return 0

    def update(self):
        view = self._int(self._view_key)
        prev = self._int(self._max_dst)
        if view > prev:
            try:
                frame = self._client.get(self._detection_key)
            except Exception:
                frame = None
            self._client.set(self._frame_dst, frame, overwrite=True)
            self._client.set(self._max_dst, view, overwrite=True)
        self.feedback_message = f"view unclassified {view}, max {max(view, prev)}"
        return Status.SUCCESS


class BtNode_CountCombine(Behaviour):
    """Combine the current count with a VLM result.

    ``mode="min"``: a VLM attribute count may only LOWER the detector's
    count (never report more people than the detector saw). ``mode="max"``:
    a VLM count of unclassifiable detections may only RAISE the classified
    member count. ``mode="attribute"`` uses the VLM only when it returns a
    positive value, bounded by the detector count. FAILURE when the other
    value is missing / not an int (the caller wraps it best-effort, so the
    detector count stands).
    """

    def __init__(self, name: str, bb_count_key: str, bb_other_key: str,
                 mode: str = "min", bb_views_key: str = None,
                 bb_confirm_key: str = None):
        super().__init__(name)
        if mode not in ("min", "max", "attribute", "attribute_confirmed", "consensus"):
            raise ValueError(
                f"unsupported count-combine mode {mode!r}"
            )
        self._count_key = bb_count_key
        self._other_key = bb_other_key
        self._mode = mode
        self._views_key = bb_views_key or bb_keys.COUNT_VIEW_COUNTS
        self._confirm_key = bb_confirm_key
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._count_key, access=Access.WRITE)
        self._client.register_key(self._other_key, access=Access.READ)
        if self._mode == "consensus":
            self._client.register_key(self._views_key, access=Access.READ)
        if self._mode == "attribute_confirmed":
            if not self._confirm_key:
                raise ValueError("attribute_confirmed mode requires bb_confirm_key")
            self._client.register_key(self._confirm_key, access=Access.READ)

    @staticmethod
    def combine(count, other, mode: str, view_counts=None, confirm=None) -> int:
        count = int(count or 0)
        if mode == "attribute_confirmed":
            values = []
            for value in (other, confirm):
                try:
                    values.append(int(value))
                except (TypeError, ValueError):
                    continue
            if len(values) == 2 and values[0] == values[1] == 0:
                return 0
            positive = [value for value in values if value > 0]
            return min(count, max(positive)) if positive else count
        other = int(other)
        if mode == "min":
            return min(count, other)
        if mode == "max":
            return max(count, other)
        if mode == "consensus":
            positive = []
            for value in view_counts or []:
                try:
                    value = int(value)
                except (TypeError, ValueError):
                    continue
                if value > 0:
                    positive.append(value)
            if positive:
                frequencies = {value: positive.count(value) for value in set(positive)}
                highest_frequency = max(frequencies.values())
                if highest_frequency >= 2:
                    modes = [value for value, frequency in frequencies.items()
                             if frequency == highest_frequency]
                    consensus = other if other in modes else max(modes)
                    return max(consensus, other)
                # With no repeated detector value, the largest view can be a
                # one-frame false positive.  A positive VLM count that agrees
                # with one of the independent views is stronger evidence than
                # that isolated spike (for example views [3, 4, 2], VLM 3).
                if other > 0 and other in positive:
                    return other
            return max(count, other)
        return min(count, other) if other > 0 else count

    def update(self):
        try:
            count = int(self._client.get(self._count_key) or 0)
        except Exception:
            count = 0
        try:
            other_value = self._client.get(self._other_key)
            other = int(other_value)
        except Exception:
            if self._mode != "attribute_confirmed":
                self.feedback_message = f"no count at {self._other_key}"
                return Status.FAILURE
            other = None
        try:
            view_counts = self._client.get(self._views_key) if self._mode == "consensus" else None
        except Exception:
            view_counts = None
        try:
            confirm = self._client.get(self._confirm_key) if self._mode == "attribute_confirmed" else None
        except Exception:
            confirm = None
        final = self.combine(count, other, self._mode, view_counts, confirm)
        self._client.set(self._count_key, final, overwrite=True)
        detail = (f"{self._mode}(detector {count}, vlm {other}, confirm {confirm}) = {final}"
                  if self._mode == "attribute_confirmed" else
                  f"{self._mode}(detector {count}, vlm {other}, views {view_counts}) = {final}"
                  if self._mode == "consensus" else
                  f"{self._mode}(detector {count}, vlm {other}) = {final}")
        self.feedback_message = detail
        return Status.SUCCESS


class BtNode_ComposeRefereeRequest(Behaviour):
    """Write the spoken ask-the-referee request for the grasp ex_machina branch.

    Sim battery 2026-09-26 (runs s70718-000/-004): the robot asked the
    referee without ever looking at the furniture. The branch now sweeps
    the furniture first (``_referee_look_sweep``); this node reads that
    sweep's detection response and composes, into ``bb_dst``:

    - seen:  "Dear referee, I can see the <detected label> on the
      <furniture>. Please put the <object name> in my gripper."
    - unseen: "Dear referee, I cannot see the <object name> on the
      <furniture>. Please hand me the <object name> and put it in my
      gripper."

    ``<furniture>`` is ``GRASP_REFEREE_LOCATION`` (the no-grasp furniture the
    branch drove to), falling back to ``TARGET_LOCATION``, then "furniture".
    Always SUCCESS -- a missing detection response simply reads as unseen.
    """

    def __init__(self, name: str, bb_dst: str,
                 bb_detection_key: str = None, bb_object_name_key: str = None,
                 bb_furniture_keys=None):
        super().__init__(name)
        self._dst = bb_dst
        self._detection_key = bb_detection_key or bb_keys.TARGET_OBJECT_DETECTION
        self._object_key = bb_object_name_key or bb_keys.TARGET_OBJECT_NAME
        self._furniture_keys = list(bb_furniture_keys or (
            bb_keys.GRASP_REFEREE_LOCATION, bb_keys.TARGET_LOCATION,
        ))
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._dst, access=Access.WRITE)
        self._client.register_key(bb_keys.REFEREE_HANDOFF_ENTITY, access=Access.WRITE)
        self._client.register_key(self._detection_key, access=Access.READ)
        self._client.register_key(self._object_key, access=Access.READ)
        for key in self._furniture_keys:
            self._client.register_key(key, access=Access.READ)

    def _get(self, key):
        try:
            return self._client.get(key)
        except Exception:
            return None

    @staticmethod
    def compose(object_name, furniture, detected_label) -> str:
        obj = str(object_name or "object").strip().replace("_", " ") or "object"
        furn_raw = str(furniture or "").strip()
        furn = furn_raw.replace("_", " ") or "furniture"
        # fix4 run 3 said "on the bedroom": a room is "in", furniture is "on".
        where = f"in the {furn}" if furn_raw and is_room_location(furn_raw) else f"on the {furn}"
        seen = str(detected_label or "").strip().replace("_", " ")
        if seen:
            return (
                f"Dear referee, I can see the {seen} {where}. "
                f"Please put the {obj} in my gripper."
            )
        return (
            f"Dear referee, I cannot see the {obj} {where}. "
            f"Please hand me the {obj} and put it in my gripper."
        )

    def update(self):
        detection = self._get(self._detection_key)
        objects = list(getattr(detection, "objects", []) or [])
        label = ""
        for item in objects:
            for field in ("cls", "label", "name"):
                value = getattr(item, field, None)
                if isinstance(value, str) and value.strip():
                    label = value.strip()
                    break
            if label:
                break
        furniture = ""
        for key in self._furniture_keys:
            value = self._get(key)
            if value is not None and str(value).strip():
                furniture = str(value)
                break
        object_name = self._get(self._object_key)
        text = self.compose(object_name, furniture, label)
        self._client.set(self._dst, text, overwrite=True)
        # Sim referee hand-off target: the KNOWN OBJECT KEY behind what the
        # look-first sweep saw, so a category target ("food") resolves to the
        # concrete spawned entity (fix4 run 3: the sim nacked "no entity for
        # 'food'"); falls back to the object name itself when nothing was seen.
        from .orchestrator import category_members, object_key_for_label  # lazy: import cycle
        entity = ""
        if " . " not in str(label or ""):  # a whole multi-class prompt echoed back is no label
            entity = object_key_for_label(label) or ""
        if not entity:
            # fix4b run 004: guessing one member ("pudding_box") of a category
            # was nacked; send every member as '|'-separated candidates and
            # let the sim hand over whichever actually exists on the furniture.
            members = category_members(object_name)
            entity = "|".join(members) if members else (object_key_for_label(object_name) or "")
        self._client.set(bb_keys.REFEREE_HANDOFF_ENTITY, entity, overwrite=True)
        self.feedback_message = text
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
    Retry-wrapped and normally propagates failure: if the arm cannot tuck, we do
    not drive with the arm sticking out.  The live simulator benchmark can set
    ``GPSR_ALLOW_NAV_WITHOUT_ARM=1`` when its TF tree does not expose the arm's
    world transform; in that mode a failed tuck is treated as best-effort so
    perception/navigation-only commands can still execute.  Manipulation steps
    retain their own strict arm action and still fail when the arm is unusable.
    """
    # The benchmark opt-out means the arm controller/TF tree is unavailable;
    # do not even send a goal in that mode.  FailureIsSuccess around a live
    # action still waits for the action server's timeout (several minutes),
    # defeating the opt-out and starving navigation.  A synthetic SUCCESS is
    # safe here because this flag is only enabled by the live-sim benchmark;
    # manipulation actions keep their own strict arm requirements.
    if os.environ.get("GPSR_ALLOW_NAV_WITHOUT_ARM", "").strip() == "1":
        return py_trees.behaviours.Success(
            f"{label} (skipped: GPSR_ALLOW_NAV_WITHOUT_ARM=1)"
        )

    retry = py_trees.decorators.Retry(
        f"retry {label}",
        BtNode_MoveArmSingle(
            label,
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=pose_key or bb_keys.ARM_ORBBEC_LOOK,
            add_octomap=False,
        ),
        num_failures=3,
    )
    return retry


def _arm_to_orbbec_look(label: str = "arm to orbbec look", prelude=None):
    """Move the arm to ``arm_pos_orbbec_look`` so it clears the ORBBEC head
    camera's field of view before an orbbec scan (find_object / count / vlm /
    find_person / describe_person). This is NOT the arm-mounted RealSense grasp
    scan, which needs the arm at ``ARM_SCAN`` — that path is left untouched.

    Best-effort: Retry-wrapped and FailureIsSuccess so a MoveIt hiccup (or an
    un-seeded ARM_ORBBEC_LOOK key) never fails the scan. The pose is read from
    ``bb_keys.ARM_ORBBEC_LOOK``, seeded once at startup alongside the nav/scan
    poses; if it is unset the move simply no-ops and the scan proceeds.
    """
    # The live simulator may intentionally run without a usable arm/TF tree.
    # In that mode the head camera can still perform the scan, while sending a
    # joint goal here can remain RUNNING or be rejected indefinitely and block
    # the whole perception subtree.  Keep this opt-out consistent with the
    # navigation tuck policy; real-arm runs retain the actual move and retries.
    if os.environ.get("GPSR_ALLOW_NAV_WITHOUT_ARM", "").strip() == "1":
        skipped = py_trees.behaviours.Success(
            f"{label} (skipped: GPSR_ALLOW_NAV_WITHOUT_ARM=1)"
        )
        if prelude is None:
            return skipped
        return py_trees.composites.Sequence(
            f"{label} prelude", memory=True, children=[prelude, skipped]
        )

    retry = py_trees.decorators.Retry(
        f"retry {label}",
        BtNode_MoveArmSingle(
            label,
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
            add_octomap=False,
        ),
        num_failures=3,
    )
    child = retry if prelude is None else py_trees.composites.Sequence(
        f"{label} prelude", memory=True, children=[prelude, retry]
    )
    return py_trees.decorators.FailureIsSuccess(
        f"{label} (best effort)",
        child,
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


# Upper bound on the door watch. Sim battery 2026-09-27 fix4 run 004: the
# previous run left the robot facing the bedroom side table, the door detector
# had no door in view and answered FAILURE for the whole 1500 s run -- the
# mission never started. A referee who has opened the door will not wait
# forever either; after this long the robot says so and enters anyway.
# Override with GPSR_DOOR_WAIT_MAX_S (seconds; <= 0 restores the unbounded watch).
DOOR_WAIT_MAX_S = float(os.environ.get("GPSR_DOOR_WAIT_MAX_S", "120") or 0)


class FailureIsRunningUntil(py_trees.decorators.Decorator):
    """``FailureIsRunning`` with a wall-clock budget: the child's FAILURE reads
    as RUNNING (keep sensing) until ``max_wait_s`` has elapsed since the first
    tick of this activation, then as SUCCESS (give up waiting, proceed). A
    non-positive budget keeps waiting forever, exactly like FailureIsRunning.
    """

    def __init__(self, name: str, child, max_wait_s: float):
        super().__init__(name=name, child=child)
        self._max_wait_s = float(max_wait_s)
        self._started = None

    def initialise(self):
        self._started = None

    def update(self):
        if self._started is None:
            self._started = time.monotonic()
        status = self.decorated.status
        # A service request can remain RUNNING forever when its server is
        # present but never completes (the live battery's door detector did
        # exactly this).  The original budget only inspected FAILURE, so that
        # path bypassed the watchdog and held the whole 1500 s command slot.
        # Apply the wall-clock bound to every non-terminal child state.
        waited = time.monotonic() - self._started
        if (
            status == Status.RUNNING
            and self._max_wait_s > 0
            and waited >= self._max_wait_s
        ):
            self.feedback_message = (
                f"door not confirmed open after {waited:.0f}s -- proceeding anyway"
            )
            return Status.SUCCESS
        if status != Status.FAILURE:
            self.feedback_message = self.decorated.feedback_message
            return status
        if self._max_wait_s > 0 and waited >= self._max_wait_s:
            self.feedback_message = (
                f"door not confirmed open after {waited:.0f}s -- proceeding anyway"
            )
            return Status.SUCCESS
        self.feedback_message = self.decorated.feedback_message
        return Status.RUNNING


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
    detect.add_child(BtNode_TurnPanTilt(
        # The entrance head pose is useful for the door detector, but a stale
        # pan/tilt feedback sample must not prevent the bounded door watch
        # from running. The detector itself remains the entry gate.
        "point head at entrance", x=0.0, y=45.0,
    ))
    detect.add_child(BtNode_Announce(
        "announce ready for gpsr", bb_source=None,
        message="Hi, I am Tinker. I am ready for GPSR. Please open the door.",
    ))
    detect.add_child(FailureIsRunningUntil(
        "keep sensing until door opens",
        BtNode_DoorDetection(
            "detect open door", bb_door_state_key=bb_keys.DOOR_STATUS,
        ),
        max_wait_s=DOOR_WAIT_MAX_S,
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
    # Keep the selector's running child between ticks.  With ``memory=False``
    # the failed guard is re-run first on every tick and py_trees invalidates
    # the still-running ``detect`` branch; that reset replays the greeting and
    # door-open request continuously while the door service is pending.  A
    # memory selector resumes the door watch until it reaches a terminal state,
    # then the blackboard latch makes the guard succeed on later restarts.
    once = py_trees.composites.Selector("enter arena (once)", memory=True)
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
    drive_retry = py_trees.decorators.Retry(
        "retry goto",
        BtNode_GotoAction("goto target", key=bb_keys.NAV_GOAL_POSE),
        num_failures=5,
    )
    # Keep the public goto tree shape stable (the parallel remains the fourth
    # child) while preparing the scratch goal immediately before navigation.
    drive = py_trees.composites.Sequence(
        "prepare + retry goto", memory=True,
        children=[
            # Keep semantic TARGET_POSE unchanged for at_robot()/step
            # evidence, but move static furniture and room anchors just
            # outside their inflated obstacle halo before Nav2 sees the goal.
            BtNode_FurnitureStandoffPose("prepare furniture standoff"),
            drive_retry,
        ],
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


def _referee_object_scan():
    """Fresh one-view scan for the grasp ex_machina branch's look-first sweep:
    the same generalist orbbec scan as ``_object_scan_and_verify`` (so a hit
    lands in ``TARGET_OBJECT_DETECTION`` for ``BtNode_ComposeRefereeRequest``
    to name), but its verify count goes to a scratch key -- this must never
    clobber ``COUNT_VALUE``, the counted() gate's artifact."""
    seq = py_trees.composites.Sequence("referee object scan+verify", memory=True)
    seq.add_child(BtNode_ScanForGeneralist(
        name="referee generalist scan",
        bb_source=bb_keys.TARGET_OBJECT_PROMPT,
        bb_key=bb_keys.TARGET_OBJECT_DETECTION,
        use_orbbec=True,
        transform_to_map=True,
        use_vlm_sam_fallback=True,
        sort_closest=True,
    ))
    seq.add_child(BtNode_CountDetections(
        "verify object seen at furniture",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.REFEREE_SEEN_COUNT,
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
    # 360 deg room survey (sim battery 2026-09-26): a room-wide search only
    # ever looked +-45 deg from each search spot. LAST branch, gated by
    # BtNode_CheckRoomSurveyDue so it fires only when the target location is
    # a room AND (inside create_search_object) this is the last filled spot,
    # i.e. after every search spot and the reduced-query retry have failed:
    # turn the base 90 deg four times, full pan/tilt sweep at each heading,
    # SUCCESS at the first detection. A direct find_object(room) surveys
    # right after its own sweep + reduction fail. See create_survey_room.
    survey = py_trees.composites.Sequence("find_object room survey", memory=True)
    survey.add_child(BtNode_CheckRoomSurveyDue("room survey due?"))
    survey.add_child(BtNode_Announce(
        "announce room survey", bb_source=None,
        message="I cannot see it from here. Let me turn around and look.",
    ))
    survey.add_child(create_survey_room(
        _object_scan_and_verify, tilts, pan_deg=pan_deg, label="find_object survey",
    ))
    sweep_with_reduction.add_child(survey)
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


def create_approach_person(
    skip_operator: bool = False,
    approach_max_distance: float = 0.0,
    fail_on_unreachable: bool = False,
):
    """Navigate to the person pose stored by the most recent ``find_person``.

    Atomic action — the LLM plans ``find_person`` then ``approach_person``
    when an interaction (describe / follow / guide / handover) needs the robot
    standing next to the person rather than across the room. Converts
    ``bb_keys.TARGET_PERSON_POSE`` (PointStamped from vision) into a nav goal
    and drives there.

    ``skip_operator`` is used only by ``create_deliver``: the operator's
    captured command-point pose is already the handoff target.

    ``approach_max_distance`` is an optional delivery override; zero keeps the
    action server's configured default for ordinary ``approach_person`` use.

    ``fail_on_unreachable`` keeps delivery closed when both navigation
    strategies fail. Asking the recipient to move is not evidence that they
    did, so a later attempt must confirm the approach before releasing.
    """
    seq = py_trees.composites.Sequence("small/approach_person", memory=True)
    # Fails (no announcement) when the person scan never stored a pose.
    seq.add_child(BtNode_PointToPoseStamped(
        "person point to nav pose",
        bb_point_key=bb_keys.TARGET_PERSON_POSE,
        bb_pose_key=bb_keys.PERSON_NAV_POSE,
    ))
    seq.add_child(_tuck_arm_for_nav("tuck arm before approach"))
    # Sim battery 2026-10-01 fix8 run s70718-000: "goto person" drove to the
    # person's OWN position -- a goal inside their costmap footprint -- and
    # Nav2 aborted 3/3 with the soup still in the gripper. go_to_approach
    # picks a free, reachable pose ~0.7 m away facing the person (and
    # recomputes against the live costmap on an abort). The raw goto stays as
    # a fallback; if the robot still cannot get there it asks the person to
    # come over. Delivery treats that request as incomplete until a later
    # approach actually succeeds.
    reach = py_trees.composites.Selector("reach person", memory=True)
    reach.add_child(py_trees.decorators.Retry(
        "retry approach",
        BtNode_Approach(
            "approach person",
            # PERSON_NAV_POSE: the same point flattened to z=0 one step
            # earlier; with target_is_person the planner uses its person
            # rules (issue-2 2d: the 3-D centroid's z selected object rules).
            bb_target_key=bb_keys.PERSON_NAV_POSE,
            max_distance=approach_max_distance,
            target_is_person=True,
        ),
        num_failures=2,
    ))
    reach.add_child(BtNode_GotoAction("goto person", key=bb_keys.PERSON_NAV_POSE))
    ask_person = BtNode_Announce(
        "ask person to come closer", bb_source=None,
        message="I cannot get any closer to you. Please come over to me.",
    )
    if fail_on_unreachable:
        ask_then_fail = py_trees.composites.Sequence(
            "ask person to come closer, then fail", memory=True,
        )
        ask_then_fail.add_child(ask_person)
        ask_then_fail.add_child(py_trees.behaviours.Failure(
            "recipient approach not confirmed",
        ))
        reach.add_child(ask_then_fail)
    else:
        reach.add_child(ask_person)
    if fail_on_unreachable:
        clear_failed_target = py_trees.composites.Sequence(
            "clear unconfirmed delivery approach", memory=True,
        )
        clear_failed_target.add_child(BtNode_BlackboardSet(
            "clear unconfirmed person nav pose", bb_keys.PERSON_NAV_POSE, None,
        ))
        clear_failed_target.add_child(py_trees.behaviours.Failure(
            "recipient approach not confirmed",
        ))
        seq.add_child(py_trees.composites.Selector(
            "delivery approach confirmation", memory=True,
            children=[reach, clear_failed_target],
        ))
    else:
        seq.add_child(reach)
    if not skip_operator:
        return seq

    # ``deliver(..., recipient_location=start_position)`` already has the
    # operator's captured pose as its navigation target.  A second person
    # approach can select a false positive (or walk away from that pose), so
    # bypass it only for this delivery-specific tree.  Standalone
    # ``approach_person`` actions retain their normal pose-driven behavior.
    direct = py_trees.composites.Sequence(
        "operator handoff: skip approach", memory=True,
    )
    direct.add_child(BtNode_CheckOperatorDelivery())
    direct.add_child(py_trees.behaviours.Success("operator already at handoff"))
    reuse = py_trees.composites.Sequence(
        "reuse resolved delivery person", memory=True,
    )
    reuse.add_child(BtNode_CheckResolvedDeliveryTarget(
        "recipient pose already resolved?", require_nav=True,
    ))
    reuse.add_child(py_trees.behaviours.Success("recipient already approached"))
    return py_trees.composites.Selector(
        "small/approach_person", memory=True, children=[direct, reuse, seq],
    )


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
        # Keep masks so BtNode_ExtractDetection can reject a near-camera
        # person box caused by the robot's own arm/gripper.  The centroid-only
        # path accepted that self detection in rerun60 and drove delivery to
        # the robot's room waypoint instead of the living-room person.
        return_segments=True,
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
        # The relaxed fallback can see the same self-arm box as the strict
        # descriptor scan, so retain masks for the shared self-occlusion filter.
        return_segments=True,
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
    # The simulator's relaxed identity mode supplies one deterministic person
    # asset; descriptor-specific VLM prompts can instead lock onto nearby
    # furniture (the living-room sofa in particular). Prefer the plain person
    # scan in that explicitly enabled validation mode.
    if os.environ.get("GPSR_SIM_IDENTITY_RELAXED") == "1":
        selector.children.remove(generic_scan_branch)
        selector.children.insert(0, generic_scan_branch)
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
        # Keep masks so BtNode_ExtractDetection can reject a near-camera
        # person box caused by the robot's own arm/gripper.
        return_segments=True,
    ))
    branch.add_child(BtNode_ExtractDetection(
        f"pick {gate} person",
        bb_detection_src=bb_keys.TARGET_PERSON_DETECTION,
        bb_object_dst=bb_keys.TARGET_OBJECT,
        bb_point_dst=bb_keys.TARGET_PERSON_POSE,
    ))
    return branch


def _person_scan_and_confirm(extra_specialist=None):
    """Fresh person detection for one pan/tilt: the scan strategies, then a
    SECOND, independent run of the same strategies at the same pose that
    must also find a person before the branch (and so the sweep) succeeds.

    Sim battery 2026-09-26 (run s70718-004): "Found a person." was announced
    at start_position from a single YOLO 'person' hit while the head frame
    showed empty floor -- a one-off false positive accepted immediately.
    Both runs are fresh node instances (a py_trees node occupies one slot);
    the confirm run's own scans re-write ``TARGET_PERSON_DETECTION`` /
    ``TARGET_PERSON_POSE`` before its extract, so the pose the sweep leaves
    behind is the CONFIRMED one. A failed confirmation fails this branch
    and the memory-Selector sweep moves on to the next (pan, tilt).
    """
    seq = py_trees.composites.Sequence("person scan + confirm", memory=True)
    seq.add_child(_person_scan_strategies(extra_specialist))
    confirm = _person_scan_strategies(extra_specialist)
    confirm.name = "person confirm strategies"
    seq.add_child(confirm)
    return seq


def create_find_person(extra_specialist=None, pan_deg=None, tilt_deg=None,
                       skip_operator: bool = False):
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

    ``skip_operator`` is delivery-only and adds a guarded fast path for the
    captured operator pose; ordinary ``find_person`` actions always scan.
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
        lambda: _person_scan_and_confirm(extra_specialist),
        pan_deg=HUMAN_PAN_SWEEP_DEG if pan_deg is None else pan_deg,
    ))
    seq.add_child(BtNode_Announce(
        "announce found person", bb_source=None,
        message="Found a person.",
    ))
    seq.add_child(BtNode_RecordResolvedPersonQuery())
    if not skip_operator:
        return seq

    # The operator is represented by the captured ``START_POSE`` rather than
    # a new visual target.  Keep this guard local to delivery; direct
    # ``find_person`` actions must still perform their requested scan.
    direct = py_trees.composites.Sequence(
        "operator handoff: skip person scan", memory=True,
    )
    direct.add_child(BtNode_CheckOperatorDelivery())
    direct.add_child(py_trees.behaviours.Success("operator pose already known"))
    reuse = py_trees.composites.Sequence(
        "reuse resolved delivery person", memory=True,
    )
    reuse.add_child(BtNode_CheckResolvedDeliveryTarget(
        "recipient pose already resolved?", require_nav=False,
    ))
    reuse.add_child(py_trees.behaviours.Success("recipient pose already known"))
    return py_trees.composites.Selector(
        "small/find_person", memory=True, children=[direct, reuse, seq],
    )


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


def _wrist_table_scan(name: str):
    """One generalist detection on the arm RealSense (camera frame, rgb +
    depth + segments -- everything the grasp server reads from
    ``TARGET_OBJECT``)."""
    return BtNode_ScanForGeneralist(
        name=name,
        bb_source=bb_keys.TARGET_OBJECT_PROMPT,
        bb_key=bb_keys.TARGET_OBJECT,
        use_orbbec=False,            # arm RealSense at ARM_SCAN
        transform_to_map=False,      # keep camera frame for the grasp server
        use_vlm_sam_fallback=True,   # open-vocab for coke / fanta / ...
        sort_closest=True,
        return_rgb_image=True,
        return_depth_image=True,
        return_segments=True,
    )


def _grasp_arm(label: str, pose_key: str, add_octomap: bool = False):
    return BtNode_MoveArmSingle(
        label, service_name=ARM_ACTION_NAME, arm_pose_bb_key=pose_key,
        add_octomap=add_octomap,
    )


def _grasp_turn_base(label: str, goal_key: str):
    """Best-effort rotate-in-place goto (x/y kept, heading from ``goal_key``)."""
    return py_trees.decorators.FailureIsSuccess(
        f"{label} (best effort)",
        py_trees.decorators.Retry(
            f"retry {label}", BtNode_GotoAction(label, key=goal_key), num_failures=2,
        ),
    )


def _arm_back_after_grasp(label: str = "retry arm back"):
    """Return the post-grasp arm reset used by the live benchmark.

    The simulator's joint action server can leave a completed/failed grasp
    with the arm goal in a terminal-but-unreported state for several minutes.
    Navigation is explicitly allowed to proceed without a tuck in this mode,
    so waiting through five more arm goals only delays the referee fallback or
    the next target.  The normal hardware path retains the five-attempt retry.
    """
    if os.environ.get("GPSR_ALLOW_NAV_WITHOUT_ARM", "").strip() == "1":
        return py_trees.behaviours.Success(
            f"{label} (skipped: GPSR_ALLOW_NAV_WITHOUT_ARM=1)"
        )
    return py_trees.decorators.Retry(
        label,
        BtNode_MoveArmSingle(
            "arm back to navigating",
            service_name=ARM_ACTION_NAME,
            arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
        ),
        num_failures=5,
    )


def _grasp_table_detection():
    """Detect the grasp target on the table from up to five DIFFERENT views.

    Sim battery 2026-09-30 fix5 (goToLoc at kitchen_table): the wrist frame
    clearly showed soup can, sugar box, banana and a mug, yet "retry detect
    on table" failed 5/5 with "no matches via vlm_sam" -- five identical
    requests from the identical ARM_SCAN pose. Now each attempt is a new
    view, first success wins:

    1. wrist at ARM_SCAN (as before);
    2. head camera: arm to the orbbec-look (nav stow) pose, head tilt
       ``GRASP_HEAD_TILT_DEG``, orbbec scan in the MAP frame. The grasp
       server needs the wrist rgb/depth/segments (BtNode_Grasp sends
       vision_result.rgb_image/depth_image/segments + header), so the head
       detection only RE-AIMS: turn the base to face it (clamped 45 deg),
       arm back to ARM_SCAN, wrist scan again;
    3./4. base yaw ``GRASP_RELOOK_YAW_DEG`` (+15 / -15 relative to the
       heading the grasp started at), wrist scan.

    Every base rotation happens with the arm at the orbbec-look pose (the
    pose used for navigation), never with the arm extended over the table.
    Detection calls <= 5 (1 + head 1 + wrist 1 + 1 + 1). If every view fails
    the robot turns back to its starting heading (best effort) and the
    sweep FAILS as before (-> primary retry / ex_machina).
    """
    settle_ticks = 6

    def _make_table_views():
        """Build a fresh view selector for each strict/reduced-query pass."""
        views = py_trees.composites.Selector("table views", memory=True)

        v1 = py_trees.composites.Sequence("table view 1: wrist at arm scan", memory=True)
        v1.add_child(_wrist_table_scan("realsense generalist detect"))
        views.add_child(v1)

        v2 = py_trees.composites.Sequence("table view 2: head camera re-aim", memory=True)
        v2.add_child(_grasp_arm("view 2: arm to orbbec look", bb_keys.ARM_ORBBEC_LOOK))
        v2.add_child(BtNode_TurnPanTilt(
            f"view 2: head tilt {GRASP_HEAD_TILT_DEG:+.0f}", x=0.0, y=GRASP_HEAD_TILT_DEG))
        v2.add_child(BtNode_ScanForGeneralist(
            name="orbbec generalist detect",
            bb_source=bb_keys.TARGET_OBJECT_PROMPT,
            bb_key=bb_keys.GRASP_HEAD_DETECTION,
            use_orbbec=True,
            transform_to_map=True,       # map-frame centroid to face
            use_vlm_sam_fallback=True,
            sort_closest=True,
            return_segments=False,
        ))
        drive_up = py_trees.composites.Sequence("view 2: drive up to the object", memory=True)
        drive_up.add_child(BtNode_ExtractDetection(
            "view 2: object point", bb_detection_src=bb_keys.GRASP_HEAD_DETECTION,
            bb_object_dst=bb_keys.GRASP_HEAD_OBJECT, bb_point_dst=bb_keys.GRASP_HEAD_POINT,
        ))
        drive_up.add_child(BtNode_Approach(
            "view 2: approach object", bb_target_key=bb_keys.GRASP_HEAD_POINT,
            desired_distance=GRASP_APPROACH_DISTANCE_M,
        ))
        # Views 3/4 and the restore turn are relative to the new, closer spot.
        drive_up.add_child(BtNode_CaptureCurrentPose(
            "view 2: re-capture start heading", bb_key=bb_keys.GRASP_START_POSE))
        v2.add_child(py_trees.decorators.FailureIsSuccess("view 2: drive up (best effort)", drive_up))
        v2.add_child(BtNode_CaptureCurrentPose("view 2: capture pose", bb_key=bb_keys.SURVEY_CAPTURE_POSE))
        v2.add_child(BtNode_YawTowardDetection(
            "view 2: face head detection", bb_keys.SURVEY_CAPTURE_POSE,
            bb_keys.GRASP_HEAD_DETECTION, bb_keys.GRASP_VIEW_GOAL,
        ))
        v2.add_child(_grasp_turn_base("view 2: turn to detection", bb_keys.GRASP_VIEW_GOAL))
        v2.add_child(BtNode_TurnPanTilt("view 2: head back down", x=0.0, y=20.0))
        v2.add_child(_grasp_arm("view 2: arm to scan", bb_keys.ARM_SCAN, add_octomap=True))
        v2.add_child(BtNode_WaitTicks("view 2: settle", settle_ticks))
        v2.add_child(_wrist_table_scan("view 2: realsense detect"))
        views.add_child(v2)

        for i, yaw in enumerate(GRASP_RELOOK_YAW_DEG, start=3):
            v = py_trees.composites.Sequence(f"table view {i}: base yaw {yaw:+.0f}", memory=True)
            v.add_child(_grasp_arm(f"view {i}: arm to orbbec look", bb_keys.ARM_ORBBEC_LOOK))
            v.add_child(BtNode_YawOffsetPose(
                f"view {i}: yaw {yaw:+.0f}", bb_keys.GRASP_START_POSE, bb_keys.GRASP_VIEW_GOAL, yaw,
            ))
            v.add_child(_grasp_turn_base(f"view {i}: turn base", bb_keys.GRASP_VIEW_GOAL))
            v.add_child(_grasp_arm(f"view {i}: arm to scan", bb_keys.ARM_SCAN, add_octomap=True))
            v.add_child(BtNode_WaitTicks(f"view {i}: settle", settle_ticks))
            v.add_child(_wrist_table_scan(f"view {i}: realsense detect"))
            views.add_child(v)
        return views

    views = _make_table_views()

    restore = py_trees.composites.Sequence("restore heading", memory=True)
    restore.add_child(BtNode_CheckBBKeySet("start heading known?", bb_keys.GRASP_START_POSE))
    restore.add_child(_grasp_arm("restore: arm to orbbec look", bb_keys.ARM_ORBBEC_LOOK))
    restore.add_child(_grasp_turn_base("restore: turn back", bb_keys.GRASP_START_POSE))
    give_up = py_trees.composites.Sequence("no view detected the object", memory=True)
    give_up.add_child(py_trees.decorators.FailureIsSuccess("restore heading (best effort)", restore))
    give_up.add_child(py_trees.behaviours.Failure("detection failed"))

    # The planner may enrich a plain object name with an attribute (for
    # example, ``bowl`` becomes ``red bowl`` from the venue vocabulary).  The
    # wrist frame can contain the object while the open-vocabulary detector
    # rejects that adjective, so give the same known-token reduction used by
    # create_find_object one complete retry before falling back to the referee.
    reduced_retry = py_trees.composites.Sequence(
        "reduced grasp query retry", memory=True,
        children=[
            BtNode_ReduceObjectQuery("reduce grasp object query"),
            _make_table_views(),
        ],
    )
    sweep = py_trees.composites.Sequence("detect on table (view sweep)", memory=True)
    sweep.add_child(BtNode_BlackboardSet("reset start heading", bb_keys.GRASP_START_POSE, None))
    sweep.add_child(py_trees.decorators.FailureIsSuccess(
        "capture start heading (best effort)",
        BtNode_CaptureCurrentPose("capture start heading", bb_key=bb_keys.GRASP_START_POSE),
    ))
    sweep.add_child(py_trees.composites.Selector(
        "table views, reduced query, or give up", memory=True,
        children=[views, reduced_retry, give_up],
    ))
    return sweep


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
    # Sim battery 2026-09-30 fix5: the old Retry x5 re-sent the identical
    # request from the identical pose (5/5 "no matches via vlm_sam" with the
    # objects in frame) -- each attempt is now a different view.
    find_and_grasp.add_child(_grasp_table_detection())
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
    primary.add_child(_arm_back_after_grasp("retry arm back"))

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
    ex_machina.add_child(_arm_back_after_grasp("retry arm back"))
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
    # Sim battery 2026-09-26 (runs s70718-000/-004): the robot asked the
    # referee blind. Look at the furniture FIRST (the same pan/tilt sweep +
    # generalist scan create_find_object uses, best effort -- an unseen
    # object still asks, just with different wording), then compose the
    # request from what was actually seen: "Dear referee, I can see the
    # <label> on the <furniture>. Please put the <object> in my gripper." /
    # "Dear referee, I cannot see the <object> on the <furniture>. Please
    # hand me the <object> and put it in my gripper."
    ex_machina.add_child(py_trees.decorators.FailureIsSuccess(
        "look for object at furniture (best effort)",
        _pantilt_sweep("grasp/referee look", OBJECT_TILT_DEG, _referee_object_scan),
    ))
    ex_machina.add_child(BtNode_ComposeRefereeRequest(
        "compose referee request", bb_dst=bb_keys.REFEREE_REQUEST_TEXT,
    ))
    ex_machina.add_child(BtNode_GripperAction("open gripper", True))
    ex_machina.add_child(BtNode_AnnounceFromBB(
        "ask referee", bb_keys.REFEREE_REQUEST_TEXT,
    ))
    ex_machina.add_child(BtNode_Announce(
        "announce put in gripper", bb_source=None,
        message="Thank you.",
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
    """Place the currently-held object on the furniture at ``bb_keys.TARGET_POSE``.

    Sim battery 2026-09-26 (run s70718-003): the old tree tucked the arm,
    drove to the side table and opened the gripper from the TUCKED pose --
    the banana fell on the floor and the bench still recorded placed(). Now:

    announce -> tuck -> goto -> ``place/on_surface`` Selector -> arm back

    ``place/on_surface`` (see :func:`_place_on_surface_or_release`) first
    runs the real placement -- arm to the RealSense scan pose (same
    ``ARM_SCAN`` pose the grasp scans from), wrist point cloud of the
    surface, free-spot selection on the detected table plane, then the
    ``place_action`` server (pick_and_place C++ executor: TCP target +
    env cloud -> octomap-aware place + gripper release + retreat) -- and on
    ANY failure falls back to announcing "I could not place it safely, I
    will put it down here" and the old open-gripper release, so the executor
    never hangs and the plan always moves on. The final arm-back is
    best-effort: the object is already down, and the next goto re-tucks.
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
    seq.add_child(_place_on_surface_or_release())
    seq.add_child(py_trees.decorators.FailureIsSuccess(
        "arm back after place (best effort)",
        py_trees.decorators.Retry(
            "retry arm back after place",
            BtNode_MoveArmSingle(
                "arm back to navigating",
                action_name=ARM_ACTION_NAME,
                arm_pose_bb_key=bb_keys.ARM_ORBBEC_LOOK,
                add_octomap=False,
            ),
            num_failures=5,
        ),
    ))
    return seq


# ---------------------------------------------------------------------------
# Real placement + 360 deg room survey (sim battery 2026-09-26, run
# s70718-003). Everything from here to create_deliver is new; the only
# pre-existing call sites are create_place (above) and create_find_object.
# ---------------------------------------------------------------------------

# place_action is served by the pick_and_place C++ executor, which BOTH the
# hardware bringup (mobile_bringup/grasp_bringup.launch.py) and the sim's
# live-manipulation stage (manipulation_planning_task_only.launch.py) start.
# scan_and_place_action (arm_api/scan_and_place_server, a VLM-driven wrapper
# around this same server) is launched by NEITHER, so the BT owns the
# "where on the surface" step itself (BtNode_SurfacePointFromCloud).
PLACE_ACTION_NAME = "place_action"
# camera_server's compat bridge (vision_bringup enable_gpsr, also launched by
# the sim's vision stage) serves the wrist/head cloud on this legacy name.
PLACE_POINT_CLOUD_SERVICE = "get_point_cloud_service"
# Place.action CONTRACT: target_point is the TCP (link_tcp) target and the
# server does NOT compensate for how far the held object hangs below the
# TCP. The grasp closes around the object's middle, so lift the TCP by
# about half a typical GPSR object (banana / can / box ~ 8-10 cm) plus a
# finger of drop height. Sim battery 2026-10-01 fix8 run s70718-003: at 0.05
# (no drop margin at all) the referee-handed banana, held off-centre below
# the fingers, was pushed into side_table_02 on the way down and ended on
# the floor 0.18 m off the edge. 0.10 = half an object + a 5 cm drop.
PLACE_TCP_CLEARANCE_M = 0.10
# Clear radius around the chosen spot (no point above the plane within it);
# scan_and_place_server's "moderate" margin band is 0.05-0.10 m.
PLACE_FREE_RADIUS_M = 0.08
# Workspace in base_link the spot must lie in (xArm7 reach from the robot's
# base, with the surface in front of the robot after the goto).
PLACE_REACH_X_M = (0.30, 0.75)
PLACE_REACH_Y_M = 0.40
PLACE_PREFERRED_XY_M = (0.50, 0.0)
# Prefer spots with plane support this far out in all four directions: sim
# battery 2026-10-01 fix8/fix9 run s70718-003 placed the banana 4 cm inside
# side_table_02's near edge (half of the 6 cm support disc was enough) and
# the off-centre-held banana tipped off onto the floor both times.
PLACE_EDGE_MARGIN_M = 0.12  # probe offset; with the 2 cm probe radius >= 10 cm of table
# Lowest z (base_link) a supporting plane may have -- rejects the floor.
PLACE_MIN_SURFACE_Z_M = 0.20

# 360 deg survey: four 90 deg base turns, full pan/tilt sweep at each.
SURVEY_HEADINGS = 4
SURVEY_STEP_DEG = 90.0


def _place_on_surface_or_release():
    """``place/on_surface``: real placement, else announce + release in place.

    primary (``place/primary``):
      arm to ``ARM_SCAN`` (octomap on, RealSense over the surface, mirrors
      create_grasp) -> settle -> [wrist point cloud -> free surface point]
      (retried) -> ``place_action`` (retried once) -> gripper open/close
      (the server already released; belt and braces) -> claim
      ``STEP_METHOD`` "autonomous".
    fallback (``place/release_fallback``):
      claim ``STEP_METHOD`` "release_fallback" -> "I could not place it
      safely, I will put it down here." -> open -> wait -> close (the
      pre-2026-09-26 behaviour). Deterministic, so the Selector never hangs.

    The place server's own goal validation rejects an empty cloud and (in
    the strict sim profile) a goal with no attached object -- e.g. after a
    referee hand-over -- as a plain FAILURE, which lands here as the
    fallback, never as a stuck step.
    """
    primary = py_trees.composites.Sequence("place/primary", memory=True)
    par_scan = py_trees.composites.Parallel(
        "arm to scan + announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par_scan.add_child(BtNode_Announce(
        "announce move arm", bb_source=None,
        message="Moving my arm to look at the surface",
    ))
    par_scan.add_child(BtNode_MoveArmSingle(
        "arm to scan",
        action_name=ARM_ACTION_NAME,
        arm_pose_bb_key=bb_keys.ARM_SCAN,
        # A held object can remain in the simulator's octomap after grasping;
        # that stale geometry makes this otherwise valid scan pose fail with
        # MoveIt status 6.  The place action still performs its own collision
        # checks, so the live benchmark may skip only this pre-scan octomap
        # attachment check while the normal hardware path stays strict.
        add_octomap=(os.environ.get("GPSR_ALLOW_NAV_WITHOUT_ARM", "").strip() != "1"),
    ))
    primary.add_child(par_scan)
    # Same reason as the grasp: the first RealSense frames after the arm
    # stops are blurred / unsynced.
    primary.add_child(BtNode_WaitTicks("settle before surface scan", 6))
    surface = py_trees.composites.Sequence("scan surface", memory=True)
    surface.add_child(BtNode_GetPointCloud(
        "realsense surface cloud",
        bb_point_cloud_key=bb_keys.PLACE_ENV_POINTS,
        service_name=PLACE_POINT_CLOUD_SERVICE,
        camera_name="realsense",
    ))
    surface.add_child(BtNode_SurfacePointFromCloud(
        "find free surface point",
        bb_cloud_key=bb_keys.PLACE_ENV_POINTS,
        bb_point_key=bb_keys.PLACE_TARGET_POINT,
        bb_orientation_key=bb_keys.PLACE_ORIENTATION,
    ))
    primary.add_child(py_trees.decorators.Retry(
        "retry scan surface", surface, num_failures=3,
    ))
    par_place = py_trees.composites.Parallel(
        "place+announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    par_place.add_child(BtNode_Announce(
        "announce place", bb_source=None, message="Placing it on the surface now.",
    ))
    par_place.add_child(py_trees.decorators.Retry(
        "retry place on surface",
        BtNode_Place(
            "place on surface",
            bb_key_point=bb_keys.PLACE_TARGET_POINT,
            bb_key_pose=bb_keys.PLACE_ORIENTATION,
            bb_key_env_points=bb_keys.PLACE_ENV_POINTS,
            action_name=PLACE_ACTION_NAME,
        ),
        num_failures=2,
    ))
    primary.add_child(par_place)
    primary.add_child(BtNode_GripperAction("open gripper after place", True))
    primary.add_child(BtNode_WaitTicks("wait after place", 2))
    primary.add_child(BtNode_GripperAction("close gripper after place", False))
    primary.add_child(BtNode_BlackboardSet(
        "mark place autonomous", bb_keys.STEP_METHOD, "autonomous",
    ))

    fallback = py_trees.composites.Sequence("place/release_fallback", memory=True)
    fallback.add_child(BtNode_BlackboardSet(
        "mark place release fallback", bb_keys.STEP_METHOD, "release_fallback",
    ))
    fallback.add_child(BtNode_Announce(
        "announce release fallback", bb_source=None,
        message="I could not place it safely, I will put it down here.",
    ))
    fallback.add_child(BtNode_GripperAction("open gripper", True))
    fallback.add_child(BtNode_WaitTicks("wait", 4))
    fallback.add_child(BtNode_GripperAction("close gripper", False))

    return py_trees.composites.Selector(
        "place/on_surface", memory=True, children=[primary, fallback],
    )


def _quaternion_to_matrix(x: float, y: float, z: float, w: float):
    """3x3 rotation matrix (numpy) of a unit quaternion."""
    import numpy as np
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


class BtNode_SurfacePointFromCloud(Behaviour):
    """Pick a free spot on the supporting surface seen in a point cloud.

    Reads the PointCloud2 at ``bb_cloud_key`` (any frame; TF'd to
    ``base_frame`` through the node's own tf2 listener, like
    BtNode_CaptureCurrentPose), finds the dominant horizontal plane inside
    the arm's workspace (1 cm z-histogram peak above ``min_surface_z``), and
    on that plane picks the grid cell nearest ``preferred_xy`` that has plane
    support underneath and NO point above the plane within ``free_radius``.
    Writes ``bb_point_key`` = PointStamped(base_frame) at (x, y, plane_z +
    tcp_clearance) -- the TCP target Place.action expects -- and
    ``bb_orientation_key`` = an all-zero Pose, which place_action treats as
    "not supplied" (it then reuses the orientation stored at the last pick,
    or a horizontal default).

    FAILURE (with the reason in ``feedback_message``) when the cloud is
    missing / empty / has no TF to ``base_frame`` / shows no plane / has no
    clear spot -- the parent Retry re-scans, then create_place's Selector
    drops to the announce + release fallback. Mock parity: when
    BtNode_GetPointCloud is mocked its cloud carries no data, so this node
    writes a synthetic in-reach point instead of failing (the place is
    mocked too in that configuration).
    """

    def __init__(
        self,
        name: str,
        bb_cloud_key: str,
        bb_point_key: str,
        bb_orientation_key: str = None,
        base_frame: str = "base_link",
        tcp_clearance_m: float = PLACE_TCP_CLEARANCE_M,
        free_radius_m: float = PLACE_FREE_RADIUS_M,
        reach_x_m=PLACE_REACH_X_M,
        reach_y_m: float = PLACE_REACH_Y_M,
        preferred_xy_m=PLACE_PREFERRED_XY_M,
        min_surface_z_m: float = PLACE_MIN_SURFACE_Z_M,
        tf_timeout_sec: float = 1.0,
    ):
        super().__init__(name)
        self.bb_cloud_key = bb_cloud_key
        self.bb_point_key = bb_point_key
        self.bb_orientation_key = bb_orientation_key
        self.base_frame = base_frame
        self.tcp_clearance_m = float(tcp_clearance_m)
        self.free_radius_m = float(free_radius_m)
        self.reach_x_m = (float(reach_x_m[0]), float(reach_x_m[1]))
        self.reach_y_m = float(reach_y_m)
        self.preferred_xy_m = (float(preferred_xy_m[0]), float(preferred_xy_m[1]))
        self.min_surface_z_m = float(min_surface_z_m)
        self.tf_timeout_sec = float(tf_timeout_sec)
        self.node = None
        self.tf_buffer = None
        self.tf_listener = None
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self.bb_cloud_key, access=Access.READ)
        self._client.register_key(self.bb_point_key, access=Access.WRITE)
        if self.bb_orientation_key:
            self._client.register_key(self.bb_orientation_key, access=Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs.get("node")
        if self.node is not None and self.tf_buffer is None:
            try:
                from tf2_ros.buffer import Buffer
                from tf2_ros.transform_listener import TransformListener
                self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
                self.tf_listener = TransformListener(self.tf_buffer, self.node)
            except Exception as exc:  # noqa: BLE001 — degrade to same-frame clouds only
                self.logger.warning(f"{self.name}: no tf2 listener ({exc})")
                self.tf_buffer = None
        return super().setup(**kwargs)

    # -- pure helpers (unit-tested) -------------------------------------

    @staticmethod
    def cloud_to_xyz(cloud):
        """Nx3 float array of the finite xyz points of a PointCloud2."""
        import numpy as np
        from sensor_msgs_py import point_cloud2 as pc2
        xyz = pc2.read_points_numpy(cloud, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        return xyz[np.isfinite(xyz).all(axis=1)]

    @staticmethod
    def pick_surface_point(
        xyz,
        reach_x=PLACE_REACH_X_M,
        reach_y=PLACE_REACH_Y_M,
        free_radius=PLACE_FREE_RADIUS_M,
        preferred_xy=PLACE_PREFERRED_XY_M,
        min_surface_z=PLACE_MIN_SURFACE_Z_M,
        bin_m: float = 0.01,
        plane_tol_m: float = 0.015,
        obstacle_height_m: float = 0.35,
        grid_m: float = 0.04,
        min_plane_points: int = 100,
        min_support: int = 8,
        max_points: int = 80000,
    ):
        """Return ``(x, y, plane_z)`` for the chosen spot, or ``None``.

        ``xyz`` is an Nx3 array in the base frame (x forward, z up). Pure
        numpy, no ROS. ``None`` carries no reason; the node adds one.
        """
        import numpy as np
        pts = np.asarray(xyz, dtype=float).reshape(-1, 3)
        if pts.shape[0] > max_points:
            pts = pts[:: int(math.ceil(pts.shape[0] / max_points))]
        # Workspace crop (a little wider than the reach box so plane support
        # and obstacles just outside the box still count).
        margin = free_radius + 0.05
        keep = (
            (pts[:, 0] >= reach_x[0] - margin) & (pts[:, 0] <= reach_x[1] + margin)
            & (np.abs(pts[:, 1]) <= reach_y + margin)
            & (pts[:, 2] >= min_surface_z) & (pts[:, 2] <= 1.6)
        )
        pts = pts[keep]
        if pts.shape[0] < min_plane_points:
            return None
        # Dominant horizontal plane: peak of the z histogram.
        z = pts[:, 2]
        bins = np.floor((z - z.min()) / bin_m).astype(int)
        counts = np.bincount(bins)
        peak = int(np.argmax(counts))
        near = np.abs(bins - peak) <= 1
        if int(near.sum()) < min_plane_points:
            return None
        plane_z = float(np.median(z[near]))
        on_plane = pts[np.abs(z - plane_z) <= plane_tol_m]
        above = pts[(z > plane_z + 0.02) & (z <= plane_z + obstacle_height_m)]
        # Candidate grid inside the reach box.
        xs = np.arange(reach_x[0], reach_x[1] + 1e-9, grid_m)
        ys = np.arange(-reach_y, reach_y + 1e-9, grid_m)
        gx, gy = np.meshgrid(xs, ys, indexing="ij")
        cand = np.column_stack([gx.ravel(), gy.ravel()])
        # Support: plane points within 0.75 * free_radius; blocked: any point
        # above the plane within free_radius. Chunked so a dense cloud never
        # allocates a giant distance matrix.
        support = np.zeros(cand.shape[0], dtype=int)
        blocked = np.zeros(cand.shape[0], dtype=bool)
        r_sup2 = (0.75 * free_radius) ** 2
        r_free2 = free_radius ** 2
        for start in range(0, cand.shape[0], 64):
            c = cand[start:start + 64]
            if on_plane.shape[0]:
                d2 = ((c[:, None, 0] - on_plane[None, :, 0]) ** 2
                      + (c[:, None, 1] - on_plane[None, :, 1]) ** 2)
                support[start:start + 64] = (d2 <= r_sup2).sum(axis=1)
            if above.shape[0]:
                d2 = ((c[:, None, 0] - above[None, :, 0]) ** 2
                      + (c[:, None, 1] - above[None, :, 1]) ** 2)
                blocked[start:start + 64] = (d2 <= r_free2).any(axis=1)
        ok = (support >= min_support) & ~blocked
        if not ok.any():
            return None
        # Edge margin: keep candidates with plane points near each of the four
        # offsets +-PLACE_EDGE_MARGIN_M (x and y); only when none has that,
        # fall back to every supported candidate.
        if on_plane.shape[0]:
            inner = ok.copy()
            r_probe2 = 0.02 ** 2
            m = PLACE_EDGE_MARGIN_M
            for dx, dy in ((m, 0.0), (-m, 0.0), (0.0, m), (0.0, -m)):
                probe = cand[:, :2] + np.array([dx, dy])
                has = np.zeros(cand.shape[0], dtype=bool)
                for start in range(0, cand.shape[0], 64):
                    c = probe[start:start + 64]
                    d2 = ((c[:, None, 0] - on_plane[None, :, 0]) ** 2
                          + (c[:, None, 1] - on_plane[None, :, 1]) ** 2)
                    has[start:start + 64] = (d2 <= r_probe2).any(axis=1)
                inner &= has
            if inner.any():
                ok = inner
        pref = np.asarray(preferred_xy, dtype=float)
        dist = ((cand[:, 0] - pref[0]) ** 2 + (cand[:, 1] - pref[1]) ** 2)
        dist[~ok] = np.inf
        best = int(np.argmin(dist))
        return float(cand[best, 0]), float(cand[best, 1]), plane_z

    # -- behaviour ---------------------------------------------------------

    def _write_outputs(self, x: float, y: float, z: float, frame: str) -> None:
        ps = PointStamped()
        ps.header.frame_id = frame
        if self.node is not None:
            try:
                ps.header.stamp = self.node.get_clock().now().to_msg()
            except Exception:  # noqa: BLE001
                pass
        ps.point.x, ps.point.y, ps.point.z = float(x), float(y), float(z)
        self._client.set(self.bb_point_key, ps, overwrite=True)
        if self.bb_orientation_key:
            # All-zero quaternion == "orientation not supplied" for
            # place_action (validate_optional_orientation). NOT Pose(): its
            # default is the identity (w=1), which the server would take as
            # an explicit, sideways TCP orientation.
            self._client.set(
                self.bb_orientation_key,
                Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0)),
                overwrite=True,
            )

    def _transform_to_base(self, xyz, frame_id: str):
        """xyz (Nx3, in ``frame_id``) -> base_frame, or None when no TF."""
        import numpy as np
        if not frame_id or frame_id == self.base_frame:
            return xyz
        if self.tf_buffer is None:
            self.feedback_message = (
                f"cloud is in '{frame_id}' but no tf2 buffer to reach '{self.base_frame}'"
            )
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout_sec),
            )
        except Exception as exc:  # noqa: BLE001
            self.feedback_message = f"TF '{self.base_frame}' <- '{frame_id}' failed: {exc}"
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        rot = _quaternion_to_matrix(q.x, q.y, q.z, q.w)
        return xyz @ rot.T + np.array([t.x, t.y, t.z], dtype=float)

    def update(self) -> Status:
        from behavior_tree.config import is_node_mocked
        try:
            cloud = self._client.get(self.bb_cloud_key)
        except KeyError:
            cloud = None
        if cloud is None:
            self.feedback_message = f"no point cloud at {self.bb_cloud_key}"
            return Status.FAILURE
        if not getattr(cloud, "data", None):
            if is_node_mocked("BtNode_GetPointCloud"):
                x, y = self.preferred_xy_m
                self._write_outputs(x, y, 0.75 + self.tcp_clearance_m, self.base_frame)
                self.feedback_message = "MOCK cloud: synthetic surface point"
                return Status.SUCCESS
            self.feedback_message = "point cloud is empty"
            return Status.FAILURE
        try:
            xyz = self.cloud_to_xyz(cloud)
        except Exception as exc:  # noqa: BLE001
            self.feedback_message = f"cannot decode point cloud: {exc}"
            return Status.FAILURE
        xyz = self._transform_to_base(xyz, getattr(cloud.header, "frame_id", ""))
        if xyz is None:
            return Status.FAILURE
        spot = self.pick_surface_point(
            xyz,
            reach_x=self.reach_x_m,
            reach_y=self.reach_y_m,
            free_radius=self.free_radius_m,
            preferred_xy=self.preferred_xy_m,
            min_surface_z=self.min_surface_z_m,
        )
        if spot is None:
            self.feedback_message = (
                f"no free spot on a surface within reach ({xyz.shape[0]} points)"
            )
            return Status.FAILURE
        x, y, plane_z = spot
        self._write_outputs(x, y, plane_z + self.tcp_clearance_m, self.base_frame)
        self.feedback_message = (
            f"surface at z={plane_z:.2f}, placing at ({x:.2f}, {y:.2f}, "
            f"{plane_z + self.tcp_clearance_m:.2f}) {self.base_frame}"
        )
        return Status.SUCCESS


# --- 360 deg room survey ------------------------------------------------------

def is_room_location(name) -> bool:
    """True when ``name`` is a room (a key of the constants' ``search_spots``
    map, the same table ``materialise_params`` sweeps for search_object)."""
    if not name:
        return False
    from .orchestrator import ROOM_SEARCH_SPOTS, _norm_loc  # lazy: avoid import cycle
    return _norm_loc(name) in ROOM_SEARCH_SPOTS


class BtNode_CheckRoomSurveyDue(Behaviour):
    """Guard for create_find_object's 360 deg survey branch.

    SUCCESS only when the target location is a room (``is_room_location``)
    AND, if this find_object sits inside a ``create_search_object`` sweep
    (an ancestor named ``search spot <i>``), no LATER spot of that sweep is
    still filled -- i.e. every search spot has already been visited and the
    survey is the last thing to try before the room search fails. Outside a
    sweep (a direct find_object at a room) the room test alone decides.
    """

    _SPOT_RE = None

    def __init__(self, name: str = "room survey due?"):
        super().__init__(name)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)
        return super().setup(**kwargs)

    def _target_location(self):
        try:
            if self._client is not None:
                return self._client.get(bb_keys.TARGET_LOCATION)
            return Blackboard.get(bb_keys.TARGET_LOCATION)
        except KeyError:
            return None

    @classmethod
    def _spot_index(cls, node) -> int:
        import re
        if cls._SPOT_RE is None:
            cls._SPOT_RE = re.compile(r"^search spot (\d+)$")
        m = cls._SPOT_RE.match(str(getattr(node, "name", "")))
        return int(m.group(1)) if m else -1

    def _search_spot_context(self):
        """``(spot_index, sweep_node)`` of the enclosing search sweep, else ``(-1, None)``."""
        node = self.parent
        while node is not None:
            idx = self._spot_index(node)
            if idx >= 0:
                return idx, node.parent
            node = node.parent
        return -1, None

    def update(self) -> Status:
        location = self._target_location()
        if not is_room_location(location):
            self.feedback_message = f"{location!r} is not a room: no survey"
            return Status.FAILURE
        idx, sweep = self._search_spot_context()
        if sweep is not None:
            for later in sweep.children:
                later_idx = self._spot_index(later)
                if later_idx <= idx:
                    continue
                try:
                    pending = Blackboard.get(f"gpsr/search_pose_{later_idx}") is not None
                except KeyError:
                    pending = False
                if pending:
                    self.feedback_message = (
                        f"search spot {later_idx} still pending: survey deferred"
                    )
                    return Status.FAILURE
        self.feedback_message = f"room survey due at {location!r}"
        return Status.SUCCESS


class BtNode_YawOffsetPose(Behaviour):
    """Write ``dst_key`` = the PoseStamped at ``src_key`` yawed by ``delta_yaw_deg``.

    Position and frame are kept; only the heading changes (wrapped to
    (-180, 180]). Pure quaternion math, no ROS backend needed.
    """

    def __init__(self, name: str, src_key: str, dst_key: str, delta_yaw_deg: float):
        super().__init__(name)
        self.src_key = src_key
        self.dst_key = dst_key
        self.delta_yaw_deg = float(delta_yaw_deg)
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self.src_key, access=Access.READ)
        self._client.register_key(self.dst_key, access=Access.WRITE)

    @staticmethod
    def wrap_deg(deg: float) -> float:
        """Wrap an angle to (-180, 180]."""
        wrapped = (float(deg) + 180.0) % 360.0 - 180.0
        return 180.0 if wrapped == -180.0 else wrapped

    @staticmethod
    def yaw_from_quaternion(q) -> float:
        """Yaw (deg) of a geometry_msgs Quaternion."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny, cosy))

    @staticmethod
    def quaternion_from_yaw(yaw_deg: float) -> Quaternion:
        half = math.radians(yaw_deg) / 2.0
        return Quaternion(x=0.0, y=0.0, z=math.sin(half), w=math.cos(half))

    @classmethod
    def rotate_pose(cls, pose: PoseStamped, delta_yaw_deg: float) -> PoseStamped:
        yaw = cls.wrap_deg(cls.yaw_from_quaternion(pose.pose.orientation) + delta_yaw_deg)
        out = PoseStamped()
        out.header.frame_id = pose.header.frame_id
        out.header.stamp = pose.header.stamp
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z
        out.pose.orientation = cls.quaternion_from_yaw(yaw)
        return out

    def update(self) -> Status:
        try:
            pose = self._client.get(self.src_key)
        except KeyError:
            pose = None
        if not isinstance(pose, PoseStamped):
            self.feedback_message = f"no PoseStamped at {self.src_key}"
            return Status.FAILURE
        out = self.rotate_pose(pose, self.delta_yaw_deg)
        self._client.set(self.dst_key, out, overwrite=True)
        self.feedback_message = (
            f"yaw {self.yaw_from_quaternion(pose.pose.orientation):+.0f} -> "
            f"{self.yaw_from_quaternion(out.pose.orientation):+.0f} deg"
        )
        return Status.SUCCESS


class BtNode_FurnitureStandoffPose(Behaviour):
    """Prepare the concrete Nav2 goal for a semantic furniture waypoint.

    Furniture poses are intentionally placed just outside the object/surface
    so the arm can work from them.  Nav2 inflates those same surfaces for the
    base footprint, however, and a direct goal can therefore be rejected or
    settle inside the goal tolerance while still being too close for the next
    interaction.  Room aliases use the configured room anchor, while object
    searches retain their independent furniture sweep spots.

    The semantic ``TARGET_POSE`` is never modified.  This keeps navigation
    evidence and room/object contracts tied to the configured waypoint while
    allowing the action to use a collision-safe scratch pose.  Unknown and
    runtime-recorded locations are copied exactly, as are start/operator
    aliases and ``command_point``.
    """

    def __init__(
        self,
        name: str,
        src_key: str = bb_keys.TARGET_POSE,
        dst_key: str = bb_keys.NAV_GOAL_POSE,
        distance_m: float = FURNITURE_STANDOFF_DISTANCE_M,
    ):
        super().__init__(name)
        self._src_key = src_key
        self._dst_key = dst_key
        self._distance_m = max(0.0, float(distance_m))
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._src_key, access=Access.READ)
        self._client.register_key(self._dst_key, access=Access.WRITE)
        self._client.register_key(bb_keys.TARGET_LOCATION, access=Access.READ)

    @staticmethod
    def _normalise(value) -> str:
        return str(value or "").strip().lower().replace(" ", "_")

    @classmethod
    def _is_static_furniture_or_room(cls, location) -> bool:
        key = cls._normalise(location)
        if not key or key in {
            "command_point", "start_position", "instruction_point", "start",
            "operator", "me", "the_user", "the_operator",
        }:
            return False
        # Import lazily to avoid the orchestrator/small_trees import cycle.
        try:
            from .orchestrator import KNOWN_LOCATIONS, ROOM_SEARCH_SPOTS
            if key in ROOM_SEARCH_SPOTS:
                return True
            # Direct furniture waypoints are calibrated for perception and
            # grasping (for example side_table_02); shifting those goals can
            # move the robot outside the working envelope. Room aliases use
            # the standoff because their anchor may be embedded in furniture.
            return False
        except Exception:
            return False

    @staticmethod
    def _copy_pose(pose: PoseStamped) -> PoseStamped:
        out = PoseStamped()
        out.header.frame_id = pose.header.frame_id
        out.header.stamp = pose.header.stamp
        out.pose.position.x = pose.pose.position.x
        out.pose.position.y = pose.pose.position.y
        out.pose.position.z = pose.pose.position.z
        out.pose.orientation = Quaternion(
            x=pose.pose.orientation.x,
            y=pose.pose.orientation.y,
            z=pose.pose.orientation.z,
            w=pose.pose.orientation.w,
        )
        return out

    @classmethod
    def _standoff_pose(cls, pose: PoseStamped, distance_m: float) -> PoseStamped:
        out = cls._copy_pose(pose)
        yaw = math.radians(BtNode_YawOffsetPose.yaw_from_quaternion(pose.pose.orientation))
        # The configured heading faces the furniture.  Move opposite that
        # heading so the footprint starts outside the inflated obstacle halo.
        out.pose.position.x -= distance_m * math.cos(yaw)
        out.pose.position.y -= distance_m * math.sin(yaw)
        return out

    def update(self) -> Status:
        try:
            pose = self._client.get(self._src_key)
        except Exception:
            pose = None
        if not isinstance(pose, PoseStamped):
            self.feedback_message = f"no PoseStamped at {self._src_key}"
            return Status.FAILURE
        try:
            location = self._client.get(bb_keys.TARGET_LOCATION)
        except Exception:
            location = None
        if self._is_static_furniture_or_room(location):
            out = self._standoff_pose(pose, self._distance_m)
            self.feedback_message = (
                f"{location}: furniture standoff {self._distance_m:.2f} m"
            )
        else:
            out = self._copy_pose(pose)
            self.feedback_message = f"{location}: exact navigation pose"
        self._client.set(self._dst_key, out, overwrite=True)
        return Status.SUCCESS


class BtNode_YawTowardDetection(Behaviour):
    """Write ``dst_key`` = the robot pose at ``pose_key`` turned to face the
    first object (map-frame centroid) of the detection at ``detection_key``.

    Position and frame are kept. The turn is clamped to ``max_deg`` (a far-
    off detection must not spin the robot away from the furniture). FAILURE
    when there is no pose or no usable centroid. Pure math, no ROS backend.
    """

    def __init__(self, name: str, pose_key: str, detection_key: str, dst_key: str,
                 max_deg: float = 45.0):
        super().__init__(name)
        self._pose_key = pose_key
        self._detection_key = detection_key
        self._dst_key = dst_key
        self._max_deg = float(max_deg)
        self._client = None

    def setup(self, **kwargs):
        self._client = self.attach_blackboard_client(name=self.name)
        self._client.register_key(self._pose_key, access=Access.READ)
        self._client.register_key(self._detection_key, access=Access.READ)
        self._client.register_key(self._dst_key, access=Access.WRITE)

    @classmethod
    def delta_to_point(cls, pose: PoseStamped, x: float, y: float, max_deg: float) -> float:
        """Clamped heading change (deg) that makes ``pose`` face (x, y)."""
        px, py = pose.pose.position.x, pose.pose.position.y
        want = math.degrees(math.atan2(float(y) - py, float(x) - px))
        have = BtNode_YawOffsetPose.yaw_from_quaternion(pose.pose.orientation)
        delta = BtNode_YawOffsetPose.wrap_deg(want - have)
        return max(-max_deg, min(max_deg, delta))

    def update(self):
        try:
            pose = self._client.get(self._pose_key)
        except Exception:
            pose = None
        if not isinstance(pose, PoseStamped):
            self.feedback_message = f"no PoseStamped at {self._pose_key}"
            return Status.FAILURE
        try:
            detection = self._client.get(self._detection_key)
        except Exception:
            detection = None
        objects = list(getattr(detection, "objects", None) or [])
        if not objects:
            self.feedback_message = "head camera saw no object to face"
            return Status.FAILURE
        c = getattr(objects[0], "centroid", None)
        try:
            cx, cy = float(c.x), float(c.y)
        except (AttributeError, TypeError, ValueError):
            self.feedback_message = "head detection has no centroid"
            return Status.FAILURE
        if abs(cx) < 1e-6 and abs(cy) < 1e-6:
            self.feedback_message = "head detection centroid unset"
            return Status.FAILURE
        delta = self.delta_to_point(pose, cx, cy, self._max_deg)
        self._client.set(self._dst_key, BtNode_YawOffsetPose.rotate_pose(pose, delta), overwrite=True)
        self.feedback_message = f"face head detection at ({cx:.2f}, {cy:.2f}): turn {delta:+.0f} deg"
        return Status.SUCCESS


class BtNode_RotateInPlace(py_trees.composites.Sequence):
    """Turn the base by ``delta_yaw_deg`` on the spot.

    Sequence: capture the current map pose (BtNode_CaptureCurrentPose, the
    node create_record_position uses) -> BtNode_YawOffsetPose -> a retried
    BtNode_GotoAction to the same x/y with the new heading (a key-based
    PoseStamped goal, like every other goto).
    """

    def __init__(
        self,
        name: str,
        delta_yaw_deg: float,
        capture_key: str = bb_keys.SURVEY_CAPTURE_POSE,
        goal_key: str = bb_keys.SURVEY_GOAL_POSE,
        goto_retries: int = 3,
    ):
        super().__init__(name, memory=True)
        self.delta_yaw_deg = float(delta_yaw_deg)
        self.add_child(BtNode_CaptureCurrentPose(f"{name}: capture pose", bb_key=capture_key))
        self.add_child(BtNode_YawOffsetPose(
            f"{name}: yaw {self.delta_yaw_deg:+.0f}", capture_key, goal_key, self.delta_yaw_deg,
        ))
        self.add_child(py_trees.decorators.Retry(
            f"retry {name}: goto heading",
            BtNode_GotoAction(f"{name}: goto heading", key=goal_key),
            num_failures=int(goto_retries),
        ))


def create_survey_room(
    scan_subtree_factory,
    tilts,
    pan_deg=None,
    label: str = "survey",
    headings: int = SURVEY_HEADINGS,
    step_deg: float = SURVEY_STEP_DEG,
):
    """360 deg survey: ``headings`` x [rotate ``step_deg``, pan/tilt sweep].

    A memory Selector that SUCCEEDS at the first heading whose sweep
    (``_pantilt_sweep`` over ``PAN_SWEEP_DEG`` (or ``pan_deg``) x ``tilts``,
    with ``scan_subtree_factory`` as the per-look detector) detects the
    target, FAILURE once every heading has been tried. The rotation is
    best-effort (a nav hiccup still scans from the current heading). Shared
    by find_object today; find_person / count can pass their own scan
    factory and tilts.
    """
    survey = py_trees.composites.Selector(f"{label} 360 survey", memory=True)
    for i in range(1, int(headings) + 1):
        branch = py_trees.composites.Sequence(f"{label} heading {i}", memory=True)
        branch.add_child(py_trees.decorators.FailureIsSuccess(
            f"{label} rotate {i} (best effort)",
            BtNode_RotateInPlace(f"{label} rotate {i}", delta_yaw_deg=step_deg),
        ))
        sweep = _pantilt_sweep(
            f"{label} heading {i}", tilts, scan_subtree_factory, pan_deg=pan_deg,
        )
        # Not "... pantilt sweep": the survey looks are NOT extra
        # find_object_sweep modification targets (the pan-tilt-sweep template
        # rebuilds the whole find_object tree, which reaches them anyway).
        sweep.name = f"{label} heading {i} survey sweep"
        branch.add_child(sweep)
        survey.add_child(branch)
    return survey


def create_deliver():
    """Bring the currently-held object to a person.

    Navigates to the recipient's room (``bb_keys.TARGET_POSE``, materialised
    from ``recipient_location``), then visually detects and approaches the
    recipient before opening the gripper. When a preceding step in the same
    target already found and approached that person and this step has no
    ``recipient_location``, the resolved person pose is reused instead of
    returning to the room waypoint.
    """
    seq = py_trees.composites.Sequence("small/deliver", memory=True)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce delivering", bb_keys.TARGET_OBJECT_NAME, prefix="Delivering the "
    ))
    seq.add_child(_tuck_arm_for_nav("tuck arm before deliver goto"))
    goto_retry = py_trees.decorators.Retry(
        "retry deliver goto",
        BtNode_GotoAction("goto recipient room", key=bb_keys.TARGET_POSE),
        num_failures=5,
    )
    # Room anchors are only a staging hint: a stale/blocked anchor must not
    # prevent delivery from making a fresh visual person scan and approach.
    # Keep the retry budget for transient Nav2 failures, then treat exhausted
    # navigation as best-effort so the recipient path can still run.
    goto_best_effort = py_trees.decorators.FailureIsSuccess(
        "deliver room navigation best effort", goto_retry,
    )
    # A prior find_person + approach_person in this target already left the
    # robot beside the requested recipient. In the common split-plan form
    # deliver has no recipient_location, so a second goto would reuse the
    # previous room waypoint and move away from that person. Keep the node's
    # public name for tree tooling while selecting the reuse path first.
    reuse_goto = py_trees.composites.Sequence(
        "reuse resolved delivery navigation", memory=True,
    )
    reuse_goto.add_child(BtNode_CheckResolvedDeliveryTarget(
        # A preceding find_person is enough to avoid the stale room goto;
        # create_deliver's own approach_person step handles the pose when the
        # planner did not emit a separate approach_person action.
        "recipient already resolved?", require_nav=False,
    ))
    reuse_goto.add_child(py_trees.behaviours.Success("already at recipient"))
    reuse_nav = py_trees.composites.Sequence(
        "reuse successful target navigation", memory=True,
    )
    reuse_nav.add_child(BtNode_CheckLastNavMatchesTarget())
    reuse_nav.add_child(py_trees.behaviours.Success("already at delivery target"))
    seq.add_child(py_trees.composites.Selector(
        "retry deliver goto", memory=True,
        children=[reuse_goto, reuse_nav, goto_best_effort],
    ))
    # For the operator, ``TARGET_POSE`` is the captured command-point pose;
    # scanning again at that pose is both redundant and able to select a
    # false-positive person.  Other recipients keep the visual find/approach
    # path.
    seq.add_child(create_find_person(skip_operator=True))
    seq.add_child(create_approach_person(
        skip_operator=True,
        approach_max_distance=DELIVERY_PERSON_MAX_DISTANCE_M,
        fail_on_unreachable=True,
    ))
    seq.add_child(BtNode_Announce(
        "announce take it", bb_source=None,
        message="Here you go, please take the object from my gripper.",
    ))
    seq.add_child(BtNode_WaitTicks("wait for take", 8))
    seq.add_child(BtNode_GripperAction("open gripper", True))
    seq.add_child(BtNode_WaitTicks("wait", DELIVERY_RELEASE_SETTLE_TICKS))
    # Leave the gripper open after the handoff.  Gripper goals are latched by
    # the facade and streamed until the next goal; closing here can reacquire
    # an object that has not fully cleared the fingers yet.
    return seq


def _count_view(label: str, pan: float, tilt: float):
    """One (pan, tilt) view of the counting sweep.

    Turn, scan for ``TARGET_OBJECT_PROMPT`` (a no-match scan still stores the
    frame, so it is FailureIsSuccess-wrapped), count the member detections of
    THIS view into ``COUNT_VIEW_VALUE`` and fold it into the running maximum
    (``BtNode_CountKeepMax``). Skipped entirely once ``COUNT_SWEEP_DONE`` has
    latched (a view already counted >= 1 and a further view added nothing).
    Best-effort: a failed turn never aborts the sweep.
    """
    look = py_trees.composites.Sequence(
        f"{label} pan={pan:+.0f} tilt={tilt:+.0f}", memory=True)
    look.add_child(BtNode_TurnPanTilt(
        f"pan {pan:+.0f} tilt {tilt:+.0f}", x=pan, y=tilt,
    ))
    look.add_child(py_trees.decorators.FailureIsSuccess(
        "scan to count (no-match keeps frame)",
        BtNode_ScanForGeneralist(
            name="scan to count",
            bb_source=bb_keys.TARGET_OBJECT_PROMPT,
            bb_key=bb_keys.TARGET_OBJECT_DETECTION,
            use_orbbec=True,
            transform_to_map=False,
            use_vlm_sam_fallback=True,
            sort_closest=True,
            # masks give each detection a pixel box (the response has no
            # bbox field) for the dedupe and the self-occlusion filter
            return_segments=True,
            return_rgb_image=True,  # keep the frame for the VLM fallback
        ),
    ))
    look.add_child(BtNode_CountDetections(
        "count view detections",
        bb_detection_src=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VIEW_VALUE,
        bb_member_prompts_key=bb_keys.COUNT_MEMBER_PROMPTS,
        bb_unclassified_dst=bb_keys.COUNT_VIEW_UNCLASSIFIED,
    ))
    look.add_child(BtNode_CountKeepMax(
        "keep best count",
        bb_view_key=bb_keys.COUNT_VIEW_VALUE,
        bb_detection_key=bb_keys.TARGET_OBJECT_DETECTION,
        bb_count_dst=bb_keys.COUNT_VALUE,
        bb_best_dst=bb_keys.COUNT_BEST_DETECTION,
        bb_done_key=bb_keys.COUNT_SWEEP_DONE,
        view_pan_deg=pan, bb_best_pan_dst=bb_keys.COUNT_BEST_PAN_RAD,
        bb_history_dst=bb_keys.COUNT_VIEW_COUNTS,
    ))
    look.add_child(BtNode_CountKeepUnclassified(
        "keep unclassified frame",
        bb_view_key=bb_keys.COUNT_VIEW_UNCLASSIFIED,
        bb_detection_key=bb_keys.TARGET_OBJECT_DETECTION,
        bb_max_dst=bb_keys.COUNT_UNCLASSIFIED,
        bb_frame_dst=bb_keys.COUNT_UNCLASSIFIED_DETECTION,
    ))
    view = py_trees.composites.Selector(
        f"{label} view pan={pan:+.0f} tilt={tilt:+.0f}", memory=True)
    view.add_child(BtNode_CheckBBTrue(
        "sweep saturated?", bb_keys.COUNT_SWEEP_DONE, quiet_on_falsy=True,
    ))
    view.add_child(py_trees.decorators.FailureIsSuccess(
        f"{label} look pan={pan:+.0f} tilt={tilt:+.0f} (best effort)", look,
    ))
    return view


def _count_pantilt(label: str, tilts, pans=None):
    """Every (pan, tilt) view of a counting sweep, tilt-outer / pan-inner like
    ``_pantilt_sweep`` -- but a memory Sequence that visits views until the
    three-view minimum and saturation latch both allow it to stop."""
    pans = PAN_SWEEP_DEG if pans is None else [float(p) for p in pans]
    sweep = py_trees.composites.Sequence(f"{label} pantilt sweep", memory=True)
    for tilt in tilts:
        for pan in pans:
            sweep.add_child(_count_view(label, pan, tilt))
    return sweep


def _count_sweep(label: str):
    """A full counting sweep: reset the running maximum / best frame / view
    history / early-stop latch, then sweep ``PAN_SWEEP_DEG`` x ``COUNT_TILT_DEG``
    (20/45/60 down -- a table 0.6 m below the camera needs the steep looks) keeping the
    MAXIMUM member count over all views. A person count (prompt mentions
    "person") sweeps ``HUMAN_TILT_DEG`` instead -- standing people are not on
    the floor. Always SUCCESS; the result is ``COUNT_VALUE`` (0 when nothing
    was seen anywhere) and ``COUNT_BEST_DETECTION`` (the busiest frame)."""
    seq = py_trees.composites.Sequence(f"{label} sweep", memory=True)
    seq.add_child(BtNode_BlackboardSet("reset count", bb_keys.COUNT_VALUE, 0))
    seq.add_child(BtNode_BlackboardSet(
        "reset sweep latch", bb_keys.COUNT_SWEEP_DONE, False))
    seq.add_child(BtNode_BlackboardSet(
        "reset view counts", bb_keys.COUNT_VIEW_COUNTS, []))
    seq.add_child(BtNode_BlackboardSet(
        "reset best frame", bb_keys.COUNT_BEST_DETECTION, None))
    seq.add_child(BtNode_BlackboardSet(
        "reset unclassified", bb_keys.COUNT_UNCLASSIFIED, 0))
    seq.add_child(BtNode_BlackboardSet(
        "reset unclassified frame", bb_keys.COUNT_UNCLASSIFIED_DETECTION, None))
    by_target = py_trees.composites.Selector(f"{label} sweep by target", memory=True)
    persons = py_trees.composites.Sequence(f"{label} persons", memory=True)
    persons.add_child(BtNode_CheckBBContains(
        "counting persons?", bb_keys.TARGET_OBJECT_PROMPT, "person",
    ))
    persons.add_child(_count_pantilt(f"{label} persons", HUMAN_TILT_DEG, HUMAN_PAN_SWEEP_DEG))
    by_target.add_child(persons)
    by_target.add_child(_count_pantilt(f"{label} objects", COUNT_TILT_DEG))
    seq.add_child(by_target)
    seq.add_child(_count_refinements(label))
    return seq


def _count_refinements(label: str):
    """Post-sweep VLM refinements of ``COUNT_VALUE`` (both best effort, so a
    VLM hiccup leaves the detector count standing):

    1. ``{label} unclassified vlm``: some view held detections labelled with
       the whole multi-class prompt (``COUNT_UNCLASSIFIED`` >= 1) -- they
       could not be matched to the category's members, so the VLM counts the
       category (``COUNT_VLM_SUBJECT``) on that frame and the count becomes
       max(member count, VLM count).
    2. ``{label} category vlm``: category counts also ask the VLM about the
       busiest sweep frame when the detector found at least one member. A
       repeated detector count is the supported estimate; a higher VLM count
       can still recover a missed member, while a one-view detector spike is
       not allowed to override agreement across multiple views.
    3. ``{label} attribute vlm``: a person count with a descriptor
       (``COUNT_ATTRIBUTE_QUESTION``, e.g. "pointing to the right") and
       N >= 1 detected persons: ask the VLM how many of the people on the
       best frame match; the count becomes min(VLM, N) -- never more than the
       detector saw.
    """
    seq = py_trees.composites.Sequence(f"{label} refinements", memory=True)

    unclassified = py_trees.composites.Sequence(f"{label} unclassified vlm", memory=True)
    unclassified.add_child(BtNode_CheckCountPositive(
        "unclassified detections?", bb_keys.COUNT_UNCLASSIFIED))
    unclassified.add_child(BtNode_VLMQuery(
        "vlm unclassified count",
        bb_detection_src=bb_keys.COUNT_UNCLASSIFIED_DETECTION,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template=(
            "How many {value} are visible in this image? Count every one of "
            "them. Answer with a number first."
        ),
        bb_fill_key=bb_keys.COUNT_VLM_SUBJECT,
    ))
    unclassified.add_child(BtNode_ParseCountFromAnswer(
        "parse unclassified count",
        bb_answer_src=bb_keys.VLM_ANSWER, bb_count_dst=bb_keys.COUNT_VLM_VALUE,
    ))
    unclassified.add_child(BtNode_CountCombine(
        "count = max(members, vlm)", bb_keys.COUNT_VALUE, bb_keys.COUNT_VLM_VALUE, mode="max",
    ))
    seq.add_child(py_trees.decorators.FailureIsSuccess(
        f"{label} unclassified vlm (best effort)", unclassified,
    ))

    # A category detector response can be classified correctly and still miss
    # partly occluded members (the live side-table battery saw two mugs out of
    # six kitchen items).  Use the busiest RGB frame for a category-specific
    # VLM recount even when the detector count is already positive.  This is
    # gated on COUNT_MEMBER_PROMPTS so specific-object and person counts keep
    # their existing detector-only behavior.
    category = py_trees.composites.Sequence(f"{label} category vlm", memory=True)
    category.add_child(BtNode_CheckBBTrue(
        "category count?", bb_keys.COUNT_MEMBER_PROMPTS, quiet_on_falsy=True,
    ))
    category.add_child(BtNode_CheckCountPositive(
        "detector saw category members?", bb_keys.COUNT_VALUE,
    ))
    category.add_child(BtNode_VLMQuery(
        "vlm category count",
        bb_detection_src=bb_keys.COUNT_BEST_DETECTION,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template=(
            "How many {value} are visible in this image? Count every visible "
            "member, including partly occluded ones. Answer with a number "
            "first."
        ),
        bb_fill_key=bb_keys.COUNT_VLM_SUBJECT,
    ))
    category.add_child(BtNode_ParseCountFromAnswer(
        "parse category count",
        bb_answer_src=bb_keys.VLM_ANSWER, bb_count_dst=bb_keys.COUNT_VLM_VALUE,
    ))
    category.add_child(BtNode_CountCombine(
        "count = consensus(members, category vlm)",
        bb_keys.COUNT_VALUE, bb_keys.COUNT_VLM_VALUE,
        mode="consensus", bb_views_key=bb_keys.COUNT_VIEW_COUNTS,
    ))
    seq.add_child(py_trees.decorators.FailureIsSuccess(
        f"{label} category vlm (best effort)", category,
    ))

    attribute = py_trees.composites.Sequence(f"{label} attribute vlm", memory=True)
    attribute.add_child(BtNode_CheckBBTrue(
        "person attribute set?", bb_keys.COUNT_ATTRIBUTE_QUESTION, quiet_on_falsy=True))
    attribute.add_child(BtNode_CheckCountPositive(
        "detector saw persons?", bb_keys.COUNT_VALUE))
    attribute.add_child(BtNode_BlackboardSet(
        "reset attribute count", bb_keys.COUNT_VLM_VALUE, None))
    attribute.add_child(BtNode_BlackboardSet(
        "reset whole-body attribute count", bb_keys.COUNT_ATTR_VLM_VALUE, None))
    question = (
        "How many people in this image match this description: {value}? "
        "Count only people whose visible pose or appearance matches it; "
        "exclude people who do not match. Answer with a number first."
    )
    attribute.add_child(BtNode_VLMQuery(
        "vlm attribute count",
        bb_detection_src=bb_keys.COUNT_BEST_DETECTION,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template=question,
        bb_fill_key=bb_keys.COUNT_ATTRIBUTE_QUESTION,
    ))
    attribute.add_child(BtNode_ParseCountFromAnswer(
        "parse attribute count",
        bb_answer_src=bb_keys.VLM_ANSWER, bb_count_dst=bb_keys.COUNT_VLM_VALUE,
    ))
    # Sim battery 2026-10-01 fix9 run s70718-002: the best counting view was
    # the tilt-45 look, which cut the person off at the chest -- the raised
    # arm ("pointing to the right") was above the frame, the VLM said 0 and
    # min(0, 1) announced 0. Re-look at the same pan with the head raised so
    # the whole body (arms included) is in frame, ask again, keep the larger
    # VLM answer (the detector count still caps it).
    whole = py_trees.composites.Sequence(f"{label} whole-body re-look", memory=True)
    whole.add_child(BtNode_BlackboardSet("reset whole-body frame", bb_keys.COUNT_ATTR_DETECTION, None))
    whole.add_child(BtNode_BlackboardSet(
        "reset whole-body attribute count", bb_keys.COUNT_ATTR_VLM_VALUE, None))
    whole.add_child(BtNode_TurnPanTilt(
        "head up at best pan", x_key=bb_keys.COUNT_BEST_PAN_RAD, y=PERSON_ATTRIBUTE_TILT_DEG))
    whole.add_child(py_trees.decorators.FailureIsSuccess(
        "whole-body scan (no-match keeps frame)",
        BtNode_ScanForGeneralist(
            name="whole-body scan",
            bb_source=bb_keys.TARGET_OBJECT_PROMPT,
            bb_key=bb_keys.COUNT_ATTR_DETECTION,
            use_orbbec=True, transform_to_map=False, use_vlm_sam_fallback=True,
            sort_closest=True, return_segments=False, return_rgb_image=True,
        ),
    ))
    # A failed scan still stores its captured frame so generic fallbacks can
    # inspect it. It is not valid attribute evidence, however: only query a
    # re-look that actually detected at least one person/object.
    whole.add_child(BtNode_CheckDetectionUsable(
        "whole-body frame usable?", bb_keys.COUNT_ATTR_DETECTION,
    ))
    whole.add_child(BtNode_VLMQuery(
        "vlm whole-body attribute count",
        bb_detection_src=bb_keys.COUNT_ATTR_DETECTION,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template=question,
        bb_fill_key=bb_keys.COUNT_ATTRIBUTE_QUESTION,
    ))
    whole.add_child(BtNode_ParseCountFromAnswer(
        "parse whole-body attribute count",
        bb_answer_src=bb_keys.VLM_ANSWER, bb_count_dst=bb_keys.COUNT_ATTR_VLM_VALUE,
    ))
    attribute.add_child(py_trees.decorators.FailureIsSuccess(
        f"{label} whole-body re-look (best effort)", whole))
    # A single VLM zero can be a crop/miss. Accept zero only when both
    # independently captured views agree; positive answers remain bounded by
    # the detector count and the larger view recovers a partially cropped pose.
    attribute.add_child(BtNode_CountCombine(
        "count = confirmed attribute(vlm, detector)", bb_keys.COUNT_VALUE,
        bb_keys.COUNT_VLM_VALUE, mode="attribute_confirmed",
        bb_confirm_key=bb_keys.COUNT_ATTR_VLM_VALUE,
    ))
    # A waving-person action detector has explicit pose evidence and is more
    # reliable than asking a VLM to infer a raised arm from the sweep's best
    # (sometimes cropped) RGB frame. Keep the branch narrowly gated so other
    # person descriptors retain the generic VLM/re-look behavior. If the
    # action server is unavailable or returns no detections, the selector falls
    # through to the generic attribute sequence above.
    waving = py_trees.composites.Sequence(
        f"{label} dedicated waving detector", memory=True)
    waving.add_child(BtNode_CheckBBContains(
        "descriptor mentions waving?", bb_keys.COUNT_ATTRIBUTE_QUESTION, "wav"))
    waving.add_child(BtNode_CheckCountPositive(
        "detector saw persons for waving?", bb_keys.COUNT_VALUE))
    waving.add_child(BtNode_BlackboardSet(
        "reset waving detections", bb_keys.ALL_WAVING_PERSONS, []))
    waving.add_child(py_trees.decorators.FailureIsSuccess(
        "head up at best waving pan (best effort)",
        BtNode_BestEffortTurnPanTilt(
            "head up at best waving pan", x_key=bb_keys.COUNT_BEST_PAN_RAD,
            y=PERSON_ATTRIBUTE_TILT_DEG)))
    waving.add_child(py_trees.decorators.FailureIsSuccess(
        "dedicated waving scan (best effort)",
        BtNode_BestEffortScanForWavingPerson(
            "find waving persons for count",
            bb_keys.ALL_WAVING_PERSONS,
            bb_keys.TARGET_PERSON_POSE,
            WAVING_THRESHOLD_METERS,
            target_frame="map",
        ),
    ))
    waving.add_child(BtNode_CountWavingPersons())
    attribute_selector = py_trees.composites.Selector(
        f"{label} attribute strategies", memory=True,
        children=[waving, attribute],
    )
    seq.add_child(py_trees.decorators.FailureIsSuccess(
        f"{label} attribute vlm (best effort)", attribute_selector))
    return seq


def create_count():
    """Count objects matching ``bb_keys.TARGET_OBJECT_PROMPT`` and announce the number.

    Counting is NOT a first-class capability of the vision stack — the
    detector enumerates what it happens to segment, which is unreliable for
    category counting. Sim battery 2026-09-26 (run s70718-001, "how many
    kitchen items on the side table"): ONE look at tilt 10 (the table was
    0.6 m below the camera -- out of frame) asking only for the category's
    enriched member prompts saw nothing, the VLM said "Zero", and the gate
    passed a 0 with bleach, mustard and a bowl on the table.

    Structure now:

    1. ``count/by_detector``: a full pan/tilt sweep (``_count_sweep``:
       ``PAN_SWEEP_DEG`` x ``COUNT_TILT_DEG`` 20/45/60 down) that keeps the
       MAXIMUM member count over all views (early stop once a view counted
       >= 1 and the next added nothing). For a category the orchestrator
       sets ``TARGET_OBJECT_PROMPT`` to EVERY known object and
       ``COUNT_MEMBER_PROMPTS`` to the members, so the detector looks for
       everything and ``BtNode_CountDetections`` filters afterwards
       (confidence floor ``COUNT_MIN_CONF``, own-arm self-occlusion). Then
       ``_count_refinements``: unclassifiable multi-class labels -> VLM
       category count on that frame (max); a person count with a descriptor
       (``COUNT_ATTRIBUTE_QUESTION``; the detector looked for plain "person")
       -> VLM "how many are <descriptor>" on the best frame (min). A
       positive count wins.
    2. ``count/vlm_fallback``: zero after the whole sweep -> ask the VLM to
       count on the best frame, naming the category members
       (``COUNT_VLM_SUBJECT``). A positive parsed count wins.
    3. ``count/recount``: still zero -> announce "I did not see any, let me
       look again", mark ``COUNT_RECOUNTED`` and run ONE more full sweep,
       accepting whatever it finds (including zero). The ``counted()`` gate
       (validators.py) rejects a zero count that did not go through this
       recount.

    Then announce + buffer ``COUNT_VALUE``.
    """
    seq = py_trees.composites.Sequence("small/count", memory=True)
    # Room anchors are useful navigation waypoints but are not necessarily a
    # good camera pose for a pose-qualified person.  materialise_params fills
    # COUNT_ROOM_POSE only for those count steps; the hop is best effort so a
    # navigation hiccup still leaves the normal detector/VLM fallback intact.
    room_view = py_trees.composites.Selector(
        "count room view navigation", memory=True,
        children=[
            py_trees.composites.Sequence(
                "navigate to count room view", memory=True,
                children=[
                    BtNode_CheckBBKeySet("count room view set?", bb_keys.COUNT_ROOM_POSE),
                    _tuck_arm_for_nav("tuck arm before count room view"),
                    py_trees.decorators.FailureIsSuccess(
                        "count room view goto (best effort)",
                        py_trees.decorators.Retry(
                            "retry count room view goto",
                            BtNode_GotoAction(
                                "goto count room view", key=bb_keys.COUNT_ROOM_POSE,
                                action_timeout_sec=30.0,
                            ),
                            num_failures=3,
                        ),
                    ),
                ],
            ),
            py_trees.behaviours.Success("count room view not needed"),
        ],
    )
    seq.add_child(_arm_to_orbbec_look(prelude=room_view))
    seq.add_child(BtNode_Announce(
        "announce scanning", bb_source=None, message="I am counting now."
    ))
    seq.add_child(BtNode_BlackboardSet(
        "mark not recounted", bb_keys.COUNT_RECOUNTED, False,
    ))

    strategies = py_trees.composites.Selector("count strategies", memory=True)

    by_detector = py_trees.composites.Sequence("count/by_detector", memory=True)
    by_detector.add_child(_count_sweep("count"))
    by_detector.add_child(BtNode_CheckCountPositive(
        "detector counted any?", bb_keys.COUNT_VALUE,
    ))
    strategies.add_child(by_detector)

    vlm_fallback = py_trees.composites.Sequence("count/vlm_fallback", memory=True)
    vlm_fallback.add_child(BtNode_Announce(
        "announce vlm fallback", bb_source=None,
        message="My detector could not count them. Let me look at the picture myself.",
    ))
    vlm_fallback.add_child(BtNode_VLMQuery(
        "vlm count",
        bb_detection_src=bb_keys.COUNT_BEST_DETECTION,
        bb_answer_dst=bb_keys.VLM_ANSWER,
        question_template=(
            "How many {value} are visible in this image? Count every one of "
            "them. Answer in one short spoken sentence that starts with the "
            "number."
        ),
        bb_fill_key=bb_keys.COUNT_VLM_SUBJECT,
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
    vlm_fallback.add_child(BtNode_CheckCountPositive(
        "vlm counted any?", bb_keys.COUNT_VALUE,
    ))
    strategies.add_child(vlm_fallback)

    recount = py_trees.composites.Sequence("count/recount", memory=True)
    recount.add_child(BtNode_Announce(
        "announce recount", bb_source=None,
        message="I did not see any, let me look again.",
    ))
    recount.add_child(BtNode_BlackboardSet(
        "mark recounted", bb_keys.COUNT_RECOUNTED, True,
    ))
    recount.add_child(_count_sweep("recount"))
    strategies.add_child(recount)

    seq.add_child(strategies)
    seq.add_child(BtNode_AnnounceFromBB(
        "announce count", bb_keys.COUNT_VALUE, prefix="I counted "
    ))
    seq.add_child(BtNode_SetReportInfo(
        "buffer count", bb_keys.COUNT_VALUE, prefix="I counted ",
    ))
    return seq


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
    orchestrator) goes to gpt-4.1 with the current date/time injected. The
    result is buffered in ``REPORT_INFO`` and must be spoken by a following
    explicit ``announce`` plan step. Questions needing live data the model
    cannot have (e.g. current weather) get an honest "I cannot access that"
    reply.
    """
    seq = py_trees.composites.Sequence("small/llm_fallback", memory=True)
    seq.add_child(BtNode_LLMQuery(
        "llm answer",
        bb_question_key=bb_keys.LLM_QUESTION,
        bb_answer_dst=bb_keys.LLM_ANSWER,
    ))
    seq.add_child(BtNode_SetReportInfo(
        "buffer llm answer", bb_keys.LLM_ANSWER, prefix="Here is the answer. ",
    ))
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
    # Only the count VLM FALLBACK ("vlm count") -- the sweep's refinement
    # queries (unclassified / person attribute) keep their fixed templates,
    # so the vlm-template modification still targets exactly one node.
    if action == "count" and node_type == "BtNode_VLMQuery" and name == "vlm count":
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
