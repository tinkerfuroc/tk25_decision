from __future__ import annotations

"""Doing Laundry — main task deciding tree (RoboCup@Home 2026 §5.4).

Hardware-aware deviations from the rulebook (see plan + laundry.md):
  * Single xArm — no bimanual basket lift (DO_BASKET_TRANSPORT=False).
  * Washer door open requires operator help (DO_REQUEST_WASHER_HELP=True).
    Net per door open: +300 - 40 (env-change penalty) = +260.
  * Single-piece grasp from washer requires operator handover
    (DO_REQUEST_SINGLE_PIECE_VIA_HANDOVER=True). Avoids -100 multi-grab.
    Net per piece: +300 retrieval - 40 handover = +260.
  * Fold action server (`fold_clothing_action`) does not yet exist.
    Tree optimistically calls `BtNode_FoldClothing`; on failure the operator-
    assisted branch fires (-100 minor help on primary, -200 on extras),
    still earning +700 / +200 per fold.

Estimated yield with these constraints: ~+1635 of the 4415 ceiling.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_FoldClothing,
    BtNode_Grasp,
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_GetPointCloud,
    BtNode_ScanFor,
    BtNode_TurnPanTilt,
)
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress

from .config import (
    ARM_POS_FOLD_START,
    ARM_POS_NAVIGATING,
    ARM_POS_PICK_BASKET,
    ARM_POS_PICK_WASHER,
    ARM_POS_PLACING,
    ARM_POS_SCAN,
    ARM_SERVICE_NAME,
    CLOTHING_SCAN_PROMPT,
    DO_FOLD_PICKED_PIECES,
    DO_PICK_FROM_BASKET,
    DO_PICK_FROM_WASHER,
    DO_REQUEST_WASHER_HELP,
    FOLD_ACTION_NAME,
    FOLD_CYCLES,
    GRASP_ACTION_NAME,
    GRASP_RETRY_LIMIT,
    KEY_ARM_FOLD_START,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PICK_BASKET,
    KEY_ARM_PICK_WASHER,
    KEY_ARM_PLACING,
    KEY_ARM_SCAN,
    KEY_DOOR_STATUS,
    KEY_ENV_POINTS,
    KEY_FOLD_COUNT,
    KEY_GRASP_ANNOUNCEMENT,
    KEY_MAX_RUNTIME,
    KEY_OBJECT_LABEL,
    KEY_PHASE_DEADLINE,
    KEY_POINT_BASKET_TOP,
    KEY_POINT_TABLE_FOLD_ZONE,
    KEY_POINT_TABLE_STACK_ZONE,
    KEY_POINT_WASHER_DRUM,
    KEY_POSE_ARENA_ENTRY,
    KEY_POSE_BASKET,
    KEY_POSE_FOLDING_TABLE,
    KEY_POSE_LAUNDRY_AREA,
    KEY_POSE_WASHING_MACHINE,
    KEY_SCORE_TRACE,
    KEY_STACK_COUNT,
    KEY_SUMMARY_MESSAGE,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    MAX_EXTRA_FOLDS,
    MAX_RUNTIME_SEC,
    NAV_RETRY_LIMIT,
    OBJECT_LABEL_CLOTHING,
    POINT_BASKET_TOP,
    POINT_TABLE_FOLD_ZONE,
    POINT_TABLE_STACK_ZONE,
    POINT_WASHER_DRUM,
    POSE_ARENA_ENTRY,
    POSE_BASKET,
    POSE_FOLDING_TABLE,
    POSE_LAUNDRY_AREA,
    POSE_WASHING_MACHINE,
    SCAN_RETRY_LIMIT,
    TARGET_FRAME,
)
from .state_nodes import (
    BtNode_BuildCompletionSummary,
    BtNode_IncrementCounter,
    BtNode_InitTaskState,
    BtNode_RecordCompletion,
    BtNode_TimeoutCutoverChecker,
)


# --------------------------------------------------------------------------- #
# Constant writer
# --------------------------------------------------------------------------- #

def createConstantWriter() -> py_trees.composites.Parallel:
    root = py_trees.composites.Parallel(
        name="Write DoingLaundry constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arena entry pose",     KEY_POSE_ARENA_ENTRY,      POSE_ARENA_ENTRY),
        ("Write laundry area pose",    KEY_POSE_LAUNDRY_AREA,     POSE_LAUNDRY_AREA),
        ("Write washer pose",          KEY_POSE_WASHING_MACHINE,  POSE_WASHING_MACHINE),
        ("Write basket pose",          KEY_POSE_BASKET,           POSE_BASKET),
        ("Write folding table pose",   KEY_POSE_FOLDING_TABLE,    POSE_FOLDING_TABLE),
        ("Write table fold zone pt",   KEY_POINT_TABLE_FOLD_ZONE, POINT_TABLE_FOLD_ZONE),
        ("Write table stack zone pt",  KEY_POINT_TABLE_STACK_ZONE, POINT_TABLE_STACK_ZONE),
        ("Write basket top pt",        KEY_POINT_BASKET_TOP,      POINT_BASKET_TOP),
        ("Write washer drum pt",       KEY_POINT_WASHER_DRUM,     POINT_WASHER_DRUM),
        ("Write arm navigating",       KEY_ARM_NAVIGATING,        ARM_POS_NAVIGATING),
        ("Write arm scan",             KEY_ARM_SCAN,              ARM_POS_SCAN),
        ("Write arm pick basket",      KEY_ARM_PICK_BASKET,       ARM_POS_PICK_BASKET),
        ("Write arm pick washer",      KEY_ARM_PICK_WASHER,       ARM_POS_PICK_WASHER),
        ("Write arm placing",          KEY_ARM_PLACING,           ARM_POS_PLACING),
        ("Write arm fold start",       KEY_ARM_FOLD_START,        ARM_POS_FOLD_START),
        ("Write target frame",         KEY_TARGET_FRAME,          TARGET_FRAME),
        ("Write max runtime",          KEY_MAX_RUNTIME,           MAX_RUNTIME_SEC),
        ("Write clothing object label", KEY_OBJECT_LABEL,         OBJECT_LABEL_CLOTHING),
    ]
    for name, key, value in writes:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name, bb_namespace="", bb_source=None, bb_key=key, object=value,
            )
        )
    return root


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #

def _moveArmRetry(name: str, arm_pose_key: str, *, add_octomap: bool = False, retries: int = 2):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_MoveArmSingle(
            name=name,
            service_name=ARM_SERVICE_NAME,
            arm_pose_bb_key=arm_pose_key,
            add_octomap=add_octomap,
        ),
        num_failures=retries,
    )


def _gripperOpenSafe(name: str = "Open gripper"):
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {name}",
        child=BtNode_GripperAction(name=name, open_gripper=True),
    )


def _visionGraspClothing(label_suffix: str) -> py_trees.composites.Sequence:
    """Scan + vision grasp with object_label='clothing'.

    Used as the *primary* attempt for washer/basket pickup before falling
    through to the operator-handover branch. Uses the canonical generalist
    detection service (`object_detection_yolo` is the default for ScanFor;
    callers can retarget via the node's `service_name` arg if needed).
    """
    seq = py_trees.composites.Sequence(name=f"Vision grasp ({label_suffix})", memory=True)
    seq.add_child(_moveArmRetry(f"Arm to scan ({label_suffix})", KEY_ARM_SCAN, add_octomap=True))
    seq.add_child(BtNode_TurnPanTilt(name=f"Tilt down ({label_suffix})", x=0.0, y=20.0))
    seq.add_child(
        py_trees.decorators.Retry(
            name=f"Retry scan clothing ({label_suffix})",
            child=BtNode_ScanFor(
                name=f"Scan for clothing ({label_suffix})",
                bb_source=None,
                bb_key=KEY_VISION_RESULT,
                object=CLOTHING_SCAN_PROMPT,
                transform_to_map=False,
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    seq.add_child(
        py_trees.decorators.Retry(
            name=f"Retry grasp clothing ({label_suffix})",
            child=BtNode_Grasp(
                name=f"Grasp clothing ({label_suffix})",
                bb_source=KEY_VISION_RESULT,
                bb_key_object_label=KEY_OBJECT_LABEL,
                action_name=GRASP_ACTION_NAME,
            ),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    return seq


def _requestOperatorHelp(
    prompt: str,
    *,
    record_label: str,
    record_points: int,
    name_suffix: str = "",
) -> py_trees.composites.Sequence:
    """Announce + wait-for-keypress + record env-change/help penalty."""
    seq = py_trees.composites.Sequence(name=f"Request help: {name_suffix or record_label}", memory=True)
    seq.add_child(BtNode_Announce(name=f"TTS: {record_label}", bb_source=None, message=prompt))
    seq.add_child(BtNode_WaitKeyboardPress(name=f"Wait operator: {record_label}", key="s"))
    seq.add_child(
        BtNode_RecordCompletion(
            name=f"Record help: {record_label}",
            score_trace_key=KEY_SCORE_TRACE,
            action_label=record_label,
            points=record_points,
            success=True,
        )
    )
    return seq


# --------------------------------------------------------------------------- #
# Phase: enter arena
# --------------------------------------------------------------------------- #

def createEnterArena() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Enter arena", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(name="Wait for arena door open", bb_door_state_key=KEY_DOOR_STATUS),
            num_failures=999,
        )
    )
    parallel = py_trees.composites.Parallel(
        name="Enter parallel", policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel.add_child(
        BtNode_Announce(name="Announce entering laundry area", bb_source=None,
                        message="Entering laundry area.")
    )
    parallel.add_child(BtNode_TurnPanTilt(name="Tilt head forward", x=0.0, y=20.0))
    parallel.add_child(
        py_trees.decorators.Retry(
            name="Retry goto arena entry",
            child=BtNode_GotoAction(name="Go to arena entry", key=KEY_POSE_ARENA_ENTRY),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(parallel)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto laundry area",
            child=BtNode_GotoAction(name="Go to laundry area", key=KEY_POSE_LAUNDRY_AREA),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(
        BtNode_RecordCompletion(
            name="Record nav to laundry",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="nav_to_laundry",
            points=15,
            success=True,
        )
    )
    return root


# --------------------------------------------------------------------------- #
# Phase: washer retrieval (operator-assisted; gated by DO_PICK_FROM_WASHER)
# --------------------------------------------------------------------------- #

def createWasherRetrievalPhase() -> py_trees.behaviour.Behaviour:
    if not DO_PICK_FROM_WASHER:
        return py_trees.behaviours.Success(name="Washer retrieval skipped (flag off)")

    root = py_trees.composites.Sequence(name="Washer retrieval", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime cutover (washer)", phase_deadline_key=KEY_PHASE_DEADLINE,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto washer",
            child=BtNode_GotoAction(name="Go to washing machine", key=KEY_POSE_WASHING_MACHINE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )

    if DO_REQUEST_WASHER_HELP:
        door = py_trees.composites.Selector(name="Open washer door", memory=False)
        ok = py_trees.composites.Sequence(name="Door opened with help", memory=True)
        ok.add_child(
            _requestOperatorHelp(
                prompt="Please open the washing machine door, then press s.",
                record_label="washer_door_help",
                record_points=-40,
            )
        )
        ok.add_child(
            BtNode_RecordCompletion(
                name="Record washer door bonus",
                score_trace_key=KEY_SCORE_TRACE,
                action_label="washer_door_opened",
                points=300,
                success=True,
            )
        )
        door.add_child(ok)
        door.add_child(
            BtNode_RecordCompletion(
                name="Record washer door miss",
                score_trace_key=KEY_SCORE_TRACE,
                action_label="washer_door_opened",
                points=0,
                success=False,
            )
        )
        root.add_child(door)

    pick = py_trees.composites.Selector(name="Pick clothing from washer", memory=False)

    # Primary: real vision grasp. Object label 'clothing' is written to the
    # blackboard by the constant writer and threaded into BtNode_Grasp.
    vision_pick = py_trees.composites.Sequence(name="Vision pick (washer)", memory=True)
    vision_pick.add_child(_visionGraspClothing("washer"))
    vision_pick.add_child(_moveArmRetry("Arm to nav (post-vision pick)", KEY_ARM_NAVIGATING))
    vision_pick.add_child(
        BtNode_RecordCompletion(
            name="Record washer vision pick bonus",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="washer_pick",
            points=300,
            success=True,
        )
    )
    pick.add_child(vision_pick)

    # Fallback: operator handover (existing behaviour).
    pick_ok = py_trees.composites.Sequence(name="Handover pick", memory=True)
    pick_ok.add_child(_moveArmRetry("Arm to washer pick pose", KEY_ARM_PICK_WASHER))
    pick_ok.add_child(_gripperOpenSafe("Open gripper for handover"))
    pick_ok.add_child(
        _requestOperatorHelp(
            prompt="Please hand me one piece of clothing into my gripper, then press s.",
            record_label="washer_handover",
            record_points=-40,
        )
    )
    pick_ok.add_child(
        py_trees.decorators.Retry(
            name="Retry close gripper",
            child=BtNode_GripperAction(name="Close gripper on cloth", open_gripper=False),
            num_failures=2,
        )
    )
    pick_ok.add_child(_moveArmRetry("Arm to nav (post-handover)", KEY_ARM_NAVIGATING))
    pick_ok.add_child(
        BtNode_RecordCompletion(
            name="Record washer pick bonus",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="washer_pick",
            points=300,
            success=True,
        )
    )
    pick.add_child(pick_ok)
    pick_fail = py_trees.composites.Sequence(name="Washer pick failed", memory=True)
    pick_fail.add_child(
        BtNode_Announce(name="Announce washer skip", bb_source=None,
                        message="Could not retrieve from washer; continuing.")
    )
    pick_fail.add_child(_gripperOpenSafe("Release any partial grasp"))
    pick_fail.add_child(
        BtNode_RecordCompletion(
            name="Record washer pick miss",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="washer_pick",
            points=0,
            success=False,
        )
    )
    pick.add_child(pick_fail)
    root.add_child(pick)

    # Carry to folding table and place at the stack zone (next to the fold target).
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto folding table",
            child=BtNode_GotoAction(name="Go to folding table", key=KEY_POSE_FOLDING_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(_moveArmRetry("Arm to placing (washer carry)", KEY_ARM_PLACING))
    # No vision grasp pose from a handover; drop via gripper-open instead of vision-Place.
    root.add_child(_gripperOpenSafe("Drop cloth on stack zone"))
    return root


# --------------------------------------------------------------------------- #
# Phase: basket fallback (default off)
# --------------------------------------------------------------------------- #

def createBasketPickPhase() -> py_trees.behaviour.Behaviour:
    if not DO_PICK_FROM_BASKET:
        return py_trees.behaviours.Success(name="Basket pick skipped (flag off)")

    root = py_trees.composites.Sequence(name="Basket pick", memory=True)
    root.add_child(BtNode_TimeoutCutoverChecker(name="Check cutover (basket)",
                                                phase_deadline_key=KEY_PHASE_DEADLINE))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto basket",
            child=BtNode_GotoAction(name="Go to basket", key=KEY_POSE_BASKET),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    pick = py_trees.composites.Selector(name="Pick clothing from basket", memory=False)

    vision_pick = py_trees.composites.Sequence(name="Vision pick (basket)", memory=True)
    vision_pick.add_child(_visionGraspClothing("basket"))
    vision_pick.add_child(_moveArmRetry("Arm to nav (post-vision basket)", KEY_ARM_NAVIGATING))
    vision_pick.add_child(
        BtNode_RecordCompletion(
            name="Record basket vision pick bonus",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="basket_pick",
            points=100,
            success=True,
        )
    )
    pick.add_child(vision_pick)

    handover = py_trees.composites.Sequence(name="Basket handover pick", memory=True)
    handover.add_child(_moveArmRetry("Arm to basket pick pose", KEY_ARM_PICK_BASKET))
    handover.add_child(_gripperOpenSafe("Open for basket handover"))
    handover.add_child(
        _requestOperatorHelp(
            prompt="Please hand me one piece of clothing from the basket, then press s.",
            record_label="basket_handover",
            record_points=-40,
        )
    )
    handover.add_child(
        py_trees.decorators.Retry(
            name="Retry close gripper (basket)",
            child=BtNode_GripperAction(name="Close gripper on basket cloth", open_gripper=False),
            num_failures=2,
        )
    )
    handover.add_child(
        BtNode_RecordCompletion(
            name="Record basket pick bonus",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="basket_pick",
            points=100,
            success=True,
        )
    )
    pick.add_child(handover)
    root.add_child(pick)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto folding table (basket)",
            child=BtNode_GotoAction(name="Go to folding table", key=KEY_POSE_FOLDING_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(_moveArmRetry("Arm to placing (basket carry)", KEY_ARM_PLACING))
    root.add_child(_gripperOpenSafe("Drop basket cloth on stack zone"))
    return root


# --------------------------------------------------------------------------- #
# Phase: primary fold (the cloth pre-placed on the table by the referee)
# --------------------------------------------------------------------------- #

def createPrimaryFoldPhase() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Primary fold (pre-placed table cloth)", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(name="Check cutover (primary fold)",
                                     phase_deadline_key=KEY_PHASE_DEADLINE)
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto folding table (primary)",
            child=BtNode_GotoAction(name="Go to folding table", key=KEY_POSE_FOLDING_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(_moveArmRetry("Arm to fold start", KEY_ARM_FOLD_START))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry env point cloud (primary fold)",
            child=BtNode_GetPointCloud(
                name="Get env point cloud (primary fold)",
                bb_point_cloud_key=KEY_ENV_POINTS,
            ),
            num_failures=2,
        )
    )
    root.add_child(BtNode_Announce(name="Announce primary fold", bb_source=None,
                                   message="Folding the cloth on the table."))

    selector = py_trees.composites.Selector(name="Fold or operator help (primary)", memory=False)

    # Branch 1: try the fold action server.
    direct = py_trees.composites.Sequence(name="Try fold action (primary)", memory=True)
    direct.add_child(
        BtNode_FoldClothing(
            name="Fold primary cloth",
            bb_key_target_point=KEY_POINT_TABLE_FOLD_ZONE,
            bb_key_object_label=KEY_OBJECT_LABEL,
            bb_key_env_points=KEY_ENV_POINTS,
            fold_cycles=FOLD_CYCLES,
            action_name=FOLD_ACTION_NAME,
        )
    )
    direct.add_child(BtNode_IncrementCounter(name="Inc fold count (primary)",
                                             counter_key=KEY_FOLD_COUNT))
    direct.add_child(
        BtNode_RecordCompletion(
            name="Record primary fold",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="primary_fold",
            points=800,
            success=True,
        )
    )
    selector.add_child(direct)

    # Branch 2: minor operator help (-100), then retry fold.
    helped = py_trees.composites.Sequence(name="Minor-help fold (primary)", memory=True)
    helped.add_child(
        _requestOperatorHelp(
            prompt="Please flatten or smooth the cloth, then press s.",
            record_label="primary_fold_help_minor",
            record_points=-100,
        )
    )
    helped.add_child(
        BtNode_FoldClothing(
            name="Fold primary cloth (post-help)",
            bb_key_target_point=KEY_POINT_TABLE_FOLD_ZONE,
            bb_key_object_label=KEY_OBJECT_LABEL,
            bb_key_env_points=KEY_ENV_POINTS,
            fold_cycles=FOLD_CYCLES,
            action_name=FOLD_ACTION_NAME,
        )
    )
    helped.add_child(BtNode_IncrementCounter(name="Inc fold count (primary, helped)",
                                             counter_key=KEY_FOLD_COUNT))
    helped.add_child(
        BtNode_RecordCompletion(
            name="Record primary fold (helped)",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="primary_fold_minor_help",
            points=800,
            success=True,
        )
    )
    selector.add_child(helped)

    # Branch 3: skip.
    skip = py_trees.composites.Sequence(name="Primary fold skip", memory=True)
    skip.add_child(BtNode_Announce(name="Announce primary fold skip", bb_source=None,
                                   message="Could not fold the cloth."))
    skip.add_child(
        BtNode_RecordCompletion(
            name="Record primary fold miss",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="primary_fold",
            points=0,
            success=False,
        )
    )
    selector.add_child(skip)

    root.add_child(selector)
    return root


# --------------------------------------------------------------------------- #
# Phase: extra folds (the picked piece + any subsequent transports)
# --------------------------------------------------------------------------- #

def _processOneExtraFold() -> py_trees.composites.Selector:
    attempt = py_trees.composites.Selector(name="One extra fold", memory=False)

    # Branch 1: try fold + (optionally) stack.
    direct = py_trees.composites.Sequence(name="Direct extra fold", memory=True)
    direct.add_child(BtNode_TimeoutCutoverChecker(name="Check cutover (extra)",
                                                  phase_deadline_key=KEY_PHASE_DEADLINE))
    direct.add_child(_moveArmRetry("Arm to fold start (extra)", KEY_ARM_FOLD_START))
    direct.add_child(
        py_trees.decorators.Retry(
            name="Retry env point cloud (extra fold)",
            child=BtNode_GetPointCloud(
                name="Get env point cloud (extra fold)",
                bb_point_cloud_key=KEY_ENV_POINTS,
            ),
            num_failures=2,
        )
    )

    fold_or_help = py_trees.composites.Selector(name="Fold or help-fold (extra)", memory=False)
    fold_direct = py_trees.composites.Sequence(name="Try fold action (extra)", memory=True)
    fold_direct.add_child(
        BtNode_FoldClothing(
            name="Fold extra cloth",
            bb_key_target_point=KEY_POINT_TABLE_STACK_ZONE,
            bb_key_object_label=KEY_OBJECT_LABEL,
            bb_key_env_points=KEY_ENV_POINTS,
            fold_cycles=FOLD_CYCLES,
            action_name=FOLD_ACTION_NAME,
        )
    )
    fold_direct.add_child(BtNode_IncrementCounter(name="Inc fold count (extra)",
                                                  counter_key=KEY_FOLD_COUNT))
    fold_direct.add_child(
        BtNode_RecordCompletion(
            name="Record extra fold",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="extra_fold",
            points=400,
            success=True,
        )
    )
    fold_or_help.add_child(fold_direct)

    fold_helped = py_trees.composites.Sequence(name="Minor-help fold (extra)", memory=True)
    fold_helped.add_child(
        _requestOperatorHelp(
            prompt="Please flatten this cloth, then press s.",
            record_label="extra_fold_help_minor",
            record_points=-200,
        )
    )
    fold_helped.add_child(
        BtNode_FoldClothing(
            name="Fold extra cloth (post-help)",
            bb_key_target_point=KEY_POINT_TABLE_STACK_ZONE,
            bb_key_object_label=KEY_OBJECT_LABEL,
            bb_key_env_points=KEY_ENV_POINTS,
            fold_cycles=FOLD_CYCLES,
            action_name=FOLD_ACTION_NAME,
        )
    )
    fold_helped.add_child(BtNode_IncrementCounter(name="Inc fold count (extra, helped)",
                                                  counter_key=KEY_FOLD_COUNT))
    fold_helped.add_child(
        BtNode_RecordCompletion(
            name="Record extra fold (helped)",
            score_trace_key=KEY_SCORE_TRACE,
            action_label="extra_fold_minor_help",
            points=400,
            success=True,
        )
    )
    fold_or_help.add_child(fold_helped)
    direct.add_child(fold_or_help)

    attempt.add_child(direct)

    # Branch 2: skip — let the loop decorator move on.
    attempt.add_child(py_trees.behaviours.Success(name="Skip extra fold"))
    return attempt


def createExtraFoldPhase() -> py_trees.behaviour.Behaviour:
    if not DO_FOLD_PICKED_PIECES or MAX_EXTRA_FOLDS <= 0:
        return py_trees.behaviours.Success(name="Extra folds skipped (flag off)")
    root = py_trees.composites.Sequence(name="Extra folds", memory=True)
    root.add_child(
        py_trees.decorators.Repeat(
            name=f"Repeat extra folds x{MAX_EXTRA_FOLDS}",
            child=_processOneExtraFold(),
            num_success=MAX_EXTRA_FOLDS,
        )
    )
    return root


# --------------------------------------------------------------------------- #
# Top-level
# --------------------------------------------------------------------------- #

def createDoingLaundryTask() -> py_trees.composites.Selector:
    mission = py_trees.composites.Sequence(name="DoingLaundry mission", memory=True)
    mission.add_child(createConstantWriter())
    mission.add_child(
        BtNode_InitTaskState(
            name="Initialize task state",
            score_trace_key=KEY_SCORE_TRACE,
            phase_deadline_key=KEY_PHASE_DEADLINE,
            max_runtime_key=KEY_MAX_RUNTIME,
            fold_count_key=KEY_FOLD_COUNT,
            stack_count_key=KEY_STACK_COUNT,
        )
    )
    mission.add_child(_moveArmRetry("Arm to nav (startup)", KEY_ARM_NAVIGATING))
    mission.add_child(
        BtNode_Announce(name="Announce start", bb_source=None,
                        message="Starting doing laundry.")
    )
    mission.add_child(createEnterArena())
    mission.add_child(py_trees.decorators.FailureIsSuccess(
        name="Washer phase best-effort", child=createWasherRetrievalPhase(),
    ))
    if DO_PICK_FROM_BASKET:
        mission.add_child(py_trees.decorators.FailureIsSuccess(
            name="Basket phase best-effort", child=createBasketPickPhase(),
        ))
    mission.add_child(py_trees.decorators.FailureIsSuccess(
        name="Primary fold best-effort", child=createPrimaryFoldPhase(),
    ))
    mission.add_child(py_trees.decorators.FailureIsSuccess(
        name="Extra folds best-effort", child=createExtraFoldPhase(),
    ))
    mission.add_child(
        BtNode_BuildCompletionSummary(
            name="Build completion summary",
            score_trace_key=KEY_SCORE_TRACE,
            fold_count_key=KEY_FOLD_COUNT,
            stack_count_key=KEY_STACK_COUNT,
            summary_key=KEY_SUMMARY_MESSAGE,
        )
    )
    mission.add_child(
        BtNode_Announce(name="Announce summary", bb_source=KEY_SUMMARY_MESSAGE, message=None)
    )

    root = py_trees.composites.Selector(name="DoingLaundry root timeout guard", memory=False)
    root.add_child(
        py_trees.decorators.Timeout(
            name="Global runtime timeout",
            child=mission,
            duration=MAX_RUNTIME_SEC,
        )
    )
    root.add_child(
        BtNode_Announce(name="Announce timeout", bb_source=None,
                        message="Time is up. Ending laundry task.")
    )
    return root
