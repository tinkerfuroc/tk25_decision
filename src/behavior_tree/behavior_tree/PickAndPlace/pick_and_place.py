from __future__ import annotations

"""Pick and Place — main mission tree (RoboCup@Home 2026 §5.2).

Mirrors the Restaurant / HRI single-file mission idiom: phase factories at
module top-level, mission assembly is a flat sequence of `create*Phase()`
calls, blackboard initialisation lives in `createConstantWriter()`.

Hardware-aware deviations from the rulebook (see `RULEBOOK_PLAN.md`):
  * Cutlery + tableware go to a wash-staging surface, not the dishwasher rack.
  * Floor pickup is skipped.

Vision: every detection call uses the tk26 generalist service
(`/object_detection_generalist`,
`tinker_vision_msgs_26/srv/ObjectDetectionGeneralist`).
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
    BtNode_Place,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_GetPointCloud,
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
)

from .config import (
    ARM_POS_DROP,
    ARM_POS_NAVIGATING,
    ARM_POS_PLACING,
    ARM_POS_PLACING_CABINET,
    ARM_POS_SCAN,
    ARM_POS_SCAN_CABINET,
    ARM_POS_TRASH,
    ARM_SERVICE_NAME,
    DESIGNATED_TRASH_LABELS,
    GRASP_ACTION_NAME,
    GRASP_RETRY_LIMIT,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_ACTIVE_PROMPT,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POINT,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ARM_DROP,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PLACING,
    KEY_ARM_SCAN,
    KEY_ARM_SCAN_CABINET,
    KEY_ARM_PLACING_CABINET,
    KEY_ARM_TRASH,
    KEY_BREAKFAST_QUEUE,
    KEY_DOOR_STATUS,
    KEY_ENV_POINTS,
    KEY_GRASP_ANNOUNCEMENT,
    KEY_GRASP_POSE,
    KEY_INVENTORY_TABLE,
    KEY_MAX_RUNTIME,
    KEY_OBJECT_LABEL,
    KEY_OBJ_SEG,
    KEY_PHASE_DEADLINE,
    KEY_PLACE_REASON,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    KEY_POINT_CABINET_DEFAULT,
    KEY_POINT_KITCHEN_SURFACE,
    KEY_POINT_SHELF_LEFT,
    KEY_POINT_SHELF_RIGHT,
    KEY_POINT_WASH_STAGING,
    KEY_POSE_CABINET,
    KEY_POSE_KITCHEN_ENTRY,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_POSE_TRASH_BIN,
    KEY_POSE_WASH_STAGING,
    KEY_SCORE_TRACE,
    KEY_SUMMARY_MESSAGE,
    KEY_TABLE_IMG,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    KEY_WORK_QUEUE,
    MAX_RUNTIME_SEC,
    NAV_RETRY_LIMIT,
    N_LAYERS,
    PLACE_ACTION_NAME,
    POINT_BREAKFAST_BOWL,
    POINT_BREAKFAST_CEREAL,
    POINT_BREAKFAST_MILK,
    POINT_BREAKFAST_SPOON,
    POINT_CABINET_DEFAULT,
    POINT_KITCHEN_SURFACE,
    POINT_SHELF_LEFT,
    POINT_SHELF_RIGHT,
    POINT_WASH_STAGING,
    POSE_CABINET,
    POSE_KITCHEN_ENTRY,
    POSE_KITCHEN_SHELF,
    POSE_TABLE,
    POSE_TRASH_BIN,
    POSE_WASH_STAGING,
    SCAN_RETRY_LIMIT,
    TABLE_SCAN_PROMPT,
    TARGET_FRAME,
    WASH_STAGING_LABELS,
)
from .custom_nodes import (
    BtNode_CategorizeGrocery,
    BtNode_FindObjTable,
    BtNode_GraspWithPose,
)
from .state_nodes import (
    BtNode_BuildBreakfastQueue,
    BtNode_BuildCompletionSummary,
    BtNode_BuildTableInventory,
    BtNode_ClassifyTableItems,
    BtNode_HasPendingWork,
    BtNode_InitTaskState,
    BtNode_IsActiveClass,
    BtNode_RecordCompletion,
    BtNode_SelectNextBreakfastItem,
    BtNode_SelectNextItem,
    BtNode_TimeoutCutoverChecker,
)

MAX_CLEANUP_ITERATIONS = 12  # rulebook caps Pick at 12x and Place at 12x
BREAKFAST_ITEM_COUNT = 4


# --------------------------------------------------------------------------- #
# Constant writer
# --------------------------------------------------------------------------- #


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write all PoseStamped / PointStamped / arm-joint constants to the blackboard."""
    root = py_trees.composites.Parallel(
        name="Write PickAndPlace constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write kitchen entry pose", KEY_POSE_KITCHEN_ENTRY, POSE_KITCHEN_ENTRY),
        ("Write table pose", KEY_POSE_TABLE, POSE_TABLE),
        ("Write wash-staging pose", KEY_POSE_WASH_STAGING, POSE_WASH_STAGING),
        ("Write trash bin pose", KEY_POSE_TRASH_BIN, POSE_TRASH_BIN),
        ("Write cabinet pose", KEY_POSE_CABINET, POSE_CABINET),
        ("Write kitchen shelf pose", KEY_POSE_KITCHEN_SHELF, POSE_KITCHEN_SHELF),
        ("Write wash-staging point", KEY_POINT_WASH_STAGING, POINT_WASH_STAGING),
        ("Write cabinet default pt", KEY_POINT_CABINET_DEFAULT, POINT_CABINET_DEFAULT),
        ("Write kitchen surface pt", KEY_POINT_KITCHEN_SURFACE, POINT_KITCHEN_SURFACE),
        ("Write breakfast bowl pt", KEY_POINT_BREAKFAST_BOWL, POINT_BREAKFAST_BOWL),
        ("Write breakfast spoon pt", KEY_POINT_BREAKFAST_SPOON, POINT_BREAKFAST_SPOON),
        (
            "Write breakfast cereal pt",
            KEY_POINT_BREAKFAST_CEREAL,
            POINT_BREAKFAST_CEREAL,
        ),
        ("Write breakfast milk pt", KEY_POINT_BREAKFAST_MILK, POINT_BREAKFAST_MILK),
        ("Write shelf left point", KEY_POINT_SHELF_LEFT, POINT_SHELF_LEFT),
        ("Write shelf right point", KEY_POINT_SHELF_RIGHT, POINT_SHELF_RIGHT),
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm scan", KEY_ARM_SCAN, ARM_POS_SCAN),
        ("Write arm scan cabinet", KEY_ARM_SCAN_CABINET, ARM_POS_SCAN_CABINET),
        ("Write arm placing", KEY_ARM_PLACING, ARM_POS_PLACING),
        ("Write arm placing cabinet", KEY_ARM_PLACING_CABINET, ARM_POS_PLACING_CABINET),
        ("Write arm drop", KEY_ARM_DROP, ARM_POS_DROP),
        ("Write arm trash", KEY_ARM_TRASH, ARM_POS_TRASH),
        ("Write target frame", KEY_TARGET_FRAME, TARGET_FRAME),
        ("Write max runtime", KEY_MAX_RUNTIME, MAX_RUNTIME_SEC),
    ]
    for name, key, value in writes:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name,
                bb_namespace="",
                bb_source=None,
                bb_key=key,
                object=value,
            )
        )
    return root


# --------------------------------------------------------------------------- #
# Shared helpers
# --------------------------------------------------------------------------- #


def _moveArmRetry(
    name: str,
    arm_pose_key: str,
    *,
    add_octomap: bool = False,
    retries: int = 2,
) -> py_trees.decorators.Retry:
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


def _gripperOpenSafe(name: str) -> py_trees.decorators.FailureIsSuccess:
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {name}",
        child=BtNode_GripperAction(name=name, open_gripper=True),
    )


def _findAndGrasp() -> py_trees.composites.Sequence:
    """FindObjTable (generalist) -> announce -> GraspWithPose."""
    root = py_trees.composites.Sequence(name="Find and grasp", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry FindObjTable",
            child=BtNode_FindObjTable(
                name="Find active object",
                bb_key_prompt=KEY_ACTIVE_PROMPT,
                bb_key_image=KEY_TABLE_IMG,
                bb_key_segment=KEY_OBJ_SEG,
                bb_key_result=KEY_VISION_RESULT,
                bb_key_announcement=KEY_GRASP_ANNOUNCEMENT,
                bb_key_object_label=KEY_OBJECT_LABEL,
                use_vlm_sam_fallback=True,
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    grasp_sequence = py_trees.composites.Sequence(
        name="Announce then grasp", memory=True
    )
    grasp_sequence.add_child(
        BtNode_Announce(name="Announce grasping", bb_source=KEY_GRASP_ANNOUNCEMENT)
    )
    grasp_sequence.add_child(
        py_trees.decorators.Retry(
            name="Retry grasp",
            child=BtNode_GraspWithPose(
                name="Grasp active object",
                bb_key_vision_res=KEY_VISION_RESULT,
                bb_key_pose=KEY_GRASP_POSE,
                action_name=GRASP_ACTION_NAME,
                bb_key_object_label=KEY_OBJECT_LABEL,
            ),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    root.add_child(grasp_sequence)
    return root


# --------------------------------------------------------------------------- #
# Phase: enter kitchen
# --------------------------------------------------------------------------- #


def createEnterKitchen() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Enter kitchen", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(
                name="Wait for door open",
                bb_door_state_key=KEY_DOOR_STATUS,
            ),
            num_failures=999,
        )
    )
    parallel = py_trees.composites.Parallel(
        name="Enter parallel",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel.add_child(
        BtNode_Announce(
            name="Announce entering kitchen",
            bb_source=None,
            message="Entering kitchen.",
        )
    )
    parallel.add_child(BtNode_TurnPanTilt(name="Tilt head forward", x=0.0, y=20.0))
    # parallel.add_child(
    #     py_trees.decorators.Retry(
    #         name="Retry goto kitchen entry",
    #         child=BtNode_GotoAction(
    #             name="Go to kitchen entry", key=KEY_POSE_KITCHEN_ENTRY,
    #         ),
    #         num_failures=NAV_RETRY_LIMIT,
    #     )
    # )
    root.add_child(parallel)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto table",
            child=BtNode_GotoAction(name="Go to dining table", key=KEY_POSE_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    return root


# --------------------------------------------------------------------------- #
# Phase: scan + classify table
# --------------------------------------------------------------------------- #


def createTableScanPhase() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Scan + classify table", memory=True)
    # root.add_child(BtNode_TurnPanTilt(name="Tilt head down", x=0.0, y=25.0))
    root.add_child(_moveArmRetry("Arm to scan", KEY_ARM_SCAN, add_octomap=False))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry table scan",
            child=BtNode_ScanForGeneralist(
                name="Scan dining table",
                bb_source=None,
                bb_key=KEY_VISION_RESULT,
                object=TABLE_SCAN_PROMPT,
                transform_to_map=True,
                use_vlm_sam_fallback=True,
                use_orbbec=False,
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    root.add_child(
        BtNode_BuildTableInventory(
            name="Build table inventory",
            vision_result_key=KEY_VISION_RESULT,
            inventory_key=KEY_INVENTORY_TABLE,
        )
    )
    root.add_child(
        BtNode_ClassifyTableItems(
            name="Classify table items",
            inventory_key=KEY_INVENTORY_TABLE,
            work_queue_key=KEY_WORK_QUEUE,
            designated_trash_labels=DESIGNATED_TRASH_LABELS,
            wash_staging_labels=WASH_STAGING_LABELS,
        )
    )
    return root


# --------------------------------------------------------------------------- #
# Phase: cleanup loop
# --------------------------------------------------------------------------- #


def _washStagingBranch() -> py_trees.composites.Sequence:
    """Cutlery + tableware -> safe surface near the dishwasher (NOT into the rack)."""
    root = py_trees.composites.Sequence(name="Wash-staging branch", memory=True)
    root.add_child(
        BtNode_IsActiveClass(
            name="Class is wash_staging?",
            expected="wash_staging",
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto wash-staging",
            child=BtNode_GotoAction(
                name="Go to wash staging", key=KEY_POSE_WASH_STAGING
            ),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(_moveArmRetry("Arm to placing", KEY_ARM_PLACING))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry env point cloud",
            child=BtNode_GetPointCloud(
                name="Get env point cloud",
                bb_point_cloud_key=KEY_ENV_POINTS,
            ),
            num_failures=2,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry place on staging",
            child=BtNode_Place(
                name="Place on wash staging",
                bb_key_point=KEY_POINT_WASH_STAGING,
                bb_key_pose=KEY_GRASP_POSE,
                bb_key_env_points=KEY_ENV_POINTS,
                action_name=PLACE_ACTION_NAME,
            ),
            num_failures=2,
        )
    )
    root.add_child(_gripperOpenSafe("Open gripper"))
    root.add_child(
        BtNode_Announce(
            name="Announce wash staged",
            bb_source=None,
            message="Placed on wash staging.",
        )
    )
    return root


def _trashBranch() -> py_trees.composites.Sequence:
    """Trash -> drop in trash bin (rulebook §3 allows drop only into a designated bin)."""
    root = py_trees.composites.Sequence(name="Trash branch", memory=True)
    root.add_child(
        BtNode_IsActiveClass(
            name="Class is trash?",
            expected="trash",
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto trash bin",
            child=BtNode_GotoAction(name="Go to trash bin", key=KEY_POSE_TRASH_BIN),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    # root.add_child(_moveArmRetry("Arm to drop (approach)", KEY_ARM_DROP))
    root.add_child(_moveArmRetry("Arm to trash (over bin)", KEY_ARM_TRASH))
    root.add_child(_gripperOpenSafe("Release into trash"))
    root.add_child(
        BtNode_Announce(
            name="Announce dropped trash",
            bb_source=None,
            message="Dropped in trash.",
        )
    )
    return root


def _cabinetBranch() -> py_trees.composites.Sequence:
    """Other objects -> cabinet shelf grouped by similarity (Categorize action)."""
    root = py_trees.composites.Sequence(name="Cabinet branch", memory=True)
    root.add_child(
        BtNode_IsActiveClass(
            name="Class is cabinet?",
            expected="cabinet",
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto cabinet",
            child=BtNode_GotoAction(name="Go to cabinet", key=KEY_POSE_CABINET),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(
        _moveArmRetry("Arm to scan (cabinet)", KEY_ARM_SCAN_CABINET, add_octomap=False)
    )
    root.add_child(BtNode_TurnPanTilt(name="Tilt head down (cabinet)", x=0.0, y=25.0))
    categorize_parallel = py_trees.composites.Parallel(
        name="Announce + categorize",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    categorize_parallel.add_child(
        BtNode_Announce(
            name="Announce categorizing",
            bb_source=None,
            message="Categorizing object for cabinet shelf.",
        )
    )
    categorize_parallel.add_child(
        BtNode_CategorizeGrocery(
            name="Categorize for cabinet",
            n_layers=N_LAYERS,
            bb_key_prompt=KEY_ACTIVE_PROMPT,
            bb_key_image=KEY_TABLE_IMG,
            bb_key_segment=KEY_OBJ_SEG,
            bb_target_frame=KEY_TARGET_FRAME,
            bb_key_result_point=KEY_POINT_CABINET_DEFAULT,
            bb_key_env_points=KEY_ENV_POINTS,
            bb_key_reason=KEY_PLACE_REASON,
            bb_key_shelf_left=KEY_POINT_SHELF_LEFT,
            bb_key_shelf_right=KEY_POINT_SHELF_RIGHT,
        )
    )
    root.add_child(categorize_parallel)
    root.add_child(
        _moveArmRetry(
            "Arm to placing (cabinet)", KEY_ARM_PLACING_CABINET, add_octomap=False
        )
    )
    root.add_child(
        BtNode_Announce(name="Announce shelf reason", bb_source=KEY_PLACE_REASON)
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry place on shelf",
            child=BtNode_Place(
                name="Place on cabinet shelf",
                bb_key_point=KEY_POINT_CABINET_DEFAULT,
                bb_key_pose=KEY_GRASP_POSE,
                bb_key_env_points=KEY_ENV_POINTS,
                action_name=PLACE_ACTION_NAME,
            ),
            num_failures=2,
        )
    )
    root.add_child(_gripperOpenSafe("Open gripper after place"))
    return root


def _serveToDestination() -> py_trees.composites.Selector:
    """Three-branch destination selector -- exactly one branch matches per item."""
    selector = py_trees.composites.Selector(name="Destination policy", memory=False)
    selector.add_child(_washStagingBranch())
    selector.add_child(_trashBranch())
    selector.add_child(_cabinetBranch())
    return selector


def _processOneCleanupItem() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Process one cleanup item", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime cutover",
            phase_deadline_key=KEY_PHASE_DEADLINE,
        )
    )
    root.add_child(
        BtNode_HasPendingWork(
            name="Has pending cleanup item", work_queue_key=KEY_WORK_QUEUE
        )
    )
    root.add_child(
        BtNode_SelectNextItem(
            name="Select next cleanup item",
            work_queue_key=KEY_WORK_QUEUE,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            active_prompt_key=KEY_ACTIVE_PROMPT,
            active_source_pose_key=KEY_ACTIVE_SOURCE_POSE,
            active_target_pose_key=KEY_ACTIVE_TARGET_POSE,
            active_target_point_key=KEY_ACTIVE_TARGET_POINT,
            pose_table_key=KEY_POSE_TABLE,
            pose_wash_staging_key=KEY_POSE_WASH_STAGING,
            pose_trash_bin_key=KEY_POSE_TRASH_BIN,
            pose_cabinet_key=KEY_POSE_CABINET,
            point_wash_staging_key=KEY_POINT_WASH_STAGING,
            point_cabinet_default_key=KEY_POINT_CABINET_DEFAULT,
        )
    )

    attempt = py_trees.composites.Selector(name="Attempt and continue", memory=False)

    success = py_trees.composites.Sequence(name="Cleanup success path", memory=True)
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry goto source",
            child=BtNode_GotoAction(
                name="Go to source area", key=KEY_ACTIVE_SOURCE_POSE
            ),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success.add_child(
        _moveArmRetry("Arm to scan (cleanup)", KEY_ARM_SCAN, add_octomap=False)
    )
    success.add_child(_findAndGrasp())
    success.add_child(_moveArmRetry("Arm to nav (post-grasp)", KEY_ARM_NAVIGATING))
    success.add_child(_serveToDestination())
    success.add_child(_moveArmRetry("Arm to nav (post-place)", KEY_ARM_NAVIGATING))
    success.add_child(
        BtNode_RecordCompletion(
            name="Record cleanup success",
            score_trace_key=KEY_SCORE_TRACE,
            active_prompt_key=KEY_ACTIVE_PROMPT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            success=True,
        )
    )

    failure = py_trees.composites.Sequence(
        name="Cleanup failure continuation", memory=True
    )
    failure.add_child(
        BtNode_Announce(
            name="Announce skip", bb_source=None, message="Skipping this item."
        )
    )
    failure.add_child(_gripperOpenSafe("Release any partial grasp"))
    failure.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Best-effort arm reset",
            child=BtNode_MoveArmSingle(
                name="Move arm to nav (failure)",
                service_name=ARM_SERVICE_NAME,
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
        )
    )
    failure.add_child(
        BtNode_RecordCompletion(
            name="Record cleanup failure",
            score_trace_key=KEY_SCORE_TRACE,
            active_prompt_key=KEY_ACTIVE_PROMPT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            success=False,
        )
    )

    attempt.add_child(success)
    attempt.add_child(failure)
    root.add_child(attempt)
    return root


def createCleanupPhase() -> py_trees.composites.Selector:
    """Repeat cleanup processing until queue is empty or runtime cutover hits."""
    root = py_trees.composites.Selector(name="Cleanup loop", memory=False)
    root.add_child(
        py_trees.decorators.Repeat(
            name="Repeat cleanup items",
            child=_processOneCleanupItem(),
            num_success=MAX_CLEANUP_ITERATIONS,
        )
    )
    root.add_child(py_trees.behaviours.Success(name="Cleanup loop done"))
    return root


# --------------------------------------------------------------------------- #
# Phase: breakfast assembly
# --------------------------------------------------------------------------- #


def _processOneBreakfastItem() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Process one breakfast item", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime cutover (breakfast)",
            phase_deadline_key=KEY_PHASE_DEADLINE,
        )
    )
    root.add_child(
        BtNode_HasPendingWork(
            name="Has pending breakfast item",
            work_queue_key=KEY_BREAKFAST_QUEUE,
        )
    )
    root.add_child(
        BtNode_SelectNextBreakfastItem(
            name="Select next breakfast item",
            breakfast_queue_key=KEY_BREAKFAST_QUEUE,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            active_prompt_key=KEY_ACTIVE_PROMPT,
            active_source_pose_key=KEY_ACTIVE_SOURCE_POSE,
            active_target_point_key=KEY_ACTIVE_TARGET_POINT,
            kitchen_shelf_pose_key=KEY_POSE_KITCHEN_SHELF,
            cabinet_pose_key=KEY_POSE_CABINET,
            bowl_point_key=KEY_POINT_BREAKFAST_BOWL,
            spoon_point_key=KEY_POINT_BREAKFAST_SPOON,
            cereal_point_key=KEY_POINT_BREAKFAST_CEREAL,
            milk_point_key=KEY_POINT_BREAKFAST_MILK,
        )
    )

    attempt = py_trees.composites.Selector(name="Attempt breakfast item", memory=False)

    success = py_trees.composites.Sequence(name="Breakfast success path", memory=True)
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry goto breakfast source",
            child=BtNode_GotoAction(
                name="Go to breakfast source", key=KEY_ACTIVE_SOURCE_POSE
            ),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success.add_child(
        _moveArmRetry("Arm to scan (breakfast)", KEY_ARM_SCAN, add_octomap=False)
    )
    success.add_child(_findAndGrasp())
    success.add_child(
        _moveArmRetry("Arm to nav (post-grasp breakfast)", KEY_ARM_NAVIGATING)
    )
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry goto table for breakfast",
            child=BtNode_GotoAction(name="Go to table (breakfast)", key=KEY_POSE_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success.add_child(
        _moveArmRetry("Arm to placing (breakfast)", KEY_ARM_PLACING, add_octomap=True)
    )
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry env point cloud (breakfast)",
            child=BtNode_GetPointCloud(
                name="Get env pc (breakfast)",
                bb_point_cloud_key=KEY_ENV_POINTS,
            ),
            num_failures=2,
        )
    )
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry place breakfast item",
            child=BtNode_Place(
                name="Place breakfast item",
                bb_key_point=KEY_ACTIVE_TARGET_POINT,
                bb_key_pose=KEY_GRASP_POSE,
                bb_key_env_points=KEY_ENV_POINTS,
                action_name=PLACE_ACTION_NAME,
            ),
            num_failures=2,
        )
    )
    success.add_child(_gripperOpenSafe("Open gripper after breakfast place"))
    success.add_child(_moveArmRetry("Arm to nav (post-breakfast)", KEY_ARM_NAVIGATING))
    success.add_child(
        BtNode_RecordCompletion(
            name="Record breakfast success",
            score_trace_key=KEY_SCORE_TRACE,
            active_prompt_key=KEY_ACTIVE_PROMPT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            success=True,
        )
    )

    failure = py_trees.composites.Sequence(
        name="Breakfast failure continuation", memory=True
    )
    failure.add_child(
        BtNode_Announce(
            name="Announce breakfast skip",
            bb_source=None,
            message="Skipping breakfast item.",
        )
    )
    failure.add_child(_gripperOpenSafe("Release any partial grasp (breakfast)"))
    failure.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Best-effort arm reset (breakfast)",
            child=BtNode_MoveArmSingle(
                name="Move arm to nav (breakfast failure)",
                service_name=ARM_SERVICE_NAME,
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
        )
    )
    failure.add_child(
        BtNode_RecordCompletion(
            name="Record breakfast failure",
            score_trace_key=KEY_SCORE_TRACE,
            active_prompt_key=KEY_ACTIVE_PROMPT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            success=False,
        )
    )

    attempt.add_child(success)
    attempt.add_child(failure)
    root.add_child(attempt)
    return root


def createBreakfastPhase() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Breakfast assembly", memory=True)
    root.add_child(
        BtNode_BuildBreakfastQueue(
            name="Seed breakfast queue",
            breakfast_queue_key=KEY_BREAKFAST_QUEUE,
            kitchen_shelf_pose_key=KEY_POSE_KITCHEN_SHELF,
            cabinet_pose_key=KEY_POSE_CABINET,
            bowl_point_key=KEY_POINT_BREAKFAST_BOWL,
            spoon_point_key=KEY_POINT_BREAKFAST_SPOON,
            cereal_point_key=KEY_POINT_BREAKFAST_CEREAL,
            milk_point_key=KEY_POINT_BREAKFAST_MILK,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce breakfast",
            bb_source=None,
            message="Setting up breakfast.",
        )
    )
    loop = py_trees.composites.Selector(name="Breakfast loop", memory=False)
    loop.add_child(
        py_trees.decorators.Repeat(
            name="Repeat breakfast items",
            child=_processOneBreakfastItem(),
            num_success=BREAKFAST_ITEM_COUNT,
        )
    )
    loop.add_child(py_trees.behaviours.Success(name="Breakfast loop done"))
    root.add_child(loop)
    return root


# --------------------------------------------------------------------------- #
# Phase: completion summary
# --------------------------------------------------------------------------- #


def createCompletionPhase() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Completion summary", memory=True)
    root.add_child(
        BtNode_BuildCompletionSummary(
            name="Build completion summary",
            score_trace_key=KEY_SCORE_TRACE,
            summary_key=KEY_SUMMARY_MESSAGE,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce summary", bb_source=KEY_SUMMARY_MESSAGE, message=None
        )
    )
    return root


# --------------------------------------------------------------------------- #
# Top-level mission
# --------------------------------------------------------------------------- #


def createPickAndPlaceTask() -> py_trees.composites.Selector:
    """Top-level Pick and Place mission tree.

    Wraps the flat phase sequence in a Selector + Timeout decorator so a
    runtime cutover terminates with an announcement instead of crashing.
    """
    mission = py_trees.composites.Sequence(name="PickAndPlace mission", memory=True)
    mission.add_child(createConstantWriter())
    mission.add_child(
        BtNode_InitTaskState(
            name="Initialize task state",
            inventory_table_key=KEY_INVENTORY_TABLE,
            work_queue_key=KEY_WORK_QUEUE,
            breakfast_queue_key=KEY_BREAKFAST_QUEUE,
            score_trace_key=KEY_SCORE_TRACE,
            phase_deadline_key=KEY_PHASE_DEADLINE,
            max_runtime_key=KEY_MAX_RUNTIME,
        )
    )
    mission.add_child(_moveArmRetry("Arm to nav (startup)", KEY_ARM_NAVIGATING))
    mission.add_child(
        BtNode_Announce(
            name="Announce start", bb_source=None, message="Starting pick and place."
        )
    )
    mission.add_child(createEnterKitchen())
    mission.add_child(createTableScanPhase())
    mission.add_child(createCleanupPhase())
    mission.add_child(createBreakfastPhase())
    mission.add_child(createCompletionPhase())

    root = py_trees.composites.Selector(
        name="PickAndPlace root timeout guard",
        memory=False,
    )
    root.add_child(
        py_trees.decorators.Timeout(
            name="Global runtime timeout",
            child=mission,
            duration=MAX_RUNTIME_SEC,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce timeout",
            bb_source=None,
            message="Time is up. Ending with completed actions.",
        )
    )
    return root
