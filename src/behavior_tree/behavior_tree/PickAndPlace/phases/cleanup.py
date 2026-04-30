from __future__ import annotations

"""Cleanup phase — tidy table via 3-class destination dispatch.

Destination classes: wash_staging, trash, cabinet.
Each class has its own branch in the destination selector.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_Drop,
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
    BtNode_Place,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_GetPointCloud, BtNode_TurnPanTilt

from ..custom_nodes import BtNode_CategorizeGrocery, BtNode_FindObjTable, BtNode_GraspWithPose
from ..config import (
    ARM_SERVICE_NAME,
    DESIGNATED_TRASH_LABELS,
    GRASP_ACTION_NAME,
    GRASP_RETRY_LIMIT,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_ACTIVE_PROMPT,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ACTIVE_TARGET_POINT,
    KEY_ARM_DROP,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PLACING,
    KEY_ARM_SCAN,
    KEY_ENV_POINTS,
    KEY_GRASP_ANNOUNCEMENT,
    KEY_GRASP_POSE,
    KEY_OBJECT_LABEL,
    KEY_OBJ_SEG,
    KEY_POINT_CABINET_DEFAULT,
    KEY_POINT_KITCHEN_SURFACE,
    KEY_POINT_SHELF_LEFT,
    KEY_POINT_SHELF_RIGHT,
    KEY_POINT_TRASH_BIN,
    KEY_POINT_WASH_STAGING,
    KEY_POSE_CABINET,
    KEY_POSE_TABLE,
    KEY_POSE_TRASH_BIN,
    KEY_POSE_WASH_STAGING,
    KEY_PHASE_DEADLINE,
    KEY_PLACE_REASON,
    KEY_SCORE_TRACE,
    KEY_TABLE_IMG,
    KEY_TARGET_FRAME,
    KEY_WORK_QUEUE,
    KEY_VISION_RESULT,
    NAV_RETRY_LIMIT,
    N_LAYERS,
    PLACE_ACTION_NAME,
    SCAN_RETRY_LIMIT,
    WASH_STAGING_LABELS,
)
from ..state_nodes import (
    BtNode_HasPendingWork,
    BtNode_IsActiveClass,
    BtNode_RecordCompletion,
    BtNode_SelectNextItem,
    BtNode_TimeoutCutoverChecker,
)


MAX_CLEANUP_ITERATIONS = 12  # rulebook caps Pick at 12x and Place at 12x


# --------------------------------------------------------------------------- #
# Shared helpers (also used by breakfast.py)
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


def _gripperOpenSafe(name: str):
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {name}",
        child=BtNode_GripperAction(name=name, open_gripper=True),
    )


# --------------------------------------------------------------------------- #
# Find and grasp — sequential announce then grasp
# --------------------------------------------------------------------------- #

def _findAndGrasp() -> py_trees.composites.Sequence:
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
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    # Announce finishes before GraspWithPose starts — object label is
    # already set by FindObjTable in the previous step.
    grasp_sequence = py_trees.composites.Sequence(name="Announce then grasp", memory=True)
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
# Destination branches
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
            child=BtNode_GotoAction(name="Go to wash staging", key=KEY_POSE_WASH_STAGING),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(_moveArmRetry("Arm to placing", KEY_ARM_PLACING))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry env point cloud",
            child=BtNode_GetPointCloud(name="Get env point cloud", bb_point_cloud_key=KEY_ENV_POINTS),
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
        BtNode_Announce(name="Announce wash staged", bb_source=None, message="Placed on wash staging.")
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
    root.add_child(_moveArmRetry("Arm to drop", KEY_ARM_DROP))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry drop in bin",
            child=BtNode_Drop(name="Drop in bin", bb_source=KEY_POINT_TRASH_BIN),
            num_failures=2,
        )
    )
    root.add_child(
        BtNode_Announce(name="Announce dropped trash", bb_source=None, message="Dropped in trash.")
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
    root.add_child(_moveArmRetry("Arm to scan (cabinet)", KEY_ARM_SCAN, add_octomap=False))
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
    root.add_child(_moveArmRetry("Arm to placing (cabinet)", KEY_ARM_PLACING, add_octomap=True))
    root.add_child(BtNode_Announce(name="Announce shelf reason", bb_source=KEY_PLACE_REASON))
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


# --------------------------------------------------------------------------- #
# Cleanup item processing + loop
# --------------------------------------------------------------------------- #

def _processOneCleanupItem() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Process one cleanup item", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime cutover", phase_deadline_key=KEY_PHASE_DEADLINE,
        )
    )
    root.add_child(BtNode_HasPendingWork(name="Has pending cleanup item", work_queue_key=KEY_WORK_QUEUE))
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
            point_trash_bin_key=KEY_POINT_TRASH_BIN,
            point_cabinet_default_key=KEY_POINT_CABINET_DEFAULT,
        )
    )

    attempt = py_trees.composites.Selector(name="Attempt and continue", memory=False)

    success = py_trees.composites.Sequence(name="Cleanup success path", memory=True)
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry goto source",
            child=BtNode_GotoAction(name="Go to source area", key=KEY_ACTIVE_SOURCE_POSE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success.add_child(_moveArmRetry("Arm to scan (cleanup)", KEY_ARM_SCAN, add_octomap=True))
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

    failure = py_trees.composites.Sequence(name="Cleanup failure continuation", memory=True)
    failure.add_child(
        BtNode_Announce(name="Announce skip", bb_source=None, message="Skipping this item.")
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


def createCleanupLoop() -> py_trees.composites.Selector:
    """Repeat _processOneCleanupItem until the queue is empty or timeout cutover."""
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