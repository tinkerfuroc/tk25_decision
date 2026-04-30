from __future__ import annotations

"""Breakfast assembly phase — bowl, spoon, cereal, milk onto dining table."""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle, BtNode_Place
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_GetPointCloud

from ..custom_nodes import BtNode_FindObjTable, BtNode_GraspWithPose
from ..config import (
    ARM_SERVICE_NAME,
    GRASP_ACTION_NAME,
    GRASP_RETRY_LIMIT,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_ACTIVE_PROMPT,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POINT,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PLACING,
    KEY_ARM_SCAN,
    KEY_BREAKFAST_QUEUE,
    KEY_ENV_POINTS,
    KEY_GRASP_ANNOUNCEMENT,
    KEY_GRASP_POSE,
    KEY_OBJECT_LABEL,
    KEY_OBJ_SEG,
    KEY_PHASE_DEADLINE,
    KEY_POSE_CABINET,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    KEY_SCORE_TRACE,
    KEY_TABLE_IMG,
    KEY_VISION_RESULT,
    NAV_RETRY_LIMIT,
    SCAN_RETRY_LIMIT,
    PLACE_ACTION_NAME,
)
from ..state_nodes import (
    BtNode_BuildBreakfastQueue,
    BtNode_HasPendingWork,
    BtNode_RecordCompletion,
    BtNode_SelectNextBreakfastItem,
    BtNode_TimeoutCutoverChecker,
)
from .cleanup import _findAndGrasp, _gripperOpenSafe, _moveArmRetry


BREAKFAST_ITEM_COUNT = 4


def _processOneBreakfastItem() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Process one breakfast item", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime cutover (breakfast)",
            phase_deadline_key=KEY_PHASE_DEADLINE,
        )
    )
    root.add_child(
        BtNode_HasPendingWork(name="Has pending breakfast item", work_queue_key=KEY_BREAKFAST_QUEUE)
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
            child=BtNode_GotoAction(name="Go to breakfast source", key=KEY_ACTIVE_SOURCE_POSE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success.add_child(_moveArmRetry("Arm to scan (breakfast)", KEY_ARM_SCAN, add_octomap=True))
    success.add_child(_findAndGrasp())
    success.add_child(_moveArmRetry("Arm to nav (post-grasp breakfast)", KEY_ARM_NAVIGATING))
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry goto table for breakfast",
            child=BtNode_GotoAction(name="Go to table (breakfast)", key=KEY_POSE_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success.add_child(_moveArmRetry("Arm to placing (breakfast)", KEY_ARM_PLACING))
    success.add_child(
        py_trees.decorators.Retry(
            name="Retry env point cloud (breakfast)",
            child=BtNode_GetPointCloud(name="Get env pc (breakfast)", bb_point_cloud_key=KEY_ENV_POINTS),
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

    failure = py_trees.composites.Sequence(name="Breakfast failure continuation", memory=True)
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
        BtNode_Announce(name="Announce breakfast", bb_source=None, message="Setting up breakfast.")
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