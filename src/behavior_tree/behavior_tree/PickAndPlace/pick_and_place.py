from __future__ import annotations

"""PickAndPlace cleanup-first v1 behavior tree."""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_Drop, BtNode_Grasp, BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor

from .config import (
    ARM_POS_NAVIGATING,
    ENDGAME_BREAKFAST_PROMPTS,
    ENDGAME_MIN_REMAINING_SEC,
    FLOOR_TRASH_PROMPTS,
    GRASP_RETRY_LIMIT,
    KEY_ACTIVE_OBJECT,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POINT,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ARM_NAV,
    KEY_INVENTORY_FLOOR,
    KEY_INVENTORY_TABLE,
    KEY_MAX_RUNTIME,
    KEY_PHASE_DEADLINE,
    KEY_POINT_CLEAN_TABLE_ZONE,
    KEY_POINT_TRASH_BIN,
    KEY_POSE_CLEAN_TABLE_ZONE,
    KEY_POSE_FLOOR_SCAN,
    KEY_POSE_START,
    KEY_POSE_TABLE,
    KEY_POSE_TRASH_BIN,
    KEY_SCORE_TRACE,
    KEY_SUMMARY_MESSAGE,
    KEY_VISION_RESULT,
    KEY_WORK_QUEUE,
    MAX_RUNTIME_SEC,
    NAV_RETRY_LIMIT,
    POINT_CLEAN_TABLE_ZONE,
    POINT_TRASH_BIN,
    POSE_CLEAN_TABLE_ZONE,
    POSE_FLOOR_SCAN,
    POSE_START,
    POSE_TABLE,
    POSE_TRASH_BIN,
    SCAN_RETRY_LIMIT,
    TABLE_RELOCATION_PROMPTS,
    TABLE_TRASH_PROMPTS,
)
from .state_nodes import (
    BtNode_BuildCleanupWorkQueue,
    BtNode_BuildCompletionSummary,
    BtNode_HasBreakfastCandidates,
    BtNode_HasRemainingTime,
    BtNode_HasPendingWork,
    BtNode_InitCleanupState,
    BtNode_IsActiveTrashClass,
    BtNode_MergeInventoryFromDetection,
    BtNode_RecordCompletion,
    BtNode_SelectNextQueueItem,
    BtNode_TimeoutCutoverChecker,
)

MAX_QUEUE_ITERATIONS = 20


def createConstantWriter():
    """Write static constants and preconfigured poses to the blackboard."""
    root = py_trees.composites.Parallel(
        name="Write PickAndPlace constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in [
        ("Write start pose", KEY_POSE_START, POSE_START),
        ("Write table pose", KEY_POSE_TABLE, POSE_TABLE),
        ("Write floor scan pose", KEY_POSE_FLOOR_SCAN, POSE_FLOOR_SCAN),
        ("Write trash bin pose", KEY_POSE_TRASH_BIN, POSE_TRASH_BIN),
        ("Write clean table pose", KEY_POSE_CLEAN_TABLE_ZONE, POSE_CLEAN_TABLE_ZONE),
        ("Write trash bin point", KEY_POINT_TRASH_BIN, POINT_TRASH_BIN),
        ("Write clean table point", KEY_POINT_CLEAN_TABLE_ZONE, POINT_CLEAN_TABLE_ZONE),
        ("Write arm navigation pose", KEY_ARM_NAV, ARM_POS_NAVIGATING),
        ("Write max runtime", KEY_MAX_RUNTIME, MAX_RUNTIME_SEC),
    ]:
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


def createBuildWorkQueuePhase():
    """Scan table/floor and create cleanup-first prioritized queue."""
    root = py_trees.composites.Sequence(name="Build cleanup work queue", memory=True)
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto table",
            child=BtNode_GotoAction(name="Go to dining table", key=KEY_POSE_TABLE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry table scan",
            child=BtNode_ScanFor(
                name="Scan table objects",
                bb_source=None,
                bb_key=KEY_VISION_RESULT,
                object=", ".join(TABLE_TRASH_PROMPTS + TABLE_RELOCATION_PROMPTS),
                transform_to_map=True,
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    root.add_child(
        BtNode_MergeInventoryFromDetection(
            name="Merge table inventory",
            vision_result_key=KEY_VISION_RESULT,
            inventory_key=KEY_INVENTORY_TABLE,
            source_area="table",
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto floor scan pose",
            child=BtNode_GotoAction(name="Go to floor scan area", key=KEY_POSE_FLOOR_SCAN),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry floor scan",
            child=BtNode_ScanFor(
                name="Scan floor trash",
                bb_source=None,
                bb_key=KEY_VISION_RESULT,
                object=", ".join(FLOOR_TRASH_PROMPTS),
                transform_to_map=True,
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    root.add_child(
        BtNode_MergeInventoryFromDetection(
            name="Merge floor inventory",
            vision_result_key=KEY_VISION_RESULT,
            inventory_key=KEY_INVENTORY_FLOOR,
            source_area="floor",
        )
    )
    root.add_child(
        BtNode_BuildCleanupWorkQueue(
            name="Build cleanup-first queue",
            inventory_table_key=KEY_INVENTORY_TABLE,
            inventory_floor_key=KEY_INVENTORY_FLOOR,
            work_queue_key=KEY_WORK_QUEUE,
        )
    )
    return root


def _createServeToDestination():
    """Deliver currently grasped object using class-based destination policy."""
    destination = py_trees.composites.Selector(name="Destination policy", memory=False)

    trash_path = py_trees.composites.Sequence(name="Trash destination", memory=True)
    trash_path.add_child(
        BtNode_IsActiveTrashClass(
            name="Check trash-class item",
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
        )
    )
    trash_path.add_child(
        py_trees.decorators.Retry(
            name="Retry goto trash bin",
            child=BtNode_GotoAction(name="Go to trash bin", key=KEY_ACTIVE_TARGET_POSE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    trash_path.add_child(
        py_trees.decorators.Retry(
            name="Retry drop in trash bin",
            child=BtNode_Drop(name="Drop in trash bin", bb_source=KEY_ACTIVE_TARGET_POINT),
            num_failures=2,
        )
    )
    destination.add_child(trash_path)

    staging_path = py_trees.composites.Sequence(name="Staging table destination", memory=True)
    staging_path.add_child(
        py_trees.decorators.Inverter(
            name="Check non-trash item",
            child=BtNode_IsActiveTrashClass(
                name="Is trash for staging check",
                active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            ),
        )
    )
    staging_path.add_child(
        py_trees.decorators.Retry(
            name="Retry goto table staging zone",
            child=BtNode_GotoAction(name="Go to table staging zone", key=KEY_ACTIVE_TARGET_POSE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    staging_path.add_child(
        py_trees.decorators.Retry(
            name="Retry drop on staging table",
            child=BtNode_Drop(name="Drop on staging table", bb_source=KEY_ACTIVE_TARGET_POINT),
            num_failures=2,
        )
    )
    destination.add_child(staging_path)
    return destination


def _createProcessOneQueueItem():
    """Attempt one queue item and always continue on failure."""
    root = py_trees.composites.Sequence(name="Process one cleanup item", memory=True)
    root.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime cutover",
            phase_deadline_key=KEY_PHASE_DEADLINE,
        )
    )
    root.add_child(BtNode_HasPendingWork(name="Has pending queue item", work_queue_key=KEY_WORK_QUEUE))
    root.add_child(
        BtNode_SelectNextQueueItem(
            name="Select next cleanup item",
            work_queue_key=KEY_WORK_QUEUE,
            active_object_key=KEY_ACTIVE_OBJECT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            active_source_pose_key=KEY_ACTIVE_SOURCE_POSE,
            active_target_pose_key=KEY_ACTIVE_TARGET_POSE,
            active_target_point_key=KEY_ACTIVE_TARGET_POINT,
            pose_table_key=KEY_POSE_TABLE,
            pose_floor_scan_key=KEY_POSE_FLOOR_SCAN,
            pose_trash_bin_key=KEY_POSE_TRASH_BIN,
            pose_clean_table_key=KEY_POSE_CLEAN_TABLE_ZONE,
            point_trash_bin_key=KEY_POINT_TRASH_BIN,
            point_clean_table_key=KEY_POINT_CLEAN_TABLE_ZONE,
        )
    )

    attempt_selector = py_trees.composites.Selector(name="Attempt and continue", memory=False)

    success_path = py_trees.composites.Sequence(name="Successful cleanup item", memory=True)
    success_path.add_child(
        py_trees.decorators.Retry(
            name="Retry goto object source",
            child=BtNode_GotoAction(name="Go to source area", key=KEY_ACTIVE_SOURCE_POSE),
            num_failures=NAV_RETRY_LIMIT,
        )
    )
    success_path.add_child(
        py_trees.decorators.Retry(
            name="Retry refine object detection",
            child=BtNode_ScanFor(
                name="Refine active object",
                bb_source=KEY_ACTIVE_OBJECT,
                bb_key=KEY_VISION_RESULT,
                object=None,
                transform_to_map=False,
            ),
            num_failures=SCAN_RETRY_LIMIT,
        )
    )
    success_path.add_child(
        py_trees.decorators.Retry(
            name="Retry grasp object",
            child=BtNode_Grasp(name="Grasp active object", bb_source=KEY_VISION_RESULT),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    success_path.add_child(_createServeToDestination())
    success_path.add_child(
        py_trees.decorators.Retry(
            name="Retry arm to navigation pose",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAV,
                add_octomap=False,
            ),
            num_failures=2,
        )
    )
    success_path.add_child(
        BtNode_RecordCompletion(
            name="Record successful action",
            score_trace_key=KEY_SCORE_TRACE,
            active_object_key=KEY_ACTIVE_OBJECT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            active_target_pose_key=KEY_ACTIVE_TARGET_POSE,
            success=True,
        )
    )

    failure_path = py_trees.composites.Sequence(name="Failure continuation", memory=True)
    failure_path.add_child(
        BtNode_Announce(
            name="Cleanup item failed announcement",
            bb_source=None,
            message="Skipping this item.",
        )
    )
    failure_path.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Try arm reset on failure",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose on failure",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAV,
                add_octomap=False,
            ),
        )
    )
    failure_path.add_child(
        BtNode_RecordCompletion(
            name="Record failed action",
            score_trace_key=KEY_SCORE_TRACE,
            active_object_key=KEY_ACTIVE_OBJECT,
            active_object_class_key=KEY_ACTIVE_OBJECT_CLASS,
            active_target_pose_key=KEY_ACTIVE_TARGET_POSE,
            success=False,
        )
    )

    attempt_selector.add_child(success_path)
    attempt_selector.add_child(failure_path)
    root.add_child(attempt_selector)
    return root


def createCleanupLoop():
    """Run bounded queue iterations and stop gracefully when done/cutover hits."""
    root = py_trees.composites.Selector(name="Cleanup loop", memory=False)
    root.add_child(
        py_trees.decorators.Repeat(
            name="Process queue items",
            child=_createProcessOneQueueItem(),
            num_success=MAX_QUEUE_ITERATIONS,
        )
    )
    root.add_child(py_trees.behaviours.Success(name="Cleanup loop finished"))
    return root


def createEndgameBranch():
    """Optional low-risk bonus branch for easy breakfast placements."""
    root = py_trees.composites.Selector(name="Endgame opportunistic branch", memory=False)
    bonus = py_trees.composites.Sequence(name="Do easy breakfast bonus", memory=True)
    bonus.add_child(
        BtNode_HasRemainingTime(
            name="Check endgame remaining time",
            phase_deadline_key=KEY_PHASE_DEADLINE,
            min_remaining_sec=ENDGAME_MIN_REMAINING_SEC,
        )
    )
    bonus.add_child(
        BtNode_HasBreakfastCandidates(
            name="Check easy breakfast candidates",
            inventory_table_key=KEY_INVENTORY_TABLE,
            candidates=ENDGAME_BREAKFAST_PROMPTS,
        )
    )
    bonus.add_child(
        BtNode_Announce(
            name="Endgame bonus announcement",
            bb_source=None,
            message="I can still stage one breakfast item quickly.",
        )
    )
    root.add_child(bonus)
    root.add_child(py_trees.behaviours.Success(name="Skip endgame bonus"))
    return root


def createPickAndPlaceTask():
    """Compose cleanup-first PickAndPlace tree with timeout-safe fallback."""
    mission = py_trees.composites.Sequence(name="PickAndPlace cleanup mission", memory=True)
    mission.add_child(createConstantWriter())
    mission.add_child(
        BtNode_InitCleanupState(
            name="Initialize cleanup runtime state",
            inventory_table_key=KEY_INVENTORY_TABLE,
            inventory_floor_key=KEY_INVENTORY_FLOOR,
            work_queue_key=KEY_WORK_QUEUE,
            score_trace_key=KEY_SCORE_TRACE,
            phase_deadline_key=KEY_PHASE_DEADLINE,
            max_runtime_key=KEY_MAX_RUNTIME,
        )
    )
    mission.add_child(
        py_trees.decorators.Retry(
            name="Retry arm setup",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAV,
                add_octomap=False,
            ),
            num_failures=2,
        )
    )
    mission.add_child(
        BtNode_Announce(
            name="Start announcement",
            bb_source=None,
            message="Starting cleanup mode.",
        )
    )
    mission.add_child(createBuildWorkQueuePhase())
    mission.add_child(createCleanupLoop())
    mission.add_child(createEndgameBranch())
    mission.add_child(
        BtNode_BuildCompletionSummary(
            name="Build completion summary",
            score_trace_key=KEY_SCORE_TRACE,
            summary_key=KEY_SUMMARY_MESSAGE,
        )
    )
    mission.add_child(
        BtNode_Announce(
            name="Completion summary announcement",
            bb_source=KEY_SUMMARY_MESSAGE,
            message=None,
        )
    )

    root = py_trees.composites.Selector(name="PickAndPlace root timeout guard", memory=False)
    root.add_child(
        py_trees.decorators.Timeout(
            name="Global runtime timeout",
            child=mission,
            duration=MAX_RUNTIME_SEC,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Timeout fallback announcement",
            bb_source=None,
            message="Time is up. Ending with completed actions.",
        )
    )
    return root
