from __future__ import annotations

"""Top-level PickAndPlace task — assembles all phases into the mission tree."""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle

from ..config import (
    ARM_POS_DROP,
    ARM_POS_NAVIGATING,
    ARM_POS_PLACING,
    ARM_POS_SCAN,
    ARM_SERVICE_NAME,
    KEY_ARM_DROP,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PLACING,
    KEY_ARM_SCAN,
    KEY_BREAKFAST_QUEUE,
    KEY_INVENTORY_TABLE,
    KEY_MAX_RUNTIME,
    KEY_PHASE_DEADLINE,
    KEY_POSE_CABINET,
    KEY_POSE_KITCHEN_ENTRY,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_POSE_TRASH_BIN,
    KEY_POSE_WASH_STAGING,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    KEY_POINT_CABINET_DEFAULT,
    KEY_POINT_KITCHEN_SURFACE,
    KEY_POINT_SHELF_LEFT,
    KEY_POINT_SHELF_RIGHT,
    KEY_POINT_TRASH_BIN,
    KEY_POINT_WASH_STAGING,
    KEY_SCORE_TRACE,
    KEY_SUMMARY_MESSAGE,
    KEY_TARGET_FRAME,
    KEY_WORK_QUEUE,
    MAX_RUNTIME_SEC,
    POSE_CABINET,
    POSE_KITCHEN_ENTRY,
    POSE_KITCHEN_SHELF,
    POSE_TABLE,
    POSE_TRASH_BIN,
    POSE_WASH_STAGING,
    POINT_BREAKFAST_BOWL,
    POINT_BREAKFAST_CEREAL,
    POINT_BREAKFAST_MILK,
    POINT_BREAKFAST_SPOON,
    POINT_CABINET_DEFAULT,
    POINT_KITCHEN_SURFACE,
    POINT_SHELF_LEFT,
    POINT_SHELF_RIGHT,
    POINT_TRASH_BIN,
    POINT_WASH_STAGING,
    TARGET_FRAME,
)
from ..state_nodes import BtNode_BuildCompletionSummary, BtNode_InitTaskState
from .cleanup import _moveArmRetry, createCleanupLoop
from .breakfast import createBreakfastPhase
from .enter_kitchen import createEnterKitchen
from .table_scan import createTableScanPhase


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
        ("Write kitchen entry pose",   KEY_POSE_KITCHEN_ENTRY,  POSE_KITCHEN_ENTRY),
        ("Write table pose",           KEY_POSE_TABLE,          POSE_TABLE),
        ("Write wash-staging pose",    KEY_POSE_WASH_STAGING,   POSE_WASH_STAGING),
        ("Write trash bin pose",       KEY_POSE_TRASH_BIN,      POSE_TRASH_BIN),
        ("Write cabinet pose",         KEY_POSE_CABINET,        POSE_CABINET),
        ("Write kitchen shelf pose",   KEY_POSE_KITCHEN_SHELF,  POSE_KITCHEN_SHELF),
        ("Write wash-staging point",   KEY_POINT_WASH_STAGING,  POINT_WASH_STAGING),
        ("Write trash bin point",      KEY_POINT_TRASH_BIN,     POINT_TRASH_BIN),
        ("Write cabinet default pt",   KEY_POINT_CABINET_DEFAULT, POINT_CABINET_DEFAULT),
        ("Write kitchen surface pt",   KEY_POINT_KITCHEN_SURFACE, POINT_KITCHEN_SURFACE),
        ("Write breakfast bowl pt",    KEY_POINT_BREAKFAST_BOWL,   POINT_BREAKFAST_BOWL),
        ("Write breakfast spoon pt",   KEY_POINT_BREAKFAST_SPOON,  POINT_BREAKFAST_SPOON),
        ("Write breakfast cereal pt",  KEY_POINT_BREAKFAST_CEREAL, POINT_BREAKFAST_CEREAL),
        ("Write breakfast milk pt",    KEY_POINT_BREAKFAST_MILK,   POINT_BREAKFAST_MILK),
        ("Write shelf left point",     KEY_POINT_SHELF_LEFT,    POINT_SHELF_LEFT),
        ("Write shelf right point",    KEY_POINT_SHELF_RIGHT,   POINT_SHELF_RIGHT),
        ("Write arm navigating",       KEY_ARM_NAVIGATING,      ARM_POS_NAVIGATING),
        ("Write arm scan",             KEY_ARM_SCAN,            ARM_POS_SCAN),
        ("Write arm placing",          KEY_ARM_PLACING,         ARM_POS_PLACING),
        ("Write arm drop",             KEY_ARM_DROP,            ARM_POS_DROP),
        ("Write target frame",         KEY_TARGET_FRAME,        TARGET_FRAME),
        ("Write max runtime",          KEY_MAX_RUNTIME,         MAX_RUNTIME_SEC),
    ]
    for name, key, value in writes:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name, bb_namespace="", bb_source=None, bb_key=key, object=value,
            )
        )
    return root


# --------------------------------------------------------------------------- #
# Top-level mission
# --------------------------------------------------------------------------- #

def createPickAndPlaceTask() -> py_trees.composites.Selector:
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
        BtNode_Announce(name="Announce start", bb_source=None, message="Starting pick and place.")
    )
    mission.add_child(createEnterKitchen())
    mission.add_child(createTableScanPhase())
    mission.add_child(createCleanupLoop())
    mission.add_child(createBreakfastPhase())
    mission.add_child(
        BtNode_BuildCompletionSummary(
            name="Build completion summary",
            score_trace_key=KEY_SCORE_TRACE,
            summary_key=KEY_SUMMARY_MESSAGE,
        )
    )
    mission.add_child(
        BtNode_Announce(name="Announce summary", bb_source=KEY_SUMMARY_MESSAGE, message=None)
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
            name="Announce timeout",
            bb_source=None,
            message="Time is up. Ending with completed actions.",
        )
    )
    return root