from __future__ import annotations

"""Scan and classify table items phase."""

import py_trees

from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Vision import BtNode_ScanFor, BtNode_TurnPanTilt

from ..config import (
    ARM_SERVICE_NAME,
    KEY_ARM_SCAN,
    KEY_INVENTORY_TABLE,
    KEY_WORK_QUEUE,
    KEY_VISION_RESULT,
    SCAN_RETRY_LIMIT,
    TABLE_SCAN_PROMPT,
    DESIGNATED_TRASH_LABELS,
    WASH_STAGING_LABELS,
)
from ..state_nodes import BtNode_BuildTableInventory, BtNode_ClassifyTableItems


def createTableScanPhase() -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Scan + classify table", memory=True)
    root.add_child(BtNode_TurnPanTilt(name="Tilt head down", x=0.0, y=25.0))
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry arm to scan",
            child=BtNode_MoveArmSingle(
                name="Move arm to scan",
                service_name=ARM_SERVICE_NAME,
                arm_pose_bb_key=KEY_ARM_SCAN,
                add_octomap=False,
            ),
            num_failures=2,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry table scan",
            child=BtNode_ScanFor(
                name="Scan dining table",
                bb_source=None,
                bb_key=KEY_VISION_RESULT,
                object=TABLE_SCAN_PROMPT,
                transform_to_map=True,
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