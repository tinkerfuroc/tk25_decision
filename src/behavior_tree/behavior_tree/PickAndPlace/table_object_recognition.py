from __future__ import annotations

"""Table Object Recognition subtree.

Mission:
    1. Vision: scan the table with the generalist detector.
    2. TTS announce what was seen so the referee can verify.
    3. Sort the scan result by grasp priority (TODO — node not yet written).

This is the *survey* step that runs before grasping. It populates a scan result
on the blackboard that downstream Table Grasp / Table Placing subtrees can
consume one object at a time.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Vision import (
    BtNode_ScanForGeneralist,
    BtNode_TurnPanTilt,
)
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_ACTION_NAME,
    ARM_POS_NAVIGATING,
    KEY_ANNOUNCEMENT_MSG,
    KEY_ARM_NAVIGATING,
    KEY_SCAN_RESULTS_TABLE,
    TABLE_SCAN_PROMPT,
)
from .custom_nodes import BtNode_WriteFoundItems


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write the blackboard constants this subtree depends on.

    Scoped to table-object-recognition only — does not duplicate
    pick_and_place.createConstantWriter. Safe to run standalone; the production
    mission writes the same keys upstream and the two entry points are not run
    concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write TableObjectRecognition constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
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


def _moveArmRetry(
    name: str, arm_pose_key: str, *, add_octomap: bool = False, retries: int = 2
):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_JointMoveAction(
            name=name,
            action_name=ARM_ACTION_NAME,
            arm_pose_bb_key=arm_pose_key,
        ),
        num_failures=retries,
    )


def _scanForGeneralistRetry(name: str, bb_key, object, use_orbbec=True, retries=5):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForGeneralist(
            name=name,
            bb_source=None,
            bb_key=bb_key,
            object=object,
            use_orbbec=use_orbbec,
            return_rgb_image=True,
            return_depth_image=True,
            force_vlm_sam=True,
            return_segments=True,
        ),
        num_failures=retries,
    )


def createTableObjectRecognition(
    *,
    prompt: str = TABLE_SCAN_PROMPT,
    pre_scan_arm_pose_key: str = KEY_ARM_NAVIGATING,
    scan_result_key: str = KEY_SCAN_RESULTS_TABLE,
    announcement_key: str = KEY_ANNOUNCEMENT_MSG,
    head_pan_deg: float = 0.0,
    head_tilt_deg: float = 20.0,
    move_arm_retries: int = 3,
):
    """Table Object Recognition subtree.

    Args:
        prompt: open-vocab detection prompt. Defaults to TABLE_SCAN_PROMPT
            from config — already covers cutlery / tableware / trash / breakfast.
        pre_scan_arm_pose_key: arm pose used to clear the orbbec view before
            scanning. Defaults to the stowed navigation pose.
        scan_result_key: blackboard key the scan bundle is written to.
        announcement_key: blackboard key the announcement text is written to.
        head_pan_deg / head_tilt_deg: head aim for the table scan. Defaults
            mirror `scanTableAndAnnounce` in pick_and_place.py.

    Assumes the robot is already positioned in front of the table.
    """
    root = py_trees.composites.Sequence(name="Table object recognition", memory=True)

    # 0. 写入本子树所需的 blackboard 常量（standalone-safe）
    root.add_child(createConstantWriter())

    # Aim the head at the table and clear the arm from the orbbec FOV.
    root.add_child(
        BtNode_TurnPanTilt(name="head to table", x=head_pan_deg, y=head_tilt_deg)
    )
    root.add_child(
        _moveArmRetry(
            name="move arm clear of table view",
            arm_pose_key=pre_scan_arm_pose_key,
            retries=move_arm_retries,
        )
    )

    # 1. 视觉识别物体 — generalist scan on the table; result goes to scan_result_key.
    root.add_child(
        _scanForGeneralistRetry(
            name="scan table objects",
            bb_key=scan_result_key,
            object=prompt,
            use_orbbec=True,
        )
    )

    # 2. 语音播报 — format the announcement message, then speak it.
    root.add_child(
        BtNode_WriteFoundItems(
            name="format table announcement",
            bb_key_vision_res=scan_result_key,
            bb_key_announcement=announcement_key,
            place_seen="on the table",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="announce table items",
            bb_source=announcement_key,
        )
    )

    # 3. 视觉识别根据优先级 sort
    # TODO: no BtNode_SortByPriority exists yet. Plug it in here when the node
    # lands; it should read `scan_result_key` and rewrite the objects list in
    # grasp-priority order (e.g. trash > tableware > breakfast > other, with
    # confidence/distance as tiebreakers).
    root.add_child(
        py_trees.behaviours.Success(
            name="TODO: sort scan_result by priority (node not implemented)"
        )
    )

    return root
