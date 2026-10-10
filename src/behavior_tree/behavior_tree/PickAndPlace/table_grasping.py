from __future__ import annotations

"""Table Grasp subtree.

Mission:
    1. Move arm to the predefined table-grasp pose.
    2. Grasp the target object on the table.

Assumes upstream phases (e.g. Table Object Recognition) have already moved the
robot to the table and selected the active grasp prompt. This subtree does the
close-range realsense scan right before the grasp so the grasp action gets a
fresh RGB / depth / segmentation bundle from the new arm pose.
"""

import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_Grasp,
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.TemplateNodes.Vision import BtNode_ScanForGeneralist
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_POS_NAVIGATING,
    ARM_POS_TABLE,
    ARM_ACTION_NAME,
    GRASP_ACTION_NAME,
    KEY_ARM_NAVIGATING,
    KEY_ARM_TABLE,
    KEY_GRASP_VISION_RES,
)


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write the arm-joint constants this subtree depends on to the blackboard.

    Scoped to table-grasp only — does not duplicate pick_and_place.createConstantWriter.
    Safe to run standalone; the production mission writes the same keys upstream
    and the two entry points are not run concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write TableGrasp constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm table", KEY_ARM_TABLE, ARM_POS_TABLE),
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


def _scanForGeneralistRetry(name: str, bb_source, bb_key, object, use_orbbec=False):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForGeneralist(
            name=name,
            bb_source=bb_source,
            bb_key=bb_key,
            object=object,
            use_orbbec=use_orbbec,
            return_rgb_image=True,
            return_depth_image=True,
            force_vlm_sam=True,
            return_segments=True,
        ),
        num_failures=5,
    )


def createTableGrasp(
    prompt: str = "bottle . cup . can . hand sanitizer",
    *,
    grasp_pose_key: str = KEY_ARM_TABLE,
    move_arm_retries: int = 3,
    grasp_retries: int = 3,
):
    """Table Grasp subtree (one shot).

    Args:
        prompt: open-vocab detection prompt for the target object.
        grasp_pose_key: blackboard key holding the preset table-grasp joint pose.
        move_arm_retries: retry budget for the preset-pose arm move.
        grasp_retries: retry budget for the grasp action.

    The outer Selector keeps overall success even on grasp failure, while still
    running cleanup (open gripper + arm back to nav) so downstream subtrees
    start from a known state.
    """
    grasp_sequence = py_trees.composites.Sequence(
        name=f"Table grasp ({prompt})", memory=True
    )

    # 0. 写入本子树所需的 blackboard 常量（standalone-safe）
    grasp_sequence.add_child(createConstantWriter())

    # 1. 机械臂移动到预定位置
    grasp_sequence.add_child(
        _moveArmRetry(
            name="move arm to table grasp pose",
            arm_pose_key=grasp_pose_key,
            add_octomap=True,
            retries=move_arm_retries,
        )
    )

    # 2. 抓取 — close-range realsense scan, then grasp, then reset arm to nav
    grasp_sequence.add_child(
        _scanForGeneralistRetry(
            name="scan for grasp target with realsense",
            bb_source=None,
            bb_key=KEY_GRASP_VISION_RES,
            object=prompt,
            use_orbbec=False,
        )
    )
    grasp_sequence.add_child(
        py_trees.decorators.Retry(
            name="Retry grasping object on table",
            child=BtNode_Grasp(
                name="grasp object on the table",
                bb_source=None,
                action_name=GRASP_ACTION_NAME,
                bb_key_vision_res=KEY_GRASP_VISION_RES,
            ),
            num_failures=grasp_retries,
        )
    )
    grasp_sequence.add_child(
        _moveArmRetry(name="move arm to nav", arm_pose_key=KEY_ARM_NAVIGATING)
    )

    failure_cleanup = py_trees.composites.Sequence(
        name="Reset state on grasp failure",
        memory=True,
        children=[
            BtNode_GripperAction(name="open gripper on failure", open_gripper=True),
            _moveArmRetry(
                name="move arm to nav on failure",
                arm_pose_key=KEY_ARM_NAVIGATING,
            ),
        ],
    )

    root = py_trees.composites.Selector(
        name="Table grasp with failure cleanup", memory=True
    )
    root.add_child(grasp_sequence)
    root.add_child(failure_cleanup)
    return root
