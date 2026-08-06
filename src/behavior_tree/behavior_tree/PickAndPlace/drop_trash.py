from __future__ import annotations

"""Drop Trash subtree.

Mission:
    1. Navigate to the placement point (trash bin pose by default).
    2. Move arm joints to a hardcoded drop pose, then open the gripper.

No vision is used — the drop pose is a fixed joint configuration tuned to
clear the bin rim, so we just need to reach it reliably. Assumes the robot is
already holding the trash item before this subtree runs.
"""

import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_ACTION_NAME,
    ARM_POS_NAVIGATING,
    ARM_POS_TRASH,
    KEY_ARM_NAVIGATING,
    KEY_ARM_TRASH,
    KEY_POSE_TRASH_BIN,
    NAV_RETRY_LIMIT,
    POSE_TRASH_BIN,
)


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write the blackboard constants this subtree depends on.

    Scoped to drop-trash only — does not duplicate pick_and_place.createConstantWriter.
    Safe to run standalone; the production mission writes the same keys upstream
    and the two entry points are not run concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write DropTrash constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm trash", KEY_ARM_TRASH, ARM_POS_TRASH),
        ("Write trash bin pose", KEY_POSE_TRASH_BIN, POSE_TRASH_BIN),
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


def createDropTrash(
    *,
    target_pose_key: str = KEY_POSE_TRASH_BIN,
    drop_arm_pose_key: str = KEY_ARM_TRASH,
    nav_retries: int = NAV_RETRY_LIMIT,
    move_arm_retries: int = 3,
):
    """Drop Trash subtree (one shot).

    Args:
        target_pose_key: blackboard key holding the navigation goal pose.
            Defaults to the trash bin; pass another pose to reuse this subtree
            for other categorized drops.
        drop_arm_pose_key: blackboard key holding the preset drop joint pose.
        nav_retries: retry budget for the navigation action.
        move_arm_retries: retry budget for the preset-pose arm move.

    The outer Selector keeps overall success even on drop failure, while still
    resetting the arm to nav so downstream subtrees start from a known state.
    """
    drop_sequence = py_trees.composites.Sequence(name="Drop trash", memory=True)

    # 0. 写入本子树所需的 blackboard 常量（standalone-safe）
    drop_sequence.add_child(createConstantWriter())

    # 1. 导航到对应物体该放置的点位
    drop_sequence.add_child(
        _moveArmRetry(
            name="move arm to nav (pre-trash)",
            arm_pose_key=KEY_ARM_NAVIGATING,
            retries=3,
        )
    )
    drop_sequence.add_child(
        py_trees.decorators.Retry(
            name="Retry goto trash bin",
            child=BtNode_GotoAction(name="goto trash bin", key=target_pose_key),
            num_failures=nav_retries,
        )
    )

    # 2. move joints 写死高度放置
    drop_sequence.add_child(
        _moveArmRetry(
            name="move arm to trash drop pose",
            arm_pose_key=drop_arm_pose_key,
            add_octomap=False,
            retries=move_arm_retries,
        )
    )
    drop_sequence.add_child(
        BtNode_GripperAction(name="open gripper to drop trash", open_gripper=True)
    )
    drop_sequence.add_child(
        _moveArmRetry(name="move arm to nav after drop", arm_pose_key=KEY_ARM_NAVIGATING)
    )

    failure_cleanup = py_trees.composites.Sequence(
        name="Reset state on drop failure",
        memory=True,
        children=[
            _moveArmRetry(
                name="move arm to nav on failure",
                arm_pose_key=KEY_ARM_NAVIGATING,
            ),
        ],
    )

    root = py_trees.composites.Selector(
        name="Drop trash with failure cleanup", memory=True
    )
    root.add_child(drop_sequence)
    root.add_child(failure_cleanup)
    return root
