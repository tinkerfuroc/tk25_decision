from __future__ import annotations

"""Table Placing subtree.

Mission:
    1. Vision provides a placing point on the table (and env point cloud).
    2. Move arm to the predefined table-placing pose.
    3. Place the held object at the resolved point.

Assumes the robot is already in front of the table and is holding an object
whose grasp orientation has been written to ``grasp_pose_key``. Vision capture
happens with the head camera (orbbec) before the arm moves into the placing
pose, so the arm does not occlude the camera view.
"""

import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_Place,
)
from behavior_tree.TemplateNodes.Vision import (
    BtNode_FindPlacingLocation,
    BtNode_GetPointCloud,
)
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_ACTION_NAME,
    ARM_POS_NAVIGATING,
    ARM_POS_TABLE,
    KEY_ARM_NAVIGATING,
    KEY_ARM_TABLE,
    KEY_ENV_POINTS,
    KEY_GRASP_POSE,
    KEY_POINT_PLACING_DYNAMIC,
    PLACE_ACTION_NAME,
    TARGET_FRAME,
)


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write the arm-joint constants this subtree depends on to the blackboard.

    Scoped to table-placing only — does not duplicate pick_and_place.createConstantWriter.
    Safe to run standalone; the production mission writes the same keys upstream
    and the two entry points are not run concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write TablePlacing constants",
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


def createTablePlacing(
    item_description: str = "the held object",
    *,
    place_pose_key: str = KEY_ARM_TABLE,
    place_point_key: str = KEY_POINT_PLACING_DYNAMIC,
    grasp_pose_key: str = KEY_GRASP_POSE,
    env_points_key: str = KEY_ENV_POINTS,
    move_arm_retries: int = 3,
    place_retries: int = 3,
    find_point_retries: int = 3,
):
    """Table Placing subtree (one shot).

    Args:
        item_description: VLM prompt describing what is being placed, used to
            bias the empty-spot search (e.g. ``"a red soda bottle"``).
        place_pose_key: blackboard key for the preset table-placing arm pose.
        place_point_key: blackboard key the VLM writes the placing PointStamped to.
        grasp_pose_key: blackboard key holding the orientation captured at grasp.
        env_points_key: blackboard key the head-camera point cloud is written to.

    The outer Selector keeps overall success even on place failure, while still
    running cleanup (open gripper + arm back to nav) so downstream subtrees
    start from a known state.
    """
    place_sequence = py_trees.composites.Sequence(name="Table placing", memory=True)

    # 0. 写入本子树所需的 blackboard 常量（standalone-safe）
    place_sequence.add_child(createConstantWriter())

    # 1. 视觉给点 — VLM places point + orbbec point cloud (in parallel; both head-orbbec).
    vision_capture = py_trees.composites.Parallel(
        name="parallel vision for placing",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    vision_capture.add_child(
        py_trees.decorators.Retry(
            name="Retry find placing location on table",
            child=BtNode_FindPlacingLocation(
                name="find placing location on table",
                bb_key_point=place_point_key,
                item_description=item_description,
                use_orbbec=True,
                target_frame=TARGET_FRAME,
            ),
            num_failures=find_point_retries,
        )
    )
    vision_capture.add_child(
        BtNode_GetPointCloud(
            name="get env point cloud for placing",
            bb_point_cloud_key=env_points_key,
            camera_name="orbbec",
        )
    )
    place_sequence.add_child(vision_capture)

    # 2. 机械臂移动到预定位置
    place_sequence.add_child(
        _moveArmRetry(
            name="move arm to table placing pose",
            arm_pose_key=place_pose_key,
            add_octomap=True,
            retries=move_arm_retries,
        )
    )

    # 3. 机械臂放置
    place_sequence.add_child(
        py_trees.decorators.Retry(
            name="Retry place on table",
            child=BtNode_Place(
                name="place object on table",
                bb_key_point=place_point_key,
                bb_key_pose=grasp_pose_key,
                bb_key_env_points=env_points_key,
                action_name=PLACE_ACTION_NAME,
            ),
            num_failures=place_retries,
        )
    )
    place_sequence.add_child(
        BtNode_GripperAction(name="open gripper after placing", open_gripper=True)
    )
    place_sequence.add_child(
        _moveArmRetry(name="move arm to nav after placing", arm_pose_key=KEY_ARM_NAVIGATING)
    )

    failure_cleanup = py_trees.composites.Sequence(
        name="Reset state on place failure",
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
        name="Table placing with failure cleanup", memory=True
    )
    root.add_child(place_sequence)
    root.add_child(failure_cleanup)
    return root
