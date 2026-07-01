from __future__ import annotations

"""Nav Test + Orbbec look subtree.

Mission (used to verify navigation accuracy at each known stop point):
    1. Set the goal pose on the blackboard.
    2. Navigate to it.
    3. Turn the orbbec head to a predefined pan/tilt so we can visually
       confirm the stop point captures the intended workspace
       (e.g. Cabinet vs Table need different tilt angles).

Pass either a literal ``goal_pose`` (PoseStamped) to write fresh, or a
``goal_pose_key`` that already holds one (e.g. KEY_POSE_TABLE after
createConstantWriter has run). For sweeping multiple destinations in one
run, see ``createNavTestSweep``.
"""

from typing import Iterable, Optional, Tuple

import py_trees

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_ACTION_NAME,
    ARM_POS_NAVIGATING,
    KEY_ARM_NAVIGATING,
    KEY_POSE_CABINET,
    KEY_POSE_DISH_WASHER,
    KEY_POSE_KITCHEN_ENTRY,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_POSE_TRASH_BIN,
    KEY_POSE_WASH_STAGING,
    NAV_RETRY_LIMIT,
    POSE_CABINET,
    POSE_DISH_WASHER,
    POSE_KITCHEN_ENTRY,
    POSE_KITCHEN_SHELF,
    POSE_TABLE,
    POSE_TRASH_BIN,
    POSE_WASH_STAGING,
)


def createConstantWriter() -> py_trees.composites.Parallel:
    """Write the blackboard constants this subtree depends on.

    Scoped to nav-test only — does not duplicate pick_and_place.createConstantWriter.
    Covers KEY_ARM_NAVIGATING plus every default nav-test destination so the
    sweep can be ticked standalone. Safe to run alongside the production mission
    (same key/value pairs); the two entry points are not run concurrently.
    """
    root = py_trees.composites.Parallel(
        name="Write NavTest constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write kitchen entry pose", KEY_POSE_KITCHEN_ENTRY, POSE_KITCHEN_ENTRY),
        ("Write table pose", KEY_POSE_TABLE, POSE_TABLE),
        ("Write wash-staging pose", KEY_POSE_WASH_STAGING, POSE_WASH_STAGING),
        ("Write trash bin pose", KEY_POSE_TRASH_BIN, POSE_TRASH_BIN),
        ("Write cabinet pose", KEY_POSE_CABINET, POSE_CABINET),
        ("Write kitchen shelf pose", KEY_POSE_KITCHEN_SHELF, POSE_KITCHEN_SHELF),
        ("Write dishwasher pose", KEY_POSE_DISH_WASHER, POSE_DISH_WASHER),
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


def _slugify(label: str) -> str:
    return label.strip().lower().replace(" ", "_")


def createNavTest(
    location_label: str,
    *,
    goal_pose=None,
    goal_pose_key: Optional[str] = None,
    look_pan_deg: float = 0.0,  # change here of the orbbec's deg for checking the stop point
    look_tilt_deg: float = 20.0,
    nav_retries: int = NAV_RETRY_LIMIT,
    move_arm_retries: int = 3,
):
    """Single-destination nav test.

    Args:
        location_label: short name (e.g. ``"cabinet"``, ``"table"``). Used to
            name nodes and derive a unique blackboard key when ``goal_pose``
            is passed.
        goal_pose: a PoseStamped to write to the blackboard for this run.
            Mutually exclusive with ``goal_pose_key``.
        goal_pose_key: blackboard key already holding a PoseStamped (e.g.
            ``KEY_POSE_CABINET``). Mutually exclusive with ``goal_pose``.
        look_pan_deg / look_tilt_deg: orbbec head aim AFTER arrival. Tune per
            destination so the stop point's workspace is fully in view.
        nav_retries: retry budget for Nav2.
        move_arm_retries: retry budget for the pre-nav arm stow.
    """
    if (goal_pose is None) == (goal_pose_key is None):
        raise ValueError(
            "createNavTest needs exactly one of `goal_pose` or `goal_pose_key`"
        )

    resolved_key = goal_pose_key or f"pp_nav_test_goal_{_slugify(location_label)}"

    root = py_trees.composites.Sequence(
        name=f"Nav test ({location_label})", memory=True
    )

    # 1. 设定 goal pose — only when a literal pose was passed in.
    if goal_pose is not None:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=f"Write goal pose for {location_label}",
                bb_namespace="",
                bb_source=None,
                bb_key=resolved_key,
                object=goal_pose,
            )
        )

    # Stow the arm so it doesn't snag during base motion (matches
    # _gotoRetryWith_Announcement in pick_and_place.py).
    root.add_child(
        _moveArmRetry(
            name=f"move arm to nav (pre-{location_label})",
            arm_pose_key=KEY_ARM_NAVIGATING,
            retries=move_arm_retries,
        )
    )

    # 2. 导航
    root.add_child(
        py_trees.decorators.Retry(
            name=f"Retry goto {location_label}",
            child=BtNode_GotoAction(name=f"goto {location_label}", key=resolved_key),
            num_failures=nav_retries,
        )
    )

    # 3. orbbec 看向预定角度
    root.add_child(
        BtNode_TurnPanTilt(
            name=f"head to {location_label} (pan={look_pan_deg}, tilt={look_tilt_deg})",
            x=look_pan_deg,
            y=look_tilt_deg,
        )
    )

    return root


# Each entry: (label, goal_pose_or_None, goal_pose_key_or_None, pan_deg, tilt_deg).
# Exactly one of pose / pose_key per entry; the other is None.
NavTestSpec = Tuple[str, object, Optional[str], float, float]


def createNavTestSweep(specs: Iterable[NavTestSpec]):
    """Sequential nav-test sweep over multiple destinations.

    Each destination is best-effort wrapped (FailureIsSuccess) so one bad stop
    doesn't abort the whole sweep — useful when calibrating multiple points
    in one run.
    """
    root = py_trees.composites.Sequence(name="Nav test sweep", memory=True)

    # 0. 写入本子树所需的 blackboard 常量（standalone-safe）
    root.add_child(createConstantWriter())

    for label, pose, pose_key, pan, tilt in specs:
        root.add_child(
            py_trees.decorators.FailureIsSuccess(
                name=f"Best-effort nav test ({label})",
                child=createNavTest(
                    location_label=label,
                    goal_pose=pose,
                    goal_pose_key=pose_key,
                    look_pan_deg=pan,
                    look_tilt_deg=tilt,
                ),
            )
        )
    return root
