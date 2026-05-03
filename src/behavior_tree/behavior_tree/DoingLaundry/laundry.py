from __future__ import annotations
from re import M

"""Doing Laundry — main task deciding tree (RoboCup@Home 2026 §5.4).

Hardware-aware deviations from the rulebook (see plan + laundry.md):
  * Single xArm — no bimanual basket lift (DO_BASKET_TRANSPORT=False).
  * Washer door open requires operator help (DO_REQUEST_WASHER_HELP=True).
    Net per door open: +300 - 40 (env-change penalty) = +260.
  * Single-piece grasp from washer requires operator handover
    (DO_REQUEST_SINGLE_PIECE_VIA_HANDOVER=True). Avoids -100 multi-grab.
    Net per piece: +300 retrieval - 40 handover = +260.
  * Fold action server (`fold_clothing_action`) does not yet exist.
    Tree optimistically calls `BtNode_FoldClothing`; on failure the operator-
    assisted branch fires (-100 minor help on primary, -200 on extras),
    still earning +700 / +200 per fold.

Estimated yield with these constraints: ~+1635 of the 4415 ceiling.
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_FoldClothing,
    BtNode_Grasp,
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_GetPointCloud,
    BtNode_ScanFor,
    BtNode_TurnPanTilt,
)
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction

from .config import (
    ARM_POS_FOLD_START,
    ARM_POS_NAVIGATING,
    ARM_POS_PRE_PICK_BASKET,
    ARM_POS_PICK_BASKET,
    ARM_POS_PRE_PICK_CLOTHING,
    ARM_POS_PICK_CLOTHING,
    # ARM_POS_PICK_WASHER,
    # ARM_POS_PLACING,
    # ARM_POS_SCAN,
    # ARM_SERVICE_NAME,
    ARM_ACTION_NAME,
    # CLOTHING_SCAN_PROMPT,
    # DO_FOLD_PICKED_PIECES,
    # DO_PICK_FROM_BASKET,
    # DO_PICK_FROM_WASHER,
    # DO_REQUEST_WASHER_HELP,
    FOLD_ACTION_NAME,
    # FOLD_CYCLES,
    # GRASP_ACTION_NAME,
    # GRASP_RETRY_LIMIT,
    KEY_ARM_FOLD_START,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PICK_BASKET,
    KEY_ARM_PRE_PICK_BASKET,
    KEY_ARM_PICK_CLOTHING,
    KEY_ARM_PRE_PICK_CLOTHING,
    # KEY_ARM_PICK_WASHER,
    # KEY_ARM_PLACING,
    # KEY_ARM_SCAN,
    # KEY_DOOR_STATUS,
    # KEY_ENV_POINTS,
    # KEY_FOLD_COUNT,
    # KEY_GRASP_ANNOUNCEMENT,
    # KEY_MAX_RUNTIME,
    # KEY_OBJECT_LABEL,
    # KEY_PHASE_DEADLINE,
    # KEY_POINT_BASKET_TOP,
    # KEY_POINT_TABLE_FOLD_ZONE,
    # KEY_POINT_TABLE_STACK_ZONE,
    # KEY_POINT_WASHER_DRUM,
    KEY_POSE_ARENA_ENTRY,
    # KEY_POSE_BASKET,
    KEY_POSE_BASKET_TABLE,
    KEY_POSE_FOLDING_TABLE,
    KEY_POSE_LAUNDRY_AREA,
    KEY_POSE_WASHING_MACHINE,
    KEY_SCORE_TRACE,
    KEY_STACK_COUNT,
    KEY_SUMMARY_MESSAGE,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    MAX_EXTRA_FOLDS,
    MAX_RUNTIME_SEC,
    NAV_RETRY_LIMIT,
    OBJECT_LABEL_CLOTHING,
    # POINT_BASKET_TOP,
    # POINT_TABLE_FOLD_ZONE,
    # POINT_TABLE_STACK_ZONE,
    # POINT_WASHER_DRUM,
    POSE_ARENA_ENTRY,
    # POSE_BASKET,
    POSE_FOLDING_TABLE,
    # POSE_LAUNDRY_AREA,
    POSE_BASKET_TABLE,
    POSE_WASHING_MACHINE,
    SCAN_RETRY_LIMIT,
    TARGET_FRAME,
)
from .state_nodes import (
    BtNode_BuildCompletionSummary,
    BtNode_IncrementCounter,
    BtNode_InitTaskState,
    BtNode_RecordCompletion,
    BtNode_TimeoutCutoverChecker,
)

# --------------------------------------------------------------------------- #
# Constant writer
# --------------------------------------------------------------------------- #


def createConstantWriter() -> py_trees.composites.Parallel:
    root = py_trees.composites.Parallel(
        name="Write DoingLaundry constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arena entry pose", KEY_POSE_ARENA_ENTRY, POSE_ARENA_ENTRY),
        # ("Write laundry area pose", KEY_POSE_LAUNDRY_AREA, POSE_LAUNDRY_AREA),
        ("Write washing machine pose", KEY_POSE_WASHING_MACHINE, POSE_WASHING_MACHINE),
        # ("Write basket pose", KEY_POSE_BASKET, POSE_BASKET),
        ("Write basket table pose", KEY_POSE_BASKET_TABLE, POSE_BASKET_TABLE),
        ("Write folding table pose", KEY_POSE_FOLDING_TABLE, POSE_FOLDING_TABLE),
        # ("Write table fold zone pt", KEY_POINT_TABLE_FOLD_ZONE, POINT_TABLE_FOLD_ZONE),
        # (
        #     "Write table stack zone pt",
        #     KEY_POINT_TABLE_STACK_ZONE,
        #     POINT_TABLE_STACK_ZONE,
        # ),
        # ("Write basket top pt", KEY_POINT_BASKET_TOP, POINT_BASKET_TOP),
        # ("Write washer drum pt", KEY_POINT_WASHER_DRUM, POINT_WASHER_DRUM),
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        # ("Write arm scan", KEY_ARM_SCAN, ARM_POS_SCAN),
        ("Write arm pick basket", KEY_ARM_PICK_BASKET, ARM_POS_PICK_BASKET),
        # ("Write arm pick washer", KEY_ARM_PICK_WASHER, ARM_POS_PICK_WASHER),
        # ("Write arm placing", KEY_ARM_PLACING, ARM_POS_PLACING),
        ("Write arm fold start", KEY_ARM_FOLD_START, ARM_POS_FOLD_START),
        ("Write arm pre pick basket", KEY_ARM_PRE_PICK_BASKET, ARM_POS_PRE_PICK_BASKET),
        ("Write arm pick clothing", KEY_ARM_PICK_CLOTHING, ARM_POS_PICK_CLOTHING),
        (
            "Write arm pre pick clothing",
            KEY_ARM_PRE_PICK_CLOTHING,
            ARM_POS_PRE_PICK_BASKET,
        ),
        ("Write target frame", KEY_TARGET_FRAME, TARGET_FRAME),
        # ("Write max runtime", KEY_MAX_RUNTIME, MAX_RUNTIME_SEC),
        # ("Write clothing object label", KEY_OBJECT_LABEL, OBJECT_LABEL_CLOTHING),
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


# --------------------------------------------------------------------------- #
# Small helpers
# --------------------------------------------------------------------------- #


def _gotoRetry(name: str, pose_key: str, *, retries: int = NAV_RETRY_LIMIT):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_GotoAction(
            name=name,
            key=pose_key,
        ),
        num_failures=retries,
    )


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


def _gripperOpenSafe(name: str = "Open gripper"):
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {name}",
        child=BtNode_GripperAction(name=name, open_gripper=True),
    )


def _gripperCloseSafe(name: str = "Close gripper"):
    return py_trees.decorators.FailureIsSuccess(
        name=f"Best-effort {name}",
        child=BtNode_GripperAction(name=name, open_gripper=False),
    )


def pickupOneClothing():
    root = py_trees.composites.Sequence(name="Pick up one clothing piece", memory=True)
    root.add_child(
        _moveArmRetry(
            name="Move arm to base moving",
            arm_pose_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
        )
    )

    nav_parallel = py_trees.composites.Parallel(
        name="Navigate to washing machine",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    nav_parallel.add_child(
        _gotoRetry(
            name="Navigate to washing machine", pose_key=KEY_POSE_WASHING_MACHINE
        )
    )
    nav_parallel.add_child(
        BtNode_Announce(
            name="Announce navigating to washing machine",
            bb_source=None,
            message="Navigating to washing machine.",
        )
    )
    root.add_child(nav_parallel)

    root.add_child(
        _moveArmRetry(
            name="preparing to pick up clothing",
            arm_pose_key=KEY_ARM_PRE_PICK_CLOTHING,
            add_octomap=False,
        )
    )
    root.add_child(
        _moveArmRetry(
            name="Move arm to pick up clothing",
            arm_pose_key=KEY_ARM_PICK_CLOTHING,
            add_octomap=False,
        )
    )
    root.add_child(_gripperCloseSafe(name="Close gripper on clothing piece"))
    root.add_child(
        _moveArmRetry(
            name="Lift clothing piece",
            arm_pose_key=KEY_ARM_PRE_PICK_CLOTHING,
            add_octomap=False,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce clothing pickup",
            bb_source=None,
            message="Picked up one piece of clothing.",
        )
    )

    root.add_child(_gripperOpenSafe(name="Release clothing piece"))
    root.add_child(
        BtNode_Announce(
            name="Announce clothing release",
            bb_source=None,
            message="Released clothing piece into basket.",
        )
    )
    return root


def pickupLaundryBasket():
    root = py_trees.composites.Sequence(name="Pick up laundry basket", memory=True)

    root.add_child(
        _moveArmRetry(
            name="preparing to pick up basket",
            arm_pose_key=KEY_ARM_PRE_PICK_BASKET,
            add_octomap=False,
        )
    )
    root.add_child(
        _moveArmRetry(
            name="Move arm to pick up basket",
            arm_pose_key=KEY_ARM_PICK_BASKET,
            add_octomap=False,
        )
    )
    root.add_child(_gripperCloseSafe(name="Close gripper on basket"))
    root.add_child(
        _moveArmRetry(
            name="Lift basket",
            arm_pose_key=KEY_ARM_PRE_PICK_BASKET,
            add_octomap=False,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce basket pickup",
            bb_source=None,
            message="Picked up one piece of basket.",
        )
    )
    return root


def goAndPlaceBasket():
    root = py_trees.composites.Sequence(name="Go and place basket", memory=True)

    nav_parallel = py_trees.composites.Parallel(
        name="Navigate to basket table",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    nav_parallel.add_child(
        _gotoRetry(name="Navigate to basket table", pose_key=KEY_POSE_BASKET_TABLE)
    )
    nav_parallel.add_child(
        BtNode_Announce(
            name="Announce navigating to basket table",
            bb_source=None,
            message="Navigating to basket table.",
        )
    )
    root.add_child(nav_parallel)

    root.add_child(
        _moveArmRetry(
            name="Move arm to place basket",
            arm_pose_key=KEY_ARM_PICK_BASKET,
            add_octomap=False,
        )
    )
    root.add_child(_gripperOpenSafe(name="Release basket"))
    root.add_child(
        BtNode_Announce(
            name="Announce basket release",
            bb_source=None,
            message="Released basket at folding table.",
        )
    )

    root.add_child(
        _moveArmRetry(
            name="retract arm after placing basket",
            arm_pose_key=KEY_ARM_PRE_PICK_BASKET,
            add_octomap=False,
        )
    )
    return root


def foldClothingOnce():
    root = py_trees.composites.Sequence(name=f"Fold clothing", memory=True)
    root.add_child(
        _moveArmRetry(
            name="Stretch out to get clothing",
            arm_pose_key=KEY_ARM_FOLD_START,
            add_octomap=False,
        )
    )
    root.add_child(_gripperOpenSafe(name="Ensure gripper empty before folding"))
    root.add_child(
        BtNode_Announce(
            name="Announce folding start",
            bb_source=None,
            message=f"Start folding, please put a clothes in my gripper",
        )
    )
    root.add_child(
        py_trees.timers.Timer(name="Wait for clothing piece to fold", duration=5.0)
    )
    root.add_child(_gripperCloseSafe(name="Wait for clothing piece to fold"))
    root.add_child(
        _moveArmRetry(
            "Move arm to fold start pose",
            arm_pose_key=KEY_ARM_FOLD_START,
            add_octomap=False,
        )
    )
    root.add_child(
        BtNode_FoldClothing(  # TODO: implement the real BtNode_Fold
            name="Fold clothing piece",
            action_name=FOLD_ACTION_NAME,
        )
    )
    root.add_child(_gripperOpenSafe(name="Release folded clothing piece"))
    root.add_child(
        BtNode_Announce(
            name="Announce folding complete",
            bb_source=None,
            message=f"Please help me to fold the current cloth into half.",
        )
    )
    # TODO: maybe sleep 5 seconds here to allow for human-assisted folding, if we want to avoid a hard dependency on the fold action server being functional
    return root


def createDoingLaundry():
    root = py_trees.composites.Sequence(name="Doing Laundry", memory=True)
    root.add_child(createConstantWriter())

    start_parallel = py_trees.composites.Parallel(
        "Setup", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    start_parallel.add_child(_gripperOpenSafe(name="Ensure gripper starts open"))
    start_parallel.add_child(
        BtNode_Announce(
            name="Announce task start",
            bb_source=None,
            message="Starting laundry task! Navigating to laundry area.",
        )
    )
    root.add_child(start_parallel)

    # root.add_child(pickupOneClothing())
    # root.add_child(pickupLaundryBasket())
    # root.add_child(goAndPlaceBasket())

    # root.add_child(
    #     _moveArmRetry(
    #         name="Move arm to base moving",
    #         arm_pose_key=KEY_ARM_NAVIGATING,
    #         add_octomap=False,
    #     )
    # )
    # root.add_child(
    #     _gotoRetry(name="Navigate to laundry area", pose_key=KEY_POSE_FOLDING_TABLE)
    # )

    root.add_child(
        BtNode_Announce(
            name="Announce folding start",
            bb_source=None,
            message="Starting to fold clothing!",
        )
    )

    root.add_child(
        py_trees.decorators.Repeat(
            name="Repeat fold action",
            child=foldClothingOnce(),
            num_success=999,
        )
    )
    return root
