from __future__ import annotations
from re import M

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_FoldClothing,
    BtNode_GripperAction,
    BtNode_Grasp,
)
# from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_GetPointCloud,
    BtNode_ScanFor,
    BtNode_TurnPanTilt,
    BtNode_ScanForGeneralist,
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
    ARM_ACTION_NAME,
    CLOTHING_SCAN_PROMPT,
    FOLD_ACTION_NAME,
    KEY_ARM_FOLD_START,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PICK_BASKET,
    KEY_ARM_PRE_PICK_BASKET,
    KEY_ARM_PICK_CLOTHING,
    KEY_ARM_PRE_PICK_CLOTHING,
    KEY_POSE_ARENA_ENTRY,
    KEY_POSE_BASKET_TABLE,
    KEY_POSE_FOLDING_TABLE,
    KEY_POSE_WASHING_MACHINE,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    NAV_RETRY_LIMIT,
    POSE_ARENA_ENTRY,
    POSE_FOLDING_TABLE,
    POSE_BASKET_TABLE,
    POSE_WASHING_MACHINE,
    TARGET_FRAME,
    SCAN_RETRY_LIMIT,
    GRASP_ACTION_NAME,
)

BASKET_SCAN_PROMPT = "laundry basket"


def createConstantWriter() -> py_trees.composites.Parallel:
    root = py_trees.composites.Parallel(
        name="Write DoingLaundry constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write arena entry pose", KEY_POSE_ARENA_ENTRY, POSE_ARENA_ENTRY),
        ("Write washing machine pose", KEY_POSE_WASHING_MACHINE, POSE_WASHING_MACHINE),
        ("Write basket table pose", KEY_POSE_BASKET_TABLE, POSE_BASKET_TABLE),
        ("Write folding table pose", KEY_POSE_FOLDING_TABLE, POSE_FOLDING_TABLE),
        ("Write arm navigating", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm pick basket", KEY_ARM_PICK_BASKET, ARM_POS_PICK_BASKET),
        ("Write arm fold start", KEY_ARM_FOLD_START, ARM_POS_FOLD_START),
        ("Write arm pre pick basket", KEY_ARM_PRE_PICK_BASKET, ARM_POS_PRE_PICK_BASKET),
        ("Write arm pick clothing", KEY_ARM_PICK_CLOTHING, ARM_POS_PICK_CLOTHING),
        (
            "Write arm pre pick clothing",
            KEY_ARM_PRE_PICK_CLOTHING,
            ARM_POS_PRE_PICK_CLOTHING,
        ),
        ("Write target frame", KEY_TARGET_FRAME, TARGET_FRAME),
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


def _scanRetry(name: str, bb_key: str, object: str, *, retries: int = SCAN_RETRY_LIMIT):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_ScanForGeneralist(
            name=name,
            bb_key=bb_key,
            object=object,
            bb_source=None,
        ),
        num_failures=retries,
    )


def _graspRetry(
    name: str,
    bb_key_vision_res: str,
    bb_key_object_label: str,
    *,
    stay: bool = False,
    use_mesh: bool = True,
    retries: int = 2,
):
    return py_trees.decorators.Retry(
        name=f"Retry {name}",
        child=BtNode_Grasp(
            name=name,
            action_name=GRASP_ACTION_NAME,
            bb_key_vision_res=bb_key_vision_res,
            bb_key_object_label=bb_key_object_label,
            use_mesh=use_mesh,
            stay=stay,
        ),
        num_failures=retries,
    )


# def _gotoRetry(name: str, pose_key: str, *, retries: int = NAV_RETRY_LIMIT):
#     return py_trees.decorators.Retry(
#         name=f"Retry {name}",
#         child=BtNode_GotoAction(
#             name=name,
#             key=pose_key,
#         ),
#         num_failures=retries,
#     )


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


def pickupOneClothing(nav: bool = False):
    root = py_trees.composites.Sequence(name="Pick up one clothing piece", memory=True)
    root.add_child(
        _moveArmRetry(
            name="Move arm to base moving",
            arm_pose_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
        )
    )

    # if nav:
    #     nav_parallel = py_trees.composites.Parallel(
    #         name="Navigate to washing machine",
    #         policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    #     )
    #     nav_parallel.add_child(
    #         _gotoRetry(
    #             name="Navigate to washing machine", pose_key=KEY_POSE_WASHING_MACHINE
    #         )
    #     )
    #     nav_parallel.add_child(
    #         BtNode_Announce(
    #             name="Announce navigating to washing machine",
    #             bb_source=None,
    #             message="Navigating to washing machine.",
    #         )
    #     )
    #     root.add_child(nav_parallel)

    root.add_child(
        _moveArmRetry(
            name="preparing to pick up clothing",
            arm_pose_key=KEY_ARM_PRE_PICK_CLOTHING,
            add_octomap=False,
        )
    )
    # root.add_child(
    #     _moveArmRetry(
    #         name="Move arm to pick up clothing",
    #         arm_pose_key=KEY_ARM_PICK_CLOTHING,
    #         add_octomap=False,
    #     )
    # )
    # root.add_child(_gripperCloseSafe(name="Close gripper on clothing piece"))
    root.add_child(
        BtNode_Announce(
            name="Announce scanning for clothing",
            bb_source=None,
            message="Scanning for clothing piece in the washing machine.",
        )
    )
    root.add_child(
        _scanRetry(
            name="Scan for clothing piece in washing machine",
            bb_key=KEY_VISION_RESULT,
            object=CLOTHING_SCAN_PROMPT,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce grasping clothing",
            bb_source=None,
            message="Grasping clothing piece in the washing machine.",
        )
    )
    root.add_child(
        _graspRetry(
            name="Grasp clothing piece in washer",
            bb_key_vision_res=KEY_VISION_RESULT,
            bb_key_object_label=CLOTHING_SCAN_PROMPT,
            retries=3,
            use_mesh=True,
            stay=True,
        )
    )
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
        BtNode_Announce(
            name="Announce scanning for basket",
            bb_source=None,
            message="Scanning for basket in the washing machine.",
        )
    )
    root.add_child(
        _scanRetry(
            name="Scan for basket in washing machine",
            bb_key=KEY_VISION_RESULT,
            object=BASKET_SCAN_PROMPT,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce grasping basket",
            bb_source=None,
            message="Grasping basket in the washing machine.",
        )
    )
    root.add_child(
        _graspRetry(
            name="Grasp basket in washer",
            bb_key_vision_res=KEY_VISION_RESULT,
            bb_key_object_label=BASKET_SCAN_PROMPT,
            retries=3,
            use_mesh=True,
            stay=True,
        )
    )
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


def goAndPlaceBasket(nav: bool = False):
    root = py_trees.composites.Sequence(name="Go and place basket", memory=True)

    # if nav:
    #     nav_parallel = py_trees.composites.Parallel(
    #         name="Navigate to basket table",
    #         policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    #     )
    #     nav_parallel.add_child(
    #         _gotoRetry(name="Navigate to basket table", pose_key=KEY_POSE_BASKET_TABLE)
    #     )
    #     nav_parallel.add_child(
    #         BtNode_Announce(
    #             name="Announce navigating to basket table",
    #             bb_source=None,
    #             message="Navigating to basket table.",
    #         )
    #     )
    #     root.add_child(nav_parallel)

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
            arm_pose_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
        )
    )
    root.add_child(_gripperOpenSafe(name="Ensure gripper empty before folding"))
    root.add_child(
        BtNode_Announce(
            name="Announce folding start",
            bb_source=None,
            message=f"Start folding, please put a clothes in my gripper, it might be on the floor or table.",
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
        BtNode_FoldClothing(
            name="Fold clothing piece",
            action_name=FOLD_ACTION_NAME,
        )
    )
    root.add_child(_gripperOpenSafe(name="Release folded clothing piece"))
    root.add_child(
        BtNode_Announce(
            name="Announce folding complete",
            bb_source=None,
            message=f"Please help me to fold the current cloth into half and move away the clothes on table.",
        )
    )
    return root


def _buildSamplingTree(task: str, nav: bool) -> py_trees.composites.Sequence:
    root = py_trees.composites.Sequence(name="Doing Laundry Behavior Tree", memory=True)
    root.add_child(createConstantWriter())

    if task == "clothing":
        root.add_child(pickupOneClothing(nav=nav))
    elif task == "basket":
        root.add_child(pickupLaundryBasket())
        root.add_child(goAndPlaceBasket(nav=nav))
    elif task == "folding":
        root.add_child(
            py_trees.decorators.Repeat(
                name="Repeat fold action",
                child=foldClothingOnce(),
                num_success=999,
            )
        )
    else:
        raise ValueError(f"Unknown sampling task: {task!r}")
    return root


def main(args=None):
    import argparse
    import sys

    from behavior_tree.runtime import run_tree

    parser = argparse.ArgumentParser(description="Doing Laundry sampling tree")
    parser.add_argument(
        "--task",
        type=str,
        default="clothing",
        choices=["clothing", "basket", "folding"],
        help="Which sampling subtree to run (default: clothing).",
    )
    parser.add_argument(
        "--nav",
        action="store_false",
        help="Include navigation steps (skipped by default for bench testing).",
    )
    parsed, _ = parser.parse_known_args(sys.argv[1:] if args is None else args)

    def factory():
        return _buildSamplingTree(task=parsed.task, nav=parsed.nav)

    run_tree(factory, period_ms=500.0, title=f"Doing Laundry — {parsed.task}")


if __name__ == "__main__":
    main()
