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
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import (
    BtNode_CheckIfEmpty,
    BtNode_WriteToBlackboard,
)
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_FoldClothingDn,
    BtNode_Grasp,
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction, BtNode_NavBack
from behavior_tree.TemplateNodes.OperatorGate import BtNode_PressEnterToSucceed
from behavior_tree.TemplateNodes.Vision import (
    BtNode_DoorDetection,
    BtNode_GetPointCloud,
    BtNode_ScanFor,
    BtNode_TurnPanTilt,
)
from behavior_tree.TemplateNodes.WaitKeyPress import BtNode_WaitKeyboardPress
from behavior_tree.visualization import create_post_tick_visualizer

from .config import (
    # ARM_POS_PICK_WASHER,
    # ARM_POS_PLACING,
    # ARM_POS_SCAN,
    # ARM_SERVICE_NAME,
    ARM_ACTION_NAME,
    ARM_POS_FOLD_START,
    ARM_POS_NAVIGATING,
    ARM_POS_ORBBEC_LOOK,
    ARM_POS_PICK_BASKET,
    ARM_POS_PICK_CLOTHING,
    ARM_POS_PRE_PICK_BASKET,
    ARM_POS_PRE_PICK_CLOTHING,
    # CLOTHING_SCAN_PROMPT,
    # DO_FOLD_PICKED_PIECES,
    # DO_PICK_FROM_BASKET,
    # DO_PICK_FROM_WASHER,
    # DO_REQUEST_WASHER_HELP,
    # FOLD_ACTION_NAME,  # no longer used — fold now uses BtNode_FoldClothingDn (fold_dn_action)
    # FOLD_CYCLES,
    # GRASP_ACTION_NAME,
    # GRASP_RETRY_LIMIT,
    KEY_ARM_FOLD_START,
    KEY_ARM_NAVIGATING,
    KEY_ARM_ORBBEC_LOOK,
    KEY_ARM_PICK_BASKET,
    KEY_ARM_PICK_CLOTHING,
    KEY_ARM_PRE_PICK_BASKET,
    KEY_ARM_PRE_PICK_CLOTHING,
    # KEY_ARM_PICK_WASHER,
    # KEY_ARM_PLACING,
    # KEY_ARM_SCAN,
    KEY_DOOR_STATUS,
    # KEY_ENV_POINTS,
    # KEY_FOLD_COUNT,
    KEY_FOLD_OUT_OF_RANGE,
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
    # POSE_LAUNDRY_AREA,
    POSE_BASKET_TABLE,
    # POSE_BASKET,
    POSE_FOLDING_TABLE,
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
        ("Write arm orbbec look", KEY_ARM_ORBBEC_LOOK, ARM_POS_ORBBEC_LOOK),
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
            ARM_POS_PRE_PICK_CLOTHING,
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
    # The fold_dn_action fold is vision-driven and fully autonomous: the server
    # moves to its own scan pose, detects the garment keypoints, and runs the
    # metric-offset folds, managing the arm AND gripper itself (returning to scan
    # when done). So the tree only needs to get a garment laid flat on the table
    # and then hand off to the action — no manual gripper/arm-pose steps here
    # (they would conflict with the server's own scan-pose + grasp/release).
    root = py_trees.composites.Sequence(name="Fold clothing", memory=True)
    # Split into two announcements: the first prompts the operator to look at
    # the screen, and its own speaking time gives them a moment to actually
    # look before the second announcement tells them to act.
    root.add_child(
        BtNode_Announce(
            name="Announce look at screen",
            bb_source=None,
            message="Dear referee, please look at the image on my screen for how to lay out the garment.",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce lay flat",
            bb_source=None,
            message="Now please help me pick up one piece of clothing and flatten it on the table.",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce lay flat",
            bb_source=None,
            message="I will wait for 10 seconds.",
        )
    )
    root.add_child(
        py_trees.timers.Timer(name="Wait for clothing to be laid flat", duration=10.0)
    )
    root.add_child(
        BtNode_Announce(
            name="Announce thank you",
            bb_source=None,
            message="Thank you.",
        )
    )

    # Fold, or (if the server reports the garment is out of reach) ask the
    # operator to move it closer so the enclosing Repeat re-attempts.
    fold_or_oor = py_trees.composites.Selector(
        name="Fold or handle out-of-range", memory=True
    )

    # Happy path: reset the OOR flag, fold, announce completion. The fold node
    # writes KEY_FOLD_OUT_OF_RANGE=True (and FAILs) if the target is too far.
    fold_once = py_trees.composites.Selector(
        name="complete fold no matter what", memory=True
    )

    fold_and_confirm = py_trees.composites.Sequence(name="Fold + confirm", memory=True)
    fold_and_confirm.add_child(
        BtNode_WriteToBlackboard(
            name="Reset out-of-range flag",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_FOLD_OUT_OF_RANGE,
            object=False,
        )
    )
    fold_and_confirm.add_child(
        BtNode_FoldClothingDn(
            name="Fold clothing piece (fold_dn_action)",
            return_to_scan=True,
            bb_key_out_of_range=KEY_FOLD_OUT_OF_RANGE,
        )
    )
    fold_and_confirm.add_child(
        BtNode_Announce(
            name="Announce folding complete",
            bb_source=None,
            message="Folding Complete. Please help me to move the folded clothes away.",
        )
    )

    fold_or_oor.add_child(fold_and_confirm)

    # Only reached when the fold above FAILed. CheckIfEmpty SUCCEEDS iff the flag
    # is truthy (out of range) -> announce and let the Selector succeed so the
    # Repeat retries. On any other fold failure the flag is False -> CheckIfEmpty
    # FAILs -> the Selector FAILs -> the task ends (unchanged behaviour).
    too_far = py_trees.composites.Sequence(
        name="Too far -> ask to reposition", memory=True
    )
    too_far.add_child(
        BtNode_CheckIfEmpty(
            name="Was fold out of range?",
            bb_source=KEY_FOLD_OUT_OF_RANGE,
        )
    )
    too_far.add_child(
        BtNode_Announce(
            name="Announce clothing too far",
            bb_source=None,
            message="Dear referee, the clothing is too far, please help me put the clothing closer.",
        )
    )
    too_far.add_child(
        BtNode_Announce(
            name="Announce clothing too far",
            bb_source=None,
            message="Waiting for 10 seconds.",
        )
    )
    too_far.add_child(
        py_trees.timers.Timer(name="Wait 15 s for referee help", duration=10.0)
    )
    too_far.add_child(
        BtNode_Announce(
            name="Announce clothing too far",
            bb_source=None,
            message="Thank you for pulling it closer",
        )
    )
    too_far.add_child(py_trees.behaviours.Failure("failure as guard"))

    fold_or_oor.add_child(too_far)

    fold_once.add_child(
        py_trees.decorators.Retry(
            name="retry three times", child=fold_or_oor, num_failures=3
        )
    )
    fold_once.add_child(
        BtNode_Announce(
            name="announce cannot complete folding",
            bb_source=None,
            message="I cannot complete folding, please fold the cloth and move it away.",
        )
    )
    root.add_child(fold_once)
    return py_trees.decorators.FailureIsSuccess(name="failure is success", child=root)


def createDoingLaundry():
    root = py_trees.composites.Sequence(name="Doing Laundry", memory=True)

    # Operator gate: nothing runs until a deliberate Enter (same gate as GPSR/HRI).
    root.add_child(BtNode_PressEnterToSucceed(name="Wait for operator to start"))

    root.add_child(createConstantWriter())

    start_parallel = py_trees.composites.Parallel(
        "Setup", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    start_parallel.add_child(_gripperOpenSafe(name="Ensure gripper starts open"))
    start_parallel.add_child(
        BtNode_Announce(
            name="Announce task start",
            bb_source=None,
            message="Starting laundry task!",
        )
    )
    # start_parallel.add_child(BtNode_TurnPanTilt("look at desktop"))
    root.add_child(start_parallel)

    # Door wait, mirroring Inspection: announce ready + aim head, then poll the
    # door_detection_srv until it reports open (FAILURE on closed keeps Retry going).
    ready = py_trees.composites.Parallel(
        name="Announce ready + aim pan-tilt",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    ready.add_child(
        BtNode_Announce(
            name="Announce ready for laundry",
            bb_source=None,
            message="I am ready for the laundry task, please open the door.",
        )
    )
    ready.add_child(BtNode_TurnPanTilt(name="Aim pan-tilt at door", x=0.0, y=45.0))
    root.add_child(ready)

    # Keep the arm out of the Orbbec head-camera's frame while it checks the door.
    root.add_child(
        _moveArmRetry(
            name="Move arm to orbbec-look (clear door cam)",
            arm_pose_key=KEY_ARM_ORBBEC_LOOK,
        )
    )

    root.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(
                name="Door detection", bb_door_state_key=KEY_DOOR_STATUS
            ),
            num_failures=999,
        )
    )

    root.add_child(
        BtNode_Announce(
            name="Announce door open",
            bb_source=None,
            message="The door is open. Heading to the washing machine.",
        )
    )

    # root.add_child(pickupOneClothing())
    # root.add_child(pickupLaundryBasket())
    # root.add_child(goAndPlaceBasket())

    root.add_child(
        _moveArmRetry(
            name="Move arm to base moving",
            arm_pose_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
        )
    )

    # Enter the arena through the doorway before making for any other target —
    # skipping straight to pose_folding_table risks clipping the doorframe/threshold.
    # root.add_child(
    #     _gotoRetry(name="Navigate to arena entry", pose_key=KEY_POSE_ARENA_ENTRY)
    # )

    # Referee-help stop: park at the washing machine and ask the referee to put
    # the clothes on the table before the robot heads for the folding table.
    root.add_child(
        _gotoRetry(
            name="Navigate to washing machine", pose_key=KEY_POSE_WASHING_MACHINE
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce referee help request",
            bb_source=None,
            message=("Dear Referee, I will fold the clothes one by one"),
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce referee help request",
            bb_source=None,
            message=(
                "Later, when I am ready to fold, I will request you to pick up a piece of clothing from the washing machine or basket."
            ),
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Announce referee help request",
            bb_source=None,
            message=("Thank you"),
        )
    )

    # Approach the folding table, then creep forward 0.1 m (NavBack distance<0
    # = forward per the fixed backward_action_server sign convention) to close
    # the Nav2 standoff gap. If the creep fails — most likely status=1
    # (nav_busy) because Nav2 hasn't fully released the base yet — redo the
    # whole goto+creep. Retry the loop up to 3 attempts.
    approach_loop = py_trees.composites.Sequence(
        name="Approach folding table (goto + creep)", memory=True
    )
    approach_loop.add_child(
        _gotoRetry(name="Navigate to laundry area", pose_key=KEY_POSE_FOLDING_TABLE)
    )
    approach_loop.add_child(
        BtNode_NavBack(
            "Getting closer to the table",
            bb_target_key=None,
            distance=-0.24,
            timeout_sec=5,
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry goto+NavBack approach (max 3)",
            child=approach_loop,
            num_failures=3,
        )
    )

    root.add_child(BtNode_TurnPanTilt("Looking at the desktop"))

    # Keep the arm out of the Orbbec head-camera's frame while it scans the table.
    root.add_child(
        _moveArmRetry(
            name="Move arm to orbbec-look (clear table cam)",
            arm_pose_key=KEY_ARM_ORBBEC_LOOK,
        )
    )

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


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=createDoingLaundry())
    tree.setup(timeout=15, node_name="doing_laundry")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="doing-laundry"
    )
    tree.tick_tock(period_ms=500.0, post_tick_handler=print_tree)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        shutdown_visualizer()
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
