from __future__ import annotations

"""Doing Laundry — finalized 2026 task tree (entry: ``doing-laundry-2026``).

A NEW parallel production tree that does not touch the canonical
``createDoingLaundry`` (``laundry.py`` / ``doing-laundry``). It composes the
existing laundry helpers but swaps the two stubbed behaviours the canonical
tree still ships, and wires the state machinery ``laundry.py`` imports but never
uses:

  * **Blind open-loop pick -> closed-loop perception pick.** The canonical
    ``pickupOneClothing`` moves the arm to fixed poses, closes the gripper, then
    immediately releases (no perception). Here we ``_scanRetry`` (generalist
    detector) then ``_graspRetry`` (AnyGrasp) the detected clothing item, then
    place it on the folding table.
  * **Mis-wired fold -> real ``FoldClothingAction``.** The canonical
    ``foldClothingOnce`` calls ``BtNode_FoldClothing`` which sends an EMPTY
    ``Fold.Goal()`` to the old ``fold_action``. Here the fold is a
    ``Selector[ real BtNode_FoldClothingAction @ fold_clothing_action,
    operator-assist fallback ]`` — the real garment-folding action
    (``arm_api/fold/fold_clothing_server.py``) with a graceful operator fallback
    if it aborts.
  * **State machinery wired.** ``BtNode_InitTaskState`` /
    ``BtNode_TimeoutCutoverChecker`` (replaces the naive ``Repeat(999)``) /
    ``BtNode_IncrementCounter`` / ``BtNode_BuildCompletionSummary``.

Run::

    ros2 run behavior_tree doing-laundry-2026

Fully offline (no servers, auto-advance)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree doing-laundry-2026
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_WaitForStart
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.FoldClothingAction import BtNode_FoldClothingAction
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction  # noqa: F401 (parity)
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection
from behavior_tree.visualization import create_post_tick_visualizer

# Reuse the canonical laundry helpers (do NOT re-derive them).
# NOTE: we deliberately do NOT reuse laundry.createConstantWriter — it has a
# pre-existing malformed 4-tuple in its `writes` list (the KEY_ARM_PRE_PICK_CLOTHING
# entry) that raises `ValueError: too many values to unpack` and crashes the
# canonical `doing-laundry` tree at construction. v2 writes its own constants.
from .laundry import (
    _gotoRetry,
    _gripperOpenSafe,
    _moveArmRetry,
)
from .sampling import _graspRetry, _scanRetry
from .state_nodes import (
    BtNode_BuildCompletionSummary,
    BtNode_IncrementCounter,
    BtNode_InitTaskState,
    BtNode_TimeoutCutoverChecker,
)

from .config import (
    ARM_POS_FOLD_START,
    ARM_POS_NAVIGATING,
    ARM_POS_PLACING,
    CLOTHING_SCAN_PROMPT,
    KEY_ARM_FOLD_START,
    KEY_ARM_NAVIGATING,
    KEY_ARM_PLACING,
    KEY_DOOR_STATUS,
    KEY_FOLD_COUNT,
    KEY_MAX_RUNTIME,
    KEY_PHASE_DEADLINE,
    KEY_POSE_FOLDING_TABLE,
    KEY_POSE_LAUNDRY_AREA,
    KEY_POSE_WASHING_MACHINE,
    KEY_SCORE_TRACE,
    KEY_STACK_COUNT,
    KEY_SUMMARY_MESSAGE,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    MAX_RUNTIME_SEC,
    OBJECT_LABEL_CLOTHING,
    POSE_FOLDING_TABLE,
    POSE_LAUNDRY_AREA,
    POSE_WASHING_MACHINE,
    TARGET_FRAME,
)


def _constantWriter() -> py_trees.composites.Parallel:
    """Write exactly the constants the v2 tree reads (self-contained, correct).

    Replaces ``laundry.createConstantWriter`` (which is broken — see the import
    note). Only the keys v2 actually consumes are written.
    """
    root = py_trees.composites.Parallel(
        name="Write DoingLaundry v2 constants",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    writes = [
        ("Write target frame", KEY_TARGET_FRAME, TARGET_FRAME),
        ("Write arm navigating pose", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm fold-start pose", KEY_ARM_FOLD_START, ARM_POS_FOLD_START),
        ("Write arm placing pose", KEY_ARM_PLACING, ARM_POS_PLACING),
        ("Write washing machine pose", KEY_POSE_WASHING_MACHINE, POSE_WASHING_MACHINE),
        ("Write folding table pose", KEY_POSE_FOLDING_TABLE, POSE_FOLDING_TABLE),
        ("Write laundry area pose", KEY_POSE_LAUNDRY_AREA, POSE_LAUNDRY_AREA),
        ("Write max runtime", KEY_MAX_RUNTIME, MAX_RUNTIME_SEC),
    ]
    for name, key, value in writes:
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name, bb_namespace="", bb_source=None, bb_key=key, object=value
            )
        )
    return root


def perceptionPickAndPlace() -> py_trees.composites.Sequence:
    """Closed-loop pick (scan -> grasp) + place on the folding table.

    Replaces the canonical blind ``pickupOneClothing``. The grasp is best-effort
    (``FailureIsSuccess``) so a perception miss does not kill the run — the fold
    loop's operator-assist fallback still scores.
    """
    root = py_trees.composites.Sequence(name="Perception pick and place", memory=True)

    root.add_child(
        _moveArmRetry(name="Move arm to base moving", arm_pose_key=KEY_ARM_NAVIGATING)
    )

    nav = py_trees.composites.Parallel(
        name="Navigate to washing machine",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    nav.add_child(
        _gotoRetry(name="Navigate to washing machine", pose_key=KEY_POSE_WASHING_MACHINE)
    )
    nav.add_child(
        BtNode_Announce(
            name="Announce navigating to washing machine",
            bb_source=None,
            message="Navigating to the washing machine to collect clothing.",
        )
    )
    root.add_child(nav)

    # Perceive a clothing item, writing the detection to KEY_VISION_RESULT.
    root.add_child(
        _scanRetry(
            name="Scan for clothing piece",
            bb_key=KEY_VISION_RESULT,
            object=CLOTHING_SCAN_PROMPT,
        )
    )
    # Grasp it (best-effort).
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Grasp clothing (best effort)",
            child=_graspRetry(
                name="Grasp clothing piece",
                bb_key_vision_res=KEY_VISION_RESULT,
                bb_key_object_label=CLOTHING_SCAN_PROMPT,
                retries=3,
                use_mesh=True,
                stay=True,
            ),
        )
    )

    nav2 = py_trees.composites.Parallel(
        name="Navigate to folding table",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    nav2.add_child(
        _gotoRetry(name="Navigate to folding table", pose_key=KEY_POSE_FOLDING_TABLE)
    )
    nav2.add_child(
        BtNode_Announce(
            name="Announce navigating to folding table",
            bb_source=None,
            message="Navigating to the folding table.",
        )
    )
    root.add_child(nav2)

    root.add_child(
        _moveArmRetry(name="Move arm to placing pose", arm_pose_key=KEY_ARM_PLACING)
    )
    root.add_child(_gripperOpenSafe(name="Release clothing on folding table"))
    root.add_child(
        BtNode_Announce(
            name="Announce clothing placed",
            bb_source=None,
            message="Placed the clothing on the folding table.",
        )
    )
    return root


def _realFoldOnce() -> py_trees.composites.Sequence:
    """Autonomous fold via the real ``fold_clothing_action`` server."""
    seq = py_trees.composites.Sequence(name="Fold autonomously", memory=True)
    seq.add_child(
        _moveArmRetry(name="Move arm to fold start pose", arm_pose_key=KEY_ARM_FOLD_START)
    )
    seq.add_child(
        BtNode_Announce(
            name="Announce autonomous fold",
            bb_source=None,
            message="I will fold this item now.",
        )
    )
    seq.add_child(
        BtNode_FoldClothingAction(
            name="Fold clothing (FoldClothing action)",
            garment_label=OBJECT_LABEL_CLOTHING,
            bottom_fold_mode=0,
            return_to_scan=True,
        )
    )
    seq.add_child(_gripperOpenSafe(name="Release folded clothing"))
    return seq


def _operatorAssistFold() -> py_trees.composites.Sequence:
    """Fallback when the real fold is unavailable / aborts (penalty-incurring)."""
    seq = py_trees.composites.Sequence(name="Operator-assisted fold", memory=True)
    seq.add_child(_gripperOpenSafe(name="Ensure gripper empty for operator fold"))
    seq.add_child(
        BtNode_Announce(
            name="Announce operator fold",
            bb_source=None,
            message=(
                "I could not fold this autonomously. Please help me fold this "
                "item and clear the table."
            ),
        )
    )
    seq.add_child(
        py_trees.timers.Timer(name="Wait for operator-assisted fold", duration=5.0)
    )
    return seq


def _guardedFoldIteration() -> py_trees.composites.Sequence:
    """One fold cycle, gated by the runtime budget and counted on success."""
    seq = py_trees.composites.Sequence(name="Fold one item (guarded)", memory=True)
    seq.add_child(
        BtNode_TimeoutCutoverChecker(
            name="Check runtime budget", phase_deadline_key=KEY_PHASE_DEADLINE
        )
    )
    fold = py_trees.composites.Selector(
        name="Fold with operator fallback", memory=True
    )
    fold.add_child(_realFoldOnce())
    fold.add_child(_operatorAssistFold())
    seq.add_child(fold)
    seq.add_child(
        BtNode_IncrementCounter(name="Increment fold count", counter_key=KEY_FOLD_COUNT)
    )
    return seq


def createDoingLaundry2026() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Doing Laundry 2026", memory=True)

    root.add_child(_constantWriter())
    root.add_child(
        BtNode_InitTaskState(
            name="Init task state",
            score_trace_key=KEY_SCORE_TRACE,
            phase_deadline_key=KEY_PHASE_DEADLINE,
            max_runtime_key=KEY_MAX_RUNTIME,
            fold_count_key=KEY_FOLD_COUNT,
            stack_count_key=KEY_STACK_COUNT,
        )
    )

    setup = py_trees.composites.Parallel(
        name="Setup", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    setup.add_child(_gripperOpenSafe(name="Ensure gripper starts open"))
    setup.add_child(
        BtNode_Announce(
            name="Announce task start",
            bb_source=None,
            message="Starting the laundry task.",
        )
    )
    root.add_child(setup)

    # Locate: wait for start, check the arena door (best-effort), drive to the area.
    root.add_child(BtNode_WaitForStart(name="Wait for start signal"))
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Door check (best effort)",
            child=BtNode_DoorDetection(
                name="Detect arena door", bb_door_state_key=KEY_DOOR_STATUS
            ),
        )
    )
    root.add_child(
        _gotoRetry(name="Navigate to laundry area", pose_key=KEY_POSE_LAUNDRY_AREA)
    )

    # Closed-loop pick + place.
    root.add_child(perceptionPickAndPlace())

    # Fold loop until the runtime budget is exhausted.
    root.add_child(
        BtNode_Announce(
            name="Announce folding start",
            bb_source=None,
            message="I will now fold the clothing.",
        )
    )
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Fold until deadline",
            child=py_trees.decorators.Repeat(
                name="Repeat guarded fold",
                child=_guardedFoldIteration(),
                num_success=999,
            ),
        )
    )

    # Completion summary.
    root.add_child(
        BtNode_BuildCompletionSummary(
            name="Build completion summary",
            score_trace_key=KEY_SCORE_TRACE,
            fold_count_key=KEY_FOLD_COUNT,
            stack_count_key=KEY_STACK_COUNT,
            summary_key=KEY_SUMMARY_MESSAGE,
        )
    )
    root.add_child(
        BtNode_Announce(name="Announce summary", bb_source=KEY_SUMMARY_MESSAGE)
    )
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Alias for the offline smoke harness."""
    return createDoingLaundry2026()


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=createDoingLaundry2026())
    tree.setup(timeout=15, node_name="doing_laundry_2026")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="doing-laundry-2026"
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
