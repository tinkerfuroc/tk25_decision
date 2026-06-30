from __future__ import annotations

"""HRI — finalized 2026 task tree (entry: ``hri-2026``).

A NEW parallel production tree that does not touch the canonical ``createHRITask``
(``hri.py`` / ``hri``). It reuses every HRI phase factory unchanged and swaps ONE
phase: the stubbed bag flow.

The canonical ``createHRITask`` ends its bag phase with ``createBagFlow()``, whose
follow-host stage is four ``BtNode_Announce("Attempting to follow")`` leaves with
the real follow + drop commented out (``# root.add_child(createFollowPerson(...))``).
Here that phase is replaced with the **real follow process**: handover ->
``createFollowHostUntilStop`` (the standalone ``FollowPerson`` tree —
``BtNode_TrackPersonAction`` @ ``/track_person`` + ``BtNode_FollowAction`` @
``follow_server`` + reacq/recovery — under a termination gate so it can end) ->
``createBagDropReal`` (real arm drop), wired to HRI's real ``KEY_ARM_DROP`` /
``KEY_ARM_NAVIGATING`` poses (seeded by ``createConstantWriter``). This restores
the follow-to-drop (200) + drop (50) scoring the stub forfeits.

Everything else — constant writer, arrival trigger, two guest intakes + seatings,
two-way introduction — is the canonical factory, imported unchanged.

Run::

    ros2 run behavior_tree hri-2026

Fully offline (no servers, auto-advance)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree hri-2026
"""

from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
import py_trees
import py_trees_ros
import rclpy

import behavior_tree.HRI.hri as hri
from behavior_tree.HRI.config import KEY_ARM_DROP, KEY_ARM_NAVIGATING
from behavior_tree.HRI.follow_real import createBagDropReal, createFollowHostUntilStop
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
)
from behavior_tree.visualization import create_post_tick_visualizer


def createBagFlowReal2026():
    """Real bag flow: handover -> real follow-host (termination-gated) -> real drop.

    Drop-in replacement for ``hri.createBagFlow``. Mirrors the canonical
    handover (gripper open -> ask -> wait -> close -> arm to nav pose with bag),
    then runs the REAL follow process until it is termination-gated by the nav
    person-stationary verdict + spoken confirmation, then the real arm drop on
    HRI's configured drop pose.
    """
    root = py_trees.composites.Sequence(
        name="HRI bag flow (real follow, 2026)", memory=True
    )

    # --- handover (mirrors hri.createBagFlow) ---
    root.add_child(
        BtNode_Announce(
            name="Announce ready for bag",
            bb_source=None,
            message="I am ready to take your bag.",
        )
    )
    root.add_child(
        BtNode_GripperAction(name="Open gripper for bag", open_gripper=True)
    )
    root.add_child(
        BtNode_Announce(
            name="Ask for bag handover",
            bb_source=None,
            message="Please place your bag in my gripper.",
        )
    )
    root.add_child(
        py_trees.timers.Timer(name="Wait for bag placement", duration=3.0)
    )
    root.add_child(
        BtNode_GripperAction(name="Close gripper with bag", open_gripper=False)
    )
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="arm to nav pose with bag (best effort)",
            child=py_trees.decorators.Retry(
                name="Retry arm to nav pose with bag",
                child=BtNode_MoveArmSingle(
                    name="Move arm to navigation pose with bag",
                    service_name="arm_joint_service",
                    arm_pose_bb_key=KEY_ARM_NAVIGATING,
                    add_octomap=False,
                ),
                num_failures=3,
            ),
        )
    )

    # --- real follow host until the host signals to stop ---
    root.add_child(BtNode_TurnPanTilt(name=f"Look at host", x=0.0, y=45.0, speed=0.0))
    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="Stand in front of me please. I'll follow you and carry the bag.",
        )
    )

    root.add_child(
        createFollowHostUntilStop()
    )

    # --- real drop on HRI's configured drop pose ---
    root.add_child(
        createBagDropReal(arm_drop_key=KEY_ARM_DROP, arm_nav_key=KEY_ARM_NAVIGATING)
    )
    return root


def createHRITask2026() -> py_trees.behaviour.Behaviour:
    """Canonical ``createHRITask`` with the stubbed bag flow swapped for the real one."""
    root = py_trees.composites.Sequence(name="HRI Task 2026", memory=True)
    root.add_child(hri.createConstantWriter())
    root.add_child(
        BtNode_Announce(
            name="HRI start announcement",
            bb_source=None,
            message="HRI task started.",
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry arm nav pose at start",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
            num_failures=2,
        )
    )
    root.add_child(hri.createWriteHostInfo())
    root.add_child(hri.createArrivalTrigger())
    root.add_child(hri.createGuestIntake(1))
    root.add_child(hri.createEscortAndSeat(1))
    root.add_child(hri.createArrivalTrigger())
    root.add_child(hri.createGuestIntake(2))
    root.add_child(hri.createEscortAndSeat(2))
    root.add_child(hri.createTwoWayIntroduction())
    root.add_child(createBagFlowReal2026())  # <-- the only change vs createHRITask
    root.add_child(
        BtNode_Announce(
            name="HRI completion announcement",
            bb_source=None,
            message="HRI task complete.",
        )
    )
    root.add_child(py_trees.behaviours.Running("running..."))
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Alias for the offline smoke harness."""
    return createHRITask2026()


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=createHRITask2026())
    tree.setup(timeout=15, node_name="hri_2026")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(title="hri-2026")
    tree.tick_tock(period_ms=200.0, post_tick_handler=print_tree)
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
