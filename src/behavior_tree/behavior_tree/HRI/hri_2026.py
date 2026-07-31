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
from behavior_tree.TemplateNodes.OperatorGate import BtNode_PressEnterToSucceed
import py_trees
import py_trees_ros
import rclpy

import behavior_tree.HRI.hri as hri
from behavior_tree.HRI.config import (
    KEY_ARM_DROP,
    KEY_ARM_HANDOVER,
    KEY_ARM_NAVIGATING,
    KEY_PERSONS,
    KEY_PERSON_CENTROIDS,
    KEY_SOFA_POSE_REVERSED,
)
from behavior_tree.HRI.follow_real import createBagDropReal, createFollowHostUntilStop
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_GripperAction,
    BtNode_MoveArmSingle,
    BtNode_PointTo,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.visualization import create_post_tick_visualizer


def createBagFlowReal2026():
    """Real bag flow: handover -> real follow-host (termination-gated) -> real drop.

    Drop-in replacement for ``hri.createBagFlow``. Mirrors the canonical
    handover (arm to handover pose aimed at guest 2 -> gripper open -> ask ->
    wait -> close -> arm to nav pose with bag), then runs the REAL follow
    process until it is termination-gated by the nav person-stationary verdict
    + spoken confirmation, then the real arm drop on HRI's configured drop pose.
    """
    root = py_trees.composites.Sequence(
        name="HRI bag flow (real follow, 2026)", memory=True
    )

    # --- handover (mirrors hri.createBagFlow, handover pose aimed at guest 2) ---
    # Joints 1-6 come from KEY_ARM_HANDOVER; joint0 is recomputed to point at
    # guest 2 (KEY_PERSONS layout [host, guest1, guest2]) with the same bearing
    # math the two-way introduction uses to point at people (pan_bias=0.0 —
    # centroids are correct in base_link). The centroids are still fresh here:
    # they were scanned by the intro's feature matching and the base does not
    # move again until the turn-around AFTER the bag is grasped. If the aim is
    # impossible (centroid missing, arm refuses), fall back to the canonical
    # fixed handover pose; if that fails too, continue — an arm refusal must
    # not forfeit the gripper handover + follow-to-drop scoring.
    arm_to_handover = py_trees.composites.Selector(
        name="Arm to bag-handover pose", memory=True
    )
    arm_to_handover.add_child(
        py_trees.decorators.Retry(
            name="Retry handover pose at guest2",
            child=BtNode_PointTo(
                name="Handover pose pointing at guest2",
                bb_key_persons=KEY_PERSONS,
                bb_key_points=KEY_PERSON_CENTROIDS,
                bb_key_init_pose=KEY_ARM_HANDOVER,
                target_id=2,
                pan_bias=0.0,
            ),
            num_failures=3,
        )
    )
    arm_to_handover.add_child(
        py_trees.decorators.Retry(
            name="Retry fixed handover pose",
            child=BtNode_MoveArmSingle(
                name="Move arm to handover pose (fixed)",
                service_name="arm_joint_service",
                arm_pose_bb_key=KEY_ARM_HANDOVER,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    parallel_handover_pose_announce = py_trees.composites.Parallel(
        name="Arm to handover pose + announce ready",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_handover_pose_announce.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Arm to bag-handover pose (best effort)",
            child=arm_to_handover,
        )
    )
    parallel_handover_pose_announce.add_child(
        BtNode_Announce(
            name="Announce ready for bag",
            bb_source=None,
            message="I am ready to take your bag.",
        )
    )
    root.add_child(parallel_handover_pose_announce)
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
        py_trees.timers.Timer(name="Wait for bag placement", duration=5.0)
    )
    root.add_child(
        BtNode_Announce(
            name="Ask for bag handover",
            bb_source=None,
            message="I will be closing my gripper. Please be careful"
        )
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

    # Turn 180 deg in place: same map point as the sofa waypoint, heading
    # flipped (POSE_SOFA_REVERSED), so the robot faces away from the sofa
    # and the host can stand in front of it for the follow. Best-effort:
    # a nav refusal must not forfeit the follow-to-drop (200) scoring.
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Turn around at sofa (best effort)",
            child=py_trees.decorators.Retry(
                name="Retry turn around at sofa",
                child=BtNode_GotoAction(
                    name="Turn around at sofa",
                    key=KEY_SOFA_POSE_REVERSED,
                ),
                num_failures=3,
            ),
        )
    )

    # --- real follow host until the host signals to stop ---
    root.add_child(BtNode_TurnPanTilt(name=f"Look at host", x=0.0, y=35.0, speed=0.0))
    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="Dear host, please stand in front of me now. I will carry the bag and follow you.",
        )
    )

    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="Waiting.",
        )
    )

    root.add_child(
        BtNode_Announce(
            name="Follow host announcement",
            bb_source=None,
            message="After this announcement, I will start following you",
        )
    )

    root.add_child(
        createFollowHostUntilStop(operator_stop_key="s")
    )

    # --- real drop on HRI's configured drop pose ---
    root.add_child(
        createBagDropReal(arm_drop_key=KEY_ARM_DROP, arm_nav_key=KEY_ARM_NAVIGATING)
    )
    return root


def createHRITask2026() -> py_trees.behaviour.Behaviour:
    """Canonical ``createHRITask`` with the stubbed bag flow swapped for the real one."""
    root = py_trees.composites.Sequence(name="HRI Task 2026", memory=True)
    root.add_child(BtNode_PressEnterToSucceed(name="Wait for operator to start"))
    root.add_child(hri.createConstantWriter())
    root.add_child(
        BtNode_Announce(
            name="HRI start announcement",
            bb_source=None,
            message="HRI task started.",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Host seating instruction",
            bb_source=None,
            message=(
                "Dear host, please sit down on the sofa."
            ),
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
