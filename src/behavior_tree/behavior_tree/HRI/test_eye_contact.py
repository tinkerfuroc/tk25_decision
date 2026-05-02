"""Tiered isolation harness for the HRI MaintainEyeContact bug.

User confirmed Tier 1 (TurnPanTilt → MaintainEyeContact in a Sequence)
tracks correctly and raw `ros2 action send_goal /follow_head_action` also
works. HRI's actual structure wraps MaintainEyeContact in a
``Parallel(SuccessOnSelected=[announce-side])`` so the gaze leaf is
cancelled when the announce sibling completes. The tiers below add
HRI-shape pieces one at a time so we can pinpoint which addition kills
tracking.

Run a specific tier:

    ros2 run behavior_tree test-eye-contact            # tier 1 (default)
    ros2 run behavior_tree test-eye-contact --tier 2
    ros2 run behavior_tree test-eye-contact --tier 3

| Tier | Structure | Tests |
|------|-----------|-------|
| 1 | ``Sequence(TurnPanTilt, MaintainEyeContact)`` | baseline (known good) |
| 2 | ``Sequence(TurnPanTilt, Parallel(SuccessOnSelected=[Sleep10s], MaintainEyeContact))`` | py_trees parallel + cancel-on-selected, no ROS announce |
| 3 | ``Sequence(TurnPanTilt, Parallel(SuccessOnSelected=[Sequence(Sleep2s, BtNode_Announce, Sleep3s)], MaintainEyeContact))`` | adds the real Announce service call (mimics HRI) |
| 4 | Tier 1 + 3 decoy ``BtNode_MaintainEyeContact`` instances created at ``tree.setup()`` (same node, same action server). Only the active one ticks. | tests the rclpy multi-client race (ros2/rclpy#1123) — does having 4 ActionClients on one node, all bound to ``/follow_head_action``, break the active client? |
| 5 | ``Sequence(TurnPanTilt(0,20), TurnPanTilt(0,35), TurnPanTilt(0,20), TurnPanTilt(90,45), MaintainEyeContact)`` | multi-step pre-aim like HRI's "look at host → look forward → look at sofa → look at guest". Tests if servo state from a chain of prior commands breaks follow_head's first-detection path. |

Reading the result:

* Tier 2 reproduces "stationary"  → parallel + cancel race is the bug.
* Tier 2 fine, Tier 3 reproduces  → ``BtNode_Announce`` (TTS) is starving
  the BT executor / cancel path.
* Both fine                       → the bug is from something earlier in
  HRI's run (prior TurnPanTilt commands, FeatureMatching, FeatureExtraction,
  PointTo arm motion, etc.). A Tier 4 with PointTo can be added next.

Stop with ``Ctrl+C``. Tail ``follow_head`` stderr in another shell while
the parallel window is open to read the ``[follow_head perf …]`` summary
and see which ``_perf_early`` key dominates when the head sits still.
"""

import argparse
import sys
import time

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Vision import (
    BtNode_MaintainEyeContact,
    BtNode_TurnPanTilt,
)


class SleepNode(py_trees.behaviour.Behaviour):
    """Pure-py_trees timer: returns RUNNING for `duration` seconds, then SUCCESS.

    No ROS dependencies. Used to drive the parallel's selected child without
    introducing TTS or service-call side effects.
    """

    def __init__(self, name: str, duration: float):
        super().__init__(name)
        self.duration = float(duration)
        self._start = None

    def initialise(self) -> None:
        self._start = time.time()

    def update(self) -> py_trees.common.Status:
        if self._start is None:
            return py_trees.common.Status.RUNNING
        if (time.time() - self._start) >= self.duration:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


def _pre_aim() -> BtNode_TurnPanTilt:
    return BtNode_TurnPanTilt(
        name="Pre-aim right (pan=90, tilt=45)",
        x=90.0,
        y=45.0,
        speed=0.0,
    )


def build_tier1() -> py_trees.behaviour.Behaviour:
    """Baseline: bare gaze leaf in a sequence."""
    root = py_trees.composites.Sequence(name="Tier 1: bare gaze", memory=True)
    root.add_child(_pre_aim())
    root.add_child(BtNode_MaintainEyeContact(name="Maintain eye contact T1"))
    return root


def build_tier2() -> py_trees.behaviour.Behaviour:
    """Gaze inside Parallel(SuccessOnSelected=[Sleep])."""
    root = py_trees.composites.Sequence(name="Tier 2: gaze in parallel", memory=True)
    root.add_child(_pre_aim())

    selected = SleepNode(name="Selected sibling sleep 10s", duration=10.0)
    eye = BtNode_MaintainEyeContact(name="Maintain eye contact T2")
    root.add_child(
        py_trees.composites.Parallel(
            name="Gaze + selected sleep",
            policy=py_trees.common.ParallelPolicy.SuccessOnSelected([selected]),
            children=[selected, eye],
        )
    )
    return root


def build_tier3() -> py_trees.behaviour.Behaviour:
    """Gaze inside Parallel(SuccessOnSelected=[Sequence(Sleep, Announce, Sleep)])."""
    root = py_trees.composites.Sequence(
        name="Tier 3: gaze + real announce", memory=True
    )
    root.add_child(_pre_aim())

    selected = py_trees.composites.Sequence(
        name="Pre-announce + announce + tail", memory=True
    )
    selected.add_child(SleepNode(name="Wait 2s before announce", duration=2.0))
    selected.add_child(
        BtNode_Announce(
            name="Announce please sit here",
            bb_source=None,
            message="Please sit here. Let me look at you.",
        )
    )
    selected.add_child(SleepNode(name="Wait 3s after announce", duration=3.0))

    eye = BtNode_MaintainEyeContact(name="Maintain eye contact T3")
    root.add_child(
        py_trees.composites.Parallel(
            name="Gaze + announce sequence",
            policy=py_trees.common.ParallelPolicy.SuccessOnSelected([selected]),
            children=[selected, eye],
        )
    )
    return root


def build_tier4() -> py_trees.behaviour.Behaviour:
    """Tier 1 + 3 decoy MaintainEyeContact instances registered at setup.

    HRI builds four BtNode_MaintainEyeContact instances at tree construction
    time. ``tree.setup()`` walks the entire tree and runs ``setup()`` on every
    behaviour, so all four end up with their own ``rclpy.action.ActionClient``
    bound to the same BT node and the same ``/follow_head_action`` server.
    Tier 1 only registers one; this tier adds three idle decoys, then ticks
    only the active one (a Selector with the active gaze leaf as the first
    child — the action stays RUNNING so the Selector never reaches the decoys).

    Hits the rclpy multi-client race surface (ros2/rclpy#1123 etc.) in the
    same shape HRI does without needing a Parallel or Announce.
    """
    root = py_trees.composites.Sequence(
        name="Tier 4: 4 instances at setup, 1 active", memory=True
    )
    root.add_child(_pre_aim())

    active = BtNode_MaintainEyeContact(name="Active gaze T4")
    decoys = [
        BtNode_MaintainEyeContact(name=f"Decoy gaze T4 #{i}") for i in range(3)
    ]

    selector = py_trees.composites.Selector(
        name="Pick first running gaze (active wins)", memory=True
    )
    selector.add_child(active)
    for d in decoys:
        selector.add_child(d)
    root.add_child(selector)
    return root


def build_tier5() -> py_trees.behaviour.Behaviour:
    """Multi-step TurnPanTilt prefix mimicking HRI's pre-eye-contact sequence.

    HRI runs TurnPanTilt(0, 20), then TurnPanTilt(0, 35), seat-scan-things,
    TurnPanTilt(0, 20) again, finally TurnPanTilt(90, 45) before the parallel
    that contains MaintainEyeContact. Each of those publishes a settle-waiting
    BT node that subscribes to /pan_tilt_controller/state.

    If Tier 5 fails, the issue is servo-state-after-multi-step-prefix.
    """
    root = py_trees.composites.Sequence(
        name="Tier 5: multi-step pre-aim", memory=True
    )
    root.add_child(BtNode_TurnPanTilt(name="Look down at host (0,20)", x=0.0, y=20.0, speed=0.0))
    root.add_child(BtNode_TurnPanTilt(name="Look forward (0,35)", x=0.0, y=35.0, speed=0.0))
    root.add_child(BtNode_TurnPanTilt(name="Look at sofa (0,20)", x=0.0, y=20.0, speed=0.0))
    root.add_child(BtNode_TurnPanTilt(name="Look at guest (90,45)", x=90.0, y=45.0, speed=0.0))
    root.add_child(BtNode_MaintainEyeContact(name="Maintain eye contact T5"))
    return root


TIERS = {
    1: build_tier1,
    2: build_tier2,
    3: build_tier3,
    4: build_tier4,
    5: build_tier5,
}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--tier",
        type=int,
        default=1,
        choices=sorted(TIERS.keys()),
        help="Which isolation tier to run (default: 1).",
    )
    parsed, ros_args = parser.parse_known_args(
        sys.argv[1:] if args is None else args
    )

    rclpy.init(args=ros_args)

    tree = py_trees_ros.trees.BehaviourTree(TIERS[parsed.tier]())
    tree.setup(node_name=f"test_eye_contact_t{parsed.tier}", timeout=15)

    py_trees.logging.level = py_trees.logging.Level.DEBUG
    # Match HRI's BT tick rate so timing-sensitive races have the same window.
    tree.tick_tock(period_ms=500.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
