# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""HRI bag-handover unit test (countdown-free).

Isolates the *bag handover* step of the HRI bag flow so it can be ticked on its
own. The production handover in ``HRI/hri.py:createBagFlow`` opens the gripper,
asks for the bag, waits on a fixed ``py_trees.timers.Timer``, then closes the
gripper — but it never *verifies* the bag is actually in the gripper. This unit
adds that verification as a non-blocking presence check while deliberately
avoiding the rulebook-penalised spoken "3-2-1" countdown (we use a silent Timer
plus a vision presence check instead of counting down out loud).

Flow (``create_tree``):
    readiness announce
    -> BtNode_GripperAction(open)
    -> ask for bag (announce)
    -> Timer (silent settle window — NOT a spoken countdown)
    -> BtNode_GripperAction(close)
    -> FailureIsSuccess( BtNode_FindObj presence check )   # best-effort verify

The presence check is wrapped in ``FailureIsSuccess`` so a missed/empty
detection never fails the handover (the human may have handed it correctly even
if vision can't confirm the bag silhouette in-gripper). ``BtNode_FindObj`` reads
no input blackboard key (``object`` is passed directly), so nothing extra needs
seeding; it writes its result to ``KEY_BAG_PRESENT``.

Run (on-robot, real audio/manipulation/vision):
    ros2 run behavior_tree hri-test-handover

Offline run (mock every subsystem; KEYPRESS auto-advances):
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree hri-test-handover
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction
from behavior_tree.TemplateNodes.Vision import BtNode_FindObj
from behavior_tree.visualization import create_post_tick_visualizer

# Blackboard key the in-gripper presence check writes its detection result to.
# Module-local (HRI/config.py loads a hardcoded constants.json path that may be
# absent), so this module is always import-safe.
KEY_BAG_PRESENT = "hri/bag_present"

# What the presence check looks for in the gripper after the close.
_BAG_LABEL = "bag"

# Silent settle window (seconds) between asking and closing the gripper. This is
# a quiet wait, NOT a spoken countdown — the spoken 3-2-1 countdown is penalised.
_HANDOVER_SETTLE_SEC = 3.0


def createBagHandover(
    bag_label: str = _BAG_LABEL,
    settle_seconds: float = _HANDOVER_SETTLE_SEC,
    bb_present_key: str = KEY_BAG_PRESENT,
):
    """A countdown-free bag-handover unit with a best-effort presence check.

    Returns a memory ``Sequence``:
        announce ready -> open gripper -> ask -> silent Timer -> close gripper
        -> FailureIsSuccess(BtNode_FindObj presence check)
    """
    seq = py_trees.composites.Sequence("HRI bag handover", memory=True)
    seq.add_child(
        BtNode_Announce(
            name="announce ready for bag",
            bb_source=None,
            message="I am ready to take your bag.",
        )
    )
    seq.add_child(
        BtNode_GripperAction(name="Open gripper for bag", open_gripper=True)
    )
    seq.add_child(
        BtNode_Announce(
            name="Ask for bag handover",
            bb_source=None,
            message="Please place your bag in my gripper.",
        )
    )
    # Silent settle window. Deliberately a Timer (no speech) so we don't speak
    # the penalised "three, two, one" countdown.
    seq.add_child(
        py_trees.timers.Timer(
            name="Silent settle window (no spoken countdown)",
            duration=settle_seconds,
        )
    )
    seq.add_child(
        BtNode_GripperAction(name="Close gripper with bag", open_gripper=False)
    )
    # Best-effort verify the bag is in the gripper. Failure must not abort the
    # handover, so wrap in FailureIsSuccess.
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="verify bag in gripper (best effort)",
            child=BtNode_FindObj(
                name="check bag presence in gripper",
                bb_source=None,
                bb_namespace=None,
                bb_key=bb_present_key,
                object=bag_label,
                target_object_cls=bag_label,
            ),
        )
    )
    return seq


def create_tree():
    """Build the standalone bag-handover unit tree.

    The handover unit needs no seeded blackboard inputs: every leaf either takes
    its arguments directly (gripper open/close flags, announce messages, the
    ``BtNode_FindObj`` ``object`` label) or only writes output. A trailing
    ``Running`` idle keeps the tree alive after the one-shot handover so the
    harness can be inspected before ctrl-c.
    """
    root = py_trees.composites.Sequence("test:hri-bag-handover", memory=True)
    root.add_child(createBagHandover())
    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="hri_test_handover")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="hri-test-handover"
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
