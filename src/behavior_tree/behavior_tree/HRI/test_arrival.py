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

"""HRI arrival-trigger unit test.

Isolates the *arrival trigger* of the HRI task — the doorbell-then-go-to-door
step that ``HRI/hri.py:createArrivalTrigger`` performs — so it can be ticked on
its own. The production version listens for the doorbell, wraps the listen in
``FailureIsSuccess`` so a timeout doesn't stall the task, then navigates to the
door pose. This unit reproduces exactly that pairing:

    BtNode_DoorbellDetection (wrapped FailureIsSuccess, like the production tree)
    -> Retry( BtNode_GotoAction -> seeded door pose )

``BtNode_DoorbellDetection`` (``HRI/doorbellDetection.py``) is an action client
for ``doorbell_action``; under the full-mock config its ``audio_input``
subsystem is mocked so it auto-advances. ``BtNode_GotoAction`` reads its goal
from a blackboard key (``KEY_DOOR_POSE``); this harness seeds that key with a
dummy ``PoseStamped`` via ``BtNode_WriteToBlackboard`` before the trigger runs,
so the goto node has a value to read (on-robot, seed the real door pose from
``HRI/config.py:POSE_DOOR`` instead).

Run (on-robot, real audio + navigation):
    ros2 run behavior_tree hri-test-arrival

Offline run (mock every subsystem; KEYPRESS auto-advances):
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree hri-test-arrival
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.visualization import create_post_tick_visualizer
from behavior_tree.HRI.doorbellDetection import BtNode_DoorbellDetection

# Blackboard key the goto node reads its door goal from. Module-local
# (HRI/config.py loads a hardcoded constants.json path that may be absent), so
# this module stays import-safe.
KEY_DOOR_POSE = "hri/door_pose"


def _make_dummy_door_pose():
    """A neutral map-frame ``PoseStamped`` so the goto node has a goal to read.

    Replace with the real door pose on-robot (``HRI/config.py:POSE_DOOR``).
    """
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose


def createArrivalUnit(door_pose_key: str = KEY_DOOR_POSE):
    """The arrival trigger as a self-contained unit.

    Mirrors ``HRI/hri.py:createArrivalTrigger``: a (FailureIsSuccess-wrapped)
    doorbell listen followed by a retried navigate to the door pose, bracketed
    with announcements for operator visibility.
    """
    seq = py_trees.composites.Sequence("HRI arrival trigger", memory=True)
    seq.add_child(
        BtNode_Announce(
            "announce waiting door bell",
            bb_source=None,
            message="Waiting for the door bell.",
        )
    )
    # Same FailureIsSuccess wrap as the production tree: a doorbell timeout must
    # not stall the task, the robot still proceeds to the door.
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(
            "doorbell (best effort)",
            BtNode_DoorbellDetection(name="Listen for doorbell"),
        )
    )
    parallel_going = py_trees.composites.Parallel(
        name="Go to door and announce",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    parallel_going.add_child(
        py_trees.decorators.Retry(
            name="Retry goto door",
            child=BtNode_GotoAction(name="Go to door", key=door_pose_key),
            num_failures=5,
        )
    )
    parallel_going.add_child(
        BtNode_Announce(
            name="Arrival announcement",
            bb_source=None,
            message="Going to check the door.",
        )
    )
    seq.add_child(parallel_going)
    return seq


def create_tree():
    """Build the standalone arrival-trigger unit tree.

    Seeds a dummy door ``PoseStamped`` on ``KEY_DOOR_POSE`` (the input the goto
    node reads), then runs the arrival unit. A trailing ``Running`` idle keeps
    the tree alive after the one-shot trigger.
    """
    root = py_trees.composites.Sequence("test:hri-arrival", memory=True)
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed door pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_DOOR_POSE,
            object=_make_dummy_door_pose(),
        )
    )
    root.add_child(createArrivalUnit())
    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="hri_test_arrival")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="hri-test-arrival"
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
