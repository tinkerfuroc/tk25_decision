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

"""Isolated runner for the DoingLaundry locate-and-approach entry phase.

Chains the real start -> door -> navigate -> scan sequence the production
tree uses to enter the laundry area:

  * ``BtNode_WaitForStart``   — block until the start signal (mock: auto).
  * ``BtNode_DoorDetection``  — check the arena door is open, writing the
    boolean to ``KEY_DOOR_STATUS``.
  * ``BtNode_GotoAction``     — Nav2 drive to the seeded laundry-area pose.
  * ``_scanRetry``            — generalist scan of the entry for a clothing
    item, writing to ``KEY_VISION_RESULT``.

  * In mock mode every leaf auto-advances (Nav2 / door / scan / start all
    mocked) so the sequence runs with no robot.
  * Against live servers it drives the real entry routine.

The goto pose is seeded on the blackboard at tree construction via
``BtNode_WriteToBlackboard`` (``POSE_LAUNDRY_AREA`` from config).

Run::

    ros2 run behavior_tree laundry-test-locate

Fully offline (no servers, auto-advance) — mock every subsystem via the
bundled full-mock config::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree laundry-test-locate
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import BtNode_WaitForStart
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
from behavior_tree.TemplateNodes.Vision import BtNode_DoorDetection
from behavior_tree.visualization import create_post_tick_visualizer

from .config import (
    CLOTHING_SCAN_PROMPT,
    KEY_DOOR_STATUS,
    KEY_POSE_LAUNDRY_AREA,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    POSE_LAUNDRY_AREA,
    TARGET_FRAME,
)
from .sampling import _scanRetry


def create_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(
        name="test:locate_and_approach", memory=True
    )

    # Seed the goto target pose + target frame the downstream nodes read.
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed laundry area pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_POSE_LAUNDRY_AREA,
            object=POSE_LAUNDRY_AREA,
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed target frame",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_TARGET_FRAME,
            object=TARGET_FRAME,
        )
    )

    # 1. Wait for the operator/start signal.
    root.add_child(BtNode_WaitForStart(name="Wait for start signal"))

    # 2. Detect the arena door is open.
    root.add_child(
        BtNode_DoorDetection(
            name="Detect arena door",
            bb_door_state_key=KEY_DOOR_STATUS,
        )
    )

    # 3. Navigate to the laundry area.
    root.add_child(
        BtNode_GotoAction(
            name="Navigate to laundry area",
            key=KEY_POSE_LAUNDRY_AREA,
        )
    )

    # 4. Scan the entry for a clothing item.
    root.add_child(
        _scanRetry(
            name="Scan entry for clothing",
            bb_key=KEY_VISION_RESULT,
            object=CLOTHING_SCAN_PROMPT,
        )
    )

    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="laundry_test_locate_and_approach")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="locate_and_approach"
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
