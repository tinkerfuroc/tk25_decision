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

"""Isolated runner for the perception-then-grasp clothing pick.

This is the closed-loop replacement for the production tree's blind
open-loop pick: scan for a clothing item with the generalist detector
(``BtNode_ScanForGeneralist``), then grasp it (``BtNode_Grasp``) using the
vision result. It reuses the real DoingLaundry sampling factories
(``_scanRetry`` / ``_graspRetry``) so the perception->grasp wiring stays in
one place.

  * In mock mode (vision + manipulation mocked) the scan publishes a mock
    detection on ``KEY_VISION_RESULT`` and the grasp advances on the
    manipulation success key, so the tree runs with no robot.
  * Against live ``/object_detection_generalist`` + ``start_grasp`` servers it
    drives a real perceive -> grasp cycle.

Run::

    ros2 run behavior_tree laundry-test-pick

Fully offline (no servers, auto-advance) — mock every subsystem via the
bundled full-mock config::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree laundry-test-pick
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .config import (
    CLOTHING_SCAN_PROMPT,
    KEY_TARGET_FRAME,
    KEY_VISION_RESULT,
    TARGET_FRAME,
)
from .sampling import _graspRetry, _scanRetry


def create_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(
        name="test:pick_clothing (scan -> grasp)", memory=True
    )

    # Seed the target frame the grasp/vision pipeline reads.
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed target frame",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_TARGET_FRAME,
            object=TARGET_FRAME,
        )
    )

    # Perception: scan for a clothing item, writing the detection to
    # KEY_VISION_RESULT (the key the grasp consumes).
    root.add_child(
        _scanRetry(
            name="Scan for clothing piece",
            bb_key=KEY_VISION_RESULT,
            object=CLOTHING_SCAN_PROMPT,
        )
    )

    # Grasp: consume the vision result + object label.
    root.add_child(
        _graspRetry(
            name="Grasp clothing piece",
            bb_key_vision_res=KEY_VISION_RESULT,
            bb_key_object_label=CLOTHING_SCAN_PROMPT,
            retries=3,
            use_mesh=True,
            stay=True,
        )
    )

    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="laundry_test_pick_clothing")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="pick_clothing"
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
