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

"""Isolated runner for the real FoldClothing manipulation integration.

Builds a one-node tree around :class:`BtNode_FoldClothingAction` and ticks it.
This is the small, testable harness for Doing-Laundry's "manipulation newest
action" integration:

  * In mock mode (manipulation subsystem mocked in ``mock_config.json``) the
    fold node advances on the manipulation success key, so the tree can be
    exercised with no robot.
  * Against a live ``fold_clothing_action`` server (``arm_api`` ->
    ``fold_clothing_server.py``) it drives a real perceive -> plan -> fold cycle.

Run::

    ros2 run behavior_tree laundry-test-fold-action

Fully offline (no servers, auto-advance) — mock every subsystem via the
bundled full-mock config::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree laundry-test-fold-action

Env knobs::

    BT_LAUNDRY_GARMENT     garment label hint        (default "shirt")
    BT_LAUNDRY_FOLD_MODE   bottom fold mode 0/1/2     (default 0)
"""

import os

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.FoldClothingAction import BtNode_FoldClothingAction
from behavior_tree.visualization import create_post_tick_visualizer

KEY_GARMENT_LABEL = "laundry/garment_label"


def create_tree() -> py_trees.behaviour.Behaviour:
    garment = os.environ.get("BT_LAUNDRY_GARMENT", "shirt")
    fold_mode = int(os.environ.get("BT_LAUNDRY_FOLD_MODE", "0"))

    root = py_trees.composites.Sequence("test:fold_clothing", memory=True)
    root.add_child(
        BtNode_WriteToBlackboard(
            "seed garment label",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_GARMENT_LABEL,
            object=garment,
        )
    )
    root.add_child(
        BtNode_FoldClothingAction(
            name=f"fold the {garment}",
            bb_key_garment_label=KEY_GARMENT_LABEL,
            garment_label=garment,
            bottom_fold_mode=fold_mode,
            return_to_scan=True,
        )
    )
    root.add_child(py_trees.behaviours.Running("idle (ctrl-c to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="laundry_test_fold_action")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="fold_clothing"
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
