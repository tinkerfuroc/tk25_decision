"""Standalone harness: announce 'at door' → doorbell action → announce 'heard doorbell'.

Mirrors the shape of `test_eye_contact.py`:

    ros2 run behavior_tree test-doorbell

Mock-mode: with `audio_input.enabled=true` in `mock_config.json`, the
doorbell leaf becomes a keypress prompt (Enter advances). With
`announcement.enabled=true` the announces print to stdout instead of
calling TTS. Hardware mode requires the action server to be running:

    ros2 run audio_pakage doorbell_action

Stop with Ctrl+C — `tick_tock` keeps re-ticking after the sequence completes.
"""

import sys

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.HRI.doorbellDetection import BtNode_DoorbellDetection
from behavior_tree.TemplateNodes.Audio import BtNode_Announce


def build_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Doorbell test", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Announce at door",
            bb_source=None,
            message="at door",
        )
    )
    root.add_child(BtNode_DoorbellDetection(name="Listen for doorbell"))
    root.add_child(
        BtNode_Announce(
            name="Announce heard doorbell",
            bb_source=None,
            message="heard doorbell",
        )
    )
    return root


def main(args=None):
    rclpy.init(args=sys.argv[1:] if args is None else args)

    tree = py_trees_ros.trees.BehaviourTree(build_tree())
    tree.setup(node_name="test_doorbell", timeout=15)

    py_trees.logging.level = py_trees.logging.Level.DEBUG
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
