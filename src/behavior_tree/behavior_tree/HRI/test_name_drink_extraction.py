"""Standalone harness: announce 'ready' -> name+drink action -> echo back what was heard.

Mirrors the shape of `test_doorbell.py`:

    ros2 run behavior_tree test-name-drink

Mock-mode: with `audio_input.enabled=true` in `mock_config.json`, the
`BtNode_NameDrinkExtractionAction` leaf KEYPRESS-waits and seeds the
blackboard with `name='Alice', drink='orange juice'`. With
`announcement.enabled=true` the announces print to stdout instead of
calling TTS. Hardware mode requires the action server to be running:

    ros2 run audio_pakage name_drink_extraction_ac

Stop with Ctrl+C — `tick_tock` keeps re-ticking after the sequence completes.
"""

import sys

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_NameDrinkExtractionAction,
)


BB_KEY_NAME = "test_extracted_name"
BB_KEY_DRINK = "test_extracted_drink"


def build_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Name+Drink extraction test", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Announce ready",
            bb_source=None,
            message="Please tell me your name and your favorite drink after the beep.",
        )
    )
    root.add_child(
        BtNode_NameDrinkExtractionAction(
            name="Extract name and drink",
            bb_name_key=BB_KEY_NAME,
            bb_drink_key=BB_KEY_DRINK,
            timeout=7.0,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Echo heard name",
            bb_source=BB_KEY_NAME,
            message="I heard your name is",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Echo heard drink",
            bb_source=BB_KEY_DRINK,
            message="and your favorite drink is",
        )
    )
    return root


def main(args=None):
    rclpy.init(args=sys.argv[1:] if args is None else args)

    tree = py_trees_ros.trees.BehaviourTree(build_tree())
    tree.setup(node_name="test_name_drink_extraction", timeout=15)

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
