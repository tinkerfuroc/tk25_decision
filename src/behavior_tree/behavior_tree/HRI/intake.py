"""Standalone harness for the HRI name/drink intake subtree.

Builds just the :func:`_create_get_info` helper from :mod:`HRI.hri` twice
(name + drink) wrapped in a root Sequence, so the audio pipeline can be
tuned without waiting on navigation or manipulation. Mirrors
:mod:`HRI.follow.main` as the follow-person isolation harness.

Usage::

    ros2 run behavior_tree hri-intake

In mock mode the action leaf KEYPRESS-waits; in live mode it dispatches
``phrase_extraction_action`` against ``tk_24_audio``.
"""
from __future__ import annotations

import py_trees


BB_KEY_TEST_NAME = "intake_test_name"
BB_KEY_TEST_DRINK = "intake_test_drink"


def build() -> py_trees.behaviour.Behaviour:
    # Deferred imports: HRI/config.py touches rclpy.time at module scope, which
    # is only populated after rclpy.init(). run_tree inits rclpy before calling
    # build(). Same deferral pattern as HRI/follow.py:main.
    from .config import DRINKS, NAMES
    from .hri import _create_get_info

    root = py_trees.composites.Sequence(name="Intake-only test", memory=True)
    root.add_child(_create_get_info("name", BB_KEY_TEST_NAME, NAMES))
    root.add_child(_create_get_info("favorite drink", BB_KEY_TEST_DRINK, DRINKS))
    return root


def main():
    from behavior_tree.runtime import run_tree

    run_tree(build, period_ms=500.0, title="Intake-only")
