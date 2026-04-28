"""Standalone harness for the HRI two-way introduction subtree.

Seeds ``KEY_PERSONS`` with two mock guest profiles (as
``BtNode_CombinePerson`` would after the full intake phase), then runs just
:func:`createTwoWayIntroduction` from :mod:`HRI.hri`. Exercises the
post-intake subtree shape — feature-match seated guests → pre-orient head
at `target_id` via ``BtNode_TurnTo`` → announce the intro under the gaze
supervisor.

Usage::

    ros2 run behavior_tree hri-intro

In mock mode the feature-matching leaf emits N synthetic centroids and
``BtNode_TurnTo`` returns SUCCESS immediately; real TTS still fires because
`announcement` defaults to `enabled: false` (real servers) in
``mock_config.json``. Set ``BT_MOCK_MODE=true`` to step through the audio
manually.
"""
from __future__ import annotations

import py_trees


def build() -> py_trees.behaviour.Behaviour:
    # Deferred imports: HRI/config.py touches rclpy.time at module-scope, only
    # populated after rclpy.init() — run_tree inits rclpy before calling build().
    from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
    from behavior_tree.TemplateNodes.structs import Person

    from .config import KEY_PERSONS
    from .hri import createTwoWayIntroduction

    # Two mock seated guests, populated the way BtNode_CombinePerson would.
    guest1 = Person()
    guest1.name = "Alex"
    guest1.fav_drink = "water"
    guest1.features = [0.1, 0.2, 0.3, 0.4, 0.5]

    guest2 = Person()
    guest2.name = "Sam"
    guest2.fav_drink = "coffee"
    guest2.features = [0.2, 0.3, 0.4, 0.5, 0.6]

    mock_persons = [guest1, guest2]

    root = py_trees.composites.Sequence(name="Intro-only test", memory=True)
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Seed mock guests on blackboard",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_PERSONS,
            object=mock_persons,
        )
    )
    root.add_child(createTwoWayIntroduction())
    return root


def main():
    from behavior_tree.runtime import run_tree

    run_tree(build, period_ms=500.0, title="Intro-only")
