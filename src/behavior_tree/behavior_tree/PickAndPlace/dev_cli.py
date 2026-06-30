"""Dev / test entry points for individual PickAndPlace subtrees.

Production entry (`pick-and-place`) lives in cli.py — leave that alone.
This file exposes one `ros2 run`-able entry per subtree so individual phases
can be exercised in isolation during development.

Add a new entry in two steps:
    1. Write a tree-factory function below (must take no args, return a root).
       Each subtree owns its own `createConstantWriter()` and prepends it to
       its sequence — no pick_and_place dependency needed.
    2. Add a thin `main`-style function that calls `run_tree(factory, ...)`,
       and register it in setup.py under `entry_points.console_scripts`.
"""

from typing import List

import py_trees

from behavior_tree.runtime import run_tree

from .nav_test import NavTestSpec, createNavTestSweep


# ---------------------------------------------------------------------------
# Tree factories — one per subtree, all no-arg, all return a root composite.
# Each subtree writes the blackboard constants it depends on at the head of
# its own sequence, so the tree can be ticked standalone.
# ---------------------------------------------------------------------------


# Default destinations for the nav-test sweep. Pan/tilt values are placeholders
# — tune each per real-robot orbbec FOV so the workspace at the stop point is
# fully in view. Format: (label, goal_pose_or_None, goal_pose_key_or_None,
# pan_deg, tilt_deg).
DEFAULT_NAV_TEST_SPECS: List[NavTestSpec] = [
    ("kitchen entry", None, "pp_pose_kitchen_entry", 0.0, 0.0),
    ("table",         None, "pp_pose_table",         0.0, 25.0),
    ("wash staging",  None, "pp_pose_wash_staging",  0.0, 15.0),
    ("trash bin",     None, "pp_pose_trash_bin",     0.0, 20.0),
    ("cabinet",       None, "pp_pose_cabinet",       0.0, 15.0),
    ("kitchen shelf", None, "pp_pose_kitchen_shelf", 0.0, 20.0),
    ("dishwasher",    None, "pp_pose_dish_washer",   0.0, 15.0),
]


def buildNavTestTree() -> py_trees.behaviour.Behaviour:
    # nav_test.createNavTestSweep() writes its own constants at the head of
    # the sequence, so it doesn't need the pick_and_place constant writer.
    return createNavTestSweep(DEFAULT_NAV_TEST_SPECS)


def buildTableObjectRecognitionTree() -> py_trees.behaviour.Behaviour:
    # table_object_recognition.createTableObjectRecognition() writes its own
    # constants at the head of the sequence, so it doesn't need the
    # pick_and_place constant writer.
    from .table_object_recognition import createTableObjectRecognition
    return createTableObjectRecognition()


def buildTableGraspTree() -> py_trees.behaviour.Behaviour:
    # table_grasping.createTableGrasp() writes its own constants at the head of
    # the sequence, so it doesn't need the pick_and_place constant writer.
    from .table_grasping import createTableGrasp
    return createTableGrasp()


def buildTablePlacingTree() -> py_trees.behaviour.Behaviour:
    # table_placing.createTablePlacing() writes its own constants at the head of
    # the sequence, so it doesn't need the pick_and_place constant writer.
    from .table_placing import createTablePlacing
    return createTablePlacing()


def buildDropTrashTree() -> py_trees.behaviour.Behaviour:
    # drop_trash.createDropTrash() writes its own constants at the head of
    # the sequence, so it doesn't need the pick_and_place constant writer.
    from .drop_trash import createDropTrash
    return createDropTrash()


def buildCabinetCategorizationTree() -> py_trees.behaviour.Behaviour:
    # cabinet_categorization.createCabinetCategorization() writes its own
    # constants at the head of the sequence, so it doesn't need the
    # pick_and_place constant writer.
    from .cabinet_categorization import createCabinetCategorization
    return createCabinetCategorization()


# ---------------------------------------------------------------------------
# Console-script entry points — referenced from setup.py.
# ---------------------------------------------------------------------------


def nav_test() -> None:
    run_tree(buildNavTestTree, period_ms=500.0, title="Nav Test")


def table_object_recognition() -> None:
    run_tree(
        buildTableObjectRecognitionTree,
        period_ms=500.0,
        title="Table Object Recognition",
    )


def table_grasp() -> None:
    run_tree(buildTableGraspTree, period_ms=500.0, title="Table Grasp")


def table_placing() -> None:
    run_tree(buildTablePlacingTree, period_ms=500.0, title="Table Placing")


def drop_trash() -> None:
    run_tree(buildDropTrashTree, period_ms=500.0, title="Drop Trash")


def cabinet_categorization() -> None:
    run_tree(
        buildCabinetCategorizationTree,
        period_ms=500.0,
        title="Cabinet Categorization",
    )
