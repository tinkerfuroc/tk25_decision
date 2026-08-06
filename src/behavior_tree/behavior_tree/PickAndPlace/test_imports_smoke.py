from __future__ import annotations

"""No-ROS import canary for the PickAndPlace (and DoingLaundry) modules.

This module imports every PickAndPlace task module plus the DoingLaundry
modules and prints OK/FAIL per module. It is the canary that would have caught
the ``manipulation_new`` ModuleNotFoundError break: any module whose import
chain is broken shows up as FAIL here without needing a running robot.

``create_tree()`` is intentionally trivial (a single Success leaf) so the
offline smoke is a pure import test — the import sweep itself runs at
module-import time and again inside ``main()``.

Run command:
    ros2 run behavior_tree pp-test-imports

Offline run command:
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree pp-test-imports
"""

import importlib

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.visualization import create_post_tick_visualizer


# (package, module) pairs to probe. pick_and_place is handled separately
# because its line-3 `from sympy import use` can raise on hosts without sympy.
_PICKANDPLACE_MODULES = [
    "table_grasping",
    "table_placing",
    "table_object_recognition",
    "drop_trash",
    "cabinet_categorization",
    "nav_test",
    "custom_nodes",
]

_DOINGLAUNDRY_MODULES = [
    "config",
    "laundry",
    "sampling",
    "state_nodes",
    "test_fold_clothing_action",
]


def _probe_imports() -> None:
    """Import each task module, printing OK/FAIL. Never raises."""
    print("=== PickAndPlace / DoingLaundry import canary ===")
    for mod in _PICKANDPLACE_MODULES:
        dotted = f"behavior_tree.PickAndPlace.{mod}"
        try:
            importlib.import_module(dotted)
            print(f"OK   {dotted}")
        except Exception as exc:  # noqa: BLE001 - canary must not abort
            print(f"FAIL {dotted}: {type(exc).__name__}: {exc}")

    for mod in _DOINGLAUNDRY_MODULES:
        dotted = f"behavior_tree.DoingLaundry.{mod}"
        try:
            importlib.import_module(dotted)
            print(f"OK   {dotted}")
        except Exception as exc:  # noqa: BLE001
            print(f"FAIL {dotted}: {type(exc).__name__}: {exc}")

    # pick_and_place.py historically carries a stray `from sympy import use`
    # at line 3. Wrap it so a missing-sympy host does not fail the canary,
    # but still surface the result.
    dotted = "behavior_tree.PickAndPlace.pick_and_place"
    try:
        importlib.import_module(dotted)
        print(f"OK   {dotted}")
    except Exception as exc:  # noqa: BLE001
        print(f"WARN {dotted} (non-fatal): {type(exc).__name__}: {exc}")

    print("=== import canary complete ===")


def create_tree() -> py_trees.behaviour.Behaviour:
    """Trivial tree — the real work is the import sweep, run here once."""
    _probe_imports()
    root = py_trees.composites.Sequence(name="imports smoke", memory=True)
    root.add_child(py_trees.behaviours.Success(name="imports ok"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="pp_test_imports")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="PickAndPlace Imports Smoke"
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
