"""Phase-2 barman-trip isolation harness for the Restaurant task.

Pre-seeds ``KEY_ORDER_LIST`` with two synthetic orders (matching the
``BtNode_RecordOrder`` schema), the bar anchor pose, and the barman-text slot,
then runs the real ``createBarmanPhase()`` from ``restaurants.py``:

    OrderListNotEmpty -> bar return -> FormatOrdersForBarman -> announce orders
    -> GetConfirmationAction (barman ready?).

No production factory is re-derived — this only seeds the blackboard and wraps
the real Phase-2 subtree.

Run (real hardware/servers):
    ros2 run behavior_tree test-restaurant-barman

Run (offline / fully mocked):
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree test-restaurant-barman
"""

from __future__ import annotations

import py_trees
import py_trees_ros
import rclpy

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .config import (
    KEY_BARMAN_TEXT,
    KEY_KITCHEN_BAR_POSE,
    KEY_ORDER_LIST,
)
from .restaurants import createBarmanPhase


def _pose_stamped(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _synthetic_order_list():
    return [
        {
            "id": 1,
            "pose": _pose_stamped(2.0, 1.0),
            "picture_path": "",
            "items": ["water", "juice"],
            "delivered_items": [],
        },
        {
            "id": 2,
            "pose": _pose_stamped(-1.5, 2.5),
            "picture_path": "",
            "items": ["coffee"],
            "delivered_items": [],
        },
    ]


def create_tree():
    root = py_trees.composites.Sequence(name="Test barman phase (Phase 2)", memory=True)

    seed = py_trees.composites.Parallel(
        name="Seed blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in (
        ("Seed kitchen bar pose", KEY_KITCHEN_BAR_POSE, _pose_stamped(0.0, 0.0)),
        ("Seed barman text", KEY_BARMAN_TEXT, ""),
        ("Seed order list", KEY_ORDER_LIST, _synthetic_order_list()),
    ):
        seed.add_child(
            BtNode_WriteToBlackboard(
                name=name,
                bb_namespace="",
                bb_source=None,
                bb_key=key,
                object=value,
            )
        )

    root.add_child(seed)
    root.add_child(createBarmanPhase())
    root.add_child(py_trees.behaviours.Running("idle"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="test_restaurant_barman")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Restaurant Barman Phase (Phase 2)"
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
