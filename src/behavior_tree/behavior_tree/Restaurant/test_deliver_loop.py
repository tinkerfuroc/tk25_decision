"""Phase-3 deliver-loop isolation harness for the Restaurant task.

Pre-seeds ``KEY_ORDER_LIST`` with two synthetic orders (matching the
``BtNode_RecordOrder`` schema: ``{id, pose, picture_path, items, delivered_items}``)
plus the runtime keys the Phase-3 subtree reads (arm poses, kitchen bar pose,
pickup-verification flag, order checklist). It then runs the real
``createDeliverAllItemsPhase()`` from ``restaurants.py`` (per-item loop:
IterateOrderItems -> pickup verification -> deliver -> MarkItemDelivered).

No production factory is re-derived — this only seeds the blackboard and wraps
the real Phase-3 subtree.

Run (real hardware/servers):
    ros2 run behavior_tree test-restaurant-deliver-loop

Run (offline / fully mocked):
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree test-restaurant-deliver-loop
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
    ARM_POS_NAVIGATING,
    ARM_POS_SERVING,
    KEY_ARM_NAVIGATING,
    KEY_ARM_SERVING,
    KEY_KITCHEN_BAR_POSE,
    KEY_ORDER_CHECKLIST,
    KEY_ORDER_LIST,
    KEY_PICKUP_VERIFIED,
)
from .restaurants import createDeliverAllItemsPhase


def _pose_stamped(x: float, y: float) -> PoseStamped:
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(
            position=Point(x=x, y=y, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _synthetic_order_list():
    """Two orders matching the BtNode_RecordOrder schema."""
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
    root = py_trees.composites.Sequence(name="Test deliver loop (Phase 3)", memory=True)

    seed = py_trees.composites.Parallel(
        name="Seed blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in (
        ("Seed arm navigating pose", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Seed arm serving pose", KEY_ARM_SERVING, ARM_POS_SERVING),
        ("Seed pickup verification", KEY_PICKUP_VERIFIED, False),
        ("Seed order checklist", KEY_ORDER_CHECKLIST, {}),
        ("Seed kitchen bar pose", KEY_KITCHEN_BAR_POSE, _pose_stamped(0.0, 0.0)),
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
    root.add_child(createDeliverAllItemsPhase())
    root.add_child(py_trees.behaviours.Running("idle"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="test_restaurant_deliver_loop")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Restaurant Deliver Loop (Phase 3)"
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
