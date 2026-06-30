"""Phase-1 collect isolation harness for the Restaurant task (detect -> approach).

Seeds the runtime state the Phase-1 subtree reads (caller queue, active-customer
slot/picture, order checklist, pickup flag) and runs the real Phase-1 perception
+ approach factories from ``restaurants.py``:

    init checklist -> scan(detect+arbitrate) -> approach selected customer.

SCOPE NOTE: this harness deliberately stops before the order-intake tail of
``createCollectOneOrder()`` (``createTakeAndConfirmOrder`` + ``BtNode_RecordOrder``
+ ``BtNode_CloseActiveCustomer``). ``createTakeAndConfirmOrder`` wraps the order
loop in a Parallel with ``BtNode_MaintainEyeContact``, whose *mock* ``send_goal``
returns without setting ``self.start_time``; its ``update`` then evaluates
``time.time() - self.start_time`` and raises ``TypeError`` under the offline
full-mock path (``TemplateNodes/Vision.py:1894/1897``). That is a pre-existing
bug in the shared eye-contact template node, not in this harness, and cannot be
worked around without editing ``restaurants.py``/``Vision.py``. So this module
exercises the largest cleanly-seedable Phase-1 sub-subtree: detection,
queue arbitration, customer selection, and approach (incl. the unreachable
show-picture fallback). The order-intake tail is covered structurally by the
production ``restaurant`` entry once the eye-contact mock bug is fixed.

The detect/arbitrate/approach factories are imported, not re-derived; only the
blackboard seeding and the InitOrderChecklist priming are local.

Run (real hardware/servers):
    ros2 run behavior_tree test-restaurant-collect-order

Run (offline / fully mocked):
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree test-restaurant-collect-order
"""

from __future__ import annotations

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.visualization import create_post_tick_visualizer

from .config import (
    KEY_ACTIVE_CUSTOMER_ID,
    KEY_ACTIVE_CUSTOMER_PICTURE,
    KEY_CUSTOMER_QUEUE,
    KEY_ORDER_CHECKLIST,
    KEY_ORDER_LIST,
    KEY_PICKUP_VERIFIED,
)
from .restaurants import createApproachCustomer, createDetectAndArbitrateCustomers
from .state_nodes import BtNode_InitOrderChecklist


def create_tree():
    root = py_trees.composites.Sequence(
        name="Test collect one order (Phase 1: detect -> approach)", memory=True
    )

    seed = py_trees.composites.Parallel(
        name="Seed blackboard",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in (
        ("Seed caller queue", KEY_CUSTOMER_QUEUE, []),
        ("Seed active caller id", KEY_ACTIVE_CUSTOMER_ID, None),
        ("Seed active caller picture", KEY_ACTIVE_CUSTOMER_PICTURE, ""),
        ("Seed pickup verification", KEY_PICKUP_VERIFIED, False),
        ("Seed order checklist", KEY_ORDER_CHECKLIST, {}),
        ("Seed order list", KEY_ORDER_LIST, []),
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
    root.add_child(
        BtNode_InitOrderChecklist(
            name="Initialize per-order checklist",
            checklist_key=KEY_ORDER_CHECKLIST,
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )

    scan_for_customers = py_trees.composites.Selector(
        name="Scan for customers", memory=True
    )
    for pan, tilt in [(0.0, 35.0), (30.0, 35.0), (-30.0, 35.0)]:
        scan_for_customers.add_child(
            createDetectAndArbitrateCustomers(x=pan, y=tilt)
        )
    root.add_child(scan_for_customers)
    root.add_child(createApproachCustomer())

    root.add_child(py_trees.behaviours.Running("idle"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="test_restaurant_collect_order")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="Restaurant Collect One Order (Phase 1)"
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
