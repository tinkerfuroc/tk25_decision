from __future__ import annotations

"""Standalone smoke test: order intake + confirmation dialogue (test tree).

A minimal, flat behaviour tree that exercises the audio order-taking leaves in
isolation — no navigation, no perception, no customer queue:

    announce prompt -> order extraction -> repeat order -> get confirmation

Distinct from ``order_intake_items.py`` (which wraps the same leaves in the full
Phase-1 detect/approach/record flow): this is a single-pass, one-customer dialogue
harness for tuning the audio nodes and prompts on the robot.

Node mapping:
  * ``BtNode_Announce``              -> speaks the fixed order prompt.
  * ``BtNode_OrderExtractionAction`` -> free speech -> Qwen-Omni -> ``items: string[]``,
                                        written to ``KEY_CUSTOMER_ORDER``.
  * ``BtNode_ConfirmOrder``          -> the "repeat" step. Reads ``KEY_CUSTOMER_ORDER``
                                        and speaks "I understand your order is [...].
                                        Is this correct?". Used instead of a plain
                                        ``BtNode_Announce(bb_source=...)`` because the
                                        order key holds a *list*, and plain Announce
                                        asserts a ``str`` and would drop the items.
  * ``BtNode_GetConfirmationAction`` -> captures the customer's yes/no.

Single pass, no ``Retry``: a "no" (or a failed extraction) simply fails the tree.

Run (real audio servers):
    ros2 run behavior_tree restaurant-test-order-confirm

Run (fully offline / mocked; extraction writes ['burger', 'coke'], keypress-advance):
    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree restaurant-test-order-confirm
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_OrderExtractionAction,
)
from behavior_tree.visualization import create_post_tick_visualizer

from .custumNodes import BtNode_ConfirmOrder
from .config import KEY_CUSTOMER_ORDER

# Recording window handed to the order-extraction server (matches
# order_intake_items.ORDER_ITEMS_TIMEOUT_S).
ORDER_TIMEOUT_S = 7.0
CONFIRM_TIMEOUT_S = 15.0


def create_tree() -> py_trees.behaviour.Behaviour:
    """announce -> extract -> repeat -> confirm, then idle."""
    root = py_trees.composites.Sequence(name="Order confirm test", memory=True)

    root.add_child(
        BtNode_Announce(
            name="Prompt for order",
            bb_source=None,
            message="Tell me both of your order, loud and clear.",
        )
    )
    root.add_child(
        BtNode_OrderExtractionAction(
            name="Extract order",
            bb_items_key=KEY_CUSTOMER_ORDER,
            timeout=ORDER_TIMEOUT_S,
        )
    )
    root.add_child(
        BtNode_ConfirmOrder(
            name="Repeat order back",
            bb_order_key=KEY_CUSTOMER_ORDER,
        )
    )
    root.add_child(
        BtNode_GetConfirmationAction(
            name="Get confirmation",
            timeout=CONFIRM_TIMEOUT_S,
        )
    )
    root.add_child(py_trees.behaviours.Running("idle (Ctrl+C to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="restaurant_order_confirm")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="restaurant-test-order-confirm"
    )
    tree.tick_tock(period_ms=300.0, post_tick_handler=print_tree)
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
