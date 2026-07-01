from __future__ import annotations

"""Restaurant order intake via the name+drink extraction action (NEW module).

Swaps Phase-1 order *capture* from the hardcoded-wordlist ``phrase_extraction_action``
(``BtNode_TakeOrder``) to tk_24_audio's free-form ``name_drink_extraction_action``
(``BtNode_NameDrinkExtractionAction``) — the SAME server HRI uses to capture a
guest's name + favourite drink in one Qwen-Omni utterance, no wordlist.

Why: ``BtNode_TakeOrder.MENU_WORDLIST`` is a fixed 12-word list; any item off it
is mis-heard or dropped. The name+drink action parses ``{name, drink}`` from free
speech, so the customer can order anything *and* we capture who ordered it (used
for a friendlier confirmation + a delivery callout).

Adapted from HRI's ``hri._create_get_name_drink`` (prompt -> ``Retry`` around the
extraction leaf). Restaurant mapping:

  * ``drink`` -> the order item(s)            -> ``KEY_CUSTOMER_ORDER`` (recorded as the order)
  * ``name``  -> the customer's name (optional) -> ``KEY_CUSTOMER_NAME`` (best-effort label)

Everything downstream is byte-for-byte the canonical Phase-1 flow: ``BtNode_ConfirmOrder``
reads ``KEY_CUSTOMER_ORDER``, ``BtNode_GetConfirmationAction`` confirms, ``BtNode_RecordOrder``
appends to ``KEY_ORDER_LIST``. Only the order-*capture* leaf changes; every other
Phase-1 factory is imported unchanged from ``restaurants.py`` / ``custumNodes.py`` /
``state_nodes.py``. This file adds NOTHING to the canonical ``restaurant`` entry —
it is consumed only by ``restaurant_v2.py`` (``restaurant-2026``) and the standalone
smoke test below.

Standalone smoke test (announce -> name+drink order -> confirm -> echo)::

    ros2 run behavior_tree restaurant-test-name-drink-order

Fully offline (KEYPRESS auto-advance; mock writes name='Alice', drink='orange juice')::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree restaurant-test-name-drink-order
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_NameDrinkExtractionAction,
)
from behavior_tree.TemplateNodes.Vision import BtNode_MaintainEyeContact
from behavior_tree.visualization import create_post_tick_visualizer

# Reuse the canonical Phase-1 factories + helper nodes unchanged.
from .restaurants import (
    createApproachCustomer,
    createDetectAndArbitrateCustomers,
)
from .custumNodes import BtNode_ConfirmOrder, BtNode_RecordOrder
from .state_nodes import (
    BtNode_CloseActiveCustomer,
    BtNode_InitOrderChecklist,
    BtNode_RequireActiveCustomer,
    BtNode_UpdateChecklistFlag,
)
from .config import (
    KEY_ACTIVE_CUSTOMER_ID,
    KEY_ACTIVE_CUSTOMER_PICTURE,
    KEY_CUSTOMER_LOCATION,
    KEY_CUSTOMER_ORDER,
    KEY_CUSTOMER_QUEUE,
    KEY_ORDER_CHECKLIST,
    KEY_ORDER_LIST,
    KEY_PICKUP_VERIFIED,
)

# The customer's spoken name from the extraction (best-effort). Restaurant config
# has no name key — keep it module-local so we don't touch shared config.
KEY_CUSTOMER_NAME = "customer_name"

# Recording window handed to the name+drink server. Matches HRI's 7 s default —
# the server records a short utterance, then Qwen-Omni parses it on top of this.
NAME_DRINK_TIMEOUT_S = 7.0


def createTakeOrderViaNameDrink(
    order_key: str = KEY_CUSTOMER_ORDER,
    name_key: str = KEY_CUSTOMER_NAME,
    timeout: float = NAME_DRINK_TIMEOUT_S,
) -> py_trees.composites.Sequence:
    """Prompt the customer, then capture name+order in one free-form utterance.

    Mirrors ``hri._create_get_name_drink``: a single prompt feeds
    ``BtNode_NameDrinkExtractionAction`` (free speech -> Qwen-Omni -> ``{name, drink}``),
    wrapped in a ``Retry`` so a fully-failed extraction (server abort) re-prompts
    instead of failing the intake. ``drink`` is written to ``order_key`` (the order),
    ``name`` to ``name_key`` (optional label).
    """
    root = py_trees.composites.Sequence(name="Take order (name+drink)", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Prompt for name and order",
            bb_source=None,
            message="May I have your name, and what would you like to order?",
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry name+order extract",
            child=BtNode_NameDrinkExtractionAction(
                name="Extract name and order",
                bb_name_key=name_key,
                bb_drink_key=order_key,
                timeout=timeout,
            ),
            num_failures=3,
        )
    )
    return root


def createTakeAndConfirmOrderNameDrink() -> py_trees.composites.Parallel:
    """Order intake + confirmation loop, using the name+drink extraction action.

    Drop-in replacement for ``restaurants.createTakeAndConfirmOrder``: same
    confirm-loop shape (capture -> confirm -> get-confirmation, retried so a "no"
    re-asks) and the same maintain-eye-contact parallel, but the capture leaf is
    the free-form name+drink action instead of the wordlist ``BtNode_TakeOrder``.
    """
    root = py_trees.composites.Sequence(name="Take and confirm order (name+drink)", memory=True)

    order_loop = py_trees.composites.Sequence(name="Order taking loop", memory=True)
    order_loop.add_child(createTakeOrderViaNameDrink())
    order_loop.add_child(
        BtNode_ConfirmOrder(name="Confirm order", bb_order_key=KEY_CUSTOMER_ORDER)
    )
    order_loop.add_child(
        BtNode_GetConfirmationAction(name="Get confirmation", timeout=5.0)
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="retry order taking",
            child=order_loop,
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_UpdateChecklistFlag(
            name="Mark order confirmed",
            checklist_key=KEY_ORDER_CHECKLIST,
            flag="order_confirmed",
            value=True,
        )
    )

    maintain_eye_contact = BtNode_MaintainEyeContact(name="Maintain eye contact (order)")
    return py_trees.composites.Parallel(
        name="Take order with eye contact (name+drink)",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([root]),
        children=[maintain_eye_contact, root],
    )


def createCollectOneOrderNameDrink() -> py_trees.composites.Sequence:
    """One Phase-1 pass: detect -> approach -> name+drink order -> record.

    Mirrors ``restaurants.createCollectOneOrder`` exactly, swapping only the
    take-and-confirm leaf for the name+drink variant. Detect, approach, checklist
    init, active-customer guard, record, and close are the canonical nodes.
    """
    root = py_trees.composites.Sequence(name="Collect one order (name+drink)", memory=True)
    root.add_child(
        BtNode_InitOrderChecklist(
            name="Initialize per-order checklist",
            checklist_key=KEY_ORDER_CHECKLIST,
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )
    scan_for_customers = py_trees.composites.Selector(name="Scan for customers", memory=True)
    for scan_pos in [(0.0, 35.0), (30.0, 35.0), (-30.0, 35.0)]:
        scan_for_customers.add_child(
            createDetectAndArbitrateCustomers(x=scan_pos[0], y=scan_pos[1])
        )
    root.add_child(scan_for_customers)
    root.add_child(createApproachCustomer())
    root.add_child(
        BtNode_RequireActiveCustomer(
            name="Require active customer (post-approach)",
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    root.add_child(createTakeAndConfirmOrderNameDrink())
    root.add_child(
        BtNode_RecordOrder(
            name="Record order to order list",
            bb_key_order_list=KEY_ORDER_LIST,
            bb_key_cur_id=KEY_ACTIVE_CUSTOMER_ID,
            bb_key_cur_pose=KEY_CUSTOMER_LOCATION,
            bb_key_cur_picture=KEY_ACTIVE_CUSTOMER_PICTURE,
            bb_key_cur_order_items=KEY_CUSTOMER_ORDER,
        )
    )
    root.add_child(
        BtNode_CloseActiveCustomer(
            name="Close active caller (order recorded)",
            queue_key=KEY_CUSTOMER_QUEUE,
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    return root


def createCollectOrdersPhaseNameDrink() -> py_trees.composites.Sequence:
    """Phase 1 (name+drink intake): two orders, one retry per order.

    Mirrors ``restaurants.createCollectOrdersPhase``; substitute this into a
    Restaurant tree to collect orders via the free-form name+drink action.
    """
    root = py_trees.composites.Sequence(name="Collect orders (2x, name+drink)", memory=True)
    for i in range(2):
        root.add_child(
            py_trees.decorators.Retry(
                name=f"retry collect order {i + 1}",
                child=createCollectOneOrderNameDrink(),
                num_failures=2,
            )
        )
    return root


# --------------------------------------------------------------------------- #
# Standalone smoke test: announce -> name+order extraction -> confirm -> echo.
# --------------------------------------------------------------------------- #
def create_tree() -> py_trees.behaviour.Behaviour:
    """Minimal isolation harness for the name+drink order-capture leaf."""
    root = py_trees.composites.Sequence(name="Name+drink order intake test", memory=True)
    root.add_child(createTakeOrderViaNameDrink())
    root.add_child(
        BtNode_ConfirmOrder(name="Confirm order", bb_order_key=KEY_CUSTOMER_ORDER)
    )
    root.add_child(
        BtNode_Announce(
            name="Echo customer name",
            bb_source=KEY_CUSTOMER_NAME,
            message="Thank you. I have your name as",
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Echo order",
            bb_source=KEY_CUSTOMER_ORDER,
            message="and your order is",
        )
    )
    root.add_child(py_trees.behaviours.Running("idle (Ctrl+C to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="restaurant_name_drink_order")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="restaurant-test-name-drink-order"
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
