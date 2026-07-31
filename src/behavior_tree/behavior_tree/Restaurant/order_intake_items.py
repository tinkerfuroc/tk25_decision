from __future__ import annotations

"""Restaurant order intake via the item-list extraction action (NEW module).

Swaps Phase-1 order *capture* from the name+drink action (``order_intake_name_drink.py``)
to tk_24_audio's ``order_extraction_action`` (``BtNode_OrderExtractionAction``) — a
free-form Qwen-Omni action that returns only ``items: string[]`` (food + drinks
combined), with no name capture at all.

Why: Restaurant doesn't need the customer's name, so the name+drink action's
"partial extraction" contract (both name AND drink must be heard, see
``name_drink_extraction_ac.py``) was forcing unnecessary re-prompts whenever a
name was missed but the order was heard fine. ``order_extraction_action`` was
purpose-built for this (see
``tk_24_audio/docs/superpowers/specs/2026-06-27-order-extraction-action-design.md``)
but was never wired into a Restaurant tree until now.

Adapted from ``order_intake_name_drink.py`` (prompt -> ``Retry`` around the
extraction leaf). Restaurant mapping:

  * ``items`` -> the order item(s) -> ``KEY_CUSTOMER_ORDER`` (recorded as the order)

Everything downstream is byte-for-byte the canonical Phase-1 flow: ``BtNode_ConfirmOrder``
reads ``KEY_CUSTOMER_ORDER``, ``BtNode_GetConfirmationAction`` confirms, ``BtNode_RecordOrder``
appends to ``KEY_ORDER_LIST`` (its `items` field already accepts a list natively —
see its docstring). Only the order-*capture* leaf changes; every other Phase-1
factory is imported unchanged from ``restaurants.py`` / ``custumNodes.py`` /
``state_nodes.py``. This file adds NOTHING to the canonical ``restaurant`` entry —
it is consumed only by ``restaurant_v2.py`` (``restaurant-2026``) and the standalone
smoke test below.

Standalone smoke test (announce -> order extraction -> confirm -> echo)::

    ros2 run behavior_tree restaurant-test-order-items

Fully offline (KEYPRESS auto-advance; mock writes items=['burger', 'coke'])::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree restaurant-test-order-items
"""

import py_trees
import py_trees_ros
import rclpy

from behavior_tree.TemplateNodes.Audio import (
    BtNode_Announce,
    BtNode_GetConfirmationAction,
    BtNode_OrderExtractionAction,
)
from behavior_tree.TemplateNodes.Vision import BtNode_MaintainEyeContact
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
from behavior_tree.visualization import create_post_tick_visualizer

# Reuse the canonical Phase-1 factories + helper nodes unchanged.
from .restaurants import (
    createScanForUpToNCustomers,
)
from .approach_narrated import createApproachCustomerNarrated
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
    KEY_REFEREE_ANNOUNCED,
)

# Recording window handed to the order-extraction server. Matches the
# name+drink action's 7 s default.
ORDER_ITEMS_TIMEOUT_S = 7.0


# Offline audio-test affordance (restaurant-2026 only). When True, Phase 1
# skips the person-scan sweep and seeds a synthetic active customer so the tree
# jumps straight to the real-audio order extraction + confirmation. Leave False
# for production. Mirrors GPSR/EGPSR's construction-time USE_NEW_SCAN_WAVING
# toggle -- a compile-time subtree swap, NOT a runtime `if MOCK_MODE` branch.
MOCK_SEED_CUSTOMER = False


def _seed_write(key: str, obj):
    """One root-namespace blackboard write for the mock-customer seed."""
    return BtNode_WriteToBlackboard(
        name=f"Seed {key}",
        bb_namespace="",
        bb_source=None,
        bb_key=key,
        object=obj,
    )


def _createSeedCustomerSubtree(customer_id: int) -> py_trees.composites.Sequence:
    """Seed the post-SelectNextQueuedCustomer blackboard state for one synthetic
    active customer, replacing the Phase-1 scan under MOCK_SEED_CUSTOMER.

    Reproduces the fields BtNode_SelectNextQueuedCustomer writes for a single
    active customer (active id, location, picture, and a single-entry queue
    marked "active"). Note: the real scan *appends* to the queue across
    customers; this seed overwrites it with one entry per order, which is
    inconsequential since nothing downstream reads multi-entry queue history.
    The customer location is a placeholder PoseStamped -- mocked
    BtNode_Approach is immediate and never navigates to it -- and the picture
    path is left empty.
    """
    pose = PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(
            position=Point(x=1.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    entry = {
        "id": customer_id,
        "pose": pose,
        "picture_path": "",
        "timestamp": 0.0,
        "confidence": 1.0,
        "status": "active",
    }
    seq = py_trees.composites.Sequence(
        name=f"Seed mock customer {customer_id}", memory=True
    )
    seq.add_child(_seed_write(KEY_CUSTOMER_QUEUE, [entry]))
    seq.add_child(_seed_write(KEY_ACTIVE_CUSTOMER_ID, customer_id))
    seq.add_child(_seed_write(KEY_CUSTOMER_LOCATION, pose))
    seq.add_child(_seed_write(KEY_ACTIVE_CUSTOMER_PICTURE, ""))
    return seq


def createTakeOrderViaItems(
    order_key: str = KEY_CUSTOMER_ORDER,
    timeout: float = ORDER_ITEMS_TIMEOUT_S,
) -> py_trees.composites.Sequence:
    """Prompt the customer, then capture the order in one free-form utterance.

    Mirrors ``order_intake_name_drink.createTakeOrderViaNameDrink``: a single
    prompt feeds ``BtNode_OrderExtractionAction`` (free speech -> Qwen-Omni ->
    ``items: string[]``), wrapped in a ``Retry`` so a fully-failed extraction
    (server abort, no items heard) re-prompts instead of failing the intake.
    """
    root = py_trees.composites.Sequence(name="Take order (items)", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Prompt for order",
            bb_source=None,
            message="What would you like to order?",
        )
    )
    root.add_child(
        py_trees.decorators.Retry(
            name="Retry order extract",
            child=BtNode_OrderExtractionAction(
                name="Extract order",
                bb_items_key=order_key,
                timeout=timeout,
            ),
            num_failures=3,
        )
    )
    return root


def createTakeAndConfirmOrderItems() -> py_trees.composites.Parallel:
    """Order intake + confirmation loop, using the item-list extraction action.

    Drop-in replacement for ``restaurants.createTakeAndConfirmOrder`` /
    ``order_intake_name_drink.createTakeAndConfirmOrderNameDrink``: same
    confirm-loop shape (capture -> confirm -> get-confirmation, retried so a "no"
    re-asks) and the same maintain-eye-contact parallel, but the capture leaf is
    the free-form order-items action instead of name+drink or the wordlist
    ``BtNode_TakeOrder``.
    """
    root = py_trees.composites.Sequence(name="Take and confirm order (items)", memory=True)
    root.add_child(
        BtNode_Announce(
            name="Order engagement prompt",
            bb_source=None,
            message="Hi.",
        )
    )

    # Note: createTakeOrderViaItems() speaks its own "What would you like to
    # order?" prompt on every (re-)attempt, so the greeting above stays a bare
    # "Hi." rather than v1's combined "Hi. What would you like to order?" --
    # otherwise the question would be asked twice back-to-back on the first try.
    order_loop = py_trees.composites.Sequence(name="Order taking loop", memory=True)
    order_loop.add_child(createTakeOrderViaItems())
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
        name="Take order with eye contact (items)",
        policy=py_trees.common.ParallelPolicy.SuccessOnSelected([root]),
        children=[maintain_eye_contact, root],
    )


def createCollectOneOrderItems(seed_customer_id: int = 1) -> py_trees.composites.Sequence:
    """One Phase-1 pass: detect -> approach -> order-items -> record.

    Mirrors ``restaurants.createCollectOneOrder`` exactly, swapping only the
    take-and-confirm leaf for the item-list variant. Detect, approach, checklist
    init, active-customer guard, record, and close are the canonical nodes.

    Args:
        seed_customer_id: id for the seeded synthetic customer when
            MOCK_SEED_CUSTOMER is enabled; ignored on the normal scan path.
    """
    root = py_trees.composites.Sequence(name="Collect one order (items)", memory=True)
    root.add_child(
        BtNode_InitOrderChecklist(
            name="Initialize per-order checklist",
            checklist_key=KEY_ORDER_CHECKLIST,
            pickup_verified_key=KEY_PICKUP_VERIFIED,
        )
    )
    if MOCK_SEED_CUSTOMER:
        root.add_child(_createSeedCustomerSubtree(seed_customer_id))
    else:
        root.add_child(
            createScanForUpToNCustomers(
                # BtNode_TurnPanTilt takes (x=pan, y=tilt) in degrees -- tilt
                # fixed at 35 deg for every position (a consistent
                # look-for-a-person angle), pan sweeps the room.
                scan_positions=[
                    (0.0, 35.0),
                    (30.0, 35.0),
                    (60.0, 35.0),
                    (-30.0, 35.0),
                    (-60.0, 35.0),
                    (-120.0, 35.0),
                    (120.0, 35.0),
                ],
                n_gate=2,
            )
        )
    root.add_child(createApproachCustomerNarrated())
    root.add_child(
        BtNode_RequireActiveCustomer(
            name="Require active customer (post-approach)",
            active_id_key=KEY_ACTIVE_CUSTOMER_ID,
        )
    )
    root.add_child(createTakeAndConfirmOrderItems())
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


def createCollectOrdersPhaseItems() -> py_trees.composites.Sequence:
    """Phase 1 (item-list intake): two orders, one retry per order.

    Mirrors ``restaurants.createCollectOrdersPhase``; substitute this into a
    Restaurant tree to collect orders via the free-form order-items action.
    """
    root = py_trees.composites.Sequence(name="Collect orders (2x, items)", memory=True)
    # Task-level init of the referee-spiel latch: the first detection of the run
    # speaks the full spiel, every later detection the short reminder. Runs once,
    # before either order's scan; never reset.
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Init referee-spiel flag",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_REFEREE_ANNOUNCED,
            object=False,
        )
    )
    for i in range(2):
        root.add_child(
            py_trees.decorators.Retry(
                name=f"retry collect order {i + 1}",
                child=createCollectOneOrderItems(seed_customer_id=i + 1),
                num_failures=2,
            )
        )
    return root


# --------------------------------------------------------------------------- #
# Standalone smoke test: announce -> order extraction -> confirm -> echo.
# --------------------------------------------------------------------------- #
def create_tree() -> py_trees.behaviour.Behaviour:
    """Minimal isolation harness for the order-items capture leaf."""
    root = py_trees.composites.Sequence(name="Order items intake test", memory=True)
    root.add_child(createTakeOrderViaItems())
    root.add_child(
        BtNode_ConfirmOrder(name="Confirm order", bb_order_key=KEY_CUSTOMER_ORDER)
    )
    root.add_child(
        BtNode_Announce(
            name="Echo order",
            bb_source=KEY_CUSTOMER_ORDER,
            message="Thank you. I have your order as",
        )
    )
    root.add_child(py_trees.behaviours.Running("idle (Ctrl+C to exit)"))
    return root


def main():
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree())
    tree.setup(timeout=15, node_name="restaurant_order_items")
    print_tree, shutdown_visualizer, _ = create_post_tick_visualizer(
        title="restaurant-test-order-items"
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
