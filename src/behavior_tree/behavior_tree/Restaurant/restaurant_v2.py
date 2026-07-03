from __future__ import annotations

"""Restaurant — finalized 2026 task tree (entry: ``restaurant-2026``).

A NEW parallel production tree that does not touch the canonical
``createRestaurantTask`` (``restaurants.py`` / ``restaurant``). Restaurant's
production pipeline is already mature, so this v2 changes exactly TWO things —
both clear, competition-affecting — and reuses every other phase factory
unchanged:

  * **Kitchen-bar pose: captured, not hardcoded to map origin.** The canonical
    ``createConstantWriter`` writes ``KEY_KITCHEN_BAR_POSE`` to a hardcoded
    ``(0,0,0)`` map pose (the real ``BtNode_CaptureCurrentPose`` is commented
    out), so every Phase-2/Phase-3 bar return drives to the map origin instead of
    the operator-placed bar. Here the bar pose is a
    ``Selector[ capture-current-pose (×3 retry), hardcoded-origin fallback ]``:
    it captures the robot's actual start pose (operator places the robot at the
    bar) when TF is available, and only falls back to the origin write if capture
    fails outright — strictly better than the canonical tree, never worse (the
    key is always set).
  * **Order capture: free-form item-list action, not a 12-word wordlist.** The
    canonical Phase-1 uses ``BtNode_TakeOrder`` (``phrase_extraction_action`` with
    a fixed ``MENU_WORDLIST``), which drops any item off the list. Phase 1 here is
    ``createCollectOrdersPhaseItems`` — identical detect/approach/record/close,
    but the order-capture leaf is ``BtNode_OrderExtractionAction`` (free-form
    Qwen-Omni, returns ``items: string[]``, no name capture). The customer can
    order anything; ``items`` -> ``KEY_CUSTOMER_ORDER``. See ``order_intake_items.py``.

Intentionally NOT changed (deliberate trade-offs / missing infra — surfaced to
the user, not silently swapped):
  * Per-item pickup stays the barman handover (``createPickupVerification``):
    autonomous grasp-from-bar is a strategy + perception decision, not a drop-in.
  * ``BtNode_ShowImage`` stays a stub (no on-robot display server exists yet).
  * Tray transport / TSP delivery ordering remain out of scope (need a tray
    ``BtNode_Pick`` wrapper + a product decision).

Run::

    ros2 run behavior_tree restaurant-2026

Fully offline (no servers, auto-advance)::

    BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json \
        ros2 run behavior_tree restaurant-2026

To skip Phase-1 person-scanning entirely and jump straight to the real-audio
order extraction/confirmation (everything-but-audio mocked), set
``order_intake_items.MOCK_SEED_CUSTOMER = True`` -- it seeds a synthetic active
customer per order instead of sweeping for waving persons.
"""

import py_trees
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header

from behavior_tree.runtime import run_tree
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle
from behavior_tree.TemplateNodes.Navigation import BtNode_CaptureCurrentPose

# Reuse the canonical Restaurant phase factories unchanged.
from .restaurants import (
    createBarmanPhase,
    createDeliverAllItemsPhase,
    createOptionalTrayTransport,
)
# Phase-1 order capture via the free-form item-list extraction action (NEW),
# replacing the canonical wordlist intake (createCollectOrdersPhase).
from .order_intake_items import createCollectOrdersPhaseItems
from .config import (
    ARM_POS_NAVIGATING,
    ARM_POS_SERVING,
    KEY_ACTIVE_CUSTOMER_ID,
    KEY_ACTIVE_CUSTOMER_PICTURE,
    KEY_ARM_NAVIGATING,
    KEY_ARM_SERVING,
    KEY_BARMAN_TEXT,
    KEY_CUSTOMER_QUEUE,
    KEY_KITCHEN_BAR_POSE,
    KEY_ORDER_CHECKLIST,
    KEY_ORDER_LIST,
    KEY_PICKUP_VERIFIED,
)


def _origin_bar_pose() -> PoseStamped:
    """The canonical hardcoded fallback bar pose (map origin)."""
    return PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )


def _createConstantWriterCaptured() -> py_trees.composites.Parallel:
    """Canonical constant writer, but the bar pose is captured (origin fallback)."""
    root = py_trees.composites.Parallel(
        name="Write constants (captured bar pose)",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    for name, key, value in (
        ("Write arm navigating pose", KEY_ARM_NAVIGATING, ARM_POS_NAVIGATING),
        ("Write arm serving pose", KEY_ARM_SERVING, ARM_POS_SERVING),
        ("Initialize caller queue", KEY_CUSTOMER_QUEUE, []),
        ("Initialize active caller id", KEY_ACTIVE_CUSTOMER_ID, None),
        ("Initialize active caller picture", KEY_ACTIVE_CUSTOMER_PICTURE, ""),
        ("Initialize pickup verification", KEY_PICKUP_VERIFIED, False),
        ("Initialize order checklist", KEY_ORDER_CHECKLIST, {}),
        ("Initialize order list", KEY_ORDER_LIST, []),
        ("Initialize barman text", KEY_BARMAN_TEXT, ""),
    ):
        root.add_child(
            BtNode_WriteToBlackboard(
                name=name, bb_namespace="", bb_source=None, bb_key=key, object=value
            )
        )

    record_parallel = py_trees.composites.Parallel(
        name="Record bar location",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(),
    )
    record_parallel.add_child(
        BtNode_Announce(
            name="Announce initialization",
            bb_source=None,
            message="Recording location of restaurant bar.",
        )
    )
    # Capture the robot's actual start pose as the kitchen bar; if TF capture
    # fails after retries, fall back to the canonical hardcoded origin so the
    # key is always populated.
    bar_pose = py_trees.composites.Selector(
        name="Bar pose: capture or origin fallback", memory=True
    )
    bar_pose.add_child(
        py_trees.decorators.Retry(
            name="Retry capture task start pose",
            child=BtNode_CaptureCurrentPose(
                name="Capture task start pose as kitchen bar",
                bb_key=KEY_KITCHEN_BAR_POSE,
            ),
            num_failures=3,
        )
    )
    bar_pose.add_child(
        BtNode_WriteToBlackboard(
            name="Fallback: write origin as kitchen bar",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_KITCHEN_BAR_POSE,
            object=_origin_bar_pose(),
        )
    )
    record_parallel.add_child(bar_pose)
    root.add_child(record_parallel)
    return root


def createRestaurantTask2026() -> py_trees.behaviour.Behaviour:
    """Canonical ``createRestaurantTask`` with the captured-bar-pose constant writer."""
    root = py_trees.composites.Sequence(name="Restaurant Task 2026", memory=True)
    root.add_child(_createConstantWriterCaptured())
    root.add_child(
        py_trees.decorators.Retry(
            name="retry arm setup",
            child=BtNode_MoveArmSingle(
                name="Move arm to navigation pose",
                action_name="joint_move_action",
                arm_pose_bb_key=KEY_ARM_NAVIGATING,
                add_octomap=False,
            ),
            num_failures=3,
        )
    )
    root.add_child(
        BtNode_Announce(
            name="Start announcement",
            bb_source=None,
            message="Restaurant service started. I'm ready for orders.",
        )
    )

    root.add_child(createCollectOrdersPhaseItems())
    root.add_child(
        BtNode_Announce(
            name="Phase 1 complete announcement",
            bb_source=None,
            message="Order collection phase complete. I'll now proceed to the barman.",
        )
    )
    root.add_child(createOptionalTrayTransport())
    root.add_child(createBarmanPhase())
    root.add_child(createDeliverAllItemsPhase())

    root.add_child(
        BtNode_Announce(
            name="Task completion",
            bb_source=None,
            message="Restaurant service complete. Thank you.",
        )
    )
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """Alias for the offline smoke harness."""
    return createRestaurantTask2026()


def main():
    run_tree(
        createRestaurantTask2026,
        period_ms=500.0,
        title="Restaurant 2026",
        node_name="restaurant_2026",
    )


if __name__ == "__main__":
    main()
