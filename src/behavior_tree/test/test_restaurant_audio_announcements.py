# Copyright 2026 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Restaurant audio-announcement correctness.

Covers the three defects found in the restaurant-2026 audio review:

1. ``BtNode_ConfirmOrder`` / ``BtNode_ServeOrder`` spoke the raw Python list
   repr once Phase 1 started storing ``items: string[]``
   (``BtNode_OrderExtractionAction``) instead of a single string — TTS said
   "left-bracket quote burger quote ...". They must join items for speech.
2. ``createDeliverOrder`` announced ``KEY_CUSTOMER_ORDER`` via
   ``BtNode_ServeOrder`` — at Phase-3 time that key still holds the LAST
   Phase-1 customer's order, so every delivery spoke the wrong customer's
   order. The per-item announce already covers the item; the closing line
   must not read the stale key.
3. ``createBarmanPhase``'s gated Selector fell through to "No customers
   served. Skipping barman." whenever the bar trip or the barman confirmation
   failed — even with a non-empty order list. The skip announcement must be
   reserved for the truly-empty case, with a truthful fallback otherwise
   (still SUCCESS so Phase 3 proceeds).
"""

import importlib
import os
import sys
import types

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.Restaurant.custumNodes import (  # noqa: E402
    BtNode_ConfirmOrder,
    BtNode_ServeOrder,
    format_order_items,
)


# _import_restaurants re-imports behavior_tree.Restaurant.restaurants against
# monkeypatched stubs (same pattern as test_restaurant_state_machine); drop the
# stubbed copy so other test files see the real module again.
@pytest.fixture(autouse=True)
def _purge_stubbed_restaurants():
    """Drop the stub-imported restaurants module after each test."""
    yield
    sys.modules.pop("behavior_tree.Restaurant.restaurants", None)


class _SuccessNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name=name)

    def update(self):
        return py_trees.common.Status.SUCCESS


class _FailNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name=name)

    def update(self):
        return py_trees.common.Status.FAILURE


class _ServeOrderStub(_SuccessNode):
    """Distinct stub so tests can detect BtNode_ServeOrder instantiations."""


def _install_stubs(monkeypatch, confirmation_cls=_SuccessNode):
    rclpy = types.ModuleType("rclpy")

    class _Time:
        def to_msg(self):
            return None

    rclpy.time = types.SimpleNamespace(Time=lambda: _Time())
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)

    geom = types.ModuleType("geometry_msgs.msg")
    geom.Point = type("Point", (), {"__init__": lambda self, x=0.0, y=0.0, z=0.0: None})
    geom.Quaternion = type(
        "Quaternion",
        (),
        {"__init__": lambda self, x=0.0, y=0.0, z=0.0, w=1.0: None},
    )
    geom.Pose = type("Pose", (), {"__init__": lambda self, position=None, orientation=None: None})
    geom.PoseStamped = type("PoseStamped", (), {"__init__": lambda self, header=None, pose=None: None})
    monkeypatch.setitem(sys.modules, "geometry_msgs.msg", geom)

    std = types.ModuleType("std_msgs.msg")
    std.Header = type("Header", (), {"__init__": lambda self, stamp=None, frame_id="": None})
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std)

    audio = types.ModuleType("behavior_tree.nodes.Audio")
    audio.BtNode_Announce = _SuccessNode
    audio.BtNode_GetConfirmationAction = confirmation_cls
    monkeypatch.setitem(sys.modules, "behavior_tree.nodes.Audio", audio)

    base = types.ModuleType("behavior_tree.nodes.BaseBehaviors")
    base.BtNode_WriteToBlackboard = _SuccessNode
    base.BtNode_CheckIfEmpty = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.nodes.BaseBehaviors", base)

    manip = types.ModuleType("behavior_tree.nodes.Manipulation")
    manip.BtNode_MoveArmSingle = _SuccessNode
    manip.BtNode_GripperAction = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.nodes.Manipulation", manip)

    nav = types.ModuleType("behavior_tree.nodes.Navigation")
    nav.BtNode_GotoAction = _SuccessNode
    nav.BtNode_Approach = _SuccessNode
    nav.BtNode_CaptureCurrentPose = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.nodes.Navigation", nav)

    vision = types.ModuleType("behavior_tree.nodes.Vision")
    vision.BtNode_ScanForWavingPerson = _SuccessNode
    vision.BtNode_MaintainEyeContact = _SuccessNode
    vision.BtNode_ShowImage = _SuccessNode
    vision.BtNode_TurnPanTilt = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.nodes.Vision", vision)

    pnp_config = types.ModuleType("behavior_tree.PickAndPlace.config")
    pnp_config.Header = std.Header
    pnp_config.Point = geom.Point
    pnp_config.Pose = geom.Pose
    pnp_config.PoseStamped = geom.PoseStamped
    pnp_config.Quaternion = geom.Quaternion
    monkeypatch.setitem(sys.modules, "behavior_tree.PickAndPlace.config", pnp_config)

    custom = types.ModuleType("behavior_tree.Restaurant.custumNodes")
    custom.BtNode_DetectCallingCustomer = _SuccessNode
    custom.BtNode_TakeOrder = _SuccessNode
    custom.BtNode_ConfirmOrder = _SuccessNode
    custom.BtNode_CommunicateWithBarman = _SuccessNode
    custom.BtNode_DetectTray = _SuccessNode
    custom.BtNode_ServeOrder = _ServeOrderStub
    custom.BtNode_ScanForCallingCustomer = _SuccessNode
    custom.BtNode_RecordOrder = _SuccessNode
    custom.BtNode_FormatOrdersForBarman = _SuccessNode
    custom.BtNode_IterateOrderItems = _SuccessNode
    custom.BtNode_MarkItemDelivered = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.Restaurant.custumNodes", custom)


def _import_restaurants(monkeypatch, confirmation_cls=_SuccessNode):
    _install_stubs(monkeypatch, confirmation_cls=confirmation_cls)
    sys.modules.pop("behavior_tree.Restaurant.restaurants", None)
    return importlib.import_module("behavior_tree.Restaurant.restaurants")


def _set_bb(**values):
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    bb = py_trees.blackboard.Client(name="test_writer")
    for key in values:
        bb.register_key(
            key=key,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
    for key, value in values.items():
        setattr(bb, key, value)


def _tick_until_terminal(root, max_ticks=20):
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(max_ticks):
        tree.tick()
        if root.status != py_trees.common.Status.RUNNING:
            return root.status
    return root.status


def _statuses_by_name(root):
    return {node.name: node.status for node in root.iterate()}


# --- format_order_items: list-valued orders must be joined for speech ------ #


def test_format_two_items_joined_with_and():
    assert format_order_items(["burger", "coke"]) == "burger and coke"


def test_format_three_items_comma_then_and():
    assert format_order_items(["burger", "fries", "coke"]) == "burger, fries and coke"


def test_format_single_item_list():
    assert format_order_items(["coke"]) == "coke"


def test_format_plain_string_passthrough():
    assert format_order_items("a bottle of water") == "a bottle of water"


def test_format_empty_and_none_say_nothing():
    assert format_order_items([]) == "nothing"
    assert format_order_items(None) == "nothing"


# --- BtNode_ConfirmOrder / BtNode_ServeOrder speak joined items ------------ #


def _initialised(node):
    # Force the mock branch so initialise() never builds a real service request
    # (same pattern as test_announce_h_word_pause).
    node.mock_mode = True
    node.initialise()
    return node


def test_confirm_order_speaks_joined_list_not_python_repr():
    _set_bb(confirm_order_items=["burger", "coke"])
    node = _initialised(
        BtNode_ConfirmOrder(name="confirm", bb_order_key="confirm_order_items")
    )
    assert node.given_msg == "I understand your order is burger and coke. Is this correct?"


def test_confirm_order_still_accepts_plain_string():
    _set_bb(confirm_order_items="coke")
    node = _initialised(
        BtNode_ConfirmOrder(name="confirm", bb_order_key="confirm_order_items")
    )
    assert node.given_msg == "I understand your order is coke. Is this correct?"


def test_serve_order_speaks_joined_list_not_python_repr():
    _set_bb(serve_order_items=["burger", "coke"])
    node = _initialised(
        BtNode_ServeOrder(name="serve", bb_order_key="serve_order_items")
    )
    assert node.given_msg == "Here is your order: burger and coke. Enjoy your meal!"


# --- createDeliverOrder: no stale KEY_CUSTOMER_ORDER announcement ---------- #


def test_deliver_order_does_not_announce_stale_customer_order(monkeypatch):
    module = _import_restaurants(monkeypatch)
    deliver = module.createDeliverOrder()
    serve_nodes = [n for n in deliver.iterate() if isinstance(n, _ServeOrderStub)]
    assert serve_nodes == [], (
        "createDeliverOrder still announces KEY_CUSTOMER_ORDER via "
        "BtNode_ServeOrder — at Phase-3 time that key holds the LAST Phase-1 "
        "customer's order, not the one being served"
    )


# --- createBarmanPhase: skip announce only when truly no orders ------------ #


def test_barman_confirm_failure_does_not_claim_no_customers(monkeypatch):
    module = _import_restaurants(monkeypatch, confirmation_cls=_FailNode)
    _set_bb(
        order_list=[
            {"id": 1, "pose": None, "picture_path": "", "items": ["coke"], "delivered_items": []}
        ],
        barman_text="",
    )
    phase = module.createBarmanPhase()
    assert _tick_until_terminal(phase) == py_trees.common.Status.SUCCESS

    statuses = _statuses_by_name(phase)
    assert "Barman not confirmed (continuing)" in statuses, (
        "expected a truthful fallback announcement when the barman "
        "confirmation fails with orders pending"
    )
    assert statuses["Barman not confirmed (continuing)"] == py_trees.common.Status.SUCCESS
    skip_status = statuses.get(
        "Announce skip (no orders)", py_trees.common.Status.INVALID
    )
    assert skip_status == py_trees.common.Status.INVALID, (
        "'No customers served' must not be announced when orders exist"
    )


def test_barman_phase_skips_with_announce_when_no_orders(monkeypatch):
    module = _import_restaurants(monkeypatch)
    _set_bb(order_list=[], barman_text="")
    phase = module.createBarmanPhase()
    assert _tick_until_terminal(phase) == py_trees.common.Status.SUCCESS

    statuses = _statuses_by_name(phase)
    assert statuses.get("Announce skip (no orders)") == py_trees.common.Status.SUCCESS
    fallback_status = statuses.get(
        "Barman not confirmed (continuing)", py_trees.common.Status.INVALID
    )
    assert fallback_status == py_trees.common.Status.INVALID
