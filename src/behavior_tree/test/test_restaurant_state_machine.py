import importlib
import sys
import types

import py_trees

from behavior_tree.Restaurant.state_nodes import (
    BtNode_AddDetectedCustomerToBatch,
    BtNode_AdvanceBatchIndex,
    BtNode_BuildBatchOrdersSummary,
    BtNode_InitCustomerBatch,
    BtNode_RequirePickupVerified,
    BtNode_ResetBatchIndex,
    BtNode_SelectNextCustomer,
    BtNode_SelectBatchCustomerByIndex,
    BtNode_StoreOrderForActiveBatchCustomer,
)


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


def _install_stubs(monkeypatch):
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

    audio = types.ModuleType("behavior_tree.TemplateNodes.Audio")
    audio.BtNode_Announce = _SuccessNode
    audio.BtNode_GetConfirmation = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Audio", audio)

    base = types.ModuleType("behavior_tree.TemplateNodes.BaseBehaviors")
    base.BtNode_WriteToBlackboard = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.BaseBehaviors", base)

    manip = types.ModuleType("behavior_tree.TemplateNodes.Manipulation")
    manip.BtNode_MoveArmSingle = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Manipulation", manip)

    nav = types.ModuleType("behavior_tree.TemplateNodes.Navigation")
    nav.BtNode_GotoAction = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Navigation", nav)

    custom = types.ModuleType("behavior_tree.Restaurant.custumNodes")
    custom.BtNode_DetectCallingCustomer = _SuccessNode
    custom.BtNode_TakeOrder = _SuccessNode
    custom.BtNode_ConfirmOrder = _SuccessNode
    custom.BtNode_CommunicateWithBarman = _SuccessNode
    custom.BtNode_DetectTray = _SuccessNode
    custom.BtNode_ServeOrder = _SuccessNode
    custom.BtNode_ScanForCallingCustomer = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.Restaurant.custumNodes", custom)


def _import_restaurants(monkeypatch):
    _install_stubs(monkeypatch)
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


def _get_bb(*keys):
    bb = py_trees.blackboard.Client(name="test_reader")
    for key in keys:
        bb.register_key(
            key=key,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
    return {key: getattr(bb, key) for key in keys}


def _tick_once(root):
    tree = py_trees.trees.BehaviourTree(root)
    tree.tick()
    return root.status


def _tick_until_terminal(root, max_ticks=20):
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(max_ticks):
        tree.tick()
        if root.status != py_trees.common.Status.RUNNING:
            return root.status
    return root.status


def test_simultaneous_callers_select_oldest():
    queue = [
        {"id": 2, "pose": "p2", "timestamp": 2.0, "confidence": 0.8, "status": "queued"},
        {"id": 1, "pose": "p1", "timestamp": 1.0, "confidence": 0.9, "status": "queued"},
    ]
    _set_bb(customer_queue=queue, customer_location=None, active_customer_id=None)
    node = BtNode_SelectNextCustomer(
        name="select",
        queue_key="customer_queue",
        selected_pose_key="customer_location",
        active_id_key="active_customer_id",
    )
    assert _tick_once(node) == py_trees.common.Status.SUCCESS
    bb = _get_bb("active_customer_id", "customer_location", "customer_queue")
    assert bb["active_customer_id"] == 1
    assert bb["customer_location"] == "p1"
    active = [item for item in bb["customer_queue"] if item["id"] == 1][0]
    assert active["status"] == "active"


def test_unreachable_customer_takes_partial_score_path(monkeypatch):
    restaurants = _import_restaurants(monkeypatch)
    monkeypatch.setattr(restaurants, "BtNode_GotoAction", _FailNode)
    monkeypatch.setattr(restaurants, "BtNode_Announce", _SuccessNode)

    _set_bb(
        order_checklist={"detected": True},
        customer_queue=[{"id": 4, "status": "active"}],
        active_customer_id=4,
    )
    root = restaurants.createApproachCustomer()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
    bb = _get_bb("order_checklist", "active_customer_id")
    assert bb["order_checklist"]["partial_score"] is True
    assert bb["active_customer_id"] is None


def test_pickup_verification_gate():
    _set_bb(pickup_verified=False)
    node = BtNode_RequirePickupVerified(
        name="pickup gate",
        pickup_verified_key="pickup_verified",
    )
    assert _tick_once(node) == py_trees.common.Status.FAILURE
    _set_bb(pickup_verified=True)
    assert _tick_once(node) == py_trees.common.Status.SUCCESS


def test_tray_and_non_tray_branches_are_executable(monkeypatch):
    restaurants = _import_restaurants(monkeypatch)
    monkeypatch.setattr(restaurants, "BtNode_Announce", _SuccessNode)

    monkeypatch.setattr(restaurants, "BtNode_DetectTray", _SuccessNode)
    with_tray = restaurants.createOptionalTrayTransport()
    assert _tick_once(with_tray) == py_trees.common.Status.SUCCESS

    monkeypatch.setattr(restaurants, "BtNode_DetectTray", _FailNode)
    without_tray = restaurants.createOptionalTrayTransport()
    assert _tick_once(without_tray) == py_trees.common.Status.SUCCESS


def test_init_customer_batch_sets_defaults():
    node = BtNode_InitCustomerBatch(
        name="init batch",
        batch_key="customer_batch",
        batch_size_limit_key="batch_size_limit",
        current_index_key="current_batch_index",
        default_limit=3,
        summary_key="batch_orders_summary",
    )
    assert _tick_once(node) == py_trees.common.Status.SUCCESS
    bb = _get_bb(
        "customer_batch",
        "batch_size_limit",
        "current_batch_index",
        "batch_orders_summary",
    )
    assert bb["customer_batch"] == []
    assert bb["batch_size_limit"] == 3
    assert bb["current_batch_index"] == 0
    assert bb["batch_orders_summary"] == ""


def test_add_detected_customers_until_limit():
    _set_bb(
        customer_batch=[],
        customer_location="pose_1",
        batch_size_limit=3,
    )
    add = BtNode_AddDetectedCustomerToBatch(
        name="add one",
        batch_key="customer_batch",
        source_pose_key="customer_location",
        batch_size_limit_key="batch_size_limit",
    )
    assert _tick_once(add) == py_trees.common.Status.SUCCESS
    _set_bb(
        customer_batch=_get_bb("customer_batch")["customer_batch"],
        customer_location="pose_2",
        batch_size_limit=3,
    )
    assert _tick_once(add) == py_trees.common.Status.SUCCESS
    _set_bb(
        customer_batch=_get_bb("customer_batch")["customer_batch"],
        customer_location="pose_3",
        batch_size_limit=3,
    )
    assert _tick_once(add) == py_trees.common.Status.SUCCESS
    _set_bb(
        customer_batch=_get_bb("customer_batch")["customer_batch"],
        customer_location="pose_4",
        batch_size_limit=3,
    )
    assert _tick_once(add) == py_trees.common.Status.FAILURE


def test_batch_index_select_advance_and_reset():
    batch = [
        {"id": 1, "pose": "p1", "order": "", "status": "detected"},
        {"id": 2, "pose": "p2", "order": "tea", "status": "ordered"},
    ]
    _set_bb(
        customer_batch=batch,
        current_batch_index=0,
        customer_location=None,
        active_customer_id=None,
        customer_order="",
    )
    select = BtNode_SelectBatchCustomerByIndex(
        name="select idx",
        batch_key="customer_batch",
        current_index_key="current_batch_index",
        customer_pose_key="customer_location",
        active_id_key="active_customer_id",
        customer_order_key="customer_order",
    )
    assert _tick_once(select) == py_trees.common.Status.SUCCESS
    bb = _get_bb("customer_location", "active_customer_id")
    assert bb["customer_location"] == "p1"
    assert bb["active_customer_id"] == 1

    advance = BtNode_AdvanceBatchIndex(
        name="advance",
        current_index_key="current_batch_index",
    )
    assert _tick_once(advance) == py_trees.common.Status.SUCCESS
    assert _get_bb("current_batch_index")["current_batch_index"] == 1

    reset = BtNode_ResetBatchIndex(
        name="reset",
        current_index_key="current_batch_index",
    )
    assert _tick_once(reset) == py_trees.common.Status.SUCCESS
    assert _get_bb("current_batch_index")["current_batch_index"] == 0


def test_store_order_and_build_summary():
    batch = [
        {"id": 1, "pose": "p1", "order": "", "status": "detected"},
        {"id": 2, "pose": "p2", "order": "coffee", "status": "ordered"},
    ]
    _set_bb(
        customer_batch=batch,
        active_customer_id=1,
        customer_order="water",
        batch_orders_summary="",
    )
    store = BtNode_StoreOrderForActiveBatchCustomer(
        name="store",
        batch_key="customer_batch",
        active_id_key="active_customer_id",
        customer_order_key="customer_order",
    )
    assert _tick_once(store) == py_trees.common.Status.SUCCESS
    summary = BtNode_BuildBatchOrdersSummary(
        name="summary",
        batch_key="customer_batch",
        summary_key="batch_orders_summary",
    )
    assert _tick_once(summary) == py_trees.common.Status.SUCCESS
    text = _get_bb("batch_orders_summary")["batch_orders_summary"]
    assert "customer 1: water" in text
    assert "customer 2: coffee" in text
