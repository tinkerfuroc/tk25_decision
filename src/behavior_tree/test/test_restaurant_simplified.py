import importlib
import sys
import types

import py_trees


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


def _import_simplified(monkeypatch):
    _install_stubs(monkeypatch)
    sys.modules.pop("behavior_tree.Restaurant.restaurant_simplified", None)
    return importlib.import_module("behavior_tree.Restaurant.restaurant_simplified")


def _tick_until_terminal(root, max_ticks=80):
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(max_ticks):
        tree.tick()
        if root.status != py_trees.common.Status.RUNNING:
            return root.status
    return root.status


def test_simplified_happy_path_n3(monkeypatch):
    module = _import_simplified(monkeypatch)

    class DetectNode(py_trees.behaviour.Behaviour):
        counter = 0

        def __init__(self, name, bb_dest_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="loc",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
            )

        def update(self):
            DetectNode.counter += 1
            self.bb.loc = f"pose_{DetectNode.counter}"
            return py_trees.common.Status.SUCCESS

    class TakeOrderNode(py_trees.behaviour.Behaviour):
        counter = 0

        def __init__(self, name, bb_dest_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="order",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
            )

        def update(self):
            TakeOrderNode.counter += 1
            self.bb.order = f"order_{TakeOrderNode.counter}"
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(module, "BtNode_DetectCallingCustomer", DetectNode)
    monkeypatch.setattr(module, "BtNode_ScanForCallingCustomer", _FailNode)
    monkeypatch.setattr(module, "BtNode_TakeOrder", TakeOrderNode)
    monkeypatch.setattr(module, "BtNode_ConfirmOrder", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_CommunicateWithBarman", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_DetectTray", _FailNode)
    monkeypatch.setattr(module, "BtNode_ServeOrder", _SuccessNode)

    root = module.createRestaurantSimplifiedTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS


def test_simplified_partial_failure_unreachable_customer(monkeypatch):
    module = _import_simplified(monkeypatch)

    class DetectNode(py_trees.behaviour.Behaviour):
        counter = 0

        def __init__(self, name, bb_dest_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="loc",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
            )

        def update(self):
            DetectNode.counter += 1
            if DetectNode.counter > 2:
                return py_trees.common.Status.FAILURE
            self.bb.loc = f"pose_{DetectNode.counter}"
            return py_trees.common.Status.SUCCESS

    class FlakyGoto(py_trees.behaviour.Behaviour):
        calls = 0

        def __init__(self, name, *args, **kwargs):
            super().__init__(name=name)

        def update(self):
            FlakyGoto.calls += 1
            if FlakyGoto.calls == 1:
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS

    class TakeOrderNode(py_trees.behaviour.Behaviour):
        def __init__(self, name, bb_dest_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="order",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
            )

        def update(self):
            self.bb.order = "tea"
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(module, "BtNode_DetectCallingCustomer", DetectNode)
    monkeypatch.setattr(module, "BtNode_ScanForCallingCustomer", _FailNode)
    monkeypatch.setattr(module, "BtNode_GotoAction", FlakyGoto)
    monkeypatch.setattr(module, "BtNode_TakeOrder", TakeOrderNode)
    monkeypatch.setattr(module, "BtNode_ConfirmOrder", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_CommunicateWithBarman", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_DetectTray", _FailNode)
    monkeypatch.setattr(module, "BtNode_ServeOrder", _SuccessNode)

    root = module.createRestaurantSimplifiedTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS


def test_simplified_no_customer_case(monkeypatch):
    module = _import_simplified(monkeypatch)
    monkeypatch.setattr(module, "BtNode_DetectCallingCustomer", _FailNode)
    monkeypatch.setattr(module, "BtNode_ScanForCallingCustomer", _FailNode)
    monkeypatch.setattr(module, "BtNode_CommunicateWithBarman", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_ServeOrder", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_DetectTray", _FailNode)

    root = module.createRestaurantSimplifiedTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS


def test_simplified_delivery_serves_only_customers_with_orders(monkeypatch):
    module = _import_simplified(monkeypatch)
    serve_events = []

    class DetectNode(py_trees.behaviour.Behaviour):
        counter = 0

        def __init__(self, name, bb_dest_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="loc",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
            )

        def update(self):
            DetectNode.counter += 1
            if DetectNode.counter > 3:
                return py_trees.common.Status.FAILURE
            self.bb.loc = f"pose_{DetectNode.counter}"
            return py_trees.common.Status.SUCCESS

    class TakeOrderNode(py_trees.behaviour.Behaviour):
        counter = 0

        def __init__(self, name, bb_dest_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="order",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_dest_key),
            )

        def update(self):
            TakeOrderNode.counter += 1
            self.bb.order = f"item_{TakeOrderNode.counter}"
            return py_trees.common.Status.SUCCESS

    class StoreOrderNode(module.BtNode_StoreOrderForActiveBatchCustomer):
        def update(self):
            if self.blackboard.active_id == 2:
                return py_trees.common.Status.FAILURE
            return super().update()

    class ServeNode(py_trees.behaviour.Behaviour):
        def __init__(self, name, *args, **kwargs):
            super().__init__(name=name)

        def update(self):
            serve_events.append(self.name)
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(module, "BtNode_DetectCallingCustomer", DetectNode)
    monkeypatch.setattr(module, "BtNode_ScanForCallingCustomer", _FailNode)
    monkeypatch.setattr(module, "BtNode_TakeOrder", TakeOrderNode)
    monkeypatch.setattr(module, "BtNode_ConfirmOrder", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_StoreOrderForActiveBatchCustomer", StoreOrderNode)
    monkeypatch.setattr(module, "BtNode_CommunicateWithBarman", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_DetectTray", _FailNode)
    monkeypatch.setattr(module, "BtNode_ServeOrder", ServeNode)

    root = module.createRestaurantSimplifiedTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
    assert len(serve_events) == 2
