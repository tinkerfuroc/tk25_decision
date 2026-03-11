import importlib
import sys
import types

import py_trees

from behavior_tree.PickAndPlace import state_nodes


def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()


def _set_bb(values):
    client = py_trees.blackboard.Client(name="pp_test_writer")
    for idx, (key, value) in enumerate(values.items()):
        local = f"k{idx}"
        client.register_key(
            key=local,
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
        setattr(client, local, value)


def _get_bb(key):
    client = py_trees.blackboard.Client(name="pp_test_reader")
    client.register_key(
        key="v",
        access=py_trees.common.Access.READ,
        remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
    )
    return client.v


class _SuccessNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, *args, **kwargs):
        super().__init__(name=name)

    def update(self):
        return py_trees.common.Status.SUCCESS


def _install_pick_and_place_stubs(monkeypatch):
    audio = types.ModuleType("behavior_tree.TemplateNodes.Audio")
    audio.BtNode_Announce = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Audio", audio)

    class WriteNode(py_trees.behaviour.Behaviour):
        def __init__(self, name, bb_namespace, bb_key, bb_source, object=None, *args, **kwargs):
            super().__init__(name=name)
            self.object = object
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="dest",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key),
            )

        def update(self):
            self.bb.dest = self.object
            return py_trees.common.Status.SUCCESS

    base = types.ModuleType("behavior_tree.TemplateNodes.BaseBehaviors")
    base.BtNode_WriteToBlackboard = WriteNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.BaseBehaviors", base)

    manip = types.ModuleType("behavior_tree.TemplateNodes.Manipulation")
    manip.BtNode_MoveArmSingle = _SuccessNode
    manip.BtNode_Grasp = _SuccessNode
    manip.BtNode_Drop = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Manipulation", manip)

    nav = types.ModuleType("behavior_tree.TemplateNodes.Navigation")
    nav.BtNode_GotoAction = _SuccessNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Navigation", nav)

    class EmptyScanNode(py_trees.behaviour.Behaviour):
        def __init__(self, name, bb_source, bb_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="result",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key),
            )

        def update(self):
            self.bb.result = types.SimpleNamespace(objects=[])
            return py_trees.common.Status.SUCCESS

    vision = types.ModuleType("behavior_tree.TemplateNodes.Vision")
    vision.BtNode_ScanFor = EmptyScanNode
    monkeypatch.setitem(sys.modules, "behavior_tree.TemplateNodes.Vision", vision)


def _import_pick_and_place(monkeypatch):
    _install_pick_and_place_stubs(monkeypatch)
    sys.modules.pop("behavior_tree.PickAndPlace.pick_and_place", None)
    return importlib.import_module("behavior_tree.PickAndPlace.pick_and_place")


def _tick_until_terminal(root, max_ticks=120):
    _clear_blackboard()
    tree = py_trees.trees.BehaviourTree(root)
    for _ in range(max_ticks):
        tree.tick()
        if root.status != py_trees.common.Status.RUNNING:
            return root.status
    return root.status


def test_queue_priority_order_is_deterministic():
    _clear_blackboard()
    _set_bb(
        {
            "pp_inventory_table": [
                {"name": "bowl", "confidence": 0.9, "source_area": "table"},
                {"name": "paper cup", "confidence": 0.7, "source_area": "table"},
            ],
            "pp_inventory_floor": [
                {"name": "plastic bag", "confidence": 0.8, "source_area": "floor"},
                {"name": "spoon", "confidence": 0.6, "source_area": "floor"},
            ],
        }
    )
    node = state_nodes.BtNode_BuildCleanupWorkQueue(
        name="build",
        inventory_table_key="pp_inventory_table",
        inventory_floor_key="pp_inventory_floor",
        work_queue_key="pp_work_queue",
    )
    assert node.update() == py_trees.common.Status.SUCCESS
    queue = _get_bb("pp_work_queue")
    ordered = [(item["source_area"], item["object_class"], item["name"]) for item in queue]
    assert ordered[0] == ("table", "trash", "paper cup")
    assert ordered[1] == ("floor", "trash", "plastic bag")
    assert ordered[2] == ("table", "relocation", "bowl")
    assert ordered[3] == ("floor", "relocation", "spoon")


def test_timeout_checker_forces_cutover():
    _clear_blackboard()
    _set_bb({"pp_phase_deadline": 0.0})
    node = state_nodes.BtNode_TimeoutCutoverChecker(
        name="timeout",
        phase_deadline_key="pp_phase_deadline",
    )
    assert node.update() == py_trees.common.Status.FAILURE


def test_cleanup_happy_path(monkeypatch):
    module = _import_pick_and_place(monkeypatch)

    scan_payloads = [
        types.SimpleNamespace(
            objects=[
                types.SimpleNamespace(class_name="paper cup", confidence=0.9),
                types.SimpleNamespace(class_name="bowl", confidence=0.8),
            ]
        ),
        types.SimpleNamespace(objects=[types.SimpleNamespace(class_name="plastic bag", confidence=0.7)]),
    ]

    class ScanNode(py_trees.behaviour.Behaviour):
        calls = 0

        def __init__(self, name, bb_source, bb_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="result",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key),
            )

        def update(self):
            idx = min(ScanNode.calls, len(scan_payloads) - 1)
            self.bb.result = scan_payloads[idx]
            ScanNode.calls += 1
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(module, "BtNode_ScanFor", ScanNode)
    monkeypatch.setattr(module, "BtNode_GotoAction", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Grasp", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Drop", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_MoveArmSingle", _SuccessNode)

    root = module.createPickAndPlaceTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
    score = _get_bb("pp_score_trace")
    assert len(score) >= 2


def test_failed_grasp_marks_item_and_continues(monkeypatch):
    module = _import_pick_and_place(monkeypatch)

    class ScanNode(py_trees.behaviour.Behaviour):
        def __init__(self, name, bb_source, bb_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="result",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key),
            )

        def update(self):
            self.bb.result = types.SimpleNamespace(
                objects=[
                    types.SimpleNamespace(class_name="paper cup", confidence=0.9),
                    types.SimpleNamespace(class_name="bowl", confidence=0.8),
                ]
            )
            return py_trees.common.Status.SUCCESS

    class FlakyGrasp(py_trees.behaviour.Behaviour):
        calls = 0

        def __init__(self, name, *args, **kwargs):
            super().__init__(name=name)

        def update(self):
            FlakyGrasp.calls += 1
            if FlakyGrasp.calls <= 2:
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(module, "BtNode_ScanFor", ScanNode)
    monkeypatch.setattr(module, "BtNode_GotoAction", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Grasp", FlakyGrasp)
    monkeypatch.setattr(module, "BtNode_Drop", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_MoveArmSingle", _SuccessNode)

    root = module.createPickAndPlaceTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
    score = _get_bb("pp_score_trace")
    assert any(not item["success"] for item in score)
    assert any(item["success"] for item in score)


def test_no_objects_found_exits_cleanly(monkeypatch):
    module = _import_pick_and_place(monkeypatch)
    monkeypatch.setattr(module, "BtNode_GotoAction", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Grasp", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Drop", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_MoveArmSingle", _SuccessNode)

    root = module.createPickAndPlaceTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
    score = _get_bb("pp_score_trace")
    assert score == []


def test_destination_policy_routes_trash_and_non_trash(monkeypatch):
    module = _import_pick_and_place(monkeypatch)

    scan_payloads = [
        types.SimpleNamespace(
            objects=[
                types.SimpleNamespace(class_name="paper cup", confidence=0.9),
                types.SimpleNamespace(class_name="bowl", confidence=0.9),
            ]
        ),
        types.SimpleNamespace(objects=[]),
    ]

    class ScanNode(py_trees.behaviour.Behaviour):
        calls = 0

        def __init__(self, name, bb_source, bb_key, *args, **kwargs):
            super().__init__(name=name)
            self.bb = self.attach_blackboard_client(name=f"{name}_bb")
            self.bb.register_key(
                key="result",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key),
            )

        def update(self):
            idx = 0 if ScanNode.calls == 0 else 1
            self.bb.result = scan_payloads[idx]
            ScanNode.calls += 1
            return py_trees.common.Status.SUCCESS

    monkeypatch.setattr(module, "BtNode_ScanFor", ScanNode)
    monkeypatch.setattr(module, "BtNode_GotoAction", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Grasp", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_Drop", _SuccessNode)
    monkeypatch.setattr(module, "BtNode_MoveArmSingle", _SuccessNode)

    root = module.createPickAndPlaceTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
    score = _get_bb("pp_score_trace")
    assert any(item["class"] == "trash" and item["target"] == "trash_bin" for item in score)
    assert any(item["class"] == "relocation" and item["target"] == "table_staging" for item in score)


def test_timeout_fallback_path(monkeypatch):
    module = _import_pick_and_place(monkeypatch)
    monkeypatch.setattr(module, "MAX_RUNTIME_SEC", 0.0)
    root = module.createPickAndPlaceTask()
    assert _tick_until_terminal(root) == py_trees.common.Status.SUCCESS
