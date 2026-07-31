"""Structural contract for the production GPSR orchestrator entry factory."""

import os
import sys
import types

os.environ.setdefault("BT_MOCK_MODE", "true")
# Planning is never invoked in these factory-structure tests. Keep the optional
# network client out of test collection on operator images that omit it.
sys.modules.setdefault("openai", types.ModuleType("openai"))

from pathlib import Path  # noqa: E402

import py_trees  # noqa: E402

import behavior_tree.GPSR.gpsr_orchestrator as gpsr  # noqa: E402


def _success(name):
    return py_trees.behaviours.Success(name=name)


def _stub_orchestrator_dependencies(monkeypatch, *, command_point):
    captured = {}

    monkeypatch.setattr(gpsr, "load_knowledge_from_constants", lambda _path: None)
    monkeypatch.setattr(
        gpsr,
        "_arm_constants_to_bb",
        lambda root: root.add_child(_success("seed arm constants")),
    )
    monkeypatch.setattr(gpsr, "create_enter_arena", lambda: _success("enter arena"))
    monkeypatch.setattr(gpsr, "has_command_point", lambda: command_point)
    monkeypatch.setattr(
        gpsr, "create_goto_command_point", lambda: _success("command point")
    )
    monkeypatch.setattr(
        gpsr, "create_orchestrator_init", lambda: _success("orchestrator init")
    )

    intake = object()

    def make_inject_intake(commands):
        captured["commands"] = list(commands)
        return intake

    def create_batch_command_flow(**kwargs):
        captured["batch_kwargs"] = kwargs
        return _success("batch command flow")

    monkeypatch.setattr(gpsr, "make_inject_intake", make_inject_intake)
    monkeypatch.setattr(gpsr, "create_batch_command_flow", create_batch_command_flow)
    return captured, intake


def test_current_factory_seeds_arm_then_enters_arena_before_command_flow(
    monkeypatch, tmp_path
):
    captured, intake = _stub_orchestrator_dependencies(
        monkeypatch, command_point=True
    )

    root = gpsr.createGPSROrchestrator(
        commands=["bring water", "find a guest"],
        plan_dir=Path(tmp_path),
        max_steps=12,
        max_corrections=2,
    )

    assert isinstance(root, py_trees.composites.Sequence)
    assert root.memory is True
    assert [child.name for child in root.children] == [
        "seed arm constants",
        "enter arena",
        "command point",
        "orchestrator init",
        "batch command flow",
        "idle",
    ]
    assert captured["commands"] == ["bring water", "find a guest"]
    assert captured["batch_kwargs"] == {
        "num_commands": 2,
        "make_intake": intake,
        "max_steps": 12,
        "max_corrections": 2,
        "emit_plan_dir": str(tmp_path),
    }


def test_command_point_step_is_optional_but_arena_entry_is_not(
    monkeypatch, tmp_path
):
    _stub_orchestrator_dependencies(monkeypatch, command_point=False)

    root = gpsr.createGPSROrchestrator(
        commands=["follow the person"],
        plan_dir=Path(tmp_path),
    )

    names = [child.name for child in root.children]
    assert names[:2] == ["seed arm constants", "enter arena"]
    assert "command point" not in names
    assert names[-3:] == ["orchestrator init", "batch command flow", "idle"]
