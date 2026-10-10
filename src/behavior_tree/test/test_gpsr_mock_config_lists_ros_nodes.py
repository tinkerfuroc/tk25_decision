"""Every ROS-backed node class in the GPSR small trees must be listed in the
sim mock configs.

Under global mock mode (mock_config.sim*.json: mock_mode.enabled = true) a
node class nobody listed defaults to MOCKED (config.is_node_mocked). Sim
battery 2026-10-01 fix8 run s70718-004: the new BtNode_Approach was unlisted,
so the person approach "auto-completed" without moving the robot and the
hand-over happened 0.93 m from the operator. Listing a class lets its
subsystem's enabled flag decide real vs mock, like every other node.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1.
"""
from __future__ import annotations

import inspect
import json
from pathlib import Path

import pytest

from behavior_tree.GPSR import small_trees
from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler

PKG = Path(small_trees.__file__).resolve().parents[1]


def _walk(node):
    yield node
    for child in getattr(node, "children", []) or []:
        yield from _walk(child)
    decorated = getattr(node, "decorated", None)
    if decorated is not None and decorated not in (getattr(node, "children", []) or []):
        yield from _walk(decorated)


def _ros_node_classes():
    found = set()
    for name, fn in inspect.getmembers(small_trees, inspect.isfunction):
        if not name.startswith("create_") or fn.__module__ != small_trees.__name__:
            continue
        sig = inspect.signature(fn)
        if any(p.default is inspect.Parameter.empty and p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)
               for p in sig.parameters.values()):
            continue
        try:
            tree = fn()
        except Exception:  # noqa: BLE001 - a builder needing runtime state is skipped
            continue
        for node in _walk(tree):
            if isinstance(node, (ServiceHandler, ActionHandler)):
                found.add(type(node).__name__)
    return found


def _listed(config_path):
    cfg = json.loads(config_path.read_text())
    names = set(cfg.get("force_mock_nodes", {}))
    for sub in cfg["mock_mode"]["subsystems"].values():
        names |= set(sub.get("nodes", {}))
    return names


@pytest.mark.parametrize("config", ["mock_config.sim.json", "mock_config.sim-hybrid.json"])
def test_every_ros_node_in_small_trees_is_listed(config):
    classes = _ros_node_classes()
    assert "BtNode_Approach" in classes  # the fix8 regression is covered
    missing = sorted(classes - _listed(PKG / config))
    assert not missing, f"{config}: unlisted ROS nodes default to MOCKED under global mock: {missing}"
