"""Render a frozen GPSR plan into the real ``py_trees`` tree and write it to disk.

Shared single source of truth for "what tree does this plan build?", used by:
  * ``render_planned_trees.py`` — the offline dev renderer (LLM -> plan -> PNG),
  * ``orchestrator.BtNode_RenderPlanTree`` — the live dry-run node that draws the
    tree the robot WOULD execute, without ticking the robot.

The tree is built only for drawing: each step's ``ACTION_FACTORIES[action]()``
small tree is *constructed* (which opens no ROS clients — those are created in
``setup()``, which the viz tree never gets) and handed to Graphviz. The actual
param values the orchestrator would put on the blackboard are baked into the
node labels instead, since a static drawing can't show blackboard traffic.

This module imports only ``py_trees`` + stdlib, so it is safe to import either
as ``behavior_tree.GPSR.plan_viz`` or as a bare ``plan_viz`` (the dev renderer
inserts its own directory on ``sys.path``). ``ACTION_FACTORIES`` is always
passed in by the caller — never imported here — to keep it import-light.
"""

import re
from pathlib import Path
from typing import Any, Dict, List

import py_trees
import py_trees.display

try:  # Keep the documented bare-module development renderer import working.
    from .tree_serialization import serialize_tree
except ImportError:  # pragma: no cover - exercised by the standalone renderer
    from tree_serialization import serialize_tree

_SAFE_LABEL_RE = re.compile(r"[^a-z0-9]+")


def safe_label(text: str, max_len: int = 40) -> str:
    s = _SAFE_LABEL_RE.sub("_", text.lower()).strip("_")
    return s[:max_len] or "cmd"


# Which leaf names inside each small tree consume which plan param. At runtime
# BtNode_PopNextAction writes the params to blackboard keys and the leaves read
# them; a static drawing can't show blackboard traffic, so we bake the actual
# value into the node label instead ("goto target [shelf]").
LEAF_PARAM_KEYS: Dict[str, Dict[str, str]] = {
    "goto": {"goto target": "location", "announce going": "location"},
    "find_object": {"generalist scan": "object", "verify objects found": "object",
                    "announce found": "object"},
    "find_person": {"descriptor to vision prompt": "descriptor",
                    "find waving persons": "descriptor",
                    "generalist person scan": "descriptor"},
    "approach_person": {},
    "describe_person": {},
    "ask_person": {"ask question": "question"},
    "report_answer": {},
    "follow": {"track person": "person"},
    "guide": {"guide to target": "location", "announce arrived": "location"},
    "grasp": {"find object on table": "object", "grasp object": "object",
              "ask referee": "object"},
    "place": {"goto place pose": "location", "announce placing": "location"},
    "deliver": {"goto recipient room": "recipient_location",
                "announce delivering": "object",
                "descriptor to vision prompt": "recipient",
                "generalist person scan": "recipient"},
    "count": {"scan to count": "object", "vlm count": "object",
              "count detections": "object"},
    "announce": {"announce text": "text"},
    "record_position": {"register labeled pose": "label"},
    "vlm_fallback": {"vlm answer": "question"},
    "llm_fallback": {"llm answer": "question"},
    "report_view": {},
}


def _short(value: Any, max_len: int = 32) -> str:
    s = str(value)
    return s if len(s) <= max_len else s[: max_len - 1] + "…"


def annotate_subtree(subtree: py_trees.behaviour.Behaviour, action: str,
                     params: Dict[str, Any]) -> None:
    """Inline the plan's actual param values into the subtree's node labels."""
    action_context = {"action": action, "params": dict(params or {})}
    # Mark only the action subtree's root as a boundary. Descendants inherit
    # the context in the serializer but are explicitly not new action scopes.
    subtree._gpsr_action_context = action_context
    if not params:
        return
    # NOTE: no ':' in node names — Graphviz parses "name: x" as node "name"
    # with port " x", which merges same-prefix nodes and draws double arrows.
    param_text = ", ".join(f"{k}={_short(v)}" for k, v in params.items())
    subtree.name = f"{subtree.name} ({param_text})"
    leaf_map = LEAF_PARAM_KEYS.get(action, {})
    for node in subtree.iterate():
        key = leaf_map.get(node.name)
        if key is not None and key in params:
            node.name = f"{node.name} [{_short(params[key])}]"


def build_planned_tree(plan: List[Dict[str, Any]], action_factories: Dict[str, Any],
                       label: str) -> py_trees.behaviour.Behaviour:
    """Compose the planned action chain into one ``py_trees.Sequence``.

    Unknown / null actions are skipped; a small tree whose factory raises is
    replaced by a visible ``Failure`` placeholder so the drawing still renders.
    """
    root = py_trees.composites.Sequence(label, memory=True)
    for step_index, step in enumerate(plan):
        if not isinstance(step, dict):
            continue
        action = step.get("action")
        params = step.get("params", {}) or {}
        factory = action_factories.get(action)
        if factory is None:
            continue  # skip unknown / null actions
        try:
            subtree = factory()
        except Exception as exc:  # noqa: BLE001 — keep rendering the rest
            root.add_child(py_trees.behaviours.Failure(
                name=f"{action} (BUILD FAILED: {exc!r})",
            ))
            continue
        annotate_subtree(subtree, action, params)
        subtree._gpsr_action_context["step_index"] = step_index
        root.add_child(subtree)
    return root


def planned_tree_document(plan: List[Dict[str, Any]], action_factories: Dict[str, Any], label: str) -> Dict[str, Any]:
    """Return a JSON-safe planned-tree topology for telemetry/debug replay.

    The live GPSR dispatcher is intentionally a separate tree.  This document
    describes the exact plan-shaped structure that operators see in the
    generated artifact and gives each node a stable structural path ID.
    """
    root = build_planned_tree(plan, action_factories, label)
    return serialize_tree(root, kind="planned", label=label)


def render_plan_tree(
    plan: List[Dict[str, Any]],
    action_factories: Dict[str, Any],
    out_dir,
    name: str,
    *,
    with_blackboard_variables: bool = False,
) -> Dict[str, str]:
    """Build + render the planned tree to ``out_dir/<name>.{dot,png,svg}``.

    Returns a dict of the artifact paths that exist after rendering.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    root = build_planned_tree(plan, action_factories, label=name)
    py_trees.display.render_dot_tree(
        root, name=name,
        target_directory=str(out_dir),
        with_blackboard_variables=with_blackboard_variables,
    )
    artifacts: Dict[str, str] = {}
    for ext in ("dot", "png", "svg"):
        candidate = out_dir / f"{name}.{ext}"
        if candidate.exists():
            artifacts[ext] = str(candidate)
    return artifacts
