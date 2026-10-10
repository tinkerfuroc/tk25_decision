"""Generate a standalone, re-runnable Python file from a frozen GPSR plan.

The orchestrator plans a command with the LLM and builds the behaviour tree
*in memory*. For check-after-run inspection and deterministic replay, this
module serialises the plan the LLM produced into a self-contained ``.py``
module that:

  * documents the command + the planned action chain,
  * exposes ``PLAN`` (the exact action list) as a Python literal,
  * ``create_tree()`` rebuilds the executed tree (two modes):
      - replay=True  : inject the frozen PLAN and run the dispatch loop —
                       re-executes the same steps with NO LLM call,
      - replay=False : the bare structural composition of small trees.
  * ``main()`` runs the replay live on the robot.

Used from two places:
  * ``orchestrator.BtNode_GeneratePlanFile`` — drops a file on every live run,
  * ``render_planned_trees.py`` — emits the ``.py`` next to the PNG in dev.
"""

from __future__ import annotations

import pprint
import re
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

# constants.json lives next to this module.
_CONSTANTS_PATH = Path(__file__).resolve().parent / "constants.json"

_SAFE_RE = re.compile(r"[^a-z0-9]+")


def safe_slug(text: str, max_len: int = 40) -> str:
    s = _SAFE_RE.sub("_", text.lower()).strip("_")
    return s[:max_len] or "cmd"


def _plan_chain(plan: List[Dict[str, Any]]) -> str:
    parts = []
    for s in plan:
        if not isinstance(s, dict):
            continue
        params = s.get("params") or {}
        inner = ", ".join(f"{k}={v}" for k, v in params.items())
        parts.append(f"{s.get('action')}({inner})")
    return " -> ".join(parts) or "(empty)"


def render_plan_module(
    command: str,
    plan: List[Dict[str, Any]],
    *,
    timestamp: Optional[str] = None,
    reasoning: Optional[str] = None,
) -> str:
    """Return the source text of a standalone plan module."""
    timestamp = timestamp or datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    chain = _plan_chain(plan)
    plan_literal = pprint.pformat(plan, indent=4, width=100, sort_dicts=False)

    # One structural add_child line per step, annotated with its params.
    struct_lines = []
    for i, s in enumerate(plan, 1):
        if not isinstance(s, dict):
            continue
        action = s.get("action")
        params = s.get("params") or {}
        inner = ", ".join(f"{k}={v}" for k, v in params.items())
        struct_lines.append(
            f'        root.add_child(ACTION_FACTORIES[{action!r}]())'
            f'  # step {i}: {action}({inner})'
        )
    struct_block = "\n".join(struct_lines) or "        pass  # empty plan"

    reasoning_doc = f"\nPlanner reasoning:\n    {reasoning}\n" if reasoning else ""

    return f'''"""GPSR decision tree — AUTO-GENERATED, do not hand-edit the header.

Command:
    {command}

Planned action chain:
    {chain}
{reasoning_doc}
Generated: {timestamp}

This file freezes the plan the LLM produced so the exact same tree can be
rebuilt and re-executed WITHOUT calling the LLM again (deterministic replay).
"""

import rclpy
import py_trees
import py_trees_ros

from behavior_tree.GPSR.small_trees import (
    ACTION_FACTORIES,
    bb_keys,
    BtNode_BlackboardSet,
)
from behavior_tree.GPSR.orchestrator import (
    create_execute_one_step,
    create_orchestrator_init,
    load_knowledge_from_constants,
)

CONSTANTS_PATH = {str(_CONSTANTS_PATH)!r}

COMMAND = {command!r}

# The frozen plan, exactly as the planner produced it.
PLAN = {plan_literal}


def create_tree(replay: bool = True):
    """Rebuild the executed GPSR tree.

    replay=True  -> inject the frozen PLAN and run the dispatch loop, so the
                    same steps execute again with NO LLM call.
    replay=False -> the bare structural composition of the small trees, in
                    plan order (for inspection / visualization).
    """
    if not replay:
        root = py_trees.composites.Sequence("GPSR (structure): " + COMMAND, memory=True)
{struct_block}
        return root

    root = py_trees.composites.Sequence("GPSR replay: " + COMMAND, memory=True)
    root.add_child(create_orchestrator_init())
    root.add_child(BtNode_BlackboardSet("inject command", bb_keys.COMMAND, COMMAND))
    root.add_child(BtNode_BlackboardSet("inject plan", bb_keys.PLAN, PLAN))
    root.add_child(BtNode_BlackboardSet("reset plan index", bb_keys.PLAN_INDEX, 0))
    loop = py_trees.decorators.Repeat(
        name="step loop",
        child=create_execute_one_step(),
        num_success=max(len(PLAN), 1),
    )
    root.add_child(py_trees.decorators.FailureIsSuccess("plan-exhausted = done", loop))
    return root


def main():
    load_knowledge_from_constants(CONSTANTS_PATH)
    rclpy.init()
    tree = py_trees_ros.trees.BehaviourTree(root=create_tree(replay=True))
    tree.setup(timeout=15, node_name="gpsr_replay")
    tree.tick_tock(period_ms=500.0)
    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
'''


def write_plan_module(
    command: str,
    plan: List[Dict[str, Any]],
    out_path,
    *,
    timestamp: Optional[str] = None,
    reasoning: Optional[str] = None,
) -> Path:
    """Render and write the plan module to ``out_path``; return the Path."""
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(render_plan_module(
        command, plan, timestamp=timestamp, reasoning=reasoning,
    ))
    return out_path
