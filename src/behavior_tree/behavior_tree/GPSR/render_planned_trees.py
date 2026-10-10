"""Render the FULL py_trees behavior tree for each random command.

For every command (generated or supplied), this script:
  1. Plans it through the LLM (same call as orchestrator.BtNode_PlanActions).
  2. Composes the planned action chain into a real ``py_trees.composites.Sequence``
     whose children are the actual ``ACTION_FACTORIES[<action>]()`` small trees.
  3. Hands that root to ``py_trees.display.render_dot_tree`` so you can see the
     exact tree the robot would execute for that command.

Output: ``GPSR/planned_trees_viz/cmd_NN.{dot,svg,png}`` plus a manifest.

Run::

    source /home/tinker/tk25_ws/install/setup.zsh
    BT_MOCK_MODE=true \
        /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
        src/behavior_tree/behavior_tree/GPSR/render_planned_trees.py \
        --n 5 --seed 7
"""

import argparse
import os
import sys
import traceback
from pathlib import Path
from typing import List

os.environ.setdefault("BT_MOCK_MODE", "true")

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import openai  # noqa: E402
import py_trees  # noqa: E402
import py_trees.display  # noqa: E402

from _official_cmd_gen import CommandGenerator  # noqa: E402
from cmd_understanding_test import (  # noqa: E402
    StubKnowledge,
    SYSTEM_PROMPT,
    build_user_prompt,
    load_user_vocabulary,
    run_one,
)
from config import OPENAI_API_KEY  # noqa: E402
# Shared plan -> tree rendering (single source of truth, also used by the live
# orchestrator's dry-run node).
from plan_viz import (  # noqa: E402
    LEAF_PARAM_KEYS,
    _short,
    annotate_subtree as _annotate_subtree,
    build_planned_tree,
    safe_label as _safe_label,
)


OUT_DIR = HERE / "planned_trees_viz"
DEFAULT_N = 5
DEFAULT_SEED = 7


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--seed", type=int, default=DEFAULT_SEED,
                   help=f"Random seed for the generator (default {DEFAULT_SEED}).")
    p.add_argument("--n", type=int, default=DEFAULT_N,
                   help=f"Number of random commands to plan + render (default {DEFAULT_N}).")
    p.add_argument("--command", action="append", default=[],
                   help="Skip the generator — render this literal command. Repeatable.")
    p.add_argument("--commands-file", type=Path, default=None,
                   help="One command per line (# = comment).")
    p.add_argument("--out", type=Path, default=OUT_DIR,
                   help=f"Output directory (default {OUT_DIR}).")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    # Import ACTION_FACTORIES lazily so that ROS-import failures produce a
    # readable error instead of a top-level traceback.
    try:
        from behavior_tree.GPSR.small_trees import ACTION_FACTORIES
    except Exception as exc:
        print(
            f"\nCould not import ACTION_FACTORIES. Likely you forgot to source "
            f"the install dir.\n  reason: {exc!r}\n"
            f"  fix:    source /home/tinker/tk25_ws/install/setup.zsh\n",
            file=sys.stderr,
        )
        return 2

    # Build command list.
    import random
    random.seed(args.seed)
    kb = load_user_vocabulary()
    gen = CommandGenerator(kb)

    commands: List[str] = list(args.command)
    if args.commands_file:
        for line in args.commands_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#"):
                commands.append(line)
    if not commands:
        half = args.n // 2
        for _ in range(half):
            commands.append(gen.generate_command_start("people"))
        for _ in range(args.n - half):
            commands.append(gen.generate_command_start("objects"))
        random.shuffle(commands)

    n_total = len(commands)
    print(f"Planning + rendering {n_total} command(s) to {args.out}/")

    # Plan + render.
    client = openai.OpenAI(api_key=OPENAI_API_KEY, base_url="https://openrouter.ai/api/v1")
    import json as _json
    known_obj_names = list(_json.loads((HERE / "constants.json").read_text())["possible_objects"].keys())

    manifest_lines = ["# GPSR planned behavior trees\n"]
    for i, cmd in enumerate(commands, 1):
        short = _safe_label(cmd)
        viz_name = f"cmd_{i:02d}_{short}"
        print(f"\n[{i:02d}/{n_total}] {cmd}")
        # Mirror the orchestrator's guard loop: a plan the shared validator
        # rejects triggers one replan; if the second attempt is also rejected
        # we render it anyway but with a visible REJECTED marker.
        guard_reason = None
        for attempt in range(2):
            # On retry, feed the rejection reason back like the runtime
            # orchestrator's rephrase_on_failure path so the model can fix it.
            result = run_one(client, cmd, kb, known_obj_names, failure_msg=guard_reason)
            if "error" in result:
                break
            validation = result.get("validation") or {}
            if validation.get("shared_ok", True):
                guard_reason = None
                break
            guard_reason = validation.get("shared_reason")
            print(f"   plan rejected by guard (attempt {attempt + 1}): {guard_reason}")
        if "error" in result:
            print(f"   ERROR planning: {result['error']}")
            manifest_lines.append(f"## {i:02d}. `{cmd}`\n- planning error: `{result['error']}`\n")
            continue

        plan = result.get("plan") or []
        if not plan:
            print(f"   (empty plan — nothing to render)")
            manifest_lines.append(
                f"## {i:02d}. `{cmd}`\n"
                f"- plan: **empty** (reason: {result.get('reasoning','?')})\n"
            )
            continue

        plan_line = " -> ".join(
            f"{s.get('action')}({','.join(s.get('params', {}).keys())})"
            for s in plan if isinstance(s, dict)
        )
        print(f"   plan: {plan_line}")

        root = build_planned_tree(plan, ACTION_FACTORIES, label=f"{i:02d} {cmd}")
        if guard_reason:
            root.insert_child(py_trees.behaviours.Failure(
                name=f"REJECTED BY GUARD — {_short(guard_reason, 80)}",
            ), 0)
            root.name = f"[REJECTED] {root.name}"
        try:
            py_trees.display.render_dot_tree(
                root, name=viz_name,
                target_directory=str(args.out),
                with_blackboard_variables=False,
            )
        except Exception:
            print(f"   render failed:")
            traceback.print_exc()
            continue
        guard_note = (
            f"- **guard: REJECTED** — {guard_reason}\n" if guard_reason
            else "- guard: passed\n"
        )
        # Freeze the plan to a standalone re-runnable .py next to the PNG.
        py_name = f"{viz_name}.py"
        try:
            from codegen import write_plan_module
            write_plan_module(cmd, plan, args.out / py_name,
                              reasoning=result.get("reasoning"))
        except Exception as exc:
            print(f"   plan-module emit failed: {exc!r}")
            py_name = "(emit failed)"
        manifest_lines.append(
            f"## {i:02d}. `{cmd}`\n"
            f"- plan: `{plan_line}`\n"
            f"{guard_note}"
            f"- viz: `{viz_name}.png` / `.svg` / `.dot`\n"
            f"- replay module: `{py_name}`\n"
        )

    manifest_path = args.out / "manifest.md"
    manifest_path.write_text("\n".join(manifest_lines))
    print(f"\nManifest written to: {manifest_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
