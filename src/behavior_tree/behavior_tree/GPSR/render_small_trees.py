"""Render every GPSR small tree as DOT/SVG/PNG using py_trees.display.

Each ``create_<action>()`` factory in ``small_trees.py`` returns a real
``py_trees`` Behaviour. This script instantiates each one under mock mode and
hands it to ``py_trees.display.render_dot_tree`` so you can see the
sub-behaviours, decorators, parallels, and selectors the planner ends up
dispatching for each atomic action.

Output: ``GPSR/small_trees_viz/small_tree_<name>.{dot,svg,png}``

Run::

    BT_MOCK_MODE=true \
        /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python \
        src/behavior_tree/behavior_tree/GPSR/render_small_trees.py
"""

import os
import sys
import traceback
from pathlib import Path

os.environ.setdefault("BT_MOCK_MODE", "true")

# install/ must be sourced for ROS msgs; if it isn't, the import below will
# raise a recognizable error. Print a clear hint instead of stacktracing.
try:
    import py_trees
    import py_trees.display
except Exception as exc:  # pragma: no cover
    print(f"failed to import py_trees: {exc!r}", file=sys.stderr)
    sys.exit(2)

HERE = Path(__file__).resolve().parent
OUT_DIR = HERE / "small_trees_viz"
OUT_DIR.mkdir(exist_ok=True)


def main() -> int:
    try:
        from behavior_tree.GPSR.small_trees import ACTION_FACTORIES
    except Exception as exc:
        print(
            f"\nCould not import ACTION_FACTORIES.\n"
            f"  reason: {exc!r}\n"
            f"  fix:    source /home/tinker/tk25_ws/install/setup.zsh, then rerun.\n",
            file=sys.stderr,
        )
        return 2

    print(f"Rendering {len(ACTION_FACTORIES)} small trees to {OUT_DIR}/")
    summary = []
    for name, factory in ACTION_FACTORIES.items():
        out_name = f"small_tree_{name}"
        try:
            tree = factory()
        except Exception as exc:
            print(f"  [FAIL build] {name}: {exc!r}")
            summary.append((name, "build_error", repr(exc)))
            traceback.print_exc()
            continue
        try:
            py_trees.display.render_dot_tree(
                tree,
                name=out_name,
                target_directory=str(OUT_DIR),
                with_blackboard_variables=False,
            )
            sizes = sorted(
                p.name for p in OUT_DIR.glob(f"{out_name}.*")
            )
            print(f"  [ OK ] {name:<16} -> {sizes}")
            summary.append((name, "ok", ""))
        except Exception as exc:
            print(f"  [FAIL render] {name}: {exc!r}")
            summary.append((name, "render_error", repr(exc)))
            traceback.print_exc()

    print()
    print("Summary:")
    n_ok = sum(1 for _, s, _ in summary if s == "ok")
    print(f"  {n_ok}/{len(summary)} rendered")
    for n, status, msg in summary:
        if status != "ok":
            print(f"  - {n}: {status}: {msg}")
    return 0 if n_ok == len(summary) else 1


if __name__ == "__main__":
    sys.exit(main())
