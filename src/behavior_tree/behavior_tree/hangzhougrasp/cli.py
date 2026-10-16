"""ROS entry point following HRI/PickAndPlace's shared runtime."""

import argparse

from behavior_tree.runtime import run_tree

from .config import load_config
from .parts import PARTS, createPart


def _build_parser():
    parser = argparse.ArgumentParser(prog="hangzhougrasp")
    parser.add_argument("--config", help="JSON overrides for constants.json")
    parser.add_argument("--enter-room", default=None, metavar="NAME",
                        help="Entry room; overrides the enter_room key")
    parser.add_argument("--grasp", action=argparse.BooleanOptionalAction, default=False,
                        help="Construct grasp/retract branch (default: --no-grasp)")
    parser.add_argument("--prompt", default=None,
                        help="Detector labels; empty string uses all label_map keys")
    parser.add_argument("--part", default="full", choices=sorted(PARTS),
                        help="Run one part of the mission; default: full. "
                        + "; ".join(f"{k}: {v}" for k, v in PARTS.items()))
    parser.add_argument("--room", default=None, help="Room for room/ground/visit-table")
    parser.add_argument("--table", type=int, default=None,
                        help="Table number n for visit-table")
    parser.add_argument("--at", default=None, metavar="POSE",
                        help="Navigate to this named pose before the part")
    parser.add_argument("--next-pose", default=None, metavar="POSE",
                        help="Route pose where drop-elsewhere drops the object")
    parser.add_argument("--print-tree", action="store_true",
                        help="Print the constructed tree without setup or hardware calls")
    return parser


def main(args=None):
    parser = _build_parser()
    options = parser.parse_args(args)
    try:
        config = load_config(
            options.config, enter_room=options.enter_room, prompt=options.prompt)
        # Validate the part now so errors precede any ROS setup.
        createPart(options.part, config, enable_grasp=options.grasp,
                   room=options.room, table=options.table,
                   at=options.at, next_pose=options.next_pose)
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))

    def factory():
        return createPart(options.part, config, enable_grasp=options.grasp,
                          room=options.room, table=options.table,
                          at=options.at, next_pose=options.next_pose)

    if options.print_tree:
        import py_trees
        print(py_trees.display.unicode_tree(factory()))
        return
    run_tree(
        factory, period_ms=500.0,
        title=f"HangzhouGrasp {options.part} (grasp={options.grasp})",
        node_name="hangzhougrasp", show_during_setup=True,
        stop_on_terminal=True,
    )


if __name__ == "__main__":
    main()
