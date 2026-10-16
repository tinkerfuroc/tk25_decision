"""ROS entry point following HRI/PickAndPlace's shared runtime."""

import argparse

from behavior_tree.runtime import run_tree

from .config import load_config
from .hangzhougrasp import createHangzhouGrasp


def _build_parser():
    parser = argparse.ArgumentParser(prog="hangzhougrasp")
    parser.add_argument("--config", help="JSON overrides for constants.json")
    parser.add_argument("--grasp", action=argparse.BooleanOptionalAction, default=False,
                        help="Construct grasp/retract branch (default: --no-grasp)")
    parser.add_argument("--scan-pose", nargs=3, type=float, metavar=("X", "Y", "YAW_DEG"))
    parser.add_argument("--destination-pose", nargs=3, type=float,
                        metavar=("X", "Y", "YAW_DEG"))
    parser.add_argument("--prompt", default=None,
                        help="YOLO-World labels; empty string uses all label_map keys")
    parser.add_argument("--print-tree", action="store_true",
                        help="Print the constructed tree without setup or hardware calls")
    return parser


def main(args=None):
    parser = _build_parser()
    options, _ = parser.parse_known_args(args)
    try:
        config = load_config(
            options.config, scan_pose=options.scan_pose,
            destination_pose=options.destination_pose, prompt=options.prompt,
        )
    except (OSError, ValueError, TypeError, KeyError) as exc:
        parser.error(str(exc))
    factory = lambda: createHangzhouGrasp(enable_grasp=options.grasp, config=config)
    if options.print_tree:
        import py_trees
        print(py_trees.display.unicode_tree(factory()))
        return
    run_tree(
        factory, period_ms=500.0,
        title=f"HangzhouGrasp (grasp={options.grasp})",
        node_name="hangzhougrasp", show_during_setup=True,
        stop_on_terminal=True,
    )


if __name__ == "__main__":
    main()

