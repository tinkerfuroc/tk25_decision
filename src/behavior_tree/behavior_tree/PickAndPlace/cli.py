import argparse

from behavior_tree.core.runtime import run_tree


def _build_parser():
    parser = argparse.ArgumentParser(prog="pick-and-place")
    parser.add_argument(
        "--place-policy",
        choices=["hardcoded", "vlm"],
        default="vlm",
        help="Surface-place strategy: 'vlm' (competition, FREE_SPACE/NEAR_SIMILAR) "
        "or 'hardcoded' (on-robot bring-up, FIXED_POINT, no VLM/network).",
    )
    return parser


def main():
    # parse_known_args so ros2 run's --ros-args ... pass through untouched.
    args, _ = _build_parser().parse_known_args()
    from .pick_and_place_rulebook import pickAndPlaceRulebook

    run_tree(
        lambda: pickAndPlaceRulebook(place_policy=args.place_policy),
        period_ms=500.0,
        title=f"Pick And Place (rulebook, place_policy={args.place_policy})",
    )
