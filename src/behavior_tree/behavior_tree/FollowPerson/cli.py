# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Follow-person CLI entry point
# =============================
#
# Runs the follow-person tree via the shared ``run_tree`` runner. The real
# ``/track_person`` action (``vision_track/person_track_server``) and the
# ``TextToSpeech`` service must already be running.
#

from behavior_tree.core.runtime import run_tree


def main():
    """Run the follow-person behaviour tree until interrupted.

    --no-nav builds the vision+audio-only tree (no follow-navigation child, no base
    motion). --breadcrumbs opts into trail-following (NavigateThroughPoses) for
    cluttered/doorway environments; the default is open following (single-goal
    standoff pursuit, no breadcrumbs). parse_known_args ignores --ros-args so the
    script still works under ros2 run behavior_tree follow-person [--no-nav]
    [--breadcrumbs].
    """
    import argparse

    parser = argparse.ArgumentParser(prog="follow-person")
    parser.add_argument(
        "--no-nav", action="store_true",
        help="vision+audio only: omit the follow-navigation child (no base motion)",
    )
    parser.add_argument(
        "--breadcrumbs", action="store_true",
        help="route through the person's trail (clutter/doorways); default is "
             "open following with single-goal pursuit",
    )
    args, _ = parser.parse_known_args()
    enable_navigation = not args.no_nav

    from behavior_tree.components.following.follow_person import create_follow_person_tree

    run_tree(
        lambda: create_follow_person_tree(
            enable_navigation=enable_navigation,
            use_breadcrumbs=args.breadcrumbs,
        ),
        period_ms=200.0,
        title="Follow Person" if enable_navigation else "Follow Person (vision+audio)",
    )
