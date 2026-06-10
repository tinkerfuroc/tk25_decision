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
# Follow-person process launch
# ============================
#
# Starts the dummy nav stub and the follow-person behaviour tree runner.
#
# PREREQUISITES (real-tracker-only; NOT started here):
#   - The real TrackPerson action server must be running, e.g.:
#         ros2 run vision_track person_track_server
#   - The real TextToSpeech service ("announce") must be running (audio stack).
#
# DEPRECATED dummy-nav (2026-06-10): the ``/follow_target`` topic the dummy nav
# node subscribes to has NO publisher since the FollowPerson rewire —
# ``BtNode_PublishFollowGoal`` (its only publisher) was removed. The follow
# pipeline now drives navigation through the ``Follow`` action on
# ``follow_server`` (tk26_navigation ``following`` package), consuming the
# tracker's ``/target_points`` topic directly. The dummy nav node below is
# retained for standalone experiments only and is NOT part of the follow
# pipeline; with the live tree it logs nothing because ``/follow_target`` has no
# publisher. Run ``follow_server`` separately to actually move the base.
#

import launch
import launch_ros.actions


def generate_launch_description():
    """Launch the dummy nav node and the follow-person BT runner."""
    dummy_nav = launch_ros.actions.Node(
        package="behavior_tree",
        executable="dummy-nav",
        name="dummy_nav_node",
        output="screen",
    )

    follow_person = launch_ros.actions.Node(
        package="behavior_tree",
        executable="follow-person",
        name="follow_person",
        output="screen",
    )

    return launch.LaunchDescription([dummy_nav, follow_person])
