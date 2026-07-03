# Copyright 2026 Tinker Team
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

"""POSE_SOFA_REVERSED: same map point as POSE_SOFA, yaw rotated exactly 180 deg.

The reversed pose is computed from POSE_SOFA at module load (not stored in
constants.json), so re-teaching pose_sofa keeps the pair consistent. Used by
the hri-2026 bag flow to turn the robot around in place before the follow.
"""

import math
import os

os.environ.setdefault("BT_MOCK_MODE", "true")

from behavior_tree.HRI.config import (  # noqa: E402
    KEY_SOFA_POSE,
    KEY_SOFA_POSE_REVERSED,
    POSE_SOFA,
    POSE_SOFA_REVERSED,
)


def _yaw(quaternion):
    """Yaw of a planar (x=y=0) quaternion."""
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


def test_reversed_key_is_distinct():
    assert KEY_SOFA_POSE_REVERSED == "hri_sofa_pose_reversed"
    assert KEY_SOFA_POSE_REVERSED != KEY_SOFA_POSE


def test_point_unchanged():
    original = POSE_SOFA.pose.position
    flipped = POSE_SOFA_REVERSED.pose.position
    assert (flipped.x, flipped.y, flipped.z) == (original.x, original.y, original.z)


def test_frame_unchanged():
    assert POSE_SOFA_REVERSED.header.frame_id == POSE_SOFA.header.frame_id == "map"


def test_yaw_flipped_exactly_180_degrees():
    delta = math.remainder(
        _yaw(POSE_SOFA_REVERSED.pose.orientation)
        - _yaw(POSE_SOFA.pose.orientation),
        2.0 * math.pi,
    )
    assert abs(abs(delta) - math.pi) < 1e-9


def test_orientation_stays_unit_quaternion():
    q = POSE_SOFA_REVERSED.pose.orientation
    norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
    assert abs(norm - 1.0) < 1e-9
