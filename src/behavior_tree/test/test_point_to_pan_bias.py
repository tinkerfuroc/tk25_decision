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

"""Arm point-to pan math (``BtNode_PointTo``), including the seat pan-bias fix.

``BtNode_PointTo`` aims the arm via ``joint0 = atan2(point.y, point.x)`` and
warns (then the arm rejects the goal) when ``joint0`` leaves ``[-pi/2, pi/2]``.

The ``seat_recommend_bbox_service`` centroid arrives rotated ``pi`` about the
base Z axis relative to ``base_link``: a seat physically in front comes back
with a raw bearing near ``+/-pi``, so the unbiased pan lands out of range and
the arm refuses to point at the recommended seat. ``pan_bias=math.pi`` removes
that offset. These tests are pure math — no ROS graph.
"""

import math

from behavior_tree.nodes.pointing_math import compute_point_to_pan

# The arm's reachable pan range, mirrored from BtNode_PointTo.send_goal.
PAN_MIN = -math.pi / 2
PAN_MAX = math.pi / 2


def _in_range(angle):
    """True iff ``angle`` is within the arm's reachable pan window."""
    return PAN_MIN <= angle <= PAN_MAX


def test_default_bias_is_legacy_atan2():
    """pan_bias=0.0 must reproduce the historical ``atan2(y, x)`` exactly."""
    for x, y in [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (1.0, -0.5), (2.0, 0.3)]:
        assert compute_point_to_pan(x, y) == math.atan2(y, x)


def test_flipped_front_seat_out_of_range_without_bias():
    """The bug: a pi-flipped front seat aims the unbiased pan out of range."""
    # Seat physically in front (+x, slightly left) returned 180-deg-flipped.
    flipped_x, flipped_y = -1.2, -0.1
    raw = compute_point_to_pan(flipped_x, flipped_y)
    assert not _in_range(raw)


def test_pan_bias_pi_restores_in_range_and_direction():
    """pan_bias=pi folds the flipped seat back to its true in-range bearing."""
    # Real seat front-left -> y>0; the flipped centroid negates x and y.
    real_x, real_y = 1.2, 0.5
    flipped_x, flipped_y = -real_x, -real_y
    corrected = compute_point_to_pan(flipped_x, flipped_y, pan_bias=math.pi)
    assert _in_range(corrected)
    # Recovers the true bearing and keeps the left (positive) direction.
    assert math.isclose(corrected, math.atan2(real_y, real_x), abs_tol=1e-9)
    assert corrected > 0.0


def test_pan_bias_sign_is_unambiguous_for_pi():
    """+pi and -pi are equivalent corrections (a half turn either way)."""
    for x, y in [(-1.2, -0.1), (-0.8, 0.4), (-1.5, -0.6)]:
        plus = compute_point_to_pan(x, y, pan_bias=math.pi)
        minus = compute_point_to_pan(x, y, pan_bias=-math.pi)
        assert math.isclose(plus, minus, abs_tol=1e-9)


def test_result_always_wrapped_to_pi():
    """Output stays within (-pi, pi] for any bias."""
    for bias in [0.0, math.pi, -math.pi, 2 * math.pi]:
        out = compute_point_to_pan(-1.0, -0.2, pan_bias=bias)
        assert -math.pi < out <= math.pi
