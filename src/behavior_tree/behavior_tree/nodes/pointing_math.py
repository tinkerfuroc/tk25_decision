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

"""Pure (ROS-free) geometry helpers for arm pointing.

Kept free of ROS / message imports so the math can be unit-tested without a
running graph (``behavior_tree.interfaces.messages`` does not import on every host).
"""

import math


def wrap_to_pi(angle: float) -> float:
    """Normalize ``angle`` (radians) into the half-open interval ``(-pi, pi]``."""
    return math.atan2(math.sin(angle), math.cos(angle))


def compute_point_to_pan(
    point_x: float, point_y: float, pan_bias: float = 0.0
) -> float:
    """Arm joint0 (pan) angle, in radians, that points at ``(point_x, point_y)``.

    ``point_x`` / ``point_y`` are the target's planar coordinates in the frame
    the point is expressed in (``base_link`` for the seat/person centroids).
    The raw bearing is ``atan2(y, x)``.

    ``pan_bias`` corrects a known constant rotation between the point's source
    frame and the arm's joint0 frame. With a correctly-calibrated camera TF the
    seat/person centroid is already correct in ``base_link`` and the arm base is
    aligned with ``base_link``, so production seat/person-pointing uses
    ``pan_bias=0.0`` (joint0 = ``atan2(y, x)`` directly). The ``math.pi`` path
    is retained for the general case: if a point's source frame is genuinely
    pi-rotated about base Z (e.g. an un-fixed backward-camera TF — the bug fixed
    2026-06-27), ``pan_bias=math.pi`` folds the raw bearing back into range.

    The biased result is wrapped into ``(-pi, pi]`` so the correction never
    pushes the command past a full turn. With the default ``pan_bias=0.0`` this
    returns exactly ``atan2(y, x)`` (the legacy behaviour, no wrap), so existing
    callers are byte-for-byte unaffected.
    """
    bearing = math.atan2(point_y, point_x)
    if pan_bias == 0.0:
        return bearing
    return wrap_to_pi(bearing - pan_bias)
