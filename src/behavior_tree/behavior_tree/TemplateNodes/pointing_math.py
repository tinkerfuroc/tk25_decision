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
running graph (``behavior_tree.messages`` does not import on every host).
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
    frame and the arm's joint0 frame. The ``seat_recommend_bbox_service``
    centroid comes back rotated ``pi`` about the base Z axis relative to
    ``base_link``, so a seat physically in front yields a raw bearing near
    ``+/-pi`` — outside the arm's reachable ``[-pi/2, pi/2]`` pan range. Passing
    ``pan_bias=math.pi`` folds it back to the true bearing.

    The biased result is wrapped into ``(-pi, pi]`` so the correction never
    pushes the command past a full turn. With the default ``pan_bias=0.0`` this
    returns exactly ``atan2(y, x)`` (the legacy behaviour, no wrap), so existing
    callers are byte-for-byte unaffected.
    """
    bearing = math.atan2(point_y, point_x)
    if pan_bias == 0.0:
        return bearing
    return wrap_to_pi(bearing - pan_bias)
