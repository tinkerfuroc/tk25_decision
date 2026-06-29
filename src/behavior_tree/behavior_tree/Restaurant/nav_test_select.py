"""Pure (no-ROS) helpers + constants for the restaurant nav test, kept separate
from the BT module so they unit-test without importing behavior_tree.messages."""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

# Sweep positions (for now): centre, then left, then right. Tunable.
SWEEP_PANS: List[float] = [0.0, -60.0, 60.0]
TILT_DEG: float = 40.0


def nearest_index(
    points_xy: List[Tuple[float, float]], robot_xy: Optional[Tuple[float, float]]
) -> int:
    """Index of the point nearest robot_xy (Euclidean). Empty -> -1; robot_xy
    None -> 0 (caller falls back to first when TF is unavailable)."""
    if not points_xy:
        return -1
    if robot_xy is None:
        return 0
    rx, ry = robot_xy
    best_i, best_d = 0, float("inf")
    for i, (x, y) in enumerate(points_xy):
        d = math.hypot(x - rx, y - ry)
        if d < best_d:
            best_i, best_d = i, d
    return best_i
