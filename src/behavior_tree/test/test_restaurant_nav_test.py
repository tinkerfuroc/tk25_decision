"""Pure unit tests for the restaurant nav-test selection helper (no ROS)."""
from behavior_tree.Restaurant.nav_test_select import nearest_index, SWEEP_PANS, TILT_DEG


def test_sweep_positions_are_centre_left_right():
    assert SWEEP_PANS == [0.0, -60.0, 60.0]
    assert TILT_DEG == 10.0


def test_nearest_index_picks_closest_to_robot():
    pts = [(5.0, 0.0), (1.0, 0.0), (3.0, 0.0)]
    assert nearest_index(pts, (0.0, 0.0)) == 1


def test_nearest_index_robot_none_returns_zero():
    assert nearest_index([(5.0, 0.0), (1.0, 0.0)], None) == 0


def test_nearest_index_empty_returns_minus_one():
    assert nearest_index([], (0.0, 0.0)) == -1
