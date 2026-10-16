import pytest
from geometry_msgs.msg import PoseStamped

from behavior_tree.hangzhougrasp.pose_index import PoseIndex


def ps(x, y):
    p = PoseStamped()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    return p


def test_tables_sorted_numerically():
    names = ["bedroom_table_10_0", "bedroom_table_2_0",
             "bedroom_table_2_10", "bedroom_table_2_2"]
    idx = PoseIndex({n: ps(i, 0) for i, n in enumerate(names)})
    assert [[r.name for r in t] for t in idx.tables("bedroom")] == [
        ["bedroom_table_2_0", "bedroom_table_2_2", "bedroom_table_2_10"],
        ["bedroom_table_10_0"],
    ]


def test_ground_sorted_and_room_filtered():
    names = ["kitchen_ground_0_0", "bedroom_ground_1_0",
             "bedroom_ground_0_10", "bedroom_ground_0_2"]
    idx = PoseIndex({n: ps(0, 0) for n in names})
    assert [r.name for r in idx.ground("bedroom")] == [
        "bedroom_ground_0_2", "bedroom_ground_0_10", "bedroom_ground_1_0"]
    assert [r.name for r in idx.ground("kitchen")] == ["kitchen_ground_0_0"]
    assert idx.rooms() == ["bedroom", "kitchen"]


def test_unmatched_names_reported():
    bad = ["grasp2", "point_area", "Bedroom_table_0_0", "bedroom_desk_0_0"]
    idx = PoseIndex({n: ps(0, 0) for n in bad + ["bedroom_table_0_0"]})
    assert sorted(idx.unmatched) == sorted(bad)
    assert [r.name for t in idx.tables("bedroom") for r in t] == ["bedroom_table_0_0"]
    assert idx.rooms() == ["bedroom"]


def test_duplicate_numeric_names_rejected():
    with pytest.raises(ValueError):
        PoseIndex({"a_table_0_0": ps(0, 0), "a_table_00_0": ps(1, 1)})


def test_unknown_room_returns_empty():
    idx = PoseIndex({"a_table_0_0": ps(0, 0)})
    assert idx.tables("x") == []
    assert idx.ground("x") == []
