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


# ---- route and config ----
import json

from behavior_tree.hangzhougrasp import config as hgcfg
from behavior_tree.hangzhougrasp.route import Stop, build_route


def _route_names(route):
    return [(s.room, s.kind, [r.name for r in s.viewpoints]) for s in route]


def test_build_route_order():
    names = ["kitchen_table_1_0", "kitchen_table_0_0", "kitchen_table_0_1",
             "kitchen_ground_0_0", "bedroom_table_0_0"]
    idx = PoseIndex({n: ps(0, 0) for n in names})
    assert _route_names(build_route(idx, ["kitchen", "bedroom"])) == [
        ("kitchen", "table", ["kitchen_table_0_0", "kitchen_table_0_1"]),
        ("kitchen", "table", ["kitchen_table_1_0"]),
        ("kitchen", "ground", ["kitchen_ground_0_0"]),
        ("bedroom", "table", ["bedroom_table_0_0"]),
    ]


def test_room_without_poses_fails():
    idx = PoseIndex({"kitchen_table_0_0": ps(0, 0)})
    with pytest.raises(ValueError, match="livingroom"):
        build_route(idx, ["kitchen", "livingroom"])


POSES = {"bedroom_table_0_0": ps(1, 1), "bedroom_ground_0_0": ps(2, 2),
         "kitchen_table_0_0": ps(3, 3), "point_area": ps(9, 9)}


def _write(tmp_path, **over):
    path = tmp_path / "run.json"
    path.write_text(json.dumps(over))
    return path


def test_load_config_builds_route_and_destination():
    c = hgcfg.load_config(enter_room="bedroom", poses=POSES)
    assert c.enter_room == "bedroom"
    assert c.rooms == ["bedroom"]
    assert c.destination_name == "point_area"
    assert c.destination.pose.position.x == 9.0
    assert [(s.room, s.kind) for s in c.route] == [
        ("bedroom", "table"), ("bedroom", "ground")]
    assert hgcfg.pose_key("x") == "hg_pose_x"
    assert not hasattr(c, "pose_scan")


def test_load_config_unknown_enter_room_fails():
    with pytest.raises(ValueError, match="attic"):
        hgcfg.load_config(enter_room="attic", poses=POSES)


def test_enter_room_not_in_rooms_fails(tmp_path):
    path = _write(tmp_path, enter_profiles={
        "bedroom": {"rooms": ["kitchen"], "destination": "point_area"}})
    with pytest.raises(ValueError, match="enter_room"):
        hgcfg.load_config(path, enter_room="bedroom", poses=POSES)


def test_missing_destination_pose_fails(tmp_path):
    path = _write(tmp_path, enter_profiles={
        "bedroom": {"rooms": ["bedroom"], "destination": "nowhere"}})
    with pytest.raises(ValueError, match="nowhere"):
        hgcfg.load_config(path, enter_room="bedroom", poses=POSES)


def test_missing_enter_profiles_fails(tmp_path):
    path = _write(tmp_path, enter_profiles={})
    with pytest.raises(ValueError, match="enter_profiles"):
        hgcfg.load_config(path, enter_room="bedroom", poses=POSES)


def test_old_keys_rejected(tmp_path):
    path = _write(tmp_path, pose_scan="grasp2")
    with pytest.raises(ValueError, match="Unknown"):
        hgcfg.load_config(path, enter_room="bedroom", poses=POSES)
