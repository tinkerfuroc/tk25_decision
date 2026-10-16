"""hangzhougrasp CLI: --enter-room, --part, removed flags, config errors."""

import json

import pytest
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from behavior_tree.hangzhougrasp import cli
from behavior_tree.hangzhougrasp import config as cfg


def _pose(x, y):
    p = PoseStamped(header=Header(frame_id="map"))
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.w = 1.0
    return p


POSES = {
    "bedroom_table_0_0": _pose(1, 2), "bedroom_table_0_1": _pose(1, 3),
    "bedroom_ground_0_0": _pose(2, 2),
    "kitchen_table_0_0": _pose(5, 2),
    "point_area": _pose(3, 4),
}
PROFILES = {
    "bedroom": {"rooms": ["bedroom", "kitchen"], "destination": "point_area"},
    "kitchen": {"rooms": ["kitchen", "bedroom"], "destination": "point_area"},
}


def _wrap(profiles, enter_room="bedroom", map_name="8902_2_edit"):
    return {"map_name": map_name, "maps": {map_name: {
        "enter_room": enter_room, "enter_profiles": profiles}}}


@pytest.fixture(autouse=True)
def poses(monkeypatch):
    import map_studio.poses
    monkeypatch.setattr(map_studio.poses, "load_poses", lambda name: dict(POSES))


@pytest.fixture
def conf_path(tmp_path):
    path = tmp_path / "run.json"
    path.write_text(json.dumps(_wrap(PROFILES)))
    return str(path)


def test_cli_unknown_enter_room_exits_nonzero(conf_path):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, "--enter-room", "nowhere", "--print-tree"])
    assert exc.value.code != 0


@pytest.mark.parametrize("flag", ["--scan-pose", "--destination-pose"])
def test_cli_rejects_removed_pose_flags(conf_path, flag):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, flag, "1", "2", "3", "--print-tree"])
    assert exc.value.code != 0


def test_print_tree_with_profile(conf_path, capsys):
    cli.main(["--config", conf_path, "--print-tree"])
    out = capsys.readouterr().out
    assert "bedroom_table_0_0" in out and "kitchen_table_0_0" in out


def test_enter_room_flag_overrides_json(conf_path, capsys):
    cli.main(["--config", conf_path, "--enter-room", "kitchen", "--print-tree"])
    out = capsys.readouterr().out
    assert out.index("kitchen_table_0_0") < out.index("bedroom_table_0_0")


def test_cli_part_scan_announce_print_tree(conf_path, capsys):
    cli.main(["--config", conf_path, "--part", "scan-announce", "--print-tree"])
    out = capsys.readouterr().out
    assert "bedroom_table_0_0" not in out.replace("Write bedroom_table_0_0", "")
    assert "nnounce" in out and "can" in out


def test_cli_part_missing_room_exits_nonzero(conf_path):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, "--part", "ground", "--print-tree"])
    assert exc.value.code != 0


def test_cli_unknown_part_exits_nonzero(conf_path):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, "--part", "bogus", "--print-tree"])
    assert exc.value.code != 0


def test_cli_part_room_print_tree(conf_path, capsys):
    cli.main(["--config", conf_path, "--part", "room", "--room", "bedroom",
              "--grasp", "--print-tree"])
    assert "bedroom_ground_0_0" in capsys.readouterr().out


def test_cli_part_needing_grasp_without_it_exits(conf_path):
    with pytest.raises(SystemExit):
        cli.main(["--config", conf_path, "--part", "deliver", "--print-tree"])


def test_cli_missing_pose_file_exits_nonzero(conf_path, monkeypatch):
    import map_studio.poses

    def boom(name):
        raise FileNotFoundError("no poses file")
    monkeypatch.setattr(map_studio.poses, "load_poses", boom)
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, "--print-tree"])
    assert exc.value.code != 0


def _write(tmp_path, profiles):
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(_wrap(profiles)))
    return str(path)


def test_non_dict_profile_is_value_error(tmp_path):
    with pytest.raises(ValueError, match="enter_profiles\\['bedroom'\\]"):
        cfg.load_config(_write(tmp_path, {"bedroom": ["bedroom"]}), poses=POSES)


def test_profile_missing_destination_is_value_error(tmp_path):
    with pytest.raises(ValueError, match="destination"):
        cfg.load_config(_write(tmp_path, {"bedroom": {"rooms": ["bedroom"]}}),
                        poses=POSES)


def test_scan_camera_key_no_longer_accepted(tmp_path):
    path = tmp_path / "c.json"
    path.write_text(json.dumps({"scan_camera": "orbbec"}))
    with pytest.raises(ValueError, match="Unknown"):
        cfg.load_config(str(path), poses=POSES)


def test_ros_args_remainder_tolerated(conf_path, capsys):
    cli.main(["--config", conf_path, "--print-tree",
              "--ros-args", "-r", "__node:=foo"])
    assert "bedroom_table_0_0" in capsys.readouterr().out


@pytest.mark.parametrize("args", [
    ["--scan-pose", "1", "2", "3"], ["--scan-pose=1"],
    ["--destination-pose", "1"], ["--destination-pose=1,2,3"]])
def test_removed_pose_flags_rejected_even_with_ros_args(conf_path, args):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, *args, "--print-tree",
                  "--ros-args", "-r", "__node:=foo"])
    assert exc.value.code != 0


def test_other_unknown_flags_still_rejected(conf_path):
    with pytest.raises(SystemExit) as exc:
        cli.main(["--config", conf_path, "--bogus", "--print-tree"])
    assert exc.value.code != 0


def test_duplicate_rooms_in_profile_rejected(tmp_path):
    profiles = {"bedroom": {"rooms": ["bedroom", "kitchen", "kitchen"],
                            "destination": "point_area"}}
    with pytest.raises(ValueError, match="duplicate.*kitchen"):
        cfg.load_config(_write(tmp_path, profiles), poses=POSES)


def test_misnamed_pose_in_profile_room_rejected(tmp_path):
    poses = dict(POSES)
    poses["bedroom_tabel_1_0"] = _pose(9, 9)
    poses["grasp2"] = _pose(8, 8)
    with pytest.raises(ValueError) as exc:
        cfg.load_config(_write(tmp_path, PROFILES), poses=poses)
    msg = str(exc.value)
    assert "bedroom_tabel_1_0" in msg and "table|ground" in msg
    assert "grasp2" not in msg


def test_unmatched_other_names_warn(tmp_path, monkeypatch):
    seen = []
    monkeypatch.setattr(
        cfg.log, "warning", lambda msg, *args: seen.append(msg % args))
    poses = dict(POSES)
    poses["grasp2"] = _pose(8, 8)
    cfg.load_config(_write(tmp_path, PROFILES), poses=poses)
    assert len(seen) == 1 and "grasp2" in seen[0]


def test_unmatched_room_prefix_only_for_profile_rooms(tmp_path):
    poses = dict(POSES)
    poses["living_room_table_0_0"] = _pose(7, 7)  # not a profile room: warning only
    cfg.load_config(_write(tmp_path, PROFILES), poses=poses)
    poses["kitchen_table_x_0"] = _pose(6, 6)
    with pytest.raises(ValueError, match="kitchen_table_x_0"):
        cfg.load_config(_write(tmp_path, PROFILES), poses=poses)


# ---- switching between known maps ----
ROBOCUP = "robocup_home_2026_measured_5cm"
# Pose names recorded in Map Studio for the robocup map (2026-10-16), with the
# diningroom_table1_1 typo fixed and the two destinations the user will label.
ROBOCUP_NAMES = [
    "livingroom_switch", "bedroom_switch", "bedroom_search", "diningroom_search",
    "kitchen_search", "livingroom_search", "register",
    "livingroom_ground_0_0", "livingroom_ground_1_0", "diningroom_ground_0_0",
    "kitchen_ground_0_0", "kitchen_ground_1_0", "diningroom_ground_1_0",
    "bedroom_ground_0_0", "bedroom_table_0_0", "bedroom_table_1_0",
    "diningroom_table_0_0", "diningroom_table_1_0", "diningroom_table_1_1",
    "diningroom_table_1_2", "diningroom_table_1_3", "livingroom_table_0_0",
    "livingroom_table_0_1", "livingroom_table_1_0", "livingroom_table_1_1",
    "livingroom_table_2_0", "livingroom_table_2_1", "kitchen_table_0_0",
    "kitchen_table_1_0", "kitchen_table_2_0", "diningroom_table_2_0",
    "livingroom_destination", "diningroom_destination",
]


# 8902_2_edit after the 2026-10-16 rename (was grasp1, grasp2, point_area).
LAB_POSES = {"room1_table_0_0": _pose(2.8, -0.1), "room2_table_0_0": _pose(1.3, -0.1),
             "room1_destination": _pose(1.8, 0.8)}


def test_lab_map_shipped_profile():
    c = cfg.load_config(map_name="8902_2_edit", poses=dict(LAB_POSES))
    assert (c.enter_room, c.rooms, c.destination_name) == (
        "room1", ["room1", "room2"], "room1_destination")
    assert [r.name for s in c.route for r in s.viewpoints] == [
        "room1_table_0_0", "room2_table_0_0"]


def _robocup_poses(names=ROBOCUP_NAMES):
    return {n: _pose(i, 0) for i, n in enumerate(names)}


def _rooms_in_route(c):
    seen = []
    for stop in c.route:
        if stop.room not in seen:
            seen.append(stop.room)
    return seen


def test_shipped_maps_and_default():
    import json as _json
    from pathlib import Path
    data = _json.loads(Path(cfg.__file__).with_name("constants.json").read_text())
    assert data["map_name"] == ROBOCUP
    assert set(data["maps"]) == {ROBOCUP, "8902_2_edit"}
    assert "enter_profiles" not in data and "enter_room" not in data


@pytest.mark.parametrize("enter, order", [
    ("livingroom", ["livingroom", "kitchen", "diningroom", "bedroom"]),
    ("diningroom", ["diningroom", "kitchen", "bedroom", "livingroom"]),
])
def test_robocup_profiles_route_order(enter, order):
    c = cfg.load_config(map_name=ROBOCUP, enter_room=enter, poses=_robocup_poses())
    assert c.map_name == ROBOCUP and c.rooms == order
    assert _rooms_in_route(c) == order
    assert c.destination_name == f"{enter}_destination"


def test_robocup_default_enter_room_is_livingroom():
    c = cfg.load_config(map_name=ROBOCUP, poses=_robocup_poses())
    assert c.enter_room == "livingroom"


def test_robocup_bedroom_is_not_an_entry_room():
    with pytest.raises(ValueError, match="bedroom"):
        cfg.load_config(map_name=ROBOCUP, enter_room="bedroom",
                        poses=_robocup_poses())


def test_robocup_helper_poses_are_ignored_not_misnamed(monkeypatch):
    seen = []
    monkeypatch.setattr(cfg.log, "warning", lambda msg, *a: seen.append(msg % a))
    cfg.load_config(map_name=ROBOCUP, poses=_robocup_poses())
    assert "bedroom_search" in seen[0] and "register" in seen[0]


def test_robocup_typo_pose_stops_startup():
    names = [n if n != "diningroom_table_1_1" else "diningroom_table1_1"
             for n in ROBOCUP_NAMES]
    with pytest.raises(ValueError, match="diningroom_table1_1"):
        cfg.load_config(map_name=ROBOCUP, poses=_robocup_poses(names))


def test_robocup_missing_destination_names_it():
    names = [n for n in ROBOCUP_NAMES if n != "livingroom_destination"]
    with pytest.raises(ValueError, match="livingroom_destination"):
        cfg.load_config(map_name=ROBOCUP, poses=_robocup_poses(names))


def test_map_flag_selects_map_and_its_poses(monkeypatch, capsys):
    import map_studio.poses
    asked = []

    def load(name):
        asked.append(name)
        return _robocup_poses() if name == ROBOCUP else dict(LAB_POSES)
    monkeypatch.setattr(map_studio.poses, "load_poses", load)
    cli.main(["--map", ROBOCUP, "--enter-room", "diningroom", "--print-tree"])
    out = capsys.readouterr().out
    assert asked == [ROBOCUP]
    assert out.index("diningroom_table_0_0") < out.index("kitchen_table_0_0")
    cli.main(["--map", "8902_2_edit", "--grasp", "--print-tree"])
    assert asked[-1] == "8902_2_edit"
    out = capsys.readouterr().out
    assert out.index("room1_table_0_0") < out.index("room2_table_0_0")
    assert "Go to destination room1_destination" in out


def test_unknown_map_lists_known_maps():
    with pytest.raises(ValueError, match="known maps.*8902_2_edit"):
        cfg.load_config(map_name="attic_map", poses=POSES)


def test_cli_unknown_map_exits_nonzero():
    with pytest.raises(SystemExit) as exc:
        cli.main(["--map", "attic_map", "--print-tree"])
    assert exc.value.code != 0


def test_top_level_profiles_rejected(tmp_path):
    path = tmp_path / "old.json"
    path.write_text(json.dumps({"enter_profiles": PROFILES, "enter_room": "bedroom"}))
    with pytest.raises(ValueError, match="Unknown"):
        cfg.load_config(str(path), poses=POSES)


def test_unknown_key_in_map_entry_rejected(tmp_path):
    data = _wrap(PROFILES)
    data["maps"]["8902_2_edit"]["destination"] = "point_area"
    path = tmp_path / "bad.json"
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match="unknown keys.*destination"):
        cfg.load_config(str(path), poses=POSES)
