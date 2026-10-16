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


@pytest.fixture(autouse=True)
def poses(monkeypatch):
    import map_studio.poses
    monkeypatch.setattr(map_studio.poses, "load_poses", lambda name: dict(POSES))


@pytest.fixture
def conf_path(tmp_path):
    path = tmp_path / "run.json"
    path.write_text(json.dumps({"enter_profiles": PROFILES, "enter_room": "bedroom"}))
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
    path.write_text(json.dumps({"enter_profiles": profiles, "enter_room": "bedroom"}))
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
