import pytest

from pathlib import Path as _Path
TEST_CONFIG = str(_Path(__file__).with_name("hangzhougrasp_test_config.json"))
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
    """Override JSON; enter_room/enter_profiles go into the 8902_2_edit map."""
    entry = {"enter_room": "bedroom", "enter_profiles": {
        "bedroom": {"rooms": ["bedroom"], "destination": "point_area"}}}
    for key in ("enter_room", "enter_profiles"):
        if key in over:
            entry[key] = over.pop(key)
    over.setdefault("map_name", "8902_2_edit")
    over.setdefault("maps", {over["map_name"]: entry})
    path = tmp_path / "run.json"
    path.write_text(json.dumps(over))
    return path


def test_load_config_builds_route_and_destination():
    c = hgcfg.load_config(TEST_CONFIG, enter_room="bedroom", poses=POSES)
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
        hgcfg.load_config(TEST_CONFIG, enter_room="attic", poses=POSES)


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


# ---- audit fixes: clearer config errors, Mandarin names ----
def test_profile_room_name_that_cannot_match_fails(tmp_path):
    path = _write(tmp_path, enter_profiles={
        "bedroom": {"rooms": ["bedroom", "living_room"], "destination": "point_area"}})
    with pytest.raises(ValueError, match="living_room.*lowercase"):
        hgcfg.load_config(path, enter_room="bedroom", poses=POSES)


def test_room_with_only_misnamed_poses_names_them(tmp_path):
    path = _write(tmp_path, enter_profiles={
        "bedroom": {"rooms": ["bedroom", "kitchen"], "destination": "point_area"}})
    poses = {"bedroom_table_0_0": ps(1, 1), "kitchen_tabel_0_0": ps(2, 2),
             "point_area": ps(9, 9)}
    with pytest.raises(ValueError, match="kitchen_tabel_0_0"):
        hgcfg.load_config(path, enter_room="bedroom", poses=poses)


@pytest.mark.parametrize("value", [None, ""])
def test_missing_enter_room_says_how_to_set_it(tmp_path, value):
    path = _write(tmp_path, enter_room=value)
    with pytest.raises(ValueError, match="--enter-room"):
        hgcfg.load_config(path, poses=POSES)


@pytest.mark.parametrize("prompt", ["coke . mug", "coke, mug", "mug"])
def test_prompt_label_without_mandarin_name_fails(prompt):
    with pytest.raises(ValueError, match="mug.*Mandarin"):
        hgcfg.load_config(TEST_CONFIG, enter_room="bedroom", prompt=prompt, poses=POSES)


@pytest.mark.parametrize("prompt", ["coke", "Coke", "", "coke , coke"])
def test_prompt_labels_with_mandarin_names_pass(prompt):
    c = hgcfg.load_config(TEST_CONFIG, enter_room="bedroom", prompt=prompt, poses=POSES)
    assert c.label_map["Coke"] == "可乐"


# Classes of yolo26n-20261016-171034-yolo26n-c8ec-best.pt (the vision pane's model).
CHECKPOINT_LABELS = ["Banana", "Pringles", "Shampoo", "Coke", "Biscuits", "IceTea",
                     "Apple", "Cup", "HandSanitizer", "SoapBox", "TeddyBear",
                     "Detergent"]


def test_label_map_covers_every_checkpoint_class():
    c = hgcfg.load_config(TEST_CONFIG, enter_room="bedroom", prompt="", poses=POSES)
    assert set(c.label_map) == set(CHECKPOINT_LABELS)
    assert all(c.label_map.values())
    assert c.prompt == " . ".join(c.label_map)


# ---- TK27 manipulation config ----
def test_tk27_config_defaults():
    c = hgcfg.load_config(TEST_CONFIG, enter_room="bedroom", poses=POSES)
    assert c.pick_action == "/manipulation/pick"
    assert c.saved_pose_action == "/manipulation/move_to_saved_pose"
    assert c.gripper_action == "/xarm_gripper/gripper_action"
    assert c.pick_plan_effort == "high"
    assert c.arm_pose_navigating == "navigation"
    assert c.arm_pose_table == "pick45"
    assert c.gripper_open_position == 0.0
    assert c.gripper_open_tolerance == 0.025
    assert c.gripper_max_effort == 50.0
    for old in ("arm_pos_navigating", "arm_pos_table", "arm_action", "grasp_action"):
        assert not hasattr(c, old)


@pytest.mark.parametrize("key", [
    "arm_pos_navigating", "arm_pos_table", "arm_action", "grasp_action"])
def test_old_arm_keys_rejected(tmp_path, key):
    with pytest.raises(ValueError, match="Unknown"):
        hgcfg.load_config(_write(tmp_path, **{key: "x"}), poses=POSES)


@pytest.mark.parametrize("effort, ok", [
    ("low", True), ("medium", True), ("high", True), ("auto", True),
    ("max", False), ("", False), ("HIGH", False)])
def test_plan_effort_validated(tmp_path, effort, ok):
    path = _write(tmp_path, pick_plan_effort=effort)
    if ok:
        assert hgcfg.load_config(path, poses=POSES).pick_plan_effort == effort
    else:
        with pytest.raises(ValueError, match="pick_plan_effort"):
            hgcfg.load_config(path, poses=POSES)


@pytest.mark.parametrize("key, value", [
    ("gripper_open_tolerance", 0), ("gripper_open_tolerance", -0.1),
    ("gripper_open_tolerance", True), ("gripper_max_effort", 0),
    ("gripper_max_effort", -1), ("gripper_open_position", "0.0"),
    ("gripper_open_position", "inf")])
def test_gripper_numbers_validated(tmp_path, key, value):
    path = _write(tmp_path, **{key: value})
    if value == "inf":  # JSON cannot hold inf; write the token json accepts
        path.write_text(path.read_text().replace('"inf"', "Infinity"))
    with pytest.raises(ValueError, match=key):
        hgcfg.load_config(path, poses=POSES)


@pytest.mark.parametrize("key", [
    "arm_pose_table", "arm_pose_navigating", "pick_action",
    "saved_pose_action", "gripper_action"])
def test_tk27_names_nonempty(tmp_path, key):
    with pytest.raises(ValueError, match=key):
        hgcfg.load_config(_write(tmp_path, **{key: " "}), poses=POSES)


# ---- Orbbec head (pan-tilt) angles ----
def test_head_angles_defaults():
    c = hgcfg.load_config(TEST_CONFIG, enter_room="bedroom", poses=POSES)
    assert c.head_pan_deg == 0.0
    assert c.table_scan_tilt_deg == 0.0
    assert c.ground_scan_tilt_deg == 0.0
    assert c.head_retry_limit == 3


@pytest.mark.parametrize("key, value", [
    ("table_scan_tilt_deg", 31), ("table_scan_tilt_deg", -31),
    ("ground_scan_tilt_deg", 45), ("ground_scan_tilt_deg", "0"),
    ("head_pan_deg", True), ("head_retry_limit", 0)])
def test_head_angles_validated(tmp_path, key, value):
    with pytest.raises(ValueError, match=key):
        hgcfg.load_config(_write(tmp_path, **{key: value}), poses=POSES)
