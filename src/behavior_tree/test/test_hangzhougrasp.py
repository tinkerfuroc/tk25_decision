"""HangzhouGrasp composition, bounded retries and grasp payload contracts."""

from collections import Counter
import json
import math

import py_trees
import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from tinker_vision_msgs_26.msg import Object
from tinker_vision_msgs_26.srv import ObjectDetectionGeneralist

from behavior_tree.hangzhougrasp import hangzhougrasp as task
from behavior_tree.hangzhougrasp import config as cfg
from behavior_tree.hangzhougrasp.cli import _build_parser, main
from behavior_tree.hangzhougrasp.custom_nodes import BtNode_SelectGraspTarget

Status = py_trees.common.Status
BB = py_trees.blackboard.Blackboard


@pytest.fixture(autouse=True)
def clear_blackboard():
    BB.clear()
    yield
    BB.clear()


def _pose(x, y):
    from geometry_msgs.msg import PoseStamped
    p = PoseStamped(header=Header(frame_id="map"))
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation.w = 1.0
    return p


SMALL_POSES = {"bedroom_table_0_0": _pose(1, 2), "bedroom_ground_0_0": _pose(2, 2),
               "point_area": _pose(3, 4)}


@pytest.fixture
def config():
    return cfg.load_config(enter_room="bedroom", poses=SMALL_POSES)


def result(labels=("bottle", "cup")):
    rgb = Image(height=2, width=2, encoding="bgr8", step=6, data=bytes([1] * 12))
    depth = Image(height=2, width=2, encoding="16UC1", step=4, data=bytes([1] * 8))
    return ObjectDetectionGeneralist.Response(
        status=0, header=Header(frame_id="camera_optical_frame"),
        objects=[Object(cls=label) for label in labels],
        segments=[Image(height=2, width=2, encoding="8UC1", step=2,
                        data=bytes([255 - i] * 4)) for i, _ in enumerate(labels)],
        rgb_image=rgb, depth_image=depth, detection_source="yolo",
    )


def test_config_pose_units_and_arm_degrees(config):
    assert config.pose_scan.header.frame_id == "map"
    assert config.pose_scan.pose.position.x == 1.0
    assert config.pose_scan.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert config.arm_pos_navigating[0] == pytest.approx(math.radians(-15))


def test_unset_real_waypoints_are_rejected(tmp_path):
    path = tmp_path / "run.json"
    path.write_text(json.dumps({"pose_scan": None}))
    with pytest.raises(ValueError, match="pose_scan is unset"):
        cfg.load_config(path)


def test_named_waypoints_use_map_studio(tmp_path, monkeypatch):
    monkeypatch.setenv("MAP_STUDIO_MAPS_DIR", str(tmp_path))
    (tmp_path / "8902_2_edit.poses.yaml").write_text(
        'map: "8902_2_edit"\nframe_id: map\nposes:\n'
        '  grasp2: {x: 1.308, y: -0.069, yaw: 1.6825}\n'
        '  point_area: {x: 1.827, y: 0.820, yaw: -1.5708}\n')
    conf = cfg.load_config()
    assert conf.pose_scan.pose.position.x == pytest.approx(1.308)
    assert conf.pose_scan.pose.position.y == pytest.approx(-0.069)
    assert conf.pose_scan.pose.orientation.z == pytest.approx(math.sin(1.6825 / 2))
    assert conf.pose_destination.pose.position.x == pytest.approx(1.827)
    assert conf.pose_destination.pose.orientation.z == pytest.approx(math.sin(-1.5708 / 2))
    assert conf.prompt == "coke" and conf.label_map == {"coke": "可乐"}
    assert conf.detection_service == "object_detection_yolo_world"
    path = tmp_path / "run.json"
    path.write_text(json.dumps({"pose_scan": "missing"}))
    with pytest.raises(ValueError, match="missing"):
        cfg.load_config(path)


@pytest.mark.parametrize("override", [
    {"nav_retry_limit": 0}, {"grasp_retry_limit": True},
    {"arm_pos_table": [1, 2]}, {"scan_camera": "wrong"},
    {"navigation_action": ""}, {"prompt": []}, {"label_map": []},
    {"typo": 1},
    {"map_name": ""}, {"prompt": "", "label_map": {}},
])
def test_invalid_configuration(tmp_path, override):
    path = tmp_path / "run.json"
    path.write_text(json.dumps(override))
    with pytest.raises((ValueError, TypeError)):
        cfg.load_config(path, scan_pose=[1, 2, 0], destination_pose=[3, 4, 0])


def test_json_dictionary_and_cli_override(tmp_path):
    path = tmp_path / "run.json"
    path.write_text(json.dumps({"prompt": "cup", "label_map": {"cup": "水杯"}}))
    conf = cfg.load_config(
        path, scan_pose=[1, 2, 0], destination_pose=[3, 4, 0], prompt="")
    assert conf.prompt == "cup" and conf.label_map == {"cup": "水杯"}


def test_no_grasp_really_omits_arm_and_grasp_nodes(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=False)
    assert isinstance(root, py_trees.composites.Sequence) and root.memory
    assert not any(isinstance(n, (task.BtNode_Grasp, task.BtNode_MoveArmSingle))
                   for n in root.iterate())
    scans = [n for n in root.iterate() if isinstance(n, task.BtNode_ScanForGeneralist)]
    assert len(scans) == 1 and scans[0].use_orbbec
    assert scans[0].object == "coke" and scans[0].sort_closest
    assert not scans[0].use_vlm_sam_fallback and not scans[0].force_vlm_sam
    assert not scans[0].return_depth_image


def test_grasp_constructs_realsense_payload_and_retries(config):
    root = task.createHangzhouGrasp(config=config, enable_grasp=True)
    assert isinstance(root.children[0], py_trees.composites.Parallel)
    assert isinstance(root.children[2], py_trees.composites.Selector)
    scans = [n for n in root.iterate() if isinstance(n, task.BtNode_ScanForGeneralist)]
    assert len(scans) == 2
    assert all(not n.use_orbbec and n.return_rgb_image and n.return_depth_image
               and n.return_segments and not n.transform_to_map for n in scans)
    retry = next(n for n in root.iterate() if n.name == "Retry reacquire and grasp")
    assert retry.num_failures == config.grasp_retry_limit
    assert retry.decorated.memory
    assert any(n.name == "Retract arm with object" for n in root.iterate())


def test_cli_branch_flags_and_print_without_setup(capsys):
    assert not _build_parser().parse_args([]).grasp
    assert _build_parser().parse_args(["--grasp"]).grasp
    assert not _build_parser().parse_args(["--grasp", "--no-grasp"]).grasp
    main(["--scan-pose", "1", "2", "90", "--destination-pose", "3", "4", "0",
          "--print-tree", "--grasp"])
    assert "Retry reacquire and grasp" in capsys.readouterr().out


def selector():
    return BtNode_SelectGraspTarget("Select", "scan", "selected", "label")


def test_nearest_mask_and_original_response_preserved():
    source = result()
    BB.set("scan", source)
    assert selector().update() == Status.SUCCESS
    selected = BB.get("selected")
    assert selected is not source
    assert [o.cls for o in selected.objects] == ["bottle"]
    assert selected.segments == [source.segments[0]]
    assert selected.rgb_image == source.rgb_image
    assert selected.depth_image == source.depth_image
    assert len(source.objects) == len(source.segments) == 2
    assert BB.get("label") == "bottle"


@pytest.mark.parametrize("bad", ["empty", "mask_count", "size", "no_rgb", "no_frame", "zero_mask"])
def test_invalid_grasp_bundle_clears_old_target(bad):
    node = selector()
    BB.set("scan", result())
    assert node.update() == Status.SUCCESS
    value = result()
    if bad == "empty":
        value.objects = []
    elif bad == "mask_count":
        value.segments = []
    elif bad == "size":
        value.depth_image.width = 3
    elif bad == "no_rgb":
        value.rgb_image.data = b""
    elif bad == "no_frame":
        value.header.frame_id = ""
    else:
        value.segments[0].data = bytes(4)
    BB.set("scan", value)
    assert node.update() == Status.FAILURE
    assert BB.get("selected") is None and BB.get("label") == ""


@pytest.fixture
def simulated(monkeypatch):
    """Run real composites/selection with recorded hardware-boundary leaves."""
    events = []
    counts = Counter()
    failures = {}
    malformed_reacquires = set()

    class Leaf(py_trees.behaviour.Behaviour):
        def __init__(self, kind, effect=None, **kwargs):
            super().__init__(kwargs["name"])
            self.kind, self.kwargs, self.effect = kind, kwargs, effect

        def update(self):
            counts[self.name] += 1
            events.append(self.name)
            if counts[self.name] <= failures.get(self.name, 0):
                return Status.FAILURE
            if self.effect:
                self.effect(self.kwargs, counts[self.name])
            return Status.SUCCESS

    def scan_effect(kw, count):
        prompt = kw.get("object")
        if prompt is None:
            prompt = BB.get(kw["bb_source"])
        labels = ("bottle", "cup") if not prompt else (prompt,)
        value = result(labels)
        if kw["bb_key"] == cfg.KEY_GRASP_SCAN_RESULT and count in malformed_reacquires:
            value.segments = []
        BB.set(kw["bb_key"], value)

    def announce_effect(kw, count):
        assert BB.get(kw["bb_source"]).objects
        assert kw["label_map"] == {"coke": "可乐"}
        BB.set(kw["bb_text_key"], "我看到了可乐。")

    def grasp_effect(kw, count):
        payload = BB.get(kw["bb_key_vision_res"])
        assert len(payload.objects) == len(payload.segments) == 1
        assert BB.get(kw["bb_key_object_label"]) == payload.objects[0].cls
        assert payload.header.frame_id == "camera_optical_frame"

    monkeypatch.setattr(task, "BtNode_GotoAction", lambda **kw: Leaf("nav", **kw))
    monkeypatch.setattr(task, "BtNode_MoveArmSingle", lambda **kw: Leaf("arm", **kw))
    monkeypatch.setattr(task, "BtNode_ScanForGeneralist",
                        lambda **kw: Leaf("scan", effect=scan_effect, **kw))
    monkeypatch.setattr(task, "BtNode_AnnounceDetections",
                        lambda **kw: Leaf("announce", effect=announce_effect, **kw))
    monkeypatch.setattr(task, "BtNode_Grasp",
                        lambda **kw: Leaf("grasp", effect=grasp_effect, **kw))
    return events, counts, failures, malformed_reacquires


def run(config, enable_grasp):
    root = task.createHangzhouGrasp(config=config, enable_grasp=enable_grasp)
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup(node=None)
    for _ in range(200):
        tree.tick()
        if root.status != Status.RUNNING:
            return root.status
    pytest.fail("bounded tree did not terminate")


def test_no_grasp_mission_order(config, simulated):
    events, _, _, _ = simulated
    assert run(config, False) == Status.SUCCESS
    assert events == [
        "Go to scan point", "Scan objects with YOLO-World",
        "Announce detected objects in Chinese", "Go to destination",
    ]


def test_grasp_mission_order_and_retraction_before_transport(config, simulated):
    events, _, _, _ = simulated
    assert run(config, True) == Status.SUCCESS
    assert events == [
        "Arm to nav before first navigation", "Go to scan point",
        "Arm to table scan pose", "Scan objects with YOLO-World",
        "Announce detected objects in Chinese",
        "Reacquire selected label with RealSense", "Grasp selected object",
        "Retract arm with object", "Go to destination",
    ]


def test_grasp_retry_reacquires_without_repeating_initial_announce(config, simulated):
    _, counts, failures, _ = simulated
    failures["Grasp selected object"] = 1
    assert run(config, True) == Status.SUCCESS
    assert counts["Reacquire selected label with RealSense"] == 2
    assert counts["Announce detected objects in Chinese"] == 1
    assert counts["Go to scan point"] == 1


def test_bad_reacquire_does_not_erase_the_requested_label(config, simulated):
    _, counts, _, malformed = simulated
    malformed.add(1)
    assert run(config, True) == Status.SUCCESS
    assert counts["Reacquire selected label with RealSense"] == 2
    assert counts["Grasp selected object"] == 1
    assert BB.get(cfg.KEY_TARGET_LABEL) == "coke"


@pytest.mark.parametrize("failed_leaf, retry_field", [
    ("Scan objects with YOLO-World", "scan_retry_limit"),
    ("Announce detected objects in Chinese", "announce_retry_limit"),
    ("Grasp selected object", "grasp_retry_limit"),
])
def test_failure_retracts_and_never_navigates_to_destination(
        config, simulated, failed_leaf, retry_field):
    _, counts, failures, _ = simulated
    failures[failed_leaf] = 100
    assert run(config, True) == Status.FAILURE
    assert counts[failed_leaf] == getattr(config, retry_field)
    assert counts["Retract arm on failure"] == 1
    assert counts["Go to destination"] == 0


def test_failed_retraction_prevents_base_motion(config, simulated):
    _, counts, failures, _ = simulated
    failures["Retract arm with object"] = 100
    failures["Retract arm on failure"] = 100
    assert run(config, True) == Status.FAILURE
    assert counts["Retract arm with object"] == config.arm_retry_limit
    assert counts["Retract arm on failure"] == config.arm_retry_limit
    assert counts["Go to destination"] == 0


def test_navigation_retries_are_bounded_before_scanning(config, simulated):
    _, counts, failures, _ = simulated
    failures["Go to scan point"] = 100
    assert run(config, False) == Status.FAILURE
    assert counts["Go to scan point"] == config.nav_retry_limit
    assert counts["Scan objects with YOLO-World"] == 0

