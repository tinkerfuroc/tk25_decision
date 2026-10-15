"""Offline tests: real control tree/state, deterministic boundary I/O; no ROS calls."""
from concurrent.futures import Future
from types import SimpleNamespace as NS
import json
import time

import pytest
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, RegionOfInterest
from std_msgs.msg import Header

from behavior_tree.HomeServiceHangzhou.config import load_config, validate, room_order
from behavior_tree.HomeServiceHangzhou.state import Context, S, StateNode, get, put
from behavior_tree.HomeServiceHangzhou.tree import create_task
from behavior_tree.HomeServiceHangzhou.perception import DetectScene, IdentifyOwners, parse_identities


def image():
    value = Image()
    value.width, value.height, value.step = 4, 4, 12
    value.encoding = "bgr8"
    value.data = bytes([90] * 48)
    value.header.stamp.sec = 100
    return value


def scene(owner=None, litter=False, status=0):
    header = Header()
    header.stamp.sec = 100
    header.frame_id = "camera_color_optical_frame"
    observations = []
    if owner or litter:
        point = PointStamped()
        point.header.frame_id = "map"
        point.header.stamp.sec = 100
        point.point.x, point.point.y = 2.0, 1.0
        labels = {1: ["sitting_on_sofa"], 2: ["lying_on_floor_possible_fall"],
                  3: ["standing_waving"]}
        observations = [NS(labels=["floor_litter"] if litter else labels[owner],
                           description="测试观测", position_valid=not litter, position=point,
                           bbox=RegionOfInterest(x_offset=0, y_offset=0, width=4, height=4))]
    return NS(header=header, status=status, detected=bool(observations),
              observations=observations, rgb_image=image(),
              log_json="/vision_log/test.json", error_msg="error" if status else "")


@pytest.fixture
def config(tmp_path):
    c = load_config()
    c["retry_delay_seconds"] = 0.0
    c["entry_pose"] = [1.0, 2.0, 0.0]
    c["exit_pose"] = [9.0, 2.0, 3.14]
    c["current_room"] = "dining_room"
    c["arm_stow"] = [0.0] * 7
    c["arm_assist"] = [1.0] * 7
    c["evidence_directory"] = str(tmp_path)
    for index, room in enumerate(c["rooms"]):
        c["room_poses"][room] = [float(index), 2.0, 0.0]
        for command, coordinate in (("on", 6.0), ("off", 7.0)):
            c["switches"][room][command] = {
                "navigation": [coordinate, 3.0, 0.0], "arm": [coordinate] * 7}
    return c


class FakeLeaves:
    def __init__(self, context, *, command="关", fail_arm=None, fail_listen=None,
                 unknown=False, detect_error=False, no_detections=False):
        self.context = context
        self.command_value = command
        self.fail_arm, self.fail_listen = fail_arm, fail_listen
        self.unknown, self.detect_error, self.no_detections = unknown, detect_error, no_detections
        self.calls = []
        self.names = iter(["小明", "小红", "小王"])
        context.fake_io = self

    def node(self, name, callback=lambda: None):
        def run():
            self.calls.append(("node", name))
            return callback()
        return StateNode(name, run)

    def say(self, name, text=None):
        return self.node(name, lambda: self.calls.append(("say", text if text is not None else get("speech"))))

    def listen(self, name):
        def run():
            if name == self.fail_listen:
                return False
            put("heard", next(self.names) if name == "Listen owner name" else "请帮我拿一杯温水，不要加糖。")
        return self.node(name, run)

    def command(self):
        return self.node("Command", lambda: put("command", self.command_value))

    def feature(self):
        def run():
            put("features", "蓝色衣服")
            put("reference_image", image())
        return self.node("Feature", run)

    def head(self, pan, tilt):
        return self.node("Head", lambda: self.calls.append(("head", self.context.room, pan, tilt)))

    def settle(self):
        return self.node("Settle")

    def detect(self, task):
        def run():
            if self.detect_error:
                return False
            owner = {"dining_room": 3, "kitchen": 1, "living_room": 2, "bedroom": None}[self.context.room]
            value = scene(None if self.no_detections else owner,
                          litter=task == "litter" and self.context.room != "bedroom"
                                 and not self.no_detections)
            self.context.record_scene(value, task)
            put("scene" if task == "behaviour" else "litter_scene", value)
            put("scene_received_at", self.context.clock())
            self.calls.append(("detect", self.context.room, task))
        return self.node("Detect " + task, run)

    def identify(self):
        def run():
            owner = {"dining_room": 3, "kitchen": 1, "living_room": 2}[self.context.room]
            put("identities", [None if self.unknown else owner])
        return self.node("Identity", run)

    def goto(self, name):
        def run():
            assert not self.context.arm_dirty
            self.calls.append(("goto", name, get("nav_goal")))
        return self.node(name, run)

    def approach(self):
        return self.node("Approach", lambda: self.calls.append(("approach", self.context.room)))

    def arm(self, name):
        def run():
            self.calls.append(("arm", name, list(get("arm_goal"))))
            if name == self.fail_arm:
                return False
        return self.node(name, run)


def run_tree(config, **kwargs):
    root = create_task(config, lambda context: FakeLeaves(context, **kwargs))
    for _ in range(100):
        root.tick_once()
        if root.status in (S.SUCCESS, S.FAILURE):
            return root, root.context.fake_io
    pytest.fail("tree failed to terminate")


@pytest.mark.parametrize("command,expected", [("开", 6.0), ("关", 7.0)])
def test_complete_random_order_and_four_room_sweep(config, command, expected):
    root, io = run_tree(config, command=command)
    assert root.status == S.SUCCESS
    c = root.context
    assert c.completed == {1, 2, 3}
    assert c.visited == set(config["rooms"])
    assert len(c.litter_rooms) == 3
    assert [e["owner"] for e in c.events if e["event"] == "interaction_completed"] == [3, 1, 2]
    assert [p.name for p in c.owners] == ["小明", "小红", "小王"]
    calls = io.calls
    scans = [call for call in calls if call[0] == "head" and call[2] in (-90, -30, 30, 90)]
    assert [(c[1], c[2]) for c in scans] == [
        (room, angle) for room in room_order(config) for angle in (-90, -30, 30, 90)]
    assert len([call for call in calls if call[0] == "detect" and call[2] == "behaviour"]) == 16
    assert len([call for call in calls if call[0] == "detect" and call[2] == "litter"]) == 4
    assert len([call for call in calls if call[0] == "head" and call[3] == 0]) == 4
    switch = [call for call in calls if call[:2] == ("goto", "Navigate to switch mark")]
    assert switch[0][2].pose.position.x == expected
    assert ("arm", "Move end effector over switch mark", [expected] * 7) in calls
    assert ("say", "我复述一下您的要求：请帮我拿一杯温水，不要加糖。") in calls
    for name in ("小明", "小红", "小王"):
        assert any(call[0] == "say" and name in call[1] and "识别到" in call[1] for call in calls)
    assert any(call[0] == "goto" and call[1] == "Leave through separate exit" for call in calls)
    assert len(list(c.evidence_dir.glob("owner_*.png"))) == 3
    records = [json.loads(line) for line in (c.evidence_dir / "events.jsonl").read_text().splitlines()]
    assert all(record["time"] for record in records)
    before = len(calls)
    for _ in range(3):
        root.tick_once()
    assert len(calls) == before  # runtime ticks cannot restart the mission


@pytest.mark.parametrize("options", [{"unknown": True}, {"no_detections": True}, {"detect_error": True}])
def test_unknown_empty_and_error_never_trigger_interactions(config, options):
    config["max_passes"] = 1
    root, io = run_tree(config, **options)
    assert root.status == S.SUCCESS
    assert not root.context.completed
    assert not any(call[0] == "approach" for call in io.calls)
    assert not any(call[:2] == ("arm", "Move end effector over switch mark") for call in io.calls)
    if options.get("detect_error"):
        assert not root.context.litter_rooms
        assert len([e for e in root.context.events if e["event"] == "perception_failed"]) == 16


def test_failed_assistance_dialogue_retracts_before_navigation(config):
    config["max_passes"] = 1
    root, io = run_tree(config, fail_listen="Listen fallen owner")
    assert root.status == S.SUCCESS
    assert root.context.completed == {1, 3}
    extended = False
    for call in io.calls:
        if call[:2] == ("arm", "Extend assistance arm over person"):
            extended = True
        if call[:2] == ("arm", "Retract arm"):
            extended = False
        if call[0] == "goto":
            assert not extended


def test_failed_retraction_stops_before_entry(config):
    root, io = run_tree(config, fail_arm="Retract arm")
    assert root.status == S.FAILURE
    assert not any(call[0] == "goto" for call in io.calls)


def test_placeholders_fail_before_any_leaf(config):
    bad = load_config()
    root, io = run_tree(bad)
    assert root.status == S.FAILURE
    assert io.calls == []


@pytest.mark.parametrize("text", ['{"owners":[1,1]}', '{"owners":[true,2]}',
                                  '{"owners":[4,null]}', '{"owners":[1]}', '{}'])
def test_identity_rejects_invalid_or_ambiguous_results(text):
    with pytest.raises(ValueError):
        parse_identities(text, 2)


def test_identity_preserves_unknown():
    assert parse_identities('\x60\x60\x60json\n{"owners":[null,3]}\n\x60\x60\x60', 2) == [None, 3]


def test_behaviour_is_not_identity(config):
    context = Context(config)
    context.owners = [NS(name=name) for name in ("甲", "乙", "丙")]
    context.active = {1: scene(3).observations[0]}  # owner 1 waving is not owner 3
    assert context.select_owner(1) is False
    assert context.select_owner(3) is False


@pytest.mark.parametrize("status,stamp,success", [(0, 100, True), (0, 80, False), (2, 100, False)])
def test_detection_error_and_pre_settle_frame_are_not_negative(config, status, stamp, success):
    context = Context(config)
    node = DetectScene("test detector", context, "behaviour")
    put("capture_after", 90)
    node.initialise()
    value = scene(status=status)
    value.header.stamp.sec = stamp
    future = Future()
    future.set_result(value)
    node.response = future
    assert node.update() == (S.SUCCESS if success else S.FAILURE)
    if not success:
        assert get("scene") is None


def test_timed_out_identity_cannot_publish_late_result(config):
    import threading
    context = Context(config)
    put("scene", scene(1))
    gate = threading.Event()

    def backend(*_):
        gate.wait(1)
        return [1]

    node = IdentifyOwners("identity", context, backend=backend)
    node.initialise()
    node.started -= config["identity_vlm_timeout_seconds"] + 3
    assert node.update() == S.FAILURE
    gate.set()
    node.future.result(timeout=2)
    assert get("identities") == []


def test_invalid_numeric_configuration(config):
    config["arm_assist"][0] = float("nan")
    with pytest.raises(ValueError, match="arm_assist"):
        validate(config)


def test_patrol_deadline_reserves_exit_without_ticking_child(config):
    from behavior_tree.HomeServiceHangzhou.state import PatrolBudget
    context = Context(config)
    context.started_at = 100.0
    context.clock = lambda: 100.0 + config["mission_timeout_seconds"] - config["exit_reserve_seconds"]
    calls = []
    child = StateNode("must not run", lambda: calls.append("ran"))
    root = PatrolBudget(child, context)
    root.tick_once()
    assert root.status == S.FAILURE
    assert calls == []


def test_live_scene_request_and_failure_semantics(config):
    from tinker_vision_msgs_26.srv import DetectRgbScene
    context = Context(config)
    node = DetectScene("real interface, fake transport", context, "behaviour")
    requests = []
    future = Future()
    reply = DetectRgbScene.Response()
    reply.status = 2
    reply.error_msg = "VLM unavailable"
    future.set_result(reply)
    node.message_type = DetectRgbScene
    node.client = NS(service_is_ready=lambda: True,
                     call_async=lambda req: (requests.append(req), future)[1])
    put("capture_after", 90)
    node.initialise()
    assert node.update() == S.RUNNING
    assert requests[0].target_frame == "map"
    assert node.update() == S.FAILURE
    assert get("scene") is None


def test_missing_position_blocks_approach(config):
    from behavior_tree.HomeServiceHangzhou.ros_nodes import FreshTarget
    context = Context(config)
    put("target", scene(2).observations[0].position)
    put("target_valid", False)
    node = FreshTarget(context)
    node.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=100_000_000_000)))
    assert node.update() == S.FAILURE
    put("target_valid", True)
    assert node.update() == S.SUCCESS
    node.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=200_000_000_000)))
    assert node.update() == S.FAILURE


def test_pan_timeout_is_not_a_photo_success(config):
    from behavior_tree.HomeServiceHangzhou.ros_nodes import SettledPanTilt
    node = SettledPanTilt("head", x=-90.0, y=30.0)
    node.mock_mode = False
    node._skip_settle = False
    node._settle_deadline = time.time() - 1
    node._latest_state = None
    node._command_at = time.monotonic()
    node._feedback_received = 0.0
    assert node.update() == S.FAILURE


def test_same_frame_identity_request_contains_three_refs_and_exact_roi(config, monkeypatch):
    import io
    import urllib.request
    from behavior_tree.HomeServiceHangzhou.perception import identify
    from kimi_api import _env
    monkeypatch.setattr(_env, "load_env", lambda: None)
    monkeypatch.setattr(_env, "resolve_qwen_target", lambda *a: ("https://test.invalid/v1", "test-key", "test-model"))
    sent = []

    def urlopen(request, timeout):
        sent.append(json.loads(request.data))
        return io.BytesIO(json.dumps({"choices": [{"message": {"content": '{"owners":[3]}'}}]}).encode())

    monkeypatch.setattr(urllib.request, "urlopen", urlopen)
    owners = [NS(features="外貌参考", comparison_image=image()) for _ in range(3)]
    assert identify(scene(3), owners, config) == [3]
    images = [part for part in sent[0]["messages"][1]["content"] if part["type"] == "image_url"]
    assert len(images) == 4  # three enrollment images and the detected person's ROI
    assert sent[0]["model"] == "test-model"
