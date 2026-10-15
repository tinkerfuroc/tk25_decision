"""Mission memory, identity association and timestamped scoring evidence."""
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace
import json
import math
import re
import time

import py_trees
from .config import BEHAVIOUR_ZH, EXPECTED, validate

PREFIX = "/hangzhou/"
S = py_trees.common.Status


def put(key, value):
    py_trees.blackboard.Blackboard.set(PREFIX + key, value)


def get(key):
    return py_trees.blackboard.Blackboard.get(PREFIX + key)


def stamp_seconds(header):
    return header.stamp.sec + header.stamp.nanosec / 1e9


def valid_point(point):
    return (point is not None and point.header.frame_id == "map"
            and stamp_seconds(point.header) > 0
            and all(math.isfinite(v) for v in (point.point.x, point.point.y, point.point.z)))


def extract_name(text):
    if not isinstance(text, str) or text.strip() in ("", "(empty)"):
        raise ValueError("姓名为空")
    name = re.sub(r"^(你好[，, ]*|我叫|我的名字是|我是)", "", str(text).strip())
    name = name.strip(" ，。！!？?、,\n\t")
    if not name or len(name) > 20 or any(c in name for c in "，。！？,!?\n"):
        raise ValueError("请仅说您的名字")
    return name


@dataclass
class Context:
    config: dict
    owners: list = field(default_factory=list)
    completed: set = field(default_factory=set)
    visited: set = field(default_factory=set)
    litter_rooms: set = field(default_factory=set)
    active: dict = field(default_factory=dict)
    room: str = ""
    arm_dirty: bool = False
    motion_actions: list = field(default_factory=list)
    motion_blocked: bool = False
    audio_actions: list = field(default_factory=list)
    audio_blocked: bool = False
    ready: bool = False
    started_at: float = 0.0
    evidence_dir: object = None
    events: list = field(default_factory=list)
    clock: object = time.monotonic

    def event(self, kind, **values):
        record = {"time": datetime.now(timezone.utc).isoformat(), "event": kind,
                  "room": self.room, **values}
        self.events.append(record)
        if self.evidence_dir:
            with (self.evidence_dir / "events.jsonl").open("a", encoding="utf-8") as stream:
                stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    def initialise(self):
        validate(self.config)
        self.owners.clear()
        self.completed.clear()
        self.visited.clear()
        self.litter_rooms.clear()
        self.active.clear()
        self.arm_dirty = False
        run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
        self.evidence_dir = Path(self.config["evidence_directory"]).expanduser() / run_id
        self.evidence_dir.mkdir(parents=True, exist_ok=False)
        put("state", self)
        put("owners", self.owners)
        self.ready = True
        self.started_at = self.clock()
        self.event("mission_started", excluded=["litter_pickup", "litter_placement"])

    def remember(self, owner):
        from cv_bridge import CvBridge
        import cv2
        name = extract_name(get("heard"))
        features, image = get("features"), get("reference_image")
        if not isinstance(features, str) or not features.strip() or image is None or image.width <= 0 or image.height <= 0:
            raise ValueError("主人特征或参考图像为空")
        if any(person.name == name for person in self.owners):
            raise ValueError("姓名重复，请使用不同称呼")
        if len(self.owners) != owner - 1:
            raise ValueError("主人登记顺序错误")
        path = self.evidence_dir / ("owner_" + str(owner) + ".png")
        if not cv2.imwrite(str(path), CvBridge().imgmsg_to_cv2(image, desired_encoding="bgr8")):
            raise ValueError("无法保存主人参考图像")
        person = SimpleNamespace(name=name, features=features, comparison_image=image)
        self.owners.append(person)
        put("owners", list(self.owners))
        suffix = "请离开镜头，让下一位主人上前。" if owner < 3 else "三位主人的信息登记完成。"
        put("speech", name + "，我已记住您的名字和外貌特征。" + suffix)
        self.event("owner_registered", owner=owner, name=name, features=features,
                   image=str(path), capture_stamp=stamp_seconds(image.header))

    def select_room(self, room):
        self.room = room
        self.active = {}

    def record_scene(self, scene, task):
        self.event(task + "_detection", status=scene.status, detected=scene.detected,
                   capture_stamp=stamp_seconds(scene.header), log_json=scene.log_json,
                   error=scene.error_msg,
                   observations=[{"labels": list(o.labels), "description": o.description,
                                  "position_valid": o.position_valid}
                                 for o in scene.observations])

    def match_scene(self):
        scene = get("scene")
        self.active = {owner: obs for owner, obs in zip(get("identities"), scene.observations)
                       if owner is not None}
        self.event("identity_association", owners=list(self.active), log_json=scene.log_json)

    def select_owner(self, owner):
        obs = self.active.get(owner)
        if owner in self.completed or obs is None or not (set(obs.labels) & EXPECTED[owner]):
            return False
        put("target", obs.position)
        put("target_valid", obs.position_valid)
        put("target_received_at", get("scene_received_at"))
        put("speech", self.owners[owner - 1].name + "，我识别到您" + BEHAVIOUR_ZH[owner] + "。")
        self.event("owner_behaviour", owner=owner, name=self.owners[owner - 1].name,
                   labels=list(obs.labels), capture_stamp=stamp_seconds(obs.position.header))
        return True

    def switch_command(self):
        command = get("command")
        if command not in ("开", "关"):
            raise ValueError("开关指令无效")
        switch = self.config["switches"][self.room]["on" if command == "开" else "off"]
        put("switch_pose", pose(switch["navigation"]))
        put("switch_arm", switch["arm"])
        put("speech", "好的，您的要求是" + command + "电器。我现在前往" + command + "的标记位置。")
        self.event("switch_requested", command=command)

    def repeat_request(self):
        heard = get("heard")
        if not isinstance(heard, str):
            raise ValueError("帮助需求为空")
        text = heard.strip()
        if not text or text == "(empty)":
            raise ValueError("帮助需求为空")
        put("speech", "我复述一下您的要求：" + text)
        self.event("help_request", original=text, repeated=text)

    def complete(self, owner):
        self.completed.add(owner)
        self.event("interaction_completed", owner=owner)

    def litter(self):
        scene = get("litter_scene")
        rows = [o for o in scene.observations if "floor_litter" in o.labels]
        room = self.config["room_names"][self.room]
        if rows:
            put("speech", "我在" + room + "的地面上发现了" + str(len(rows)) + "处疑似垃圾。")
        else:
            put("speech", "本次检查未在" + room + "的地面图像中发现垃圾。")
        self.event("litter_observed", count=len(rows), log_json=scene.log_json)

    def litter_announced(self):
        scene = get("litter_scene")
        rows = [o for o in scene.observations if "floor_litter" in o.labels]
        if rows:
            self.litter_rooms.add(self.room)
        self.event("litter_announced", count=len(rows), log_json=scene.log_json)

    def summary(self):
        put("speech", "巡游结束。已完成" + str(len(self.completed))
            + "位主人的交互，在" + str(len(self.litter_rooms))
            + "个房间发现垃圾。本次未执行垃圾抓取和放置，现在从出口离场。")
        self.event("patrol_summary", completed=sorted(self.completed),
                   visited=sorted(self.visited), litter_rooms=sorted(self.litter_rooms),
                   scope_complete=(len(self.completed) == 3 and len(self.visited) == 4
                                   and len(self.litter_rooms) >= 3))


def pose(values):
    from geometry_msgs.msg import PoseStamped
    output = PoseStamped()
    output.header.frame_id = "map"
    output.pose.position.x, output.pose.position.y = map(float, values[:2])
    output.pose.orientation.z = math.sin(float(values[2]) / 2)
    output.pose.orientation.w = math.cos(float(values[2]) / 2)
    return output


class StateNode(py_trees.behaviour.Behaviour):
    def __init__(self, name, callback):
        super().__init__(name)
        self.callback = callback

    def update(self):
        try:
            result = self.callback()
            return S.FAILURE if result is False else S.SUCCESS
        except (ValueError, KeyError, TypeError, AttributeError, OSError, RuntimeError, ImportError) as exc:
            self.feedback_message = str(exc)
            self.logger.error(self.feedback_message)
            return S.FAILURE


class PatrolBudget(py_trees.decorators.Decorator):
    """Reserve exit time from match start, including entry and enrollment."""
    def __init__(self, child, context):
        super().__init__(name="Patrol budget with exit reserve", child=child)
        self.context = context

    def initialise(self):
        self.started = self.context.clock()

    def tick(self):
        now = self.context.clock()
        config = self.context.config
        expired = now >= self.context.started_at + config["mission_timeout_seconds"] - config["exit_reserve_seconds"]
        if self.status == S.RUNNING:
            expired = expired or now >= self.started + config["patrol_timeout_seconds"]
        if expired:
            self.decorated.stop(S.INVALID)
            self.stop(S.FAILURE)
            self.feedback_message = "Patrol budget reached; retract then leave"
            yield self
            return
        yield from super().tick()

    def update(self):
        return self.decorated.status
