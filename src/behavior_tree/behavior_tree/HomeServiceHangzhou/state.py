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
from .config import BEHAVIOUR_ZH, EXPECTED, OWNER_NAMES, ROLE_PRIORITY, validate
from .speech import match_owner_name

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
            and all(math.isfinite(v) for v in (point.point.x, point.point.y, point.point.z)))


def extract_name(text):
    if not isinstance(text, str) or text.strip() in ("", "(empty)"):
        raise ValueError("姓名为空")
    name = re.sub(r"^(你好[，, ]*|我叫|我的名字是|我是)", "", str(text).strip())
    name = name.strip(" ，。！!？?、,\n\t")
    if not name or len(name) > 20 or any(c in name for c in "，。！？,!?\n"):
        raise ValueError("请仅说您的名字")
    return name


def extract_owner_name(text):
    """Normalize enrollment speech against the configured owner names."""
    return match_owner_name(text)


@dataclass
class Context:
    config: dict
    owners: list = field(default_factory=list)
    completed: set = field(default_factory=set)
    serviced_roles: set = field(default_factory=set)
    visited: set = field(default_factory=set)
    litter_rooms: set = field(default_factory=set)
    active: dict = field(default_factory=dict)
    room: str = ""
    arm_dirty: bool = False
    motion_actions: list = field(default_factory=list)
    motion_blocked: bool = False
    audio_actions: list = field(default_factory=list)
    audio_blocked: bool = False
    feature_actions: list = field(default_factory=list)
    feature_blocked: bool = False
    ready: bool = False
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
        self.serviced_roles.clear()
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
        self.event("mission_started", excluded=["litter_pickup", "litter_placement"])

    def remember(self, owner):
        from cv_bridge import CvBridge
        import cv2
        name = extract_owner_name(get("heard"))
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

    def match_owner_name(self):
        raw = get("heard")
        try:
            name = extract_owner_name(raw)
        except ValueError as exc:
            self.event("owner_name_match", transcript=raw, matched=None, error=str(exc))
            raise
        self.event("owner_name_match", transcript=raw, matched=name)
        put("heard_raw", raw)
        put("heard", name)

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

    def observe_behaviours(self):
        """Select behaviour targets without requiring visible faces or identity."""
        self.active.clear()
        scene = get("scene")
        for index, obs in enumerate(scene.observations):
            roles = [role for role in ROLE_PRIORITY if set(obs.labels) & EXPECTED[role]]
            # A served role is not reassigned to a lower-priority label: a person
            # still waving from a chair after service is not a new resting target.
            if roles and roles[0] not in self.serviced_roles:
                self.active.setdefault(roles[0], (index, obs))
        self.event("behaviour_targets", roles=list(self.active), log_json=scene.log_json)

    def select_behaviour(self, role):
        if role in self.serviced_roles or role not in self.active:
            return False
        index, obs = self.active[role]
        scene = get("scene")
        # Preserve the selected person's original ROI for pre-service identity.
        put("behaviour_identity_scene", SimpleNamespace(
            header=scene.header, rgb_image=scene.rgb_image,
            observations=[obs], log_json=scene.log_json))
        put("matched_owner", None)
        put("target", obs.position)
        put("target_valid", obs.position_valid)

        self.event("behaviour_selected", role=role, observation=index,
                   capture_stamp=stamp_seconds(scene.header))
        return True

    def prepare_facing_identity(self):
        """The feature service returns the foreground person's cropped image."""
        image = get("reference_image")
        if image is None or image.width <= 0 or image.height <= 0:
            raise ValueError("正面人物图像为空")
        put("scene", SimpleNamespace(header=image.header, rgb_image=image,
            observations=[SimpleNamespace(bbox=SimpleNamespace(
                x_offset=0, y_offset=0, width=image.width, height=image.height))],
            log_json="facing_person_crop"))

    def accept_identity(self, source):
        identities = get("identities")
        if len(identities) != 1 or type(identities[0]) is not int or not 1 <= identities[0] <= len(self.owners):
            self.event("identity_attempt_unresolved", source=source, identities=list(identities))
            return False
        owner = identities[0]
        put("matched_owner", owner)
        self.event("owner_identified", owner=owner,
                   name=self.owners[owner - 1].name, source=source)
        return True

    def prepare_observation_report(self, role):
        """Report location, matched identity and behaviour before approaching."""
        owner = get("matched_owner")
        name = (self.owners[owner - 1].name if owner is not None
                else "\u4e00\u4f4d\u8eab\u4efd\u5c1a\u672a\u786e\u8ba4\u7684\u4eba")
        room = self.config["room_names"][self.room]
        put("speech", f"\u5728{room}\u53d1\u73b0{name}\uff0c\u6b63\u5728{BEHAVIOUR_ZH[role]}\u3002")
        self.event("observation_report_prepared", owner=owner, role=role, name=name)

    def finish_behaviour(self, role):
        # Do not repeat a completed physical service just because recognition
        # was inconclusive. Unidentified service is never scored as a known owner.
        self.serviced_roles.add(role)
        owner = get("matched_owner")
        if owner is not None:
            self.completed.add(owner)
            self.event("interaction_completed", owner=owner, role=role)
        else:
            self.event("interaction_completed_unidentified", role=role)

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
        if command not in ("on", "off", "none"):
            raise ValueError("Switch intent is unresolved")
        if command == "none":
            put("speech", "\u597d\u7684\uff0c\u4fdd\u6301\u7535\u5668\u5f53\u524d\u72b6\u6001\uff0c\u4e0d\u64cd\u4f5c\u5f00\u5173\u3002")
        else:
            switch = self.config["switches"][self.room][command]
            put("switch_arm", switch["arm"])
            operation = "\u6253\u5f00" if command == "on" else "\u5173\u95ed"
            put("speech", f"\u597d\u7684\uff0c\u60a8\u7684\u8981\u6c42\u662f{operation}\u7535\u5668\u3002")
        self.event("switch_requested", command=command, transcript=get("heard"))

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
            + "\u4e2a\u623f\u95f4\u53d1\u73b0\u5783\u573e\u3002\u672c\u6b21\u672a\u6267\u884c\u5783\u573e\u6293\u53d6\u548c\u653e\u7f6e\uff0c\u4efb\u52a1\u7ed3\u675f\u3002")
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
