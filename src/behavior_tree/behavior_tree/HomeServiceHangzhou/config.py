"""Configuration and strict placeholder validation; no ROS side effects."""
import json
import math
import os
from pathlib import Path

PAN_ANGLES = [-90, -30, 30, 90]
OWNER_NAMES = ("杰克", "安东", "李华")
REST = {"lying_resting_or_sleeping", "sitting_on_bed", "sitting_on_sofa", "sitting_on_chair"}
FALL = {"lying_on_floor_possible_fall"}
WAVE = {"standing_waving", "sitting_waving"}
EXPECTED = {1: REST, 2: FALL, 3: WAVE}
BEHAVIOUR_ZH = {1: "躺卧或坐着休息", 2: "倒在地上，疑似摔倒", 3: "挥手示意"}
RETRY_DEFAULTS = {"navigation_attempts": 5, "approach_attempts": 3,
                  "arm_attempts": 3, "head_attempts": 3,
                  "dialogue_attempts": 3, "feature_attempts": 4,
                  "perception_attempts": 2, "retry_delay_seconds": 1.0,
                  "cancel_timeout_seconds": 10.0, "arm_timeout_seconds": 120.0}


def load_config(path=None):
    path = path or os.environ.get("HANGZHOU_CONFIG") or Path(__file__).with_name("settings.json")
    with open(path, encoding="utf-8") as stream:
        return {**RETRY_DEFAULTS, **json.load(stream)}


def room_order(config):
    rooms = list(config["rooms"])
    index = rooms.index(config["current_room"])
    return rooms[index:] + rooms[:index]


def validate(config):
    """All fixed poses must be deliberately calibrated before any robot I/O."""
    errors = []
    for key, default in RETRY_DEFAULTS.items():
        value = config.get(key, default)
        if key.endswith("_attempts"):
            if type(value) is not int or not 1 <= value <= 10:
                errors.append(key + " must be an integer in [1, 10]")
        elif (type(value) not in (int, float) or not math.isfinite(value)
              or value < 0 or (key != "retry_delay_seconds" and value == 0)):
            errors.append(key + " must be finite and positive (retry delay may be zero)")
    rooms = config.get("rooms", [])
    if len(rooms) != 4 or len(set(rooms)) != 4:
        errors.append("rooms must contain four unique rooms")
    if config.get("current_room") not in rooms:
        errors.append("current_room must be one of rooms")
    if config.get("pan_angles_deg") != PAN_ANGLES:
        errors.append("pan_angles_deg must be [-90, -30, 30, 90]")
    if config.get("litter_depression_deg") != 30:
        errors.append("litter_depression_deg must be 30")

    def vector(value, size, name):
        if not isinstance(value, list) or len(value) != size or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in value
        ):
            errors.append(name + ": configure " + str(size) + " finite numbers")

    for key in ("entry_pose", "exit_pose"):
        vector(config.get(key), 3, key)
    for key in ("arm_stow", "arm_assist"):
        vector(config.get(key), 7, key)
    for room in rooms:
        vector(config.get("room_poses", {}).get(room), 3, "room_poses." + room)
        if not config.get("room_names", {}).get(room):
            errors.append("room_names." + room + " is missing")
        for command in ("on", "off"):
            switch = config.get("switches", {}).get(room, {}).get(command, {})
            vector(switch.get("navigation"), 3, "switches." + room + "." + command + ".navigation")
            vector(switch.get("arm"), 7, "switches." + room + "." + command + ".arm")
    if config.get("entry_pose") is not None and config.get("exit_pose") == config["entry_pose"]:
        errors.append("exit_pose must differ from entry_pose (different exit)")
    for key in ("settle_seconds", "patrol_timeout_seconds", "mission_timeout_seconds",
                "identity_vlm_timeout_seconds", "target_max_age_seconds", "exit_reserve_seconds",
                "listen_timeout_seconds", "service_timeout_seconds", "navigation_timeout_seconds"):
        value = config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            errors.append(key + " must be positive and finite")
    if type(config.get("max_passes")) is not int or config["max_passes"] < 1:
        errors.append("max_passes must be a positive integer")
    if config.get("identity_qwen_backend") not in ("dashscope", "openrouter"):
        errors.append("identity_qwen_backend must be dashscope or openrouter")
    if not isinstance(config.get("identity_model", ""), str):
        errors.append("identity_model must be a string")
    reserve = config.get("exit_reserve_seconds")
    total = config.get("mission_timeout_seconds")
    if type(reserve) in (int, float) and type(total) in (int, float) and reserve >= total:
        errors.append("exit_reserve_seconds must be less than mission_timeout_seconds")
    level = config.get("level_tilt_deg")
    if type(level) not in (int, float) or not math.isfinite(level) or not 0 <= level <= 90:
        errors.append("level_tilt_deg must be within [0, 90]")
    if (isinstance(config.get("patrol_timeout_seconds"), (int, float))
            and isinstance(config.get("mission_timeout_seconds"), (int, float))
            and config["patrol_timeout_seconds"] >= config["mission_timeout_seconds"]):
        errors.append("patrol timeout must leave time for retract and exit")
    for key in ("announce", "listen", "phrase", "feature_extraction",
                "behaviour_detection", "litter_detection", "navigate", "approach", "arm"):
        if not isinstance(config.get("endpoints", {}).get(key), str) or not config["endpoints"][key].strip():
            errors.append("endpoints." + key + " is missing")
    if errors:
        raise ValueError("\n".join(errors))
