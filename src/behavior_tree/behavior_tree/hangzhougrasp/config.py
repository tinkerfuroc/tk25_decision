"""Task-local constants and configuration, following HRI/PickAndPlace."""

from dataclasses import dataclass
import json
import logging
import math
from pathlib import Path

from geometry_msgs.msg import PoseStamped

from .pose_index import PoseIndex
from .route import build_route


log = logging.getLogger(__name__)

KEY_ARM_NAVIGATING = "hg_arm_navigating"
KEY_ARM_TABLE = "hg_arm_table"
KEY_SCAN_RESULT = "hg_scan_result"
KEY_GRASP_SCAN_RESULT = "hg_grasp_scan_result"
KEY_GRASP_VISION_RES = "hg_grasp_vision_result"
KEY_TARGET_LABEL = "hg_target_label"
KEY_GRASP_LABEL = "hg_grasp_label"
KEY_ANNOUNCEMENT = "hg_announcement"
KEY_HOLDING = "hg_holding"


def pose_key(name: str) -> str:
    return "hg_pose_" + name


@dataclass
class TaskConfig:
    map_name: str
    enter_room: str
    rooms: list
    destination_name: str
    destination: PoseStamped
    route: list
    arm_pos_navigating: list
    arm_pos_table: list
    label_map: dict
    prompt: str
    detection_service: str
    announce_service: str
    navigation_action: str
    arm_action: str
    grasp_action: str
    nav_retry_limit: int
    scan_retry_limit: int
    arm_retry_limit: int
    grasp_retry_limit: int
    announce_retry_limit: int


def _report_unmatched(index, rooms, destination_name):
    """Fail on misnamed poses of a profile room; warn about other strays."""
    stray = [n for n in index.unmatched if n != destination_name]
    misnamed = sorted(n for n in stray if any(n.startswith(r + "_") for r in rooms))
    if misnamed:
        raise ValueError(
            f"pose names {misnamed} start with a profile room but do not match "
            f"<room>_<table|ground>_<n>_<m> (e.g. {rooms[0]}_table_0_0)")
    other = sorted(n for n in stray if n not in misnamed)
    if other:
        log.warning("Ignoring poses not named <room>_<table|ground>_<n>_<m>: %s",
                    other)


def load_config(path=None, *, enter_room=None, prompt=None, poses=None):
    """Load packaged defaults, overlay an optional JSON file and CLI values."""
    with Path(__file__).with_name("constants.json").open(encoding="utf-8") as stream:
        data = json.load(stream)
    if path is not None:
        with Path(path).expanduser().open(encoding="utf-8") as stream:
            overrides = json.load(stream)
        unknown = set(overrides) - set(data)
        if unknown:
            raise ValueError(f"Unknown configuration keys: {sorted(unknown)}")
        data.update(overrides)
    if enter_room is not None:
        data["enter_room"] = enter_room
    if prompt is not None:
        data["prompt"] = prompt
    if not isinstance(data["map_name"], str) or not data["map_name"].strip():
        raise ValueError("map_name must be a nonempty map name")
    profiles = data.pop("enter_profiles", None)
    if not isinstance(profiles, dict) or not profiles:
        raise ValueError("enter_profiles is missing or empty")
    room = data["enter_room"]
    if room not in profiles:
        raise ValueError(
            f"enter_room {room!r} is not a key of enter_profiles {sorted(profiles)}")
    profile = profiles[room]
    if not isinstance(profile, dict):
        raise ValueError(f"enter_profiles[{room!r}] must be an object with "
                         f"'rooms' and 'destination'")
    rooms = profile.get("rooms")
    if (not isinstance(rooms, list) or not rooms
            or not all(isinstance(r, str) and r for r in rooms)):
        raise ValueError(f"enter_profiles[{room!r}].rooms must be a nonempty list")
    duplicates = sorted({r for r in rooms if rooms.count(r) > 1})
    if duplicates:
        raise ValueError(
            f"enter_profiles[{room!r}].rooms has duplicate rooms {duplicates}")
    if room not in rooms:
        raise ValueError(
            f"enter_room {room!r} is not in its own rooms {rooms}")
    destination_name = profile.get("destination")
    if not isinstance(destination_name, str) or not destination_name.strip():
        raise ValueError(
            f"enter_profiles[{room!r}] needs a 'destination' pose name")
    if poses is None:
        from map_studio.poses import load_poses
        poses = load_poses(data["map_name"])
    if destination_name not in poses:
        raise ValueError(
            f"destination pose {destination_name!r} is missing from map "
            f"{data['map_name']!r}")
    data["rooms"] = list(rooms)
    data["destination_name"] = destination_name
    data["destination"] = poses[destination_name]
    index = PoseIndex(poses)
    data["route"] = build_route(index, data["rooms"])
    _report_unmatched(index, data["rooms"], destination_name)
    for key in ("arm_pos_navigating", "arm_pos_table"):
        values = data[key]
        if len(values) != 7 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f"{key} requires seven finite joint angles in degrees")
        data[key] = [math.radians(float(v)) for v in values]
    for key in ("nav_retry_limit", "scan_retry_limit", "arm_retry_limit",
                "grasp_retry_limit", "announce_retry_limit"):
        if type(data[key]) is not int or data[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if not isinstance(data["prompt"], str):
        raise ValueError("prompt must be a string")
    if not isinstance(data["label_map"], dict):
        raise ValueError("label_map must be a dictionary")
    for label, translation in data["label_map"].items():
        if (not isinstance(label, str) or not label.strip()
                or not isinstance(translation, str) or not translation.strip()):
            raise ValueError("label_map keys and values must be nonempty strings")
    # An empty task prompt means all configured dictionary labels.
    if not data["prompt"].strip():
        data["prompt"] = " . ".join(data["label_map"])
        if not data["prompt"]:
            raise ValueError("a prompt or a nonempty label_map is required")
    for key in ("detection_service", "announce_service", "navigation_action",
                "arm_action", "grasp_action"):
        if not isinstance(data[key], str) or not data[key].strip():
            raise ValueError(f"{key} must be a nonempty endpoint")
    return TaskConfig(**data)
