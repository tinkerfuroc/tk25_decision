"""Task-local constants and configuration, following HRI/PickAndPlace."""

from dataclasses import dataclass
import json
import math
from pathlib import Path

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header


KEY_POSE_SCAN = "hg_pose_scan"
KEY_POSE_DESTINATION = "hg_pose_destination"
KEY_ARM_NAVIGATING = "hg_arm_navigating"
KEY_ARM_TABLE = "hg_arm_table"
KEY_SCAN_RESULT = "hg_scan_result"
KEY_GRASP_SCAN_RESULT = "hg_grasp_scan_result"
KEY_GRASP_VISION_RES = "hg_grasp_vision_result"
KEY_TARGET_LABEL = "hg_target_label"
KEY_GRASP_LABEL = "hg_grasp_label"
KEY_ANNOUNCEMENT = "hg_announcement"


def pose_from_xyyaw(x, y, yaw_deg):
    """CLI poses use metres and degrees, in map."""
    yaw = math.radians(float(yaw_deg))
    return {
        "point": {"x": float(x), "y": float(y), "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0,
                        "z": math.sin(yaw / 2), "w": math.cos(yaw / 2)},
    }


def _pose_reader(value, name):
    if value is None:
        raise ValueError(f"{name} is unset; configure it or pass its CLI pose option")
    point, quat = value["point"], value["orientation"]
    xyz = [float(point["x"]), float(point["y"]), float(point.get("z", 0.0))]
    q = [float(quat[k]) for k in ("x", "y", "z", "w")]
    if not all(math.isfinite(v) for v in xyz + q):
        raise ValueError(f"{name} must contain finite coordinates")
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-6:
        raise ValueError(f"{name} quaternion must be nonzero")
    q = [v / norm for v in q]
    return PoseStamped(
        header=Header(frame_id="map"),
        pose=Pose(position=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
                  orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])),
    )


@dataclass
class TaskConfig:
    map_name: str
    pose_scan: PoseStamped
    pose_destination: PoseStamped
    arm_pos_navigating: list
    arm_pos_table: list
    label_map: dict
    prompt: str
    scan_camera: str
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


def load_config(path=None, *, scan_pose=None, destination_pose=None, prompt=None):
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
    if scan_pose is not None:
        data["pose_scan"] = pose_from_xyyaw(*scan_pose)
    if destination_pose is not None:
        data["pose_destination"] = pose_from_xyyaw(*destination_pose)
    if prompt is not None:
        data["prompt"] = prompt
    if not isinstance(data["map_name"], str) or not data["map_name"].strip():
        raise ValueError("map_name must be a nonempty map name")
    named_poses = None
    for key in ("pose_scan", "pose_destination"):
        value = data[key]
        if isinstance(value, str):
            if named_poses is None:
                from map_studio.poses import load_poses
                named_poses = load_poses(data["map_name"])
            if value not in named_poses:
                raise ValueError(
                    f"{key}: pose {value!r} is missing from map {data['map_name']!r}")
            data[key] = named_poses[value]
        else:
            data[key] = _pose_reader(value, key)
    for key in ("arm_pos_navigating", "arm_pos_table"):
        values = data[key]
        if len(values) != 7 or not all(math.isfinite(float(v)) for v in values):
            raise ValueError(f"{key} requires seven finite joint angles in degrees")
        data[key] = [math.radians(float(v)) for v in values]
    for key in ("nav_retry_limit", "scan_retry_limit", "arm_retry_limit",
                "grasp_retry_limit", "announce_retry_limit"):
        if type(data[key]) is not int or data[key] < 1:
            raise ValueError(f"{key} must be a positive integer")
    if data["scan_camera"] not in ("orbbec", "realsense"):
        raise ValueError("scan_camera must be orbbec or realsense")
    if not isinstance(data["prompt"], str):
        raise ValueError("prompt must be a string")
    if not isinstance(data["label_map"], dict):
        raise ValueError("label_map must be a dictionary")
    for label, translation in data["label_map"].items():
        if (not isinstance(label, str) or not label.strip()
                or not isinstance(translation, str) or not translation.strip()):
            raise ValueError("label_map keys and values must be nonempty strings")
    # YOLO-World has no fixed checkpoint vocabulary. An empty task prompt
    # means all configured dictionary labels.
    if not data["prompt"].strip():
        data["prompt"] = " . ".join(data["label_map"])
        if not data["prompt"]:
            raise ValueError("YOLO-World requires a prompt or a nonempty label_map")
    for key in ("detection_service", "announce_service", "navigation_action",
                "arm_action", "grasp_action"):
        if not isinstance(data[key], str) or not data[key].strip():
            raise ValueError(f"{key} must be a nonempty endpoint")
    return TaskConfig(**data)

