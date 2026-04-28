"""Live ROS-graph endpoint verifier for HRI / Restaurant / PickAndPlace tasks.

Compares the live ROS service / action / topic graph against a per-task spec
and prints an OK/MISS table. Always exits 0 — the operator reads the table.

Usage:
    ros2 run behavior_tree verify-task-endpoints --task hri
    ros2 run behavior_tree verify-task-endpoints --task all --json
"""

from __future__ import annotations

import argparse
import enum
import json
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.action import get_action_names_and_types
from rclpy.node import Node


class Status(str, enum.Enum):
    OK = "OK"
    MISS = "MISS"
    TYPE_MISMATCH = "TYPE_MISMATCH"


_STATUS_COLOR = {
    Status.OK: "\033[32m",
    Status.MISS: "\033[31m",
    Status.TYPE_MISMATCH: "\033[33m",
}
_RESET = "\033[0m"
_BOLD = "\033[1m"


TASK_SPECS: Dict[str, Dict[str, List]] = {
    "hri": {
        "services": [
            ("/door_detection_srv",          "tinker_vision_msgs_26/srv/DoorDetection"),
            ("/feature_extraction_service",  "tinker_vision_msgs_26/srv/FeatureExtraction"),
            ("/seat_recommend_bbox_service", "tinker_vision_msgs_26/srv/SeatRecommendBbox"),
            ("/arm_joint_service",           "tinker_arm_msgs/srv/ArmJointService"),
            ("/announce",                    "tinker_audio_msgs/srv/TextToSpeech"),
            ("/phrase_extraction_service",   "tinker_audio_msgs/srv/PhraseExtraction"),
        ],
        "actions": [
            ("/get_confirmation_action",     "tinker_audio_msgs/action/GetConfirmation"),
            ("/follow_head_action",          "tinker_vision_msgs_26/action/FollowHeadAction"),
            ("/navigate_to_pose",            "nav2_msgs/action/NavigateToPose"),
            ("/xarm_gripper/gripper_action", "control_msgs/action/GripperCommand"),
        ],
        "topics_subscriber": [],
        "topics_publisher": [],
    },
    "restaurant": {
        "services": [
            ("/object_detection_generalist", "tinker_vision_msgs_26/srv/ObjectDetectionGeneralist"),
            ("/object_detection",            "tinker_vision_msgs_26/srv/ObjectDetection"),
            ("/arm_joint_service",           "tinker_arm_msgs/srv/ArmJointService"),
            ("/announce",                    "tinker_audio_msgs/srv/TextToSpeech"),
            ("/phrase_extraction_service",   "tinker_audio_msgs/srv/PhraseExtraction"),
        ],
        "actions": [
            ("/navigate_to_pose",            "nav2_msgs/action/NavigateToPose"),
        ],
        "topics_subscriber": ["/pan_tilt_controller/cmd"],
        "topics_publisher": [],
    },
    "pickandplace": {
        "services": [
            ("/object_detection_yolo",       "tinker_vision_msgs_26/srv/ObjectDetection"),
            ("/get_point_cloud_service",     "tinker_vision_msgs_26/srv/GetPointCloud"),
            ("/door_detection_srv",          "tinker_vision_msgs_26/srv/DoorDetection"),
            ("/arm_joint_service",           "tinker_arm_msgs/srv/ArmJointService"),
            ("/start_drop",                  "tinker_arm_msgs/srv/Drop"),
            ("/announce",                    "tinker_audio_msgs/srv/TextToSpeech"),
        ],
        "actions": [
            ("/start_grasp",                 "tinker_arm_msgs/action/Grasp"),
            ("/place_action",                "tinker_arm_msgs/action/Place"),
            ("/grocery_categorize",          "tinker_vision_msgs_26/action/Categorize"),
            ("/navigate_to_pose",            "nav2_msgs/action/NavigateToPose"),
            ("/xarm_gripper/gripper_action", "control_msgs/action/GripperCommand"),
        ],
        "topics_subscriber": [],
        "topics_publisher": [],
    },
}


def _check_typed(name: str, expected: str, name_to_types: Dict[str, List[str]]) -> Tuple[Status, Optional[str]]:
    actual = name_to_types.get(name, [])
    if not actual:
        return Status.MISS, None
    if expected in actual:
        return Status.OK, None
    return Status.TYPE_MISMATCH, ",".join(actual)


def verify_task(
    node: Node,
    task: str,
    services: Dict[str, List[str]],
    actions: Dict[str, List[str]],
) -> Dict[str, List[Dict[str, str]]]:
    spec = TASK_SPECS[task]
    rows: Dict[str, List[Dict[str, str]]] = {"services": [], "actions": [], "topics": []}

    for name, expected in spec["services"]:
        status, detail = _check_typed(name, expected, services)
        rows["services"].append({"name": name, "expected": expected, "status": status.value, "detail": detail or ""})

    for name, expected in spec["actions"]:
        status, detail = _check_typed(name, expected, actions)
        rows["actions"].append({"name": name, "expected": expected, "status": status.value, "detail": detail or ""})

    for name in spec.get("topics_subscriber", []):
        ok = bool(node.get_subscriptions_info_by_topic(name))
        rows["topics"].append({"name": name, "expected": "subscriber", "status": (Status.OK if ok else Status.MISS).value, "detail": ""})
    for name in spec.get("topics_publisher", []):
        ok = bool(node.get_publishers_info_by_topic(name))
        rows["topics"].append({"name": name, "expected": "publisher", "status": (Status.OK if ok else Status.MISS).value, "detail": ""})

    return rows


def _colorize(status_str: str) -> str:
    color = _STATUS_COLOR.get(Status(status_str), "")
    return f"{color}{status_str}{_RESET}" if color else status_str


def print_table(task: str, rows: Dict[str, List[Dict[str, str]]]) -> None:
    print(f"\n{_BOLD}=== {task} ==={_RESET}")
    name_w, type_w = 38, 50
    n_total = n_ok = 0
    for kind in ("services", "actions", "topics"):
        if not rows[kind]:
            continue
        print(f"\n  {_BOLD}{kind}{_RESET}")
        for row in rows[kind]:
            n_total += 1
            if row["status"] == Status.OK.value:
                n_ok += 1
            tail = f"  ({row['detail']})" if row["detail"] else ""
            print(f"    {row['name']:<{name_w}} {row['expected']:<{type_w}} {_colorize(row['status'])}{tail}")
    print(f"\n  Summary: {n_ok}/{n_total} OK")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Verify ROS graph endpoints for a behavior-tree task.")
    parser.add_argument("--task", choices=list(TASK_SPECS.keys()) + ["all"], default="all")
    parser.add_argument("--timeout", type=float, default=2.0,
                        help="Seconds to wait before querying graph (lets DDS discovery propagate)")
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of table")
    args = parser.parse_args(argv)

    rclpy.init()
    node = rclpy.create_node("task_endpoint_verifier")
    time.sleep(args.timeout)

    services = dict(node.get_service_names_and_types())
    actions = dict(get_action_names_and_types(node))

    tasks = list(TASK_SPECS.keys()) if args.task == "all" else [args.task]
    all_rows = {t: verify_task(node, t, services, actions) for t in tasks}

    if args.json:
        print(json.dumps(all_rows, indent=2))
    else:
        for t in tasks:
            print_table(t, all_rows[t])

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
