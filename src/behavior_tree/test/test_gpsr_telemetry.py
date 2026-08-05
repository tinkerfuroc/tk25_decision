from __future__ import annotations

import json
import sys
import types

from behavior_tree.GPSR.telemetry import GpsrTelemetry


def test_telemetry_is_run_scoped_causal_ndjson(tmp_path) -> None:
    telemetry = GpsrTelemetry(tmp_path, trajectory_id="trajectory-test", enabled=True)
    first = telemetry.emit("task.command_received", {"command": "go to kitchen"}, task_id="task-1")
    second = telemetry.emit("planner.request", {"messages": [{"role": "user", "content": "go"}]}, task_id="task-1")
    telemetry.close()
    assert first and second
    assert second["parent_event_id"] == first["event_id"]
    records = [json.loads(line) for line in (tmp_path / "debug" / "trajectory-test" / "events.jsonl").read_text().splitlines()]
    assert records[0]["event_type"] == "run.started"
    assert records[2]["causation_ids"]
    assert all(item["trajectory_id"] == "trajectory-test" for item in records)


def test_ros_attach_replays_startup_events_and_batch_completion_is_terminal(tmp_path, monkeypatch) -> None:
    class String:
        def __init__(self):
            self.data = ""

    class Policy:
        KEEP_LAST = RELIABLE = VOLATILE = object()

    class QoSProfile:
        def __init__(self, **_kwargs):
            pass

    qos_module = types.ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = Policy
    qos_module.HistoryPolicy = Policy
    qos_module.QoSProfile = QoSProfile
    qos_module.ReliabilityPolicy = Policy
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")
    std_msgs_msg_module.String = String
    monkeypatch.setitem(sys.modules, "rclpy.qos", qos_module)
    monkeypatch.setitem(sys.modules, "std_msgs", std_msgs_module)
    monkeypatch.setitem(sys.modules, "std_msgs.msg", std_msgs_msg_module)

    published = []

    class Publisher:
        def publish(self, message):
            published.append(json.loads(message.data))

    class Node:
        def create_publisher(self, *_args):
            return Publisher()

    telemetry = GpsrTelemetry(tmp_path, trajectory_id="trajectory-ros", enabled=True)
    telemetry.emit("run.configured", {"expected_task_count": 1})
    telemetry.attach_ros(Node())
    assert [event["event_type"] for event in published] == ["run.started", "run.configured"]

    class Status:
        def __init__(self, name):
            self.name = name

    class Behaviour:
        def __init__(self, name, status, children=()):
            self.name = name
            self.status = Status(status)
            self.feedback_message = ""
            self.children = list(children)

    batch = Behaviour("batch_command_flow", "SUCCESS")
    root = Behaviour("GPSR orchestrator", "RUNNING", [batch, Behaviour("idle", "RUNNING")])
    tree = types.SimpleNamespace(root=root, count=1)
    handler = telemetry.post_tick_handler()
    handler(tree)
    handler(tree)
    telemetry.close()

    event_types = [event["event_type"] for event in published]
    assert event_types.count("run.finished") == 1
    assert event_types.count("tree.tick_observed") == 1
    terminal = next(event for event in published if event["event_type"] == "run.finished")
    assert terminal["payload"]["status"] == "succeeded"
