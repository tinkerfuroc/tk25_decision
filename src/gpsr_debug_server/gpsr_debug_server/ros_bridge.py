"""Optional ROS bridge for the developer GPSR debugger.

Trace payloads use ``std_msgs/String`` so the debugger can be used before a
dedicated message package is installed. The payload itself is versioned JSON;
future deployments can add a typed control action without changing the web
protocol.
"""
from __future__ import annotations

import json
import threading
from typing import Any, Callable


class RosBridge:
    def __init__(self, on_event: Callable[[dict[str, Any]], None], on_warning: Callable[[str], None] | None = None):
        self.on_event = on_event
        self.on_warning = on_warning or (lambda _message: None)
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._context = None
        self._node = None
        self._executor = None
        self._command_publisher = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="gpsr-debug-ros", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._executor is not None:
            try:
                self._executor.shutdown(timeout_sec=2.0)
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=3.0)

    def send_command(self, command: dict[str, Any]) -> dict[str, Any]:
        if self._command_publisher is None:
            return {"status": "unavailable", "message": "ROS debug command publisher is not ready"}
        from std_msgs.msg import String
        message = String()
        message.data = json.dumps(command, ensure_ascii=False, separators=(",", ":"))
        self._command_publisher.publish(message)
        return {"status": "queued", "request_id": command.get("request_id")}

    def _run(self) -> None:  # pragma: no cover - requires a ROS host
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import String
        except ImportError as exc:
            self.on_warning(f"ROS debugger bridge unavailable: {exc}")
            return

        bridge = self

        class DebugNode(Node):
            def __init__(self) -> None:
                super().__init__("gpsr_debug_server", context=bridge._context)
                qos = QoSProfile(
                    history=HistoryPolicy.KEEP_LAST,
                    depth=256,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                )
                self.create_subscription(String, "/gpsr/debug/events", self._event, qos)
                self.create_subscription(String, "/tinker/gpsr/events", self._event, qos)
                bridge._command_publisher = self.create_publisher(String, "/gpsr/debug/commands", qos)

            def _event(self, message: String) -> None:
                try:
                    value = json.loads(message.data)
                    if isinstance(value, dict):
                        bridge.on_event(value)
                except Exception as exc:
                    bridge.on_warning(f"invalid GPSR event: {exc}")

        try:
            self._context = Context()
            rclpy.init(context=self._context)
            self._node = DebugNode()
            self._executor = MultiThreadedExecutor(num_threads=2, context=self._context)
            self._executor.add_node(self._node)
            self._executor.spin()
        except Exception as exc:
            self.on_warning(f"ROS debugger bridge stopped: {type(exc).__name__}: {exc}")
        finally:
            if self._node is not None:
                try:
                    self._node.destroy_node()
                except Exception:
                    pass
            if self._context is not None:
                try:
                    rclpy.shutdown(context=self._context)
                except Exception:
                    pass
