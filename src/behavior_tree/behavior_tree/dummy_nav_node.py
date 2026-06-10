# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# Dummy navigation node
# =====================
#
# A standalone ``rclpy`` stub that subscribes to the follow-target topic
# (``PointStamped``, default ``/follow_target``) and logs where it "would
# navigate", throttled to ~1 Hz. Pure stub: no motion, no nav action.
#
# DEPRECATED (2026-06-10): the ``/follow_target`` topic this node subscribes to
# has NO publisher since the FollowPerson rewire — ``BtNode_PublishFollowGoal``
# (its only publisher) was removed, and the follow pipeline now drives
# navigation through the ``Follow`` action on ``follow_server`` (tk26_navigation
# ``following`` package), which consumes the tracker's ``/target_points`` topic
# directly. This stub is retained for standalone experiments only and is NOT
# part of the follow pipeline; running it against the live tree logs nothing
# because no one publishes ``/follow_target``.
#

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class DummyNavNode(Node):
    """Subscribes the follow target and logs it; performs no motion."""

    def __init__(self):
        """Create the subscriber on the configured follow-target topic."""
        super().__init__("dummy_nav_node")

        self.declare_parameter("follow_target_topic", "/follow_target")
        topic = self.get_parameter("follow_target_topic").value

        self._subscription = self.create_subscription(
            PointStamped,
            topic,
            self._on_target,
            10,
        )
        self.get_logger().info(f"[dummy_nav] subscribed to '{topic}' (stub; no motion)")

    def _on_target(self, msg: PointStamped):
        """Log the latest follow target, throttled to ~1 Hz."""
        p = msg.point
        frame = msg.header.frame_id
        self.get_logger().info(
            f"[dummy_nav] would navigate toward "
            f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) in frame '{frame}'",
            throttle_duration_sec=1.0,
        )


def main(args=None):
    """Spin the dummy nav node until interrupted."""
    rclpy.init(args=args)
    node = DummyNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
