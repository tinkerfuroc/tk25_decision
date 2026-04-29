"""HRI-specific BT nodes.

`BtNode_BestOfNSeatAndMatch` performs a 3-pose pan-tilt sweep, firing
`seat_recommend_bbox_service` and `feature_matching_service` concurrently
at each pose, and commits the best-scoring pose's responses to the
blackboard. Compensates for chassis park positions that cannot see the
entire seating area in one frame.
"""

from __future__ import annotations

import math
import time
from typing import Any

import py_trees as pytree
from py_trees.common import Status

from behavior_tree.config import is_node_mocked
from behavior_tree.messages import (
    BoundingBox,
    FeatureMatching,
    PanTiltCommand,
    PanTiltState,
    SeatRecommendBbox,
)
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header


_STATE_INIT = "init"
_STATE_MOVE = "move"
_STATE_SETTLE = "settle"
_STATE_CALL = "call"
_STATE_AWAIT = "await"
_STATE_SCORE = "score"
_STATE_RESTORE_MOVE = "restore_move"
_STATE_RESTORE_SETTLE = "restore_settle"
_STATE_DONE = "done"


class BtNode_BestOfNSeatAndMatch(pytree.behaviour.Behaviour):
    """Sweep N pan-tilt poses, score each by seat-rec + feature-match results,
    commit the best pose's outputs to the blackboard.
    """

    def __init__(
        self,
        name: str,
        bb_persons_key: str,
        bb_centroids_key: str,
        bb_recommendation_key: str,
        bb_bbox_key: str,
        bb_point_key: str,
        pan_angles_deg: list[float] | None = None,
        tilt_deg: float = 20.0,
        settle_count: int = 4,
        settle_eps_deg: float = 2.0,
        settle_max_ticks: int = 12,
        service_call_timeout_sec: float = 12.0,
        seat_service: str = "seat_recommend_bbox_service",
        match_service: str = "feature_matching_service",
        command_topic: str = "/pan_tilt_controller/cmd",
        state_topic: str = "/pan_tilt_state",
        target_frame: str = "base_link",
        use_orbbec: bool = True,
        max_distance: float = 2.0,
    ):
        super().__init__(name=name)
        self.bb_persons_key = bb_persons_key
        self.bb_centroids_key = bb_centroids_key
        self.bb_recommendation_key = bb_recommendation_key
        self.bb_bbox_key = bb_bbox_key
        self.bb_point_key = bb_point_key
        self.pan_angles_deg = list(pan_angles_deg or [-30.0, 0.0, 30.0])
        self.tilt_deg = float(tilt_deg)
        self.settle_count = int(settle_count)
        self.settle_eps_rad = math.radians(float(settle_eps_deg))
        self.settle_max_ticks = int(settle_max_ticks)
        self.service_call_timeout_sec = float(service_call_timeout_sec)
        self.seat_service = seat_service
        self.match_service = match_service
        self.command_topic = command_topic
        self.state_topic = state_topic
        self.target_frame = target_frame
        self.camera = "orbbec" if use_orbbec else "realsense"
        self.max_distance = float(max_distance)

        self.mock_mode = is_node_mocked(self.__class__.__name__)

        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="persons",
            access=pytree.common.Access.READ,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_persons_key),
        )
        self.blackboard.register_key(
            key="centroids",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_centroids_key),
        )
        self.blackboard.register_key(
            key="recommendation",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_recommendation_key),
        )
        self.blackboard.register_key(
            key="bbox",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_bbox_key),
        )
        self.blackboard.register_key(
            key="point",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_point_key),
        )

        self.node = None
        self.cmd_pub = None
        self.state_sub = None
        self.seat_client = None
        self.match_client = None
        self._latest_state: Any = None

    def setup(self, **kwargs) -> None:
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(
                f"didn't find 'node' in setup's kwargs [{self.name}][{self.__class__.__name__}]"
            ) from e

        if self.mock_mode:
            print(f"MOCK MODE: Skipping ROS setup for {self.name}")
            return

        # Importing rclpy/QoS lazily so mock-mode tree compiles without ROS.
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.cmd_pub = self.node.create_publisher(PanTiltCommand, self.command_topic, 1)
        self.state_sub = self.node.create_subscription(
            PanTiltState, self.state_topic, self._state_cb, state_qos
        )
        self.seat_client = self.node.create_client(SeatRecommendBbox, self.seat_service)
        self.match_client = self.node.create_client(FeatureMatching, self.match_service)

    def _state_cb(self, msg: PanTiltState) -> None:
        self._latest_state = msg

    def initialise(self) -> None:
        self._state = _STATE_INIT
        self._pose_idx = 0
        self._best_score = -1
        self._best_seat: Any = None
        self._best_match: Any = None
        self._best_pose_idx = -1
        self._target_pan_rad = 0.0
        self._target_tilt_rad = 0.0
        self._settle_good_ticks = 0
        self._settle_total_ticks = 0
        self._seat_future = None
        self._match_future = None
        self._call_deadline_ns = 0
        self._mock_ticks = 0
        self._restore_attempted = False

    def update(self) -> Status:
        if self.mock_mode:
            return self._update_mock()

        if self._state == _STATE_INIT:
            self._state = _STATE_MOVE
            return Status.RUNNING

        if self._state == _STATE_MOVE:
            self._publish_pose(self.pan_angles_deg[self._pose_idx], self.tilt_deg)
            self._settle_good_ticks = 0
            self._settle_total_ticks = 0
            self._state = _STATE_SETTLE
            return Status.RUNNING

        if self._state == _STATE_SETTLE:
            self._settle_total_ticks += 1
            if self._is_settled():
                self._settle_good_ticks += 1
            else:
                self._settle_good_ticks = 0
            if self._settle_good_ticks >= self.settle_count:
                self._fire_service_calls()
                self._state = _STATE_AWAIT
                return Status.RUNNING
            if self._settle_total_ticks >= self.settle_max_ticks:
                # Bail: skip this pose's measurements; advance.
                self.feedback_message = (
                    f"Settle timed out at pose {self._pose_idx} "
                    f"(target_pan={self.pan_angles_deg[self._pose_idx]}°). Skipping."
                )
                self.node.get_logger().warn(self.feedback_message)
                self._seat_future = None
                self._match_future = None
                self._state = _STATE_SCORE
                return Status.RUNNING
            return Status.RUNNING

        if self._state == _STATE_AWAIT:
            seat_done = self._seat_future is None or self._seat_future.done()
            match_done = self._match_future is None or self._match_future.done()
            if seat_done and match_done:
                self._state = _STATE_SCORE
                return Status.RUNNING
            if time.monotonic_ns() > self._call_deadline_ns:
                self.feedback_message = (
                    f"Service call timeout at pose {self._pose_idx} "
                    f"(seat_done={seat_done}, match_done={match_done})."
                )
                self.node.get_logger().warn(self.feedback_message)
                self._state = _STATE_SCORE
                return Status.RUNNING
            return Status.RUNNING

        if self._state == _STATE_SCORE:
            seat_resp = (
                self._seat_future.result()
                if self._seat_future is not None and self._seat_future.done()
                else None
            )
            match_resp = (
                self._match_future.result()
                if self._match_future is not None and self._match_future.done()
                else None
            )
            score = 0
            if seat_resp is not None and seat_resp.status == 0:
                score += 1
            if match_resp is not None and match_resp.status == 0:
                score += len(match_resp.centroids)
            self.node.get_logger().info(
                f"Pose {self._pose_idx} pan={self.pan_angles_deg[self._pose_idx]}° score={score} "
                f"(seat_ok={seat_resp is not None and seat_resp.status == 0}, "
                f"n_centroids={len(match_resp.centroids) if match_resp is not None and match_resp.status == 0 else 0})"
            )
            if score > self._best_score:
                self._best_score = score
                self._best_seat = seat_resp
                self._best_match = match_resp
                self._best_pose_idx = self._pose_idx
            self._pose_idx += 1
            if self._pose_idx < len(self.pan_angles_deg):
                self._state = _STATE_MOVE
            else:
                self._state = _STATE_RESTORE_MOVE
            return Status.RUNNING

        if self._state == _STATE_RESTORE_MOVE:
            self._publish_pose(0.0, self.tilt_deg)
            self._settle_good_ticks = 0
            self._settle_total_ticks = 0
            self._restore_attempted = True
            self._state = _STATE_RESTORE_SETTLE
            return Status.RUNNING

        if self._state == _STATE_RESTORE_SETTLE:
            self._settle_total_ticks += 1
            if self._is_settled():
                self._settle_good_ticks += 1
            else:
                self._settle_good_ticks = 0
            if (
                self._settle_good_ticks >= self.settle_count
                or self._settle_total_ticks >= self.settle_max_ticks
            ):
                self._state = _STATE_DONE
                return self._commit_result()
            return Status.RUNNING

        if self._state == _STATE_DONE:
            return self._commit_result()

        # Defensive default
        self.feedback_message = f"Unknown state: {self._state}"
        return Status.FAILURE

    def _update_mock(self) -> Status:
        self._mock_ticks += 1
        if self._mock_ticks < 2:
            self.feedback_message = "MOCK: sweep advancing"
            return Status.RUNNING
        mock_point = PointStamped()
        mock_point.header = Header(frame_id=self.target_frame)
        mock_point.point = Point(x=1.0, y=0.5, z=0.6)
        self.blackboard.point = mock_point
        self.blackboard.bbox = BoundingBox(xmin=300, ymin=200, xmax=380, ymax=280)
        self.blackboard.recommendation = (
            "Dear guest, I recommend the seat on the left side of the table."
        )
        # Mirror BtNode_FeatureMatching mock: emit one centroid per registered
        # person (minus the newest who hasn't sat yet), spread laterally.
        try:
            persons = self.blackboard.persons or []
        except KeyError:
            persons = []
        n = max(len(persons) - 1, 0)
        centroids = []
        for i in range(n):
            mc = PointStamped()
            mc.header = Header(frame_id=self.target_frame)
            mc.point = Point(x=1.5, y=-0.5 + i * 1.0, z=1.3)
            centroids.append(mc)
        self.blackboard.centroids = centroids
        self.feedback_message = (
            f"MOCK: best-of-N committed (n_centroids={len(centroids)})"
        )
        return Status.SUCCESS

    def _publish_pose(self, pan_deg: float, tilt_deg: float) -> None:
        pan_rad = math.radians(pan_deg)
        tilt_rad = math.radians(tilt_deg)
        msg = PanTiltCommand()
        msg.header = Header(frame_id="")
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.mode = PanTiltCommand.ABSOLUTE
        msg.pan_rad = float(pan_rad)
        msg.tilt_rad = float(tilt_rad)
        msg.speed_raw = 0
        msg.accel_raw = 0
        self.cmd_pub.publish(msg)
        self._target_pan_rad = pan_rad
        self._target_tilt_rad = tilt_rad
        self.node.get_logger().info(
            f"Sweep move: pan={pan_deg:+.1f}° tilt={tilt_deg:+.1f}°"
        )

    def _is_settled(self) -> bool:
        if self._latest_state is None:
            return False
        if not getattr(self._latest_state, "feedback_ok", False):
            return False
        d_pan = abs(float(self._latest_state.pan_rad) - self._target_pan_rad)
        d_tilt = abs(float(self._latest_state.tilt_rad) - self._target_tilt_rad)
        return d_pan < self.settle_eps_rad and d_tilt < self.settle_eps_rad

    def _fire_service_calls(self) -> None:
        try:
            persons = list(self.blackboard.persons or [])
        except KeyError:
            persons = []
        # Exclude the newest registered person (current escortee not yet seated).
        seated_persons = persons[:-1] if len(persons) > 0 else []

        seat_req = SeatRecommendBbox.Request()
        seat_req.camera = self.camera
        seat_req.names = [p.name for p in seated_persons if p.name is not None]
        seat_req.features = [p.features for p in seated_persons if p.features is not None]
        seat_req.target_frame = self.target_frame
        if self.seat_client.service_is_ready():
            self._seat_future = self.seat_client.call_async(seat_req)
        else:
            self.node.get_logger().warn(f"{self.seat_service} not ready")
            self._seat_future = None

        match_req = FeatureMatching.Request()
        match_req.camera = self.camera
        match_req.features = [p.features for p in seated_persons]
        from sensor_msgs.msg import Image as ImageMsg
        match_req.comparison_images = [
            p.comparison_image if p.comparison_image is not None else ImageMsg()
            for p in seated_persons
        ]
        match_req.max_distance = self.max_distance
        match_req.target_frame = self.target_frame
        if len(seated_persons) > 0 and self.match_client.service_is_ready():
            self._match_future = self.match_client.call_async(match_req)
        else:
            # Match call is meaningless with zero seated guests; leave future
            # as None so SCORE skips it without timing out.
            self._match_future = None

        self._call_deadline_ns = time.monotonic_ns() + int(
            self.service_call_timeout_sec * 1e9
        )
        self.node.get_logger().info(
            f"Pose {self._pose_idx}: fired seat-rec (n_seated={len(seated_persons)}) "
            f"+ match (skipped={self._match_future is None})"
        )

    def _commit_result(self) -> Status:
        if self._best_score <= 0 or self._best_seat is None:
            self.feedback_message = (
                f"Best-of-N sweep produced no usable seat (best_score={self._best_score})."
            )
            return Status.FAILURE
        self.blackboard.recommendation = "Dear guest, " + self._best_seat.recommendation
        self.blackboard.bbox = self._best_seat.bbox
        self.blackboard.point = self._best_seat.centroid
        if self._best_match is not None and self._best_match.status == 0:
            self.blackboard.centroids = list(self._best_match.centroids)
        else:
            self.blackboard.centroids = []
        self.feedback_message = (
            f"Committed pose {self._best_pose_idx} "
            f"(pan={self.pan_angles_deg[self._best_pose_idx]:+.1f}°, score={self._best_score})"
        )
        self.node.get_logger().info(self.feedback_message)
        return Status.SUCCESS
