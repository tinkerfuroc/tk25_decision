"""Restaurant pure-nav test: pan-tilt sweep -> scan for waving -> approach the
closest waver. Composes the test_scan sweep + BtNode_Approach, publishes JSON
status on /restaurant_nav_test/status, and is wrapped in OneShot (runs once,
then latches/idles until the process is stopped).

Run:                ros2 run behavior_tree restaurant-nav-test
Mock smoke:         BT_MOCK_MODE=true ros2 run behavior_tree restaurant-nav-test
Stop with Ctrl+C (or the dashboard's Stop, which SIGTERMs -> run_tree unwinds).
"""
from __future__ import annotations

import json

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped  # noqa: F401  (type used via blackboard)
from std_msgs.msg import String

from behavior_tree.runtime import run_tree
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach
from .nav_test_select import SWEEP_PANS, TILT_DEG, nearest_index
from .test_scan import scan_once, KEY_CUSTOMER_CENTROIDS

STATUS_TOPIC = "/restaurant_nav_test/status"
KEY_APPROACH_TARGET = "restaurant_nav_test_target"
TARGET_FRAME = "map"
ROBOT_FRAME = "base_link"


class BtNode_SelectClosestWaver(py_trees.behaviour.Behaviour):
    """Pick the centroid nearest the robot (TF map->base_link; falls back to
    index 0 if TF unavailable) from KEY_CUSTOMER_CENTROIDS, write it as the
    approach target. FAILURE when no centroids were accumulated."""

    def __init__(self, name, bb_centroids_key=KEY_CUSTOMER_CENTROIDS,
                 bb_target_key=KEY_APPROACH_TARGET):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="centroids", access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_centroids_key))
        self.blackboard.register_key(
            key="target", access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_key))
        self._tf_buffer = None
        self._tf_listener = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(f"'{self.name}': 'node' missing in setup kwargs") from e
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

    def _robot_xy(self):
        try:
            t = self._tf_buffer.lookup_transform(
                TARGET_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:  # noqa: BLE001 — TF not ready -> caller falls back
            return None

    def update(self):
        if not self.blackboard.exists("centroids") or not self.blackboard.centroids:
            self.feedback_message = "no waving centroids accumulated"
            return py_trees.common.Status.FAILURE
        centroids = list(self.blackboard.centroids)
        xy = [(c.point.x, c.point.y) for c in centroids]
        idx = nearest_index(xy, self._robot_xy())
        self.blackboard.target = centroids[idx]
        self.feedback_message = f"selected waver {idx} of {len(centroids)}"
        return py_trees.common.Status.SUCCESS


class BtNode_PublishNavTestStatus(py_trees.behaviour.Behaviour):
    """Publish a JSON status snapshot on STATUS_TOPIC for the web dashboard."""

    def __init__(self, name, phase, result=None,
                 bb_centroids_key=KEY_CUSTOMER_CENTROIDS,
                 bb_target_key=KEY_APPROACH_TARGET):
        super().__init__(name)
        self.phase = phase
        self.result = result
        self._pub = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="centroids", access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_centroids_key))
        self.blackboard.register_key(
            key="target", access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_key))

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(f"'{self.name}': 'node' missing in setup kwargs") from e
        self._pub = self.node.create_publisher(String, STATUS_TOPIC, 10)

    def update(self):
        centroids = (list(self.blackboard.centroids)
                     if self.blackboard.exists("centroids") else [])
        target = self.blackboard.target if self.blackboard.exists("target") else None
        doc = {
            "phase": self.phase,
            "result": self.result,
            "waver_count": len(centroids),
            "wavers": [{"x": c.point.x, "y": c.point.y} for c in centroids],
            "target": ({"x": target.point.x, "y": target.point.y}
                       if target is not None else None),
        }
        if self._pub is not None:
            msg = String()
            msg.data = json.dumps(doc)
            self._pub.publish(msg)
        self.feedback_message = f"status: {self.phase} result={self.result}"
        return py_trees.common.Status.SUCCESS


def _status(phase, result=None):
    return BtNode_PublishNavTestStatus(
        name=f"status: {phase}", phase=phase, result=result)


def build_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Restaurant Nav Test", memory=True)
    root.add_child(BtNode_Announce(
        name="announce start", bb_source=None,
        message="Starting restaurant nav test."))
    root.add_child(_status("scanning"))
    sweep = py_trees.composites.Sequence(name="scan sweep", memory=True)
    for pan in SWEEP_PANS:
        sweep.add_child(scan_once(pan, tilt=TILT_DEG, target_frame=TARGET_FRAME))
    root.add_child(sweep)
    root.add_child(_status("scanned"))
    outcome = py_trees.composites.Selector(name="approach outcome", memory=False)
    success = py_trees.composites.Sequence(name="select+approach", memory=True)
    success.add_child(BtNode_SelectClosestWaver(name="select closest waver"))
    success.add_child(_status("approaching"))
    success.add_child(BtNode_Approach(
        name="approach closest waver", bb_target_key=KEY_APPROACH_TARGET))
    success.add_child(_status("done", result="success"))
    outcome.add_child(success)
    outcome.add_child(_status("done", result="failed"))
    root.add_child(outcome)
    # OneShot: run the scan->approach once, then latch + idle (no auto re-run
    # under tick_tock). The dashboard re-runs by respawning the process.
    return py_trees.decorators.OneShot(
        name="nav test (one-shot)", child=root,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION)


def main():
    run_tree(build_tree, period_ms=500.0, title="Restaurant Nav Test",
             node_name="restaurant_nav_test")


if __name__ == "__main__":
    main()
