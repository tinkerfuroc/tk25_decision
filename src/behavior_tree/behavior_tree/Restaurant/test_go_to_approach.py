"""Layer-3 operator test for ``BtNode_Approach`` against ``go_to_approach``.

Detects a single person via ``/object_detection_generalist``, stashes
the centroid as a ``PointStamped(map)`` on the blackboard, then runs
``BtNode_Approach`` against that target. Announces "approach successful"
on action status=0, "approach failed" otherwise.

Run:

    ros2 run behavior_tree test-go-to-approach

Mock smoke (no robot, no nav, no vision):

    BT_MOCK_MODE=true ros2 run behavior_tree test-go-to-approach

Stop with Ctrl+C.
"""

import sys

import py_trees as pytree
import py_trees_ros
import rclpy
from geometry_msgs.msg import PointStamped

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach
from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt
from behavior_tree.messages import ObjectDetectionGeneralist


# Module-local blackboard key — kept distinct from Restaurant /
# HRI keys so a stale entry does not bleed across tests.
KEY_APPROACH_TARGET = "test_go_to_approach_target"


class BtNode_DetectAndStashPersonInMap(ServiceHandler):
    """Single-shot person detection + blackboard stash in map frame.

    Calls ``/object_detection_generalist`` with ``prompt="person"``,
    ``target_frame="map"``, ``sort_closest=True``, takes the first
    object, and writes ``PointStamped(map, point=obj.centroid)`` to
    ``bb_target``. Returns FAILURE on empty / status != 0; wrap in
    ``Retry`` to ride out transient empty frames.

    Mock mode: synthesizes ``PointStamped(map, x=2.0, y=0.0, z=1.3)``
    so the BT exercises the seeded path under mocks.
    """

    def __init__(
        self,
        name: str,
        bb_target: str,
        prompt: str = "person",
        service_name: str = "object_detection_generalist",
        target_frame: str = "map",
        use_orbbec: bool = True,
        use_vlm_sam_fallback: bool = True,
    ):
        super().__init__(name, service_name, ObjectDetectionGeneralist)
        self._bb_target = bb_target
        self._prompt = prompt
        self._target_frame = target_frame
        self._camera = "orbbec" if use_orbbec else "realsense"
        self._use_vlm_sam_fallback = use_vlm_sam_fallback

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb_write = self.attach_blackboard_client(name=f"{self.name}_write")
        self._bb_write.register_key(
            key="target",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self._bb_target),
        )

    def initialise(self):
        super().initialise()

        if self.mock_mode:
            ps = PointStamped()
            ps.header.frame_id = self._target_frame
            ps.point.x = 2.0
            ps.point.y = 0.0
            ps.point.z = 1.3
            self._bb_write.set("target", ps, overwrite=True)
            self.feedback_message = (
                f"MOCK: stashed synthetic person centroid "
                f"{ps.point.x:.2f},{ps.point.y:.2f},{ps.point.z:.2f} "
                f"in {ps.header.frame_id}"
            )
            print(f"🔍 {self.feedback_message}")
            return

        request = ObjectDetectionGeneralist.Request()
        request.camera = self._camera
        request.prompt = self._prompt
        request.target_frame = self._target_frame
        request.sort_closest = True
        request.sort_highest = False
        request.return_rgb_image = False
        request.return_depth_image = False
        request.return_segments = False
        request.force_vlm_sam = False
        request.use_vlm_sam_fallback = self._use_vlm_sam_fallback
        self.response = self.call_service_async(request)
        self.feedback_message = (
            f"Detect+stash request sent (prompt={self._prompt!r}, "
            f"target_frame={self._target_frame!r}, camera={self._camera})"
        )

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()

        if self.response is None:
            self.feedback_message = "No response object"
            print(
                f"❌ DetectAndStash FAIL: {self.feedback_message} — service "
                f"client never sent (check /object_detection_generalist is up).",
                file=sys.stderr, flush=True,
            )
            return pytree.common.Status.FAILURE

        if not self.response.done():
            self.feedback_message = "Still detecting person..."
            return pytree.common.Status.RUNNING

        result = self.response.result()
        if result.status != 0:
            self.feedback_message = (
                f"Detect failed status={result.status}: {result.error_msg}"
            )
            print(
                f"❌ DetectAndStash FAIL: {self.feedback_message}\n"
                f"   target_frame={self._target_frame!r} — if status=1 and "
                f"error_msg mentions TF, ensure map<->camera TF is published, "
                f"or set target_frame='' on the request.",
                file=sys.stderr, flush=True,
            )
            return pytree.common.Status.FAILURE
        if not result.objects:
            self.feedback_message = (
                "Detect returned 0 objects (no person in frame, "
                f"detection_source={result.detection_source})."
            )
            print(
                f"❌ DetectAndStash FAIL: {self.feedback_message} — make sure "
                "exactly one person is in the camera FoV, well-lit, and "
                "facing the robot.",
                file=sys.stderr, flush=True,
            )
            return pytree.common.Status.FAILURE

        chosen = result.objects[0]
        ps = PointStamped()
        ps.header = result.header
        if not ps.header.frame_id:
            ps.header.frame_id = self._target_frame
        ps.point = chosen.centroid
        self._bb_write.set("target", ps, overwrite=True)
        self.feedback_message = (
            f"Stashed person centroid xyz=({ps.point.x:.3f}, "
            f"{ps.point.y:.3f}, {ps.point.z:.3f}) in {ps.header.frame_id} "
            f"(detection_source={result.detection_source}, "
            f"chose 1/{len(result.objects)})"
        )
        print(f"🎯 {self.feedback_message}", flush=True)
        return pytree.common.Status.SUCCESS


def build_tree() -> pytree.behaviour.Behaviour:
    root = pytree.composites.Sequence(
        name="Test BtNode_Approach", memory=True,
    )
    # 1. Look forward so the person is centered in the camera before detection.
    root.add_child(
        BtNode_TurnPanTilt(name="Look forward (0, 30)", x=0.0, y=30.0)
    )
    # 2. Pre-test announce.
    root.add_child(
        BtNode_Announce(
            name="Pre-test announce",
            bb_source=None,
            message="Starting approach testing.",
        )
    )
    # 3. Detect + stash, retry against transient empty frames.
    root.add_child(
        pytree.decorators.Retry(
            name="Retry detect+stash person",
            child=BtNode_DetectAndStashPersonInMap(
                name="Detect person + stash centroid (map)",
                bb_target=KEY_APPROACH_TARGET,
            ),
            num_failures=3,
        )
    )
    # 4. Approach + outcome announce. Selector falls through to the
    # failure announce if BtNode_Approach returns FAILURE, so the
    # operator hears either "Approach successful" or "Approach failed".
    outcome = pytree.composites.Selector(
        name="Approach + announce outcome", memory=False,
    )
    success_branch = pytree.composites.Sequence(
        name="Approach + success announce", memory=True,
    )
    success_branch.add_child(
        BtNode_Approach(
            name="Approach detected person",
            bb_target_key=KEY_APPROACH_TARGET,
        )
    )
    success_branch.add_child(
        BtNode_Announce(
            name="Approach success announce",
            bb_source=None,
            message="Approach successful.",
        )
    )
    outcome.add_child(success_branch)
    outcome.add_child(
        BtNode_Announce(
            name="Approach failure announce",
            bb_source=None,
            message="Approach failed.",
        )
    )
    root.add_child(outcome)
    return root


def main(args=None):
    rclpy.init(args=sys.argv[1:] if args is None else args)

    tree = py_trees_ros.trees.BehaviourTree(build_tree())
    tree.setup(node_name="test_go_to_approach", timeout=15)

    pytree.logging.level = pytree.logging.Level.DEBUG
    tree.tick_tock(period_ms=500.0)

    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
