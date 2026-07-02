"""Layer-3 operator test for the seeded follow_head fresh-lock path.

Detects a single person via ``/object_detection_generalist``, stashes the
centroid as a ``PointStamped(base_link)`` on the blackboard, then runs
``BtNode_MaintainEyeContact`` in seeded mode against that centroid for
~30 s. The post-detection announcement is intentionally long enough to
give a bystander time to walk into the frame closer than the originally-
detected person, so the operator can observe whether the gaze stays on
the seeded target.

Run:

    ros2 run behavior_tree test-follow-head-fresh-lock

Mock smoke (no robot):

    BT_MOCK_MODE=true ros2 run behavior_tree test-follow-head-fresh-lock

Stop with Ctrl+C. See ``test-follow-head-fresh-lock.md`` for the operator
script and pass criteria.
"""

import sys
import time

import py_trees as pytree
import py_trees_ros
import rclpy
from geometry_msgs.msg import PointStamped

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.Vision import (
    BtNode_MaintainEyeContact,
    BtNode_TurnPanTilt,
)
from behavior_tree.messages import ObjectDetectionGeneralist


# Module-local blackboard keys — kept distinct from the receptionist /
# HRI keys so a stale entry doesn't bleed across tests.
KEY_PERSONS = "test_follow_head_persons"
KEY_POINT_CENTROIDS = "test_follow_head_centroids"


class BtNode_DetectAndStashPersonCentroid(ServiceHandler):
    """Single-shot person detection + blackboard stash for the seeded test.

    Calls ``/object_detection_generalist`` with ``prompt="person"`` and
    ``target_frame="base_link"``, picks the first object, and writes:

    - ``[PointStamped(point=obj.centroid, frame_id="base_link")]`` to
      ``bb_points``
    - ``["person"]`` to ``bb_persons``

    These are the two lists ``BtNode_MaintainEyeContact(target_id=0,
    bb_key_persons=..., bb_key_points=...)`` reads to build the seeded
    goal. Returns FAILURE on empty / status != 0; wrap in ``Retry`` if
    you want to ride out transient empty frames.
    """

    def __init__(
        self,
        name: str,
        bb_persons: str,
        bb_points: str,
        prompt: str = "person",
        service_name: str = "object_detection_generalist",
        target_frame: str = "base_link",
        use_orbbec: bool = True,
        use_vlm_sam_fallback: bool = True,
        # base_link sanity gate for the chosen object. The generalist's
        # `sort_closest=True` ranks by camera-frame depth, so a
        # close-camera ghost (poster / laptop screen / operator at the
        # keyboard) often beats the actual test guest. Reject objects
        # whose base_link X (forward) is < min_forward_m or > max_forward_m,
        # or whose |Y| (lateral) exceeds max_lateral_m. Tuned so a guest
        # standing 1-3 m forward, within ±1.5 m laterally, passes.
        min_forward_m: float = 0.3,
        max_forward_m: float = 5.0,
        max_lateral_m: float = 1.5,
    ):
        super().__init__(name, service_name, ObjectDetectionGeneralist)
        self._bb_persons = bb_persons
        self._bb_points = bb_points
        self._prompt = prompt
        self._target_frame = target_frame
        self._camera = "orbbec" if use_orbbec else "realsense"
        self._use_vlm_sam_fallback = use_vlm_sam_fallback
        self._min_forward_m = float(min_forward_m)
        self._max_forward_m = float(max_forward_m)
        self._max_lateral_m = float(max_lateral_m)

    def setup(self, **kwargs):
        super().setup(**kwargs)
        self._bb_write = self.attach_blackboard_client(name=f"{self.name}_write")
        self._bb_write.register_key(
            key="persons",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self._bb_persons),
        )
        self._bb_write.register_key(
            key="points",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", self._bb_points),
        )

    def initialise(self):
        super().initialise()

        if self.mock_mode:
            # Synthesize a sensible centroid in base_link so the rest of
            # the tree exercises the seeded path under mocks.
            ps = PointStamped()
            ps.header.frame_id = self._target_frame
            ps.point.x = 1.5
            ps.point.y = 0.0
            ps.point.z = 1.3
            self._bb_write.set("persons", ["person"], overwrite=True)
            self._bb_write.set("points", [ps], overwrite=True)
            self.feedback_message = (
                f"MOCK: stashed synthetic person centroid "
                f"{ps.point.x:.2f},{ps.point.y:.2f},{ps.point.z:.2f}"
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
                f"client never sent (check {self._service_name_str()} is up).",
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
            # Most common cause: target_frame TF lookup failure on the
            # server (base_link <-> camera frame). Hint the operator
            # explicitly so they can either bring up TF or rerun with
            # target_frame="" (raw camera frame).
            print(
                f"❌ DetectAndStash FAIL: {self.feedback_message}\n"
                f"   target_frame={self._target_frame!r} — if status=1 and "
                f"error_msg mentions TF, ensure base_link <-> camera TF is "
                f"published, or set target_frame='' on the request.",
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

        # Sanity-filter: the generalist's `sort_closest=True` ranks by
        # camera-frame depth, so a close-camera ghost (poster, laptop
        # screen, the operator at the keyboard) often outranks the
        # actual test guest. Pick the first object whose base_link
        # centroid is in the reasonable forward-half-plane envelope.
        chosen = None
        rejected = []
        for cand in result.objects:
            c = cand.centroid
            if not (self._min_forward_m <= c.x <= self._max_forward_m):
                rejected.append((c.x, c.y, c.z, "x_out_of_range"))
                continue
            if abs(c.y) > self._max_lateral_m:
                rejected.append((c.x, c.y, c.z, "y_out_of_range"))
                continue
            chosen = cand
            break

        if chosen is None:
            rejected_str = ", ".join(
                f"({x:.2f},{y:.2f},{z:.2f}|{why})" for (x, y, z, why) in rejected
            )
            self.feedback_message = (
                f"All {len(result.objects)} detection(s) rejected by "
                f"sanity gate (X in [{self._min_forward_m}, "
                f"{self._max_forward_m}] m, |Y|<={self._max_lateral_m} m). "
                f"Rejected: [{rejected_str}]"
            )
            print(
                f"❌ DetectAndStash FAIL: {self.feedback_message}\n"
                "   The generalist usually picks the closest-to-camera\n"
                "   object first, which can be a poster / laptop screen\n"
                "   / the operator near the keyboard. Make sure the\n"
                "   intended guest is the only person in the camera FoV\n"
                "   and stands ~1.5 m forward of the robot.",
                file=sys.stderr, flush=True,
            )
            return pytree.common.Status.FAILURE

        ps = PointStamped()
        ps.header = result.header  # frame_id should already be base_link
        if not ps.header.frame_id:
            ps.header.frame_id = self._target_frame
        ps.point = chosen.centroid
        self._bb_write.set("persons", ["person"], overwrite=True)
        self._bb_write.set("points", [ps], overwrite=True)
        self.feedback_message = (
            f"Stashed person centroid xyz=({ps.point.x:.3f}, "
            f"{ps.point.y:.3f}, {ps.point.z:.3f}) in {ps.header.frame_id} "
            f"(detection_source={result.detection_source}, "
            f"chose {1 + len(rejected)}/{len(result.objects)} after sanity gate)"
        )
        print(f"🎯 {self.feedback_message}", flush=True)
        return pytree.common.Status.SUCCESS

    def _service_name_str(self) -> str:
        return getattr(self, "service_name", "object_detection_generalist")


def build_tree() -> pytree.behaviour.Behaviour:
    root = pytree.composites.Sequence(
        name="Test follow_head fresh-lock (seeded)", memory=True,
    )
    # 1. Look forward so the person is centered in the camera before we
    # detect. Using (0, 30) tilts up slightly toward face level.
    root.add_child(
        BtNode_TurnPanTilt(
            name="Look forward (0, 30)", x=0.0, y=30.0
        )
    )
    # 2. Pre-detection cue. Long enough to give the operator time to
    # position themselves; the BtNode_Announce blocks until TTS finishes
    # so no separate sleep is needed.
    root.add_child(
        BtNode_Announce(
            name="Pre-detect announce",
            bb_source=None,
            message=(
                "Starting"
            ),
        )
    )
    # 3. Detect + stash. Retry against transient empty frames.
    root.add_child(
        pytree.decorators.Retry(
            name="Retry detect+stash person",
            child=BtNode_DetectAndStashPersonCentroid(
                name="Detect person + stash centroid",
                bb_persons=KEY_PERSONS,
                bb_points=KEY_POINT_CENTROIDS,
            ),
            num_failures=3,
        )
    )
    # 4. Bystander-walk-in window. The announcement plays for ~5 s of
    # TTS, which is the operator's cue to introduce a closer-but-
    # off-center bystander while the robot is still about to start
    # follow_head. Verbose body intentional — describes exactly what
    # the test is doing so the operator and any observer know what to
    # watch for.
    root.add_child(
        BtNode_Announce(
            name="Post-detect announce (long)",
            bb_source=None,
            message=(
                "Person detected."
            ),
        )
    )
    # 5. Seeded follow_head. follow_timeout=30 gives ample room for the
    # bystander to walk in, linger, and walk out while the gaze stays
    # locked on the seeded target.
    root.add_child(
        BtNode_MaintainEyeContact(
            name="Seeded follow (target_id=0)",
            target_id=0,
            bb_key_persons=KEY_PERSONS,
            bb_key_points=KEY_POINT_CENTROIDS,
            # 1.0 m radius (vs 0.6 m default) absorbs the centroid
            # mismatch between the generalist's bbox-center heuristic
            # and follow_head's bbox-head-window heuristic for the same
            # YOLO person bbox. A standing person 1.5 m away can produce
            # XY centroids that differ by 0.4-0.7 m between the two
            # heuristics; 0.6 m is too tight, 1.0 m comfortably covers
            # the gap without admitting a bystander on the other side
            # of the room.
            seed_radius_m=1.0,
            follow_timeout=30.0,
        )
    )
    # 6. Final cue.
    root.add_child(
        BtNode_Announce(
            name="Test complete",
            bb_source=None,
            message="Follow head test complete.",
        )
    )
    return root


def main(args=None):
    rclpy.init(args=sys.argv[1:] if args is None else args)

    tree = py_trees_ros.trees.BehaviourTree(build_tree())
    tree.setup(node_name="test_follow_head_fresh_lock", timeout=15)

    pytree.logging.level = pytree.logging.Level.DEBUG
    # Match the HRI BT tick cadence so timing-sensitive behaviour matches
    # the production tree.
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
