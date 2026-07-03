from __future__ import annotations

"""PickAndPlace-specific BT nodes — copied from StoringGroceries to avoid cross-task imports.

These are functionally identical to the StoringGroceries originals.
"""

from typing import Any

import action_msgs.msg as action_msgs
import py_trees

from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.TemplateNodes.BaseBehaviors import ServiceHandler
from behavior_tree.TemplateNodes.Manipulation import BtNode_Grasp
from behavior_tree.messages import Categorize, ObjectDetectionGeneralist, GetImage
from geometry_msgs.msg import Pose

import time

from behavior_tree.config import get_config
from behavior_tree.PickAndPlace.categorization import classify_destination
from behavior_tree.PickAndPlace.config import (
    CUTLERY_LABELS,
    TABLEWARE_LABELS,
    DESIGNATED_TRASH_LABELS,
    CATEGORY_MAP,
    DESTINATION_ROUTING,
    PLACEMENT_MODE_FIXED_POINT,
    PLACEMENT_MODE_NONE,
    KEY_SCAN_RESULTS_TABLE,
    KEY_INVENTORY_TABLE,
    KEY_WORK_QUEUE,
    KEY_POSE_TABLE,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_OBJECT_LABEL,
    KEY_ACTIVE_PROMPT,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ACTIVE_TARGET_POINT,
    KEY_SCORE_TRACE,
)


class BtNode_WriteFoundItems(py_trees.behaviour.Behaviour):
    """Writes found items from the generalist detection result to the blackboard for TTS announcement.

    Always SUCCESS: a missing vision-result key (upstream scan never ran /
    was skipped), an empty `.objects`, or a malformed result (unexpected
    shape, missing `.cls`) all degrade to the same "found nothing" outcome
    rather than failing the tree. This node only formats an announcement —
    it must never be the thing that aborts a mission.
    """

    def __init__(
        self,
        name: str,
        bb_key_vision_res: str,
        bb_key_announcement: str,
        place_seen:str = None
    ):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="vision_result",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_vision_res),
        )
        self.blackboard.register_key(
            key="announcement_msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_announcement),
        )
        self.place_seen = place_seen

    def _found_nothing(self, reason):
        self.blackboard.announcement_msg = "I could not find any objects"
        self.feedback_message = reason
        return py_trees.common.Status.SUCCESS

    def update(self):
        try:
            vision_result = self.blackboard.vision_result
        except Exception as e:
            return self._found_nothing(
                f"No vision result on blackboard, treating as none found: {e}"
            )

        objects = getattr(vision_result, "objects", None)
        if not objects:
            return self._found_nothing("No objects found in vision result")

        try:
            object_names = [
                "one " + obj.cls for obj in objects if getattr(obj, "cls", None)
            ]
        except Exception as e:
            return self._found_nothing(
                f"Malformed vision result, treating as none found: {e}"
            )

        if not object_names:
            return self._found_nothing("No valid object labels in vision result")

        announcement_msg = f"I see {', '.join(object_names)}"
        if self.place_seen is not None:
            announcement_msg += f"  {self.place_seen}."
        else:
            announcement_msg += "."
        self.blackboard.announcement_msg = announcement_msg
        self.feedback_message = f"Wrote {announcement_msg} to blackboard"
        return py_trees.common.Status.SUCCESS


class BtNode_GetImage(ServiceHandler):
    """Gets an image and/or depth from a specified camera and writes it to the blackboard.

    Uses `tinker_vision_msgs_26/srv/GetImage` on `/get_image_service`. 
    The `camera` field specifies which camera to get the image from (e.g., "realsense" or "orbbec").
    The `depth` field specifies whether to return the depth image in addition to the RGB image.
    """

    def __init__(
        self,
        name: str,
        camera: str,
        bb_key_rgb_image: str,
        get_depth_image: bool = False,
        bb_key_depth_image: str = None,
        service_name: str = "get_image_service",
        service_type=GetImage,
    ):
        super().__init__(name=name, service_name=service_name, service_type=service_type)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="rgb_image",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_rgb_image),
        )
        if bb_key_depth_image is not None:
            self.blackboard.register_key(
                key="depth_image",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_depth_image),
            )
        self.camera = camera
        self.get_depth_image = get_depth_image
    
    def initialise(self):
        super().initialise()
        if self.mock_mode:
            from behavior_tree.mock_messages import MockMessage
            placeholder = MockMessage()
            placeholder.status = 0
            self.blackboard.rgb_image = placeholder
            if self.get_depth_image and self.blackboard.exists("depth_image"):
                self.blackboard.depth_image = placeholder
            self.feedback_message = f"MOCK: GetImage from {self.camera}"
            return
        request = GetImage.Request()
        request.camera = self.camera
        request.depth = self.get_depth_image
        self.response = self.client.call_async(request)

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        if self.response.done():
            if self.response.result().status == 0:
                response = self.response.result()
                self.blackboard.rgb_image = response.rgb_image
                if self.get_depth_image and self.blackboard.exists("depth_image"):
                    self.blackboard.depth_image = response.depth_image
                self.feedback_message = f"Successfully got image from {self.camera} camera"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = (
                    f"Failed to get image from {self.camera} camera with status {self.response.result().status} "
                    f"and error message {self.response.result().error_msg}"
                )
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"Waiting for response from get image service for {self.camera} camera"
            return py_trees.common.Status.RUNNING



class BtNode_FindObjTable(ServiceHandler):
    """Find object on table using the generalist detection service.

    Uses `tinker_vision_msgs_26/srv/ObjectDetectionGeneralist` on
    `/object_detection_generalist`. Open-vocab prompts fall through to the
    YOLO-World / Gemini + FastSAM path when `use_vlm_sam_fallback=True`.
    Sorts closest-first so the nearest match is the grasp candidate.
    """

    def __init__(
        self,
        name: str,
        bb_key_prompt: str,
        bb_key_image: str,
        bb_key_segment: str,
        bb_key_result: str,
        bb_key_announcement: str,
        bb_key_object_label: str = None,
        target_frame: str = "base_link",
        use_realsense: bool = True,
        service_name: str = "object_detection_generalist",
        service_type=ObjectDetectionGeneralist,
        use_vlm_sam_fallback: bool = True,
    ):
        super().__init__(name=name, service_name=service_name, service_type=service_type)
        self.bb_key_prompt = bb_key_prompt
        self.bb_key_image = bb_key_image
        self.bb_key_segment = bb_key_segment
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt),
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image),
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment),
        )
        self.blackboard.register_key(
            key="result",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result),
        )
        self.blackboard.register_key(
            key="announcement_msg",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_announcement),
        )
        if bb_key_object_label is not None:
            self.blackboard.register_key(
                key="object_label",
                access=py_trees.common.Access.WRITE,
                remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_object_label),
            )
        self.use_realsense = use_realsense
        self.target_frame = target_frame
        self.use_vlm_sam_fallback = use_vlm_sam_fallback

    def initialise(self):
        super().initialise()
        if self.mock_mode:
            from behavior_tree.mock_messages import MockMessage
            mock_result = MockMessage()
            mock_result.status = 0
            mock_result.objects = []
            mock_result.segments = [MockMessage()]
            mock_result.rgb_image = MockMessage()
            mock_result.depth_image = MockMessage()
            self.blackboard.image = mock_result.rgb_image
            self.blackboard.segmentation = mock_result.segments[0]
            self.blackboard.result = mock_result
            self.blackboard.announcement_msg = "MOCK: pretending to find object"
            try:
                self.blackboard.object_label = "mock_object"
            except AttributeError:
                pass
            self.feedback_message = "MOCK: FindObjTable"
            return
        request = ObjectDetectionGeneralist.Request()
        request.prompt = self.blackboard.prompt
        request.camera = "realsense" if self.use_realsense else "orbbec"
        request.target_frame = self.target_frame
        request.sort_closest = True
        request.sort_highest = False
        request.return_rgb_image = True
        request.return_depth_image = True
        request.return_segments = True
        request.force_vlm_sam = False
        request.use_vlm_sam_fallback = self.use_vlm_sam_fallback
        self.response = self.client.call_async(request)

    def update(self):
        if self.mock_mode:
            return self.wait_for_keypress_in_mock()
        if self.response.done():
            if self.response.result().status == 0:
                response = self.response.result()
                self.blackboard.image = response.rgb_image
                self.blackboard.segmentation = response.segments[0]
                self.blackboard.result = response
                self.blackboard.announcement_msg = f"Grasping {response.objects[0].cls}"
                try:
                    self.blackboard.object_label = response.objects[0].cls
                except AttributeError:
                    pass
                self.feedback_message = f"Found object: {response.objects[0].cls}"
                return py_trees.common.Status.SUCCESS
            else:
                self.feedback_message = (
                    f"Failed to find object with {self.response.result().status} "
                    f"and error message {self.response.result().error_msg}"
                )
                return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = "Waiting for response from find object service"
            return py_trees.common.Status.RUNNING


class BtNode_CategorizeGrocery(ActionHandler):
    """Categorize grocery item for cabinet shelf placement."""

    def __init__(
        self,
        name: str,
        n_layers: int,
        bb_key_prompt: str,
        bb_key_image: str,
        bb_key_segment: str,
        bb_target_frame: str,
        bb_key_result_point: str,
        bb_key_env_points: str,
        bb_key_reason: str,
        bb_key_shelf_left: str,
        bb_key_shelf_right: str,
        action_name: str = "grocery_categorize",
        wait_for_server_timeout_sec: float = -3,
    ):
        super().__init__(
            name, Categorize, action_name, None, wait_for_server_timeout_sec
        )
        self.n_layers = n_layers
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_prompt),
        )
        self.blackboard.register_key(
            key="image",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_image),
        )
        self.blackboard.register_key(
            key="segmentation",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_segment),
        )
        self.blackboard.register_key(
            key="target_frame",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_frame),
        )
        self.blackboard.register_key(
            key="shelf_left",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_shelf_left),
        )
        self.blackboard.register_key(
            key="shelf_right",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_shelf_right),
        )
        self.blackboard.register_key(
            key="env_points",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_env_points),
        )
        self.blackboard.register_key(
            key="result_point",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_result_point),
        )
        self.blackboard.register_key(
            key="reason",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_key_reason),
        )

    def send_goal(self):
        try:
            goal = Categorize.Goal()
            goal.n_layers = self.n_layers
            goal.prompt = self.blackboard.prompt
            goal.img_table = self.blackboard.image
            goal.segment_object = self.blackboard.segmentation
            goal.target_frame = self.blackboard.target_frame
            goal.pt_shelf_left = self.blackboard.shelf_left
            goal.pt_shelf_right = self.blackboard.shelf_right
            self.send_goal_request(goal)
            self.feedback_message = f"Sent goal to categorize grocery with prompt: {self.blackboard.prompt}"
        except Exception as e:
            self.feedback_message = f"Failed to send goal: {e}"
            return py_trees.common.Status.FAILURE

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            self.feedback_message = (
                f"Categorize grocery failed with status: {self.result_status} "
                f"and error message {self.result_message.result.error_msg}"
            )
            return py_trees.common.Status.FAILURE
        else:
            result = self.result_message.result
            self.blackboard.result_point = result.place_point
            self.blackboard.env_points = result.env_points
            self.blackboard.reason = result.place_reason
            self.feedback_message = (
                f"Categorize grocery succeeded with target layer {result.shelf_layer} "
                f"and target point {result.place_point}"
            )
            return py_trees.common.Status.SUCCESS

    def feedback_callback(self, msg):
        feedback = msg.feedback
        if feedback.status != 0:
            self.feedback_message = f"ERROR:  {feedback.status} - {feedback.message}"
        else:
            self.feedback_message = f"INFO:  {feedback.status} - {feedback.message}"


def _abs(key):
    return py_trees.blackboard.Blackboard.absolute_name("/", key)


# Lazily-built map from a config KEY_* name to its materialized constant. Built
# on first use so config import order never matters.
_CONST_BY_KEY = None


def _const_by_key():
    global _CONST_BY_KEY
    if _CONST_BY_KEY is None:
        import behavior_tree.PickAndPlace.config as cfg
        _CONST_BY_KEY = {
            cfg.KEY_POSE_TABLE: cfg.POSE_TABLE,
            cfg.KEY_POSE_WASH_STAGING: cfg.POSE_WASH_STAGING,
            cfg.KEY_POSE_CABINET: cfg.POSE_CABINET,
            cfg.KEY_POSE_TRASH_BIN: cfg.POSE_TRASH_BIN,
            cfg.KEY_POSE_KITCHEN_SHELF: cfg.POSE_KITCHEN_SHELF,
            cfg.KEY_POSE_EXTRA_SURFACE: cfg.POSE_EXTRA_SURFACE,
            cfg.KEY_ARM_TABLE: cfg.ARM_POS_TABLE,
            cfg.KEY_ARM_WASH: cfg.ARM_POS_WASH,
            cfg.KEY_ARM_CABINET: cfg.ARM_POS_CABINET,
            cfg.KEY_ARM_TRASH: cfg.ARM_POS_TRASH,
            cfg.KEY_POINT_WASH_STAGING: cfg.POINT_WASH_STAGING,
            cfg.KEY_POINT_CABINET_DEFAULT: cfg.POINT_CABINET_DEFAULT,
            cfg.KEY_POINT_EXTRA_SURFACE: cfg.POINT_EXTRA_SURFACE,
        }
    return _CONST_BY_KEY


def _new_score_trace():
    return {"visited_phases": [], "events": [], "place_policy": ""}


def record_event(blackboard, phase, item, action, outcome, points_est=0):
    """Append a scored event to the global KEY_SCORE_TRACE.

    `blackboard` is accepted for call-site symmetry (nodes pass their own
    client); the score-trace is a single global key, so a dedicated client does
    the read-modify-write to avoid per-caller key-registration coupling.
    Shape: {'visited_phases': [], 'events': [{phase,item,action,outcome,points_est}], 'place_policy': str}.
    """
    client = py_trees.blackboard.Client(name="pp_record_event")
    client.register_key(key="trace", access=py_trees.common.Access.READ, remap_to=_abs(KEY_SCORE_TRACE))
    client.register_key(key="trace", access=py_trees.common.Access.WRITE, remap_to=_abs(KEY_SCORE_TRACE))
    try:
        trace = client.trace
    except Exception:
        trace = None
    if not isinstance(trace, dict):
        trace = _new_score_trace()
    trace.setdefault("visited_phases", [])
    trace.setdefault("events", [])
    trace.setdefault("place_policy", "")
    trace["events"].append({
        "phase": phase, "item": item, "action": action,
        "outcome": outcome, "points_est": points_est,
    })
    client.trace = trace
    return None


class BtNode_BuildInventory(py_trees.behaviour.Behaviour):
    """Build the cleanup inventory + work queue from a generalist scan result.

    Reads the scan result, classifies each label via classify_destination,
    sorts cabinet-bound items by category (so same-category placements run
    consecutively for the +20 grouping), and writes inventory + queue. In mock
    mode (or when `mock_seed` is set) with an empty upstream result, seeds a
    canned queue so the per-item loop body actually runs. Always SUCCESS.
    Plain Behaviour — never mocked, always runs real logic.
    """

    def __init__(self, name, in_key=KEY_SCAN_RESULTS_TABLE, out_inventory=KEY_INVENTORY_TABLE,
                 out_queue=KEY_WORK_QUEUE, source_pose_key=KEY_POSE_TABLE, mock_seed=None):
        super().__init__(name)
        self.source_pose_key = source_pose_key
        self.mock_seed = mock_seed
        self._in = self.attach_blackboard_client(name=f"{name}_in")
        self._in.register_key(key="scan", access=py_trees.common.Access.READ, remap_to=_abs(in_key))
        self._out = self.attach_blackboard_client(name=f"{name}_out")
        self._out.register_key(key="inventory", access=py_trees.common.Access.WRITE, remap_to=_abs(out_inventory))
        self._out.register_key(key="queue", access=py_trees.common.Access.WRITE, remap_to=_abs(out_queue))

    def _labels_from_scan(self):
        try:
            scan = self._in.scan
        except Exception:
            return []
        objs = getattr(scan, "objects", None) or []
        labels = []
        for o in objs:
            lbl = getattr(o, "cls", None) or getattr(o, "class_name", None)
            if lbl:
                labels.append((str(lbl), getattr(o, "segment", None)))
        return labels

    def update(self):
        labels = self._labels_from_scan()
        if not labels and (self.mock_seed is not None or get_config().is_mock_mode()):
            seed = self.mock_seed or ["bowl", "paper cup", "pringles"]
            labels = [(str(s), None) for s in seed]

        items = []
        for label, segment in labels:
            dest = classify_destination(
                label, cutlery=CUTLERY_LABELS, tableware=TABLEWARE_LABELS,
                trash=DESIGNATED_TRASH_LABELS, category_map=CATEGORY_MAP,
            )
            items.append({
                "label": label, "segment": segment, "destination": dest.klass,
                "reference_label": dest.reference_label, "source_pose_key": self.source_pose_key,
            })

        cabinet = [it for it in items if it["destination"] == "cabinet"]
        other = [it for it in items if it["destination"] != "cabinet"]
        cabinet.sort(key=lambda it: it["reference_label"])
        ordered = other + cabinet

        self._out.inventory = items
        self._out.queue = ordered
        self.feedback_message = f"inventory={len(items)} queue={len(ordered)}"
        return py_trees.common.Status.SUCCESS


class BtNode_PopWorkItem(py_trees.behaviour.Behaviour):
    """Pop the front work item and stamp the active-item blackboard keys.

    Resolves DESTINATION_ROUTING and applies place_policy to derive the
    effective placement_mode + fixed_target. SUCCESS if an item was popped,
    FAILURE on an empty queue — this is the cleanup-loop terminator and must
    stay un-masked (only its FAILURE exits the Repeat loop). Plain Behaviour —
    never mocked.
    """

    def __init__(self, name, queue=KEY_WORK_QUEUE, place_policy="vlm"):
        super().__init__(name)
        self.place_policy = place_policy
        self._q = self.attach_blackboard_client(name=f"{name}_q")
        self._q.register_key(key="queue", access=py_trees.common.Access.READ, remap_to=_abs(queue))
        self._q.register_key(key="queue", access=py_trees.common.Access.WRITE, remap_to=_abs(queue))
        self._w = self.attach_blackboard_client(name=f"{name}_w")
        self._writes = {
            "object_class": KEY_ACTIVE_OBJECT_CLASS,
            "object_label": KEY_OBJECT_LABEL,
            "prompt": KEY_ACTIVE_PROMPT,
            "source_pose": KEY_ACTIVE_SOURCE_POSE,
            "target_pose": KEY_ACTIVE_TARGET_POSE,
            "target_point": KEY_ACTIVE_TARGET_POINT,
            "reference_label": "pp_active_reference_label",
            "placement_mode": "pp_active_placement_mode",
            "scan_pose": "pp_active_scan_pose",
            "skip_scan": "pp_active_skip_scan",
        }
        for local, key in self._writes.items():
            self._w.register_key(key=local, access=py_trees.common.Access.WRITE, remap_to=_abs(key))

    def update(self):
        try:
            queue = self._q.queue
        except Exception:
            queue = None
        if not queue:
            self.feedback_message = "work queue empty -> FAILURE (loop exit)"
            return py_trees.common.Status.FAILURE

        item = queue[0]
        self._q.queue = list(queue[1:])

        klass = item["destination"]
        nav_pose_key, arm_pose_key, vlm_mode, hardcoded_point_key = DESTINATION_ROUTING[klass]
        consts = _const_by_key()

        self._w.object_class = klass
        self._w.object_label = item["label"]
        self._w.prompt = item["label"]
        self._w.reference_label = item.get("reference_label", "")
        self._w.source_pose = consts.get(item["source_pose_key"])
        self._w.target_pose = consts.get(nav_pose_key)
        # The BT positions the arm itself (handleOneItem._arm); hand the server an
        # empty scan pose + skip_scan_move=True so it does NOT re-move the arm.
        # NB: ARM_POS_* are RADIANS but scan_pose_deg is DEGREES — never forward
        # the arm_pose_key constant (arm_pose_key) here as degrees.
        self._w.scan_pose = []
        self._w.skip_scan = True

        if klass == "trash":
            self._w.placement_mode = PLACEMENT_MODE_NONE
            self._w.target_point = None
        elif self.place_policy == "hardcoded":
            self._w.placement_mode = PLACEMENT_MODE_FIXED_POINT
            self._w.target_point = consts.get(hardcoded_point_key)
        else:  # 'vlm'
            self._w.placement_mode = vlm_mode
            self._w.target_point = consts.get(hardcoded_point_key)

        self.feedback_message = f"popped {item['label']} -> {klass} (mode {self._w.placement_mode})"
        return py_trees.common.Status.SUCCESS


class BtNode_DeadlineGuard(py_trees.behaviour.Behaviour):
    """Wall-clock budget guard. Plain Behaviour — never mocked, so it runs its
    real logic even in mock (large budgets never fire in a fast run).

    initialise() latches deadline = clock() + budget_sec (on entry, not at
    construction). update() returns RUNNING until clock() >= deadline, then
    SUCCESS. Never returns FAILURE. `clock` is injectable for deterministic
    tests; default time.monotonic needs no ROS node handle.
    """

    def __init__(self, name, budget_sec, clock=None):
        super().__init__(name)
        self.budget_sec = float(budget_sec)
        self.clock = clock or time.monotonic
        self._deadline = None

    def initialise(self):
        self._deadline = self.clock() + self.budget_sec

    def update(self):
        if self._deadline is None:
            self._deadline = self.clock() + self.budget_sec
        if self.clock() >= self._deadline:
            self.feedback_message = "deadline reached"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "within budget"
        return py_trees.common.Status.RUNNING


class BtNode_GuardActiveClass(py_trees.behaviour.Behaviour):
    """Route guard: SUCCESS iff the active object class matches `expected`,
    else FAILURE. Plain condition Behaviour — never a Handler (a mocked guard
    would auto-succeed and route everything to the first branch)."""

    def __init__(self, name, expected, key=KEY_ACTIVE_OBJECT_CLASS):
        super().__init__(name)
        self.expected = expected
        self._bb = self.attach_blackboard_client(name=f"{name}_g")
        self._bb.register_key(key="klass", access=py_trees.common.Access.READ, remap_to=_abs(key))

    def update(self):
        try:
            klass = self._bb.klass
        except Exception:
            klass = None
        if klass == self.expected:
            self.feedback_message = f"class {klass} == {self.expected}"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"class {klass} != {self.expected}"
        return py_trees.common.Status.FAILURE


class BtNode_MarkPhase(py_trees.behaviour.Behaviour):
    """Append `phase` to the score-trace visited_phases. Always SUCCESS."""

    def __init__(self, name, phase, key=KEY_SCORE_TRACE):
        super().__init__(name)
        self.phase = phase
        self._bb = self.attach_blackboard_client(name=f"{name}_phase")
        self._bb.register_key(key="trace", access=py_trees.common.Access.READ, remap_to=_abs(key))
        self._bb.register_key(key="trace", access=py_trees.common.Access.WRITE, remap_to=_abs(key))

    def update(self):
        try:
            trace = self._bb.trace
        except Exception:
            trace = None
        if not isinstance(trace, dict):
            trace = _new_score_trace()
        trace.setdefault("visited_phases", [])
        trace.setdefault("events", [])
        trace.setdefault("place_policy", "")
        if self.phase not in trace["visited_phases"]:
            trace["visited_phases"].append(self.phase)
        self._bb.trace = trace
        self.feedback_message = f"phase {self.phase} marked"
        return py_trees.common.Status.SUCCESS
