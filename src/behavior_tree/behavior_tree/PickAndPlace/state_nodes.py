from __future__ import annotations

"""State and policy helper nodes for the PickAndPlace main task.

Three table-cleanup destinations: wash_staging, trash, cabinet.
Plus a separate breakfast queue (bowl, spoon, cereal, milk).
"""

import time
from typing import Any, Dict, List, Sequence

import py_trees


def _abs_key(key: str) -> str:
    return py_trees.blackboard.Blackboard.absolute_name("/", key)


def _safe_object_name(item: Any) -> str:
    if isinstance(item, dict):
        return str(item.get("class_name") or item.get("name") or item.get("cls") or "").strip()
    for attr in ("class_name", "name", "cls"):
        value = getattr(item, attr, None)
        if value:
            return str(value).strip()
    return ""


def _safe_confidence(item: Any) -> float:
    if isinstance(item, dict):
        value = item.get("confidence", 1.0)
    else:
        value = getattr(item, "confidence", 1.0)
    try:
        return float(value)
    except (TypeError, ValueError):
        return 1.0


def _label_matches(item_name: str, labels: Sequence[str]) -> bool:
    """True if any label appears as a substring (case-insensitive) of item_name."""
    lower = item_name.lower()
    return any(label.lower() in lower for label in labels)


class BtNode_InitTaskState(py_trees.behaviour.Behaviour):
    """Reset inventories, queues, score trace, and runtime deadline."""

    def __init__(
        self,
        name: str,
        *,
        inventory_table_key: str,
        work_queue_key: str,
        breakfast_queue_key: str,
        score_trace_key: str,
        phase_deadline_key: str,
        max_runtime_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        bindings = {
            "inv_table": (inventory_table_key, py_trees.common.Access.WRITE),
            "queue": (work_queue_key, py_trees.common.Access.WRITE),
            "breakfast_queue": (breakfast_queue_key, py_trees.common.Access.WRITE),
            "score": (score_trace_key, py_trees.common.Access.WRITE),
            "deadline": (phase_deadline_key, py_trees.common.Access.WRITE),
            "max_runtime": (max_runtime_key, py_trees.common.Access.READ),
        }
        for local, (bb_key, access) in bindings.items():
            self.blackboard.register_key(key=local, access=access, remap_to=_abs_key(bb_key))

    def update(self) -> py_trees.common.Status:
        max_runtime = self.blackboard.max_runtime
        if not isinstance(max_runtime, (int, float)):
            max_runtime = 390.0
        self.blackboard.inv_table = []
        self.blackboard.queue = []
        self.blackboard.breakfast_queue = []
        self.blackboard.score = []
        self.blackboard.deadline = time.time() + float(max_runtime)
        return py_trees.common.Status.SUCCESS


class BtNode_BuildTableInventory(py_trees.behaviour.Behaviour):
    """Merge latest table-scan result into the table inventory with name-based dedupe."""

    def __init__(
        self,
        name: str,
        *,
        vision_result_key: str,
        inventory_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="vision",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(vision_result_key),
        )
        self.blackboard.register_key(
            key="inventory",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(inventory_key),
        )
        self.blackboard.register_key(
            key="inventory_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(inventory_key),
        )

    def update(self) -> py_trees.common.Status:
        current = list(self.blackboard.inventory or [])
        existing = {str(item.get("name", "")).lower() for item in current if isinstance(item, dict)}
        vision = self.blackboard.vision
        objects = list(getattr(vision, "objects", []) or [])

        added = 0
        for obj in objects:
            name = _safe_object_name(obj)
            if not name or name.lower() in existing:
                continue
            existing.add(name.lower())
            current.append(
                {
                    "id": len(current) + 1,
                    "name": name,
                    "confidence": _safe_confidence(obj),
                }
            )
            added += 1

        self.blackboard.inventory_write = current
        self.feedback_message = f"Added {added} new table items (total {len(current)})"
        return py_trees.common.Status.SUCCESS


class BtNode_ClassifyTableItems(py_trees.behaviour.Behaviour):
    """Tag each table inventory entry with object_class ∈ {wash_staging, trash, cabinet}.

    Priority order on the work queue: wash_staging (priority 0) → trash (1) → cabinet (2).
    Within the same priority, higher-confidence items are processed first.
    """

    PRIORITY_MAP = {"wash_staging": 0, "trash": 1, "cabinet": 2}

    def __init__(
        self,
        name: str,
        *,
        inventory_key: str,
        work_queue_key: str,
        designated_trash_labels: Sequence[str],
        wash_staging_labels: Sequence[str],
    ):
        super().__init__(name=name)
        self.designated_trash_labels = list(designated_trash_labels)
        self.wash_staging_labels = list(wash_staging_labels)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="inventory",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(inventory_key),
        )
        self.blackboard.register_key(
            key="queue_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(work_queue_key),
        )

    def _classify(self, item_name: str) -> str:
        # Designated trash takes priority over wash staging in case of a label
        # overlap (e.g. "paper cup" overlaps "cup" in tableware).
        if _label_matches(item_name, self.designated_trash_labels):
            return "trash"
        if _label_matches(item_name, self.wash_staging_labels):
            return "wash_staging"
        return "cabinet"

    def update(self) -> py_trees.common.Status:
        queue: List[Dict[str, Any]] = []
        for item in list(self.blackboard.inventory or []):
            if not isinstance(item, dict) or not item.get("name"):
                continue
            name = item["name"]
            object_class = self._classify(name)
            queue.append(
                {
                    "name": name,
                    "prompt": name,
                    "confidence": float(item.get("confidence", 1.0)),
                    "object_class": object_class,
                    "priority": self.PRIORITY_MAP[object_class],
                    "status": "queued",
                }
            )
        queue.sort(key=lambda x: (x["priority"], -x["confidence"], x["name"]))
        self.blackboard.queue_write = queue
        self.feedback_message = (
            f"Classified {len(queue)} items: "
            f"{sum(1 for q in queue if q['object_class']=='wash_staging')} wash, "
            f"{sum(1 for q in queue if q['object_class']=='trash')} trash, "
            f"{sum(1 for q in queue if q['object_class']=='cabinet')} cabinet"
        )
        return py_trees.common.Status.SUCCESS


class BtNode_HasPendingWork(py_trees.behaviour.Behaviour):
    """Condition: queue is not empty (parameterized so it works for cleanup or breakfast)."""

    def __init__(self, name: str, *, work_queue_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="queue",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(work_queue_key),
        )

    def update(self) -> py_trees.common.Status:
        if list(self.blackboard.queue or []):
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "Queue empty"
        return py_trees.common.Status.FAILURE


class BtNode_SelectNextItem(py_trees.behaviour.Behaviour):
    """Pop next cleanup item; publish active prompt, source pose, target pose+point, class."""

    def __init__(
        self,
        name: str,
        *,
        work_queue_key: str,
        active_object_class_key: str,
        active_prompt_key: str,
        active_source_pose_key: str,
        active_target_pose_key: str,
        active_target_point_key: str,
        pose_table_key: str,
        pose_wash_staging_key: str,
        pose_trash_bin_key: str,
        pose_cabinet_key: str,
        point_wash_staging_key: str,
        point_cabinet_default_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        bindings = {
            "queue":               (work_queue_key,            py_trees.common.Access.READ),
            "queue_write":         (work_queue_key,            py_trees.common.Access.WRITE),
            "active_class":        (active_object_class_key,   py_trees.common.Access.WRITE),
            "active_prompt":       (active_prompt_key,         py_trees.common.Access.WRITE),
            "active_source_pose":  (active_source_pose_key,    py_trees.common.Access.WRITE),
            "active_target_pose":  (active_target_pose_key,    py_trees.common.Access.WRITE),
            "active_target_point": (active_target_point_key,   py_trees.common.Access.WRITE),
            "pose_table":          (pose_table_key,            py_trees.common.Access.READ),
            "pose_wash":           (pose_wash_staging_key,     py_trees.common.Access.READ),
            "pose_trash":          (pose_trash_bin_key,        py_trees.common.Access.READ),
            "pose_cabinet":        (pose_cabinet_key,          py_trees.common.Access.READ),
            "point_wash":          (point_wash_staging_key,    py_trees.common.Access.READ),
            "point_cabinet":       (point_cabinet_default_key, py_trees.common.Access.READ),
        }
        for local, (bb_key, access) in bindings.items():
            self.blackboard.register_key(key=local, access=access, remap_to=_abs_key(bb_key))

    def update(self) -> py_trees.common.Status:
        queue = list(self.blackboard.queue or [])
        if not queue:
            self.feedback_message = "Queue empty"
            return py_trees.common.Status.FAILURE
        item = queue.pop(0)
        self.blackboard.queue_write = queue

        cls = item["object_class"]
        self.blackboard.active_class = cls
        self.blackboard.active_prompt = item["prompt"]
        self.blackboard.active_source_pose = self.blackboard.pose_table

        if cls == "wash_staging":
            self.blackboard.active_target_pose = self.blackboard.pose_wash
            self.blackboard.active_target_point = self.blackboard.point_wash
        elif cls == "trash":
            self.blackboard.active_target_pose = self.blackboard.pose_trash
            # Trash drop uses joint-space arm move (KEY_ARM_TRASH); no Cartesian point.
            self.blackboard.active_target_point = None
        else:  # cabinet
            self.blackboard.active_target_pose = self.blackboard.pose_cabinet
            self.blackboard.active_target_point = self.blackboard.point_cabinet

        self.feedback_message = f"Selected '{item['name']}' → {cls}"
        return py_trees.common.Status.SUCCESS


class BtNode_IsActiveClass(py_trees.behaviour.Behaviour):
    """Guard: succeed iff the active item's class matches `expected`."""

    def __init__(self, name: str, *, expected: str, active_object_class_key: str):
        super().__init__(name=name)
        self.expected = str(expected)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="active_class",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_object_class_key),
        )

    def update(self) -> py_trees.common.Status:
        if str(self.blackboard.active_class or "") == self.expected:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_BuildBreakfastQueue(py_trees.behaviour.Behaviour):
    """Seed the breakfast queue: bowl → spoon → cereal → milk.

    Order chosen so spoon arrives directly after bowl on the same trip
    (kitchen shelf), then cereal+milk on a single trip to the cabinet.
    """

    def __init__(
        self,
        name: str,
        *,
        breakfast_queue_key: str,
        kitchen_shelf_pose_key: str,
        cabinet_pose_key: str,
        bowl_point_key: str,
        spoon_point_key: str,
        cereal_point_key: str,
        milk_point_key: str,
    ):
        super().__init__(name=name)
        self.kitchen_shelf_pose_key = kitchen_shelf_pose_key
        self.cabinet_pose_key = cabinet_pose_key
        self.bowl_point_key = bowl_point_key
        self.spoon_point_key = spoon_point_key
        self.cereal_point_key = cereal_point_key
        self.milk_point_key = milk_point_key
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="queue_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(breakfast_queue_key),
        )

    def update(self) -> py_trees.common.Status:
        queue = [
            {
                "name": "bowl",
                "prompt": "bowl",
                "source_pose_key": self.kitchen_shelf_pose_key,
                "target_point_key": self.bowl_point_key,
                "object_class": "breakfast",
            },
            {
                "name": "spoon",
                "prompt": "spoon",
                "source_pose_key": self.kitchen_shelf_pose_key,
                "target_point_key": self.spoon_point_key,
                "object_class": "breakfast",
            },
            {
                "name": "cereal",
                "prompt": "cereal box",
                "source_pose_key": self.cabinet_pose_key,
                "target_point_key": self.cereal_point_key,
                "object_class": "breakfast",
            },
            {
                "name": "milk",
                "prompt": "milk box",
                "source_pose_key": self.cabinet_pose_key,
                "target_point_key": self.milk_point_key,
                "object_class": "breakfast",
            },
        ]
        self.blackboard.queue_write = queue
        self.feedback_message = f"Seeded {len(queue)} breakfast items"
        return py_trees.common.Status.SUCCESS


class BtNode_SelectNextBreakfastItem(py_trees.behaviour.Behaviour):
    """Pop next breakfast item; publish active prompt, source pose, target point, class."""

    def __init__(
        self,
        name: str,
        *,
        breakfast_queue_key: str,
        active_object_class_key: str,
        active_prompt_key: str,
        active_source_pose_key: str,
        active_target_point_key: str,
        kitchen_shelf_pose_key: str,
        cabinet_pose_key: str,
        bowl_point_key: str,
        spoon_point_key: str,
        cereal_point_key: str,
        milk_point_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self._key_to_local: Dict[str, str] = {}
        rw_bindings = {
            "queue":               (breakfast_queue_key,      py_trees.common.Access.READ),
            "queue_write":         (breakfast_queue_key,      py_trees.common.Access.WRITE),
            "active_class":        (active_object_class_key,  py_trees.common.Access.WRITE),
            "active_prompt":       (active_prompt_key,        py_trees.common.Access.WRITE),
            "active_source_pose":  (active_source_pose_key,   py_trees.common.Access.WRITE),
            "active_target_point": (active_target_point_key,  py_trees.common.Access.WRITE),
        }
        for local, (bb_key, access) in rw_bindings.items():
            self.blackboard.register_key(key=local, access=access, remap_to=_abs_key(bb_key))

        candidate_keys = [
            ("src_kitchen_shelf", kitchen_shelf_pose_key),
            ("src_cabinet",       cabinet_pose_key),
            ("pt_bowl",   bowl_point_key),
            ("pt_spoon",  spoon_point_key),
            ("pt_cereal", cereal_point_key),
            ("pt_milk",   milk_point_key),
        ]
        for local, bb_key in candidate_keys:
            self.blackboard.register_key(
                key=local,
                access=py_trees.common.Access.READ,
                remap_to=_abs_key(bb_key),
            )
            self._key_to_local[bb_key] = local

    def update(self) -> py_trees.common.Status:
        queue = list(self.blackboard.queue or [])
        if not queue:
            self.feedback_message = "Breakfast queue empty"
            return py_trees.common.Status.FAILURE
        item = queue.pop(0)
        self.blackboard.queue_write = queue

        src_local = self._key_to_local.get(item["source_pose_key"])
        pt_local = self._key_to_local.get(item["target_point_key"])
        if src_local is None or pt_local is None:
            self.feedback_message = (
                f"Unmapped breakfast keys for '{item['name']}': "
                f"src={item['source_pose_key']} pt={item['target_point_key']}"
            )
            return py_trees.common.Status.FAILURE

        self.blackboard.active_class = "breakfast"
        self.blackboard.active_prompt = item["prompt"]
        self.blackboard.active_source_pose = getattr(self.blackboard, src_local)
        self.blackboard.active_target_point = getattr(self.blackboard, pt_local)
        self.feedback_message = f"Selected breakfast item '{item['name']}'"
        return py_trees.common.Status.SUCCESS


class BtNode_RecordCompletion(py_trees.behaviour.Behaviour):
    """Append a completed-or-skipped action record for the score trace / summary."""

    def __init__(
        self,
        name: str,
        *,
        score_trace_key: str,
        active_prompt_key: str,
        active_object_class_key: str,
        success: bool,
    ):
        super().__init__(name=name)
        self.success = bool(success)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="score",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(score_trace_key),
        )
        self.blackboard.register_key(
            key="score_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(score_trace_key),
        )
        self.blackboard.register_key(
            key="prompt",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_prompt_key),
        )
        self.blackboard.register_key(
            key="klass",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_object_class_key),
        )

    def update(self) -> py_trees.common.Status:
        score = list(self.blackboard.score or [])
        score.append(
            {
                "object": self.blackboard.prompt,
                "class": self.blackboard.klass,
                "success": self.success,
                "timestamp": time.time(),
            }
        )
        self.blackboard.score_write = score
        return py_trees.common.Status.SUCCESS


class BtNode_TimeoutCutoverChecker(py_trees.behaviour.Behaviour):
    """Condition: runtime deadline has not been exceeded (FAILURE = stop the loop)."""

    def __init__(self, name: str, *, phase_deadline_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="deadline",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(phase_deadline_key),
        )

    def update(self) -> py_trees.common.Status:
        deadline = self.blackboard.deadline
        if isinstance(deadline, (int, float)) and time.time() >= float(deadline):
            self.feedback_message = "Reached runtime cutover"
            return py_trees.common.Status.FAILURE
        return py_trees.common.Status.SUCCESS


class BtNode_BuildCompletionSummary(py_trees.behaviour.Behaviour):
    """Build a concise completion message from the score trace."""

    def __init__(self, name: str, *, score_trace_key: str, summary_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="score",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(score_trace_key),
        )
        self.blackboard.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(summary_key),
        )

    def update(self) -> py_trees.common.Status:
        score = list(self.blackboard.score or [])
        succeeded = sum(1 for item in score if item.get("success"))
        failed = len(score) - succeeded
        cleanup = sum(1 for item in score if item.get("class") in ("wash_staging", "trash", "cabinet"))
        breakfast = sum(1 for item in score if item.get("class") == "breakfast")
        if not score:
            self.blackboard.summary = "Pick and place complete. No items handled."
        else:
            self.blackboard.summary = (
                f"Pick and place complete. Handled {succeeded} items ({failed} skipped). "
                f"Cleanup: {cleanup}. Breakfast: {breakfast}."
            )
        return py_trees.common.Status.SUCCESS
