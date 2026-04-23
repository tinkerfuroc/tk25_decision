from __future__ import annotations

"""State and policy helper nodes for PickAndPlace cleanup-first strategy."""

import time
from typing import Any, Dict, List

import py_trees


def _abs_key(key: str) -> str:
    return py_trees.blackboard.Blackboard.absolute_name("/", key)


def _safe_object_name(item: Any) -> str:
    """Extract an object class/name from a detection item."""
    if isinstance(item, dict):
        return str(item.get("class_name") or item.get("name") or "").strip()
    return str(getattr(item, "class_name", "") or getattr(item, "name", "")).strip()


def _safe_confidence(item: Any) -> float:
    if isinstance(item, dict):
        value = item.get("confidence", 1.0)
    else:
        value = getattr(item, "confidence", 1.0)
    try:
        return float(value)
    except (TypeError, ValueError):
        return 1.0


def _is_trash_label(label: str) -> bool:
    lower = label.lower()
    trash_tokens = [
        "trash",
        "garbage",
        "paper",
        "napkin",
        "bottle",
        "can",
        "cup",
        "wrapper",
        "bag",
    ]
    return any(token in lower for token in trash_tokens)


class BtNode_InitCleanupState(py_trees.behaviour.Behaviour):
    """Initialize runtime state for cleanup-first execution."""

    def __init__(
        self,
        name: str,
        *,
        inventory_table_key: str,
        inventory_floor_key: str,
        work_queue_key: str,
        score_trace_key: str,
        phase_deadline_key: str,
        max_runtime_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        for key, access in [
            ("inv_table", py_trees.common.Access.WRITE),
            ("inv_floor", py_trees.common.Access.WRITE),
            ("queue", py_trees.common.Access.WRITE),
            ("score", py_trees.common.Access.WRITE),
            ("deadline", py_trees.common.Access.WRITE),
            ("max_runtime", py_trees.common.Access.READ),
        ]:
            self.blackboard.register_key(
                key=key,
                access=access,
                remap_to=_abs_key(
                    {
                        "inv_table": inventory_table_key,
                        "inv_floor": inventory_floor_key,
                        "queue": work_queue_key,
                        "score": score_trace_key,
                        "deadline": phase_deadline_key,
                        "max_runtime": max_runtime_key,
                    }[key]
                ),
            )

    def update(self) -> py_trees.common.Status:
        max_runtime = self.blackboard.max_runtime
        if not isinstance(max_runtime, (int, float)):
            max_runtime = 390.0
        self.blackboard.inv_table = []
        self.blackboard.inv_floor = []
        self.blackboard.queue = []
        self.blackboard.score = []
        self.blackboard.deadline = time.time() + float(max_runtime)
        return py_trees.common.Status.SUCCESS


class BtNode_MergeInventoryFromDetection(py_trees.behaviour.Behaviour):
    """Merge latest scan result into per-area inventory with dedupe."""

    def __init__(
        self,
        name: str,
        *,
        vision_result_key: str,
        inventory_key: str,
        source_area: str,
    ):
        super().__init__(name=name)
        self.source_area = source_area
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
        existing = {
            (str(item.get("name", "")).lower(), str(item.get("source_area", "")).lower())
            for item in current
            if isinstance(item, dict)
        }
        vision = self.blackboard.vision
        objects = list(getattr(vision, "objects", []) or [])

        for obj in objects:
            name = _safe_object_name(obj)
            if not name:
                continue
            dedupe_key = (name.lower(), self.source_area.lower())
            if dedupe_key in existing:
                continue
            existing.add(dedupe_key)
            current.append(
                {
                    "id": len(current) + 1,
                    "name": name,
                    "confidence": _safe_confidence(obj),
                    "source_area": self.source_area,
                }
            )

        self.blackboard.inventory_write = current
        self.feedback_message = f"Merged {len(objects)} detections into {self.source_area} inventory"
        return py_trees.common.Status.SUCCESS


class BtNode_BuildCleanupWorkQueue(py_trees.behaviour.Behaviour):
    """Build deterministic priority queue: table_trash > floor_trash > table_relocation."""

    def __init__(
        self,
        name: str,
        *,
        inventory_table_key: str,
        inventory_floor_key: str,
        work_queue_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="table_items",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(inventory_table_key),
        )
        self.blackboard.register_key(
            key="floor_items",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(inventory_floor_key),
        )
        self.blackboard.register_key(
            key="queue_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(work_queue_key),
        )

    def update(self) -> py_trees.common.Status:
        queue: List[Dict[str, Any]] = []

        def append_item(item: Dict[str, Any], priority: int, object_class: str):
            queue.append(
                {
                    "name": item["name"],
                    "confidence": float(item.get("confidence", 1.0)),
                    "source_area": item["source_area"],
                    "object_class": object_class,
                    "priority": priority,
                    "status": "queued",
                }
            )

        for item in list(self.blackboard.table_items or []):
            if not isinstance(item, dict) or not item.get("name"):
                continue
            if _is_trash_label(item["name"]):
                append_item(item, priority=0, object_class="trash")
            else:
                append_item(item, priority=2, object_class="relocation")

        for item in list(self.blackboard.floor_items or []):
            if not isinstance(item, dict) or not item.get("name"):
                continue
            obj_class = "trash" if _is_trash_label(item["name"]) else "relocation"
            priority = 1 if obj_class == "trash" else 3
            append_item(item, priority=priority, object_class=obj_class)

        queue.sort(key=lambda x: (x["priority"], -x["confidence"], x["name"]))
        self.blackboard.queue_write = queue
        self.feedback_message = f"Built cleanup queue with {len(queue)} items"
        return py_trees.common.Status.SUCCESS


class BtNode_HasPendingWork(py_trees.behaviour.Behaviour):
    """Condition: queue is not empty."""

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
        self.feedback_message = "No pending queue items"
        return py_trees.common.Status.FAILURE


class BtNode_SelectNextQueueItem(py_trees.behaviour.Behaviour):
    """Pop the next queue item and publish active object metadata."""

    def __init__(
        self,
        name: str,
        *,
        work_queue_key: str,
        active_object_key: str,
        active_object_class_key: str,
        active_source_pose_key: str,
        active_target_pose_key: str,
        active_target_point_key: str,
        pose_table_key: str,
        pose_floor_scan_key: str,
        pose_trash_bin_key: str,
        pose_clean_table_key: str,
        point_trash_bin_key: str,
        point_clean_table_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        key_map = {
            "queue": (work_queue_key, py_trees.common.Access.READ),
            "queue_write": (work_queue_key, py_trees.common.Access.WRITE),
            "active_name": (active_object_key, py_trees.common.Access.WRITE),
            "active_class": (active_object_class_key, py_trees.common.Access.WRITE),
            "active_source_pose": (active_source_pose_key, py_trees.common.Access.WRITE),
            "active_target_pose": (active_target_pose_key, py_trees.common.Access.WRITE),
            "active_target_point": (active_target_point_key, py_trees.common.Access.WRITE),
            "pose_table": (pose_table_key, py_trees.common.Access.READ),
            "pose_floor": (pose_floor_scan_key, py_trees.common.Access.READ),
            "pose_trash": (pose_trash_bin_key, py_trees.common.Access.READ),
            "pose_clean": (pose_clean_table_key, py_trees.common.Access.READ),
            "point_trash": (point_trash_bin_key, py_trees.common.Access.READ),
            "point_clean": (point_clean_table_key, py_trees.common.Access.READ),
        }
        for local_key, (bb_key, access) in key_map.items():
            self.blackboard.register_key(
                key=local_key,
                access=access,
                remap_to=_abs_key(bb_key),
            )

    def update(self) -> py_trees.common.Status:
        queue = list(self.blackboard.queue or [])
        if not queue:
            self.feedback_message = "Queue empty"
            return py_trees.common.Status.FAILURE

        item = queue.pop(0)
        self.blackboard.queue_write = queue
        self.blackboard.active_name = item["name"]
        self.blackboard.active_class = item["object_class"]
        if item["source_area"] == "floor":
            self.blackboard.active_source_pose = self.blackboard.pose_floor
        else:
            self.blackboard.active_source_pose = self.blackboard.pose_table

        if item["object_class"] == "trash":
            self.blackboard.active_target_pose = self.blackboard.pose_trash
            self.blackboard.active_target_point = self.blackboard.point_trash
        else:
            self.blackboard.active_target_pose = self.blackboard.pose_clean
            self.blackboard.active_target_point = self.blackboard.point_clean

        self.feedback_message = f"Selected {item['name']} ({item['object_class']})"
        return py_trees.common.Status.SUCCESS


class BtNode_IsActiveTrashClass(py_trees.behaviour.Behaviour):
    """Condition: currently selected queue item is trash-class."""

    def __init__(self, name: str, *, active_object_class_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="active_class",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_object_class_key),
        )

    def update(self) -> py_trees.common.Status:
        if str(self.blackboard.active_class or "") == "trash":
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_RecordCompletion(py_trees.behaviour.Behaviour):
    """Append a completed action record for score trace / summary."""

    def __init__(
        self,
        name: str,
        *,
        score_trace_key: str,
        active_object_key: str,
        active_object_class_key: str,
        active_target_pose_key: str,
        success: bool,
    ):
        super().__init__(name=name)
        self.success = success
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
            key="name",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_object_key),
        )
        self.blackboard.register_key(
            key="klass",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_object_class_key),
        )
        self.blackboard.register_key(
            key="target_pose",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_target_pose_key),
        )

    def update(self) -> py_trees.common.Status:
        score = list(self.blackboard.score or [])
        score.append(
            {
                "object": self.blackboard.name,
                "class": self.blackboard.klass,
                "target": "trash_bin" if self.blackboard.klass == "trash" else "table_staging",
                "success": bool(self.success),
                "timestamp": time.time(),
            }
        )
        self.blackboard.score_write = score
        return py_trees.common.Status.SUCCESS


class BtNode_TimeoutCutoverChecker(py_trees.behaviour.Behaviour):
    """Condition: runtime deadline has not been exceeded."""

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


class BtNode_HasBreakfastCandidates(py_trees.behaviour.Behaviour):
    """Condition: table inventory contains easy breakfast items."""

    def __init__(
        self,
        name: str,
        *,
        inventory_table_key: str,
        candidates: List[str],
    ):
        super().__init__(name=name)
        self.candidates = {item.lower() for item in candidates}
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="table_items",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(inventory_table_key),
        )

    def update(self) -> py_trees.common.Status:
        for item in list(self.blackboard.table_items or []):
            if not isinstance(item, dict):
                continue
            name = str(item.get("name", "")).lower()
            if any(token in name for token in self.candidates):
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_HasRemainingTime(py_trees.behaviour.Behaviour):
    """Condition: enough time remains before deadline for optional branch."""

    def __init__(self, name: str, *, phase_deadline_key: str, min_remaining_sec: float):
        super().__init__(name=name)
        self.min_remaining_sec = float(min_remaining_sec)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="deadline",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(phase_deadline_key),
        )

    def update(self) -> py_trees.common.Status:
        deadline = self.blackboard.deadline
        if not isinstance(deadline, (int, float)):
            return py_trees.common.Status.FAILURE
        if float(deadline) - time.time() >= self.min_remaining_sec:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_BuildCompletionSummary(py_trees.behaviour.Behaviour):
    """Build concise completion message from score trace."""

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
        if not score:
            self.blackboard.summary = "Cleanup complete. No items were handled."
        elif failed == 0:
            self.blackboard.summary = f"Cleanup complete. Handled {succeeded} items."
        else:
            self.blackboard.summary = (
                f"Cleanup complete. Handled {succeeded} items with {failed} skipped."
            )
        return py_trees.common.Status.SUCCESS
