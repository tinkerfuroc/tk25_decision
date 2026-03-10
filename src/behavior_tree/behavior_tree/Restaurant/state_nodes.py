from __future__ import annotations

import time
from typing import Any, Dict, List

import py_trees


def _abs_key(key: str) -> str:
    return py_trees.blackboard.Blackboard.absolute_name("/", key)


class BtNode_AppendCustomerCandidate(py_trees.behaviour.Behaviour):
    """Append a detected customer pose to the queue with deterministic metadata."""

    def __init__(
        self,
        name: str,
        *,
        queue_key: str,
        source_pose_key: str,
        active_id_key: str,
        confidence: float = 1.0,
    ):
        super().__init__(name=name)
        self.confidence = confidence
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="queue",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(queue_key),
        )
        self.blackboard.register_key(
            key="queue_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(queue_key),
        )
        self.blackboard.register_key(
            key="pose",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(source_pose_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        pose = self.blackboard.pose
        if pose is None:
            self.feedback_message = "No detected customer pose to enqueue"
            return py_trees.common.Status.FAILURE

        queue: List[Dict[str, Any]] = list(self.blackboard.queue or [])
        next_id = 1 + max((item["id"] for item in queue), default=0)
        active_id = self.blackboard.active_id
        if isinstance(active_id, int):
            next_id = max(next_id, active_id + 1)

        queue.append(
            {
                "id": next_id,
                "pose": pose,
                "timestamp": time.time(),
                "confidence": float(self.confidence),
                "status": "queued",
            }
        )
        queue.sort(key=lambda item: (item["timestamp"], item["id"]))
        self.blackboard.queue_write = queue
        self.feedback_message = f"Queued caller id={next_id}"
        return py_trees.common.Status.SUCCESS


class BtNode_SelectNextCustomer(py_trees.behaviour.Behaviour):
    """Select oldest valid queued customer and expose it as active target."""

    def __init__(
        self,
        name: str,
        *,
        queue_key: str,
        selected_pose_key: str,
        active_id_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="queue",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(queue_key),
        )
        self.blackboard.register_key(
            key="queue_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(queue_key),
        )
        self.blackboard.register_key(
            key="selected_pose",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(selected_pose_key),
        )
        self.blackboard.register_key(
            key="active_id_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        queue: List[Dict[str, Any]] = list(self.blackboard.queue or [])
        candidates = [item for item in queue if item.get("status") == "queued"]
        if not candidates:
            self.feedback_message = "No queued customers available"
            return py_trees.common.Status.FAILURE

        selected = sorted(candidates, key=lambda item: (item["timestamp"], item["id"]))[0]
        self.blackboard.selected_pose = selected["pose"]
        self.blackboard.active_id_write = selected["id"]
        for item in queue:
            if item["id"] == selected["id"]:
                item["status"] = "active"
        self.blackboard.queue_write = queue
        self.feedback_message = f"Selected caller id={selected['id']}"
        return py_trees.common.Status.SUCCESS


class BtNode_InitOrderChecklist(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, checklist_key: str, pickup_verified_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="checklist",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(checklist_key),
        )
        self.blackboard.register_key(
            key="pickup_verified",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(pickup_verified_key),
        )

    def update(self) -> py_trees.common.Status:
        self.blackboard.checklist = {
            "detected": True,
            "reached": False,
            "order_confirmed": False,
            "pickup_verified": False,
            "served": False,
            "partial_score": False,
        }
        self.blackboard.pickup_verified = False
        return py_trees.common.Status.SUCCESS


class BtNode_UpdateChecklistFlag(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, checklist_key: str, flag: str, value: bool = True):
        super().__init__(name=name)
        self.flag = flag
        self.value = value
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="checklist",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(checklist_key),
        )
        self.blackboard.register_key(
            key="checklist_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(checklist_key),
        )

    def update(self) -> py_trees.common.Status:
        checklist = dict(self.blackboard.checklist or {})
        checklist[self.flag] = self.value
        self.blackboard.checklist_write = checklist
        return py_trees.common.Status.SUCCESS


class BtNode_MarkPickupVerified(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, checklist_key: str, pickup_verified_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="pickup_verified",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(pickup_verified_key),
        )
        self.blackboard.register_key(
            key="checklist",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(checklist_key),
        )
        self.blackboard.register_key(
            key="checklist_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(checklist_key),
        )

    def update(self) -> py_trees.common.Status:
        self.blackboard.pickup_verified = True
        checklist = dict(self.blackboard.checklist or {})
        checklist["pickup_verified"] = True
        self.blackboard.checklist_write = checklist
        return py_trees.common.Status.SUCCESS


class BtNode_RequirePickupVerified(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, pickup_verified_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="pickup_verified",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(pickup_verified_key),
        )

    def update(self) -> py_trees.common.Status:
        if bool(self.blackboard.pickup_verified):
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "Pickup not verified yet"
        return py_trees.common.Status.FAILURE


class BtNode_CloseActiveCustomer(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, queue_key: str, active_id_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="queue",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(queue_key),
        )
        self.blackboard.register_key(
            key="queue_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(queue_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )
        self.blackboard.register_key(
            key="active_id_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        active_id = self.blackboard.active_id
        queue = list(self.blackboard.queue or [])
        if active_id is None:
            return py_trees.common.Status.SUCCESS
        for item in queue:
            if item["id"] == active_id:
                item["status"] = "done"
        self.blackboard.queue_write = queue
        self.blackboard.active_id_write = None
        return py_trees.common.Status.SUCCESS
