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


class BtNode_SelectNextQueuedCustomer(py_trees.behaviour.Behaviour):
    """Select oldest valid queued customer and expose it as active target.

    Previously named ``BtNode_SelectNextCustomer``. Renamed to disambiguate from
    ``Restaurant.custumNodes.BtNode_SelectNextCustomer`` (the list-of-orders
    variant used by ``restaurants_fake.py``).

    Optional ``picture_path_key`` — if set, also writes the selected queue
    entry's ``picture_path`` field (populated by ``BtNode_QueueWavingCandidates``)
    to that blackboard key. Entries without a ``picture_path`` field resolve to
    the empty string.
    """

    def __init__(
        self,
        name: str,
        *,
        queue_key: str,
        selected_pose_key: str,
        active_id_key: str,
        picture_path_key: str = None,
    ):
        super().__init__(name=name)
        self._picture_path_key = picture_path_key
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
        if picture_path_key is not None:
            self.blackboard.register_key(
                key="picture_path_write",
                access=py_trees.common.Access.WRITE,
                remap_to=_abs_key(picture_path_key),
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
        if self._picture_path_key is not None:
            self.blackboard.picture_path_write = selected.get("picture_path", "") or ""
        for item in queue:
            if item["id"] == selected["id"]:
                item["status"] = "active"
        self.blackboard.queue_write = queue
        self.feedback_message = f"Selected caller id={selected['id']}"
        return py_trees.common.Status.SUCCESS


class BtNode_QueueWavingCandidates(py_trees.behaviour.Behaviour):
    """Enqueue up to ``max_candidates`` waving persons (with matching picture paths).

    Reads two parallel lists populated by ``BtNode_ScanForWavingPerson``:
      - ``all_poses_key``    → list[PointStamped] (closest-first)
      - ``all_pictures_key`` → list[str] filesystem paths (may be shorter or empty)

    Each entry appended to ``queue_key`` has the shape::

        {id, pose, picture_path, timestamp, confidence, status="queued"}

    FAILURE if the pose list is empty or missing; SUCCESS otherwise.
    """

    def __init__(
        self,
        name: str,
        *,
        all_poses_key: str,
        all_pictures_key: str,
        queue_key: str,
        active_id_key: str,
        max_candidates: int = 2,
        confidence: float = 1.0,
    ):
        super().__init__(name=name)
        self.max_candidates = max_candidates
        self.confidence = confidence
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="all_poses",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(all_poses_key),
        )
        self.blackboard.register_key(
            key="all_pictures",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(all_pictures_key),
        )
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

    def update(self) -> py_trees.common.Status:
        try:
            poses = list(self.blackboard.all_poses or [])
        except Exception:
            poses = []
        if not poses:
            self.feedback_message = "No waving poses to queue"
            return py_trees.common.Status.FAILURE
        try:
            pictures = list(self.blackboard.all_pictures or [])
        except Exception:
            pictures = []

        queue: List[Dict[str, Any]] = list(self.blackboard.queue or [])
        next_id = 1 + max((item["id"] for item in queue), default=0)
        try:
            active_id = self.blackboard.active_id
        except Exception:
            active_id = None
        if isinstance(active_id, int):
            next_id = max(next_id, active_id + 1)

        added = 0
        now = time.time()
        for idx, pose in enumerate(poses[: self.max_candidates]):
            picture_path = pictures[idx] if idx < len(pictures) else ""
            queue.append(
                {
                    "id": next_id + added,
                    "pose": pose,
                    "picture_path": picture_path or "",
                    "timestamp": now + added * 1e-6,
                    "confidence": float(self.confidence),
                    "status": "queued",
                }
            )
            added += 1
        queue.sort(key=lambda item: (item["timestamp"], item["id"]))
        self.blackboard.queue_write = queue
        self.feedback_message = f"Queued {added} waving candidate(s)"
        return py_trees.common.Status.SUCCESS


class BtNode_SplitOrderItems(py_trees.behaviour.Behaviour):
    """Convert the raw customer order into a list of individual items.

    ``BtNode_TakeOrder`` writes a string to ``KEY_CUSTOMER_ORDER``; this node
    wraps it in a single-element list (or passes through a list if already one).
    Writes to ``items_key``. Always SUCCESS.
    """

    def __init__(
        self,
        name: str,
        *,
        order_key: str,
        items_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="order",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(order_key),
        )
        self.blackboard.register_key(
            key="items_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(items_key),
        )

    def update(self) -> py_trees.common.Status:
        try:
            raw = self.blackboard.order
        except Exception:
            raw = None
        if raw is None or raw == "":
            items: List[str] = []
        elif isinstance(raw, str):
            items = [raw]
        else:
            items = list(raw)
        self.blackboard.items_write = items
        self.feedback_message = f"Split order into {len(items)} item(s)"
        return py_trees.common.Status.SUCCESS


class BtNode_PopNextItem(py_trees.behaviour.Behaviour):
    """Pop the first item off ``items_key`` and publish current-item fields.

    Writes:
      - ``current_item_key``         ← the popped item (str)
      - ``current_item_summary_key`` ← "<item> for customer <active_id>"

    FAILURE when the list is empty (parent loop terminates).
    """

    def __init__(
        self,
        name: str,
        *,
        items_key: str,
        current_item_key: str,
        current_item_summary_key: str,
        active_id_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="items_read",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(items_key),
        )
        self.blackboard.register_key(
            key="items_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(items_key),
        )
        self.blackboard.register_key(
            key="current_item",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(current_item_key),
        )
        self.blackboard.register_key(
            key="current_item_summary",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(current_item_summary_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        try:
            items = list(self.blackboard.items_read or [])
        except Exception:
            items = []
        if not items:
            self.feedback_message = "No more items"
            return py_trees.common.Status.FAILURE
        current = items.pop(0)
        try:
            active_id = self.blackboard.active_id
        except Exception:
            active_id = None
        self.blackboard.items_write = items
        self.blackboard.current_item = current
        self.blackboard.current_item_summary = (
            f"{current} for customer {active_id}"
            if active_id is not None
            else f"{current}"
        )
        self.feedback_message = f"Popped item: {current}"
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


class BtNode_InitCustomerBatch(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        *,
        batch_key: str,
        batch_size_limit_key: str,
        current_index_key: str,
        default_limit: int,
        summary_key: str,
    ):
        super().__init__(name=name)
        self.default_limit = int(default_limit)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="batch_size_limit",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(batch_size_limit_key),
        )
        self.blackboard.register_key(
            key="current_index",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(current_index_key),
        )
        self.blackboard.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(summary_key),
        )

    def update(self) -> py_trees.common.Status:
        self.blackboard.batch = []
        self.blackboard.batch_size_limit = self.default_limit
        self.blackboard.current_index = 0
        self.blackboard.summary = ""
        return py_trees.common.Status.SUCCESS


class BtNode_AddDetectedCustomerToBatch(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        *,
        batch_key: str,
        source_pose_key: str,
        batch_size_limit_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="batch_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="pose",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(source_pose_key),
        )
        self.blackboard.register_key(
            key="batch_size_limit",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_size_limit_key),
        )

    def update(self) -> py_trees.common.Status:
        pose = self.blackboard.pose
        if pose is None:
            self.feedback_message = "No detected customer pose to add"
            return py_trees.common.Status.FAILURE

        batch: List[Dict[str, Any]] = list(self.blackboard.batch or [])
        limit = int(self.blackboard.batch_size_limit or 0)
        if limit > 0 and len(batch) >= limit:
            self.feedback_message = "Batch already full"
            return py_trees.common.Status.FAILURE

        next_id = 1 + max((item["id"] for item in batch), default=0)
        batch.append(
            {
                "id": next_id,
                "pose": pose,
                "order": "",
                "status": "detected",
                "detected_at": time.time(),
            }
        )
        self.blackboard.batch_write = batch
        self.feedback_message = f"Added customer {next_id} to batch"
        return py_trees.common.Status.SUCCESS


class BtNode_SelectBatchCustomerByIndex(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        *,
        batch_key: str,
        current_index_key: str,
        customer_pose_key: str,
        active_id_key: str,
        customer_order_key: str,
        require_order: bool = False,
        require_unserved: bool = False,
    ):
        super().__init__(name=name)
        self.require_order = require_order
        self.require_unserved = require_unserved
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="current_index",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(current_index_key),
        )
        self.blackboard.register_key(
            key="customer_pose",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(customer_pose_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(active_id_key),
        )
        self.blackboard.register_key(
            key="customer_order",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(customer_order_key),
        )

    def update(self) -> py_trees.common.Status:
        batch = list(self.blackboard.batch or [])
        idx = int(self.blackboard.current_index or 0)
        if idx < 0 or idx >= len(batch):
            self.feedback_message = "Batch index out of range"
            return py_trees.common.Status.FAILURE

        customer = batch[idx]
        if self.require_order and not customer.get("order"):
            self.feedback_message = f"Customer {customer['id']} has no order"
            return py_trees.common.Status.FAILURE
        if self.require_unserved and customer.get("status") == "served":
            self.feedback_message = f"Customer {customer['id']} already served"
            return py_trees.common.Status.FAILURE

        self.blackboard.customer_pose = customer.get("pose")
        self.blackboard.active_id = customer["id"]
        self.blackboard.customer_order = customer.get("order", "")
        self.feedback_message = f"Selected batch customer {customer['id']}"
        return py_trees.common.Status.SUCCESS


class BtNode_StoreOrderForActiveBatchCustomer(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        *,
        batch_key: str,
        active_id_key: str,
        customer_order_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="batch_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )
        self.blackboard.register_key(
            key="customer_order",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(customer_order_key),
        )

    def update(self) -> py_trees.common.Status:
        active_id = self.blackboard.active_id
        order = (self.blackboard.customer_order or "").strip()
        if active_id is None:
            return py_trees.common.Status.FAILURE
        if not order:
            return py_trees.common.Status.FAILURE

        batch = list(self.blackboard.batch or [])
        updated = False
        for customer in batch:
            if customer["id"] == active_id:
                customer["order"] = order
                customer["status"] = "ordered"
                updated = True
                break
        if not updated:
            return py_trees.common.Status.FAILURE
        self.blackboard.batch_write = batch
        self.feedback_message = f"Stored order for customer {active_id}"
        return py_trees.common.Status.SUCCESS


class BtNode_AdvanceBatchIndex(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, current_index_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="current_index",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(current_index_key),
        )
        self.blackboard.register_key(
            key="current_index_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(current_index_key),
        )

    def update(self) -> py_trees.common.Status:
        self.blackboard.current_index_write = int(self.blackboard.current_index or 0) + 1
        return py_trees.common.Status.SUCCESS


class BtNode_ResetBatchIndex(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, current_index_key: str, value: int = 0):
        super().__init__(name=name)
        self.value = int(value)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="current_index",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(current_index_key),
        )

    def update(self) -> py_trees.common.Status:
        self.blackboard.current_index = self.value
        return py_trees.common.Status.SUCCESS


class BtNode_BuildBatchOrdersSummary(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, batch_key: str, summary_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(summary_key),
        )

    def update(self) -> py_trees.common.Status:
        batch = list(self.blackboard.batch or [])
        ordered = [item for item in batch if item.get("order")]
        if not ordered:
            self.blackboard.summary = ""
            return py_trees.common.Status.FAILURE
        text = "; ".join(
            [f"customer {item['id']}: {item['order']}" for item in ordered]
        )
        self.blackboard.summary = text
        self.feedback_message = text
        return py_trees.common.Status.SUCCESS


class BtNode_MarkBatchCustomerServed(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, batch_key: str, active_id_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="batch_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        active_id = self.blackboard.active_id
        if active_id is None:
            return py_trees.common.Status.FAILURE
        batch = list(self.blackboard.batch or [])
        for customer in batch:
            if customer["id"] == active_id:
                customer["status"] = "served"
                self.blackboard.batch_write = batch
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_HasUnservedBatchCustomers(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, batch_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )

    def update(self) -> py_trees.common.Status:
        batch = list(self.blackboard.batch or [])
        has_unserved = any(
            item.get("order") and item.get("status") != "served" for item in batch
        )
        if has_unserved:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_SetActiveBatchCustomerStatus(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name: str,
        *,
        batch_key: str,
        active_id_key: str,
        status: str,
    ):
        super().__init__(name=name)
        self.status = status
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="batch_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        active_id = self.blackboard.active_id
        if active_id is None:
            return py_trees.common.Status.FAILURE
        batch = list(self.blackboard.batch or [])
        for customer in batch:
            if customer["id"] == active_id:
                customer["status"] = self.status
                self.blackboard.batch_write = batch
                return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class BtNode_CurrentBatchCustomerHasOrder(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, batch_key: str, active_id_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        active_id = self.blackboard.active_id
        batch = list(self.blackboard.batch or [])
        for customer in batch:
            if customer["id"] == active_id:
                if customer.get("order"):
                    return py_trees.common.Status.SUCCESS
                return py_trees.common.Status.FAILURE
        return py_trees.common.Status.FAILURE


class BtNode_CurrentBatchCustomerNeedsService(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, *, batch_key: str, active_id_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="batch",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(batch_key),
        )
        self.blackboard.register_key(
            key="active_id",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(active_id_key),
        )

    def update(self) -> py_trees.common.Status:
        active_id = self.blackboard.active_id
        batch = list(self.blackboard.batch or [])
        for customer in batch:
            if customer["id"] == active_id:
                needs = bool(customer.get("order")) and customer.get("status") != "served"
                return (
                    py_trees.common.Status.SUCCESS
                    if needs
                    else py_trees.common.Status.FAILURE
                )
        return py_trees.common.Status.FAILURE
