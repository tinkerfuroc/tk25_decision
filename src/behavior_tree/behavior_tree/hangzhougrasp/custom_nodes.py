"""Task-local adapters; ROS primitives stay in TemplateNodes."""

import math
import time

import py_trees

from behavior_tree.TemplateNodes.Vision import BtNode_TurnPanTilt

from .config import KEY_HOLDING


class BtNode_HeadTilt(BtNode_TurnPanTilt):
    """Absolute pan/tilt for the Orbbec head that only succeeds when settled.

    The base leaf reports SUCCESS on a settle timeout; here the head must
    report fresh, in-tolerance feedback after the command, else FAILURE
    (modelled on HomeServiceHangzhou.ros_nodes.SettledPanTilt).
    """
    mock_config_name = "BtNode_TurnPanTilt"

    def initialise(self):
        self._feedback_received = 0.0
        self._command_at = time.monotonic()
        super().initialise()

    def _state_cb(self, message):
        super()._state_cb(message)
        self._feedback_received = time.monotonic()

    def update(self):
        result = super().update()
        if result != py_trees.common.Status.SUCCESS or self.mock_mode:
            return result
        state = self._latest_state
        if (state is None or not state.feedback_ok
                or self._feedback_received < self._command_at
                or time.monotonic() - self._feedback_received > 0.5
                or abs(math.degrees(state.pan_rad) - self.x) >= self.SETTLE_EPS_DEG
                or abs(math.degrees(state.tilt_rad) - self.y) >= self.SETTLE_EPS_DEG):
            self.feedback_message = (
                f"Head did not settle at pan {self.x} tilt {self.y}")
            if time.time() <= self._settle_deadline:
                return py_trees.common.Status.RUNNING
            return py_trees.common.Status.FAILURE
        self.feedback_message = f"Head at pan {self.x} tilt {self.y}"
        return py_trees.common.Status.SUCCESS


class BtNode_SetHolding(py_trees.behaviour.Behaviour):
    """Record whether the gripper currently holds an object."""

    def __init__(self, name, value):
        super().__init__(name)
        if type(value) is not bool:
            raise TypeError("value must be a boolean")
        self.value = value
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(KEY_HOLDING, access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.set(KEY_HOLDING, self.value, overwrite=True)
        return py_trees.common.Status.SUCCESS


def _scan_objects(blackboard, key):
    """Objects of a successful detection response at key, else an empty list."""
    try:
        result = blackboard.get(key)
    except KeyError:
        return []
    if result is None or getattr(result, "status", 1) != 0:
        return []
    return list(getattr(result, "objects", None) or [])


class BtNode_ClearKey(py_trees.behaviour.Behaviour):
    """Write None to a key, so a later reader never sees a stale value."""

    def __init__(self, name, key):
        super().__init__(name)
        self.key = key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key, access=py_trees.common.Access.WRITE)

    def update(self):
        self.blackboard.set(self.key, None, overwrite=True)
        return py_trees.common.Status.SUCCESS


class BtNode_ScanSawObjects(py_trees.behaviour.Behaviour):
    """SUCCESS iff the detection response at bb_source has status 0 and objects."""

    def __init__(self, name, bb_source):
        super().__init__(name)
        self.bb_source = bb_source
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(bb_source, access=py_trees.common.Access.READ)

    def update(self):
        if _scan_objects(self.blackboard, self.bb_source):
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "Scan saw no objects"
        return py_trees.common.Status.FAILURE


class BtNode_SelectNearestLabel(py_trees.behaviour.Behaviour):
    """Write the first (nearest, sort_closest) detection's label, stripped."""

    def __init__(self, name, bb_source, label_key):
        super().__init__(name)
        self.bb_source, self.label_key = bb_source, label_key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(bb_source, access=py_trees.common.Access.READ)
        self.blackboard.register_key(label_key, access=py_trees.common.Access.WRITE)

    def update(self):
        # A failed selection must never leave a usable old label behind.
        self.blackboard.set(self.label_key, "", overwrite=True)
        objects = _scan_objects(self.blackboard, self.bb_source)
        label = getattr(objects[0], "cls", "") if objects else ""
        label = label.strip() if isinstance(label, str) else ""
        if not label:
            self.feedback_message = "No labelled detection to pick"
            return py_trees.common.Status.FAILURE
        self.blackboard.set(self.label_key, label, overwrite=True)
        self.feedback_message = f"Selected nearest {label}"
        return py_trees.common.Status.SUCCESS


class BtNode_LogError(py_trees.behaviour.Behaviour):
    """Log an ERROR and succeed, so a non-essential failure stays visible."""

    def __init__(self, name, message):
        super().__init__(name)
        self.message = message

    def update(self):
        self.logger.error(self.message)
        self.feedback_message = self.message
        return py_trees.common.Status.SUCCESS
