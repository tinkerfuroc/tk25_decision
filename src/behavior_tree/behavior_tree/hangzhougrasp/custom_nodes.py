"""Task-local adapters; ROS primitives stay in TemplateNodes."""

import copy

import py_trees

from .config import KEY_HOLDING


class BtNode_SelectGraspTarget(py_trees.behaviour.Behaviour):
    """Keep the nearest object's mask and label together in a grasp response."""

    def __init__(self, name, bb_source, bb_key, label_key):
        super().__init__(name)
        self.bb_source, self.bb_key, self.label_key = bb_source, bb_key, label_key
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(bb_source, access=py_trees.common.Access.READ)
        self.blackboard.register_key(bb_key, access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(label_key, access=py_trees.common.Access.WRITE)

    def update(self):
        # Failed retries must never leave a usable old grasp bundle.
        self.blackboard.set(self.bb_key, None, overwrite=True)
        self.blackboard.set(self.label_key, "", overwrite=True)
        try:
            result = self.blackboard.get(self.bb_source)
            if result.status != 0 or not result.objects:
                raise ValueError("no successful detection to grasp")
            if len(result.segments) != len(result.objects):
                raise ValueError("object/mask count mismatch")
            rgb, depth, mask = result.rgb_image, result.depth_image, result.segments[0]
            shape = (rgb.height, rgb.width)
            if min(shape) <= 0 or not rgb.data or not depth.data or not mask.data:
                raise ValueError("grasp requires RGB, depth and a nonempty mask")
            if (depth.height, depth.width) != shape or (mask.height, mask.width) != shape:
                raise ValueError("RGB/depth/mask dimensions must match")
            if not any(mask.data):
                raise ValueError("selected mask has no foreground")
            if not result.header.frame_id:
                raise ValueError("grasp requires a camera frame_id")
            selected = copy.deepcopy(result)
            # sort_closest=True on the scanner guarantees nearest-first order.
            selected.objects = [selected.objects[0]]
            selected.segments = [selected.segments[0]]
            self.blackboard.set(self.bb_key, selected, overwrite=True)
            self.blackboard.set(self.label_key, selected.objects[0].cls, overwrite=True)
        except (AttributeError, IndexError, KeyError, TypeError, ValueError) as exc:
            self.feedback_message = f"Cannot select grasp target: {exc}"
            return py_trees.common.Status.FAILURE
        self.feedback_message = f"Selected nearest {selected.objects[0].cls}"
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
