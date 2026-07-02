from __future__ import annotations

"""State and policy helper nodes for the DoingLaundry main task."""

import time
from typing import Any

import py_trees

from behavior_tree.config import is_node_mocked


def _abs_key(key: str) -> str:
    return py_trees.blackboard.Blackboard.absolute_name("/", key)


class BtNode_InitTaskState(py_trees.behaviour.Behaviour):
    """Reset score trace, fold/stack counters, and runtime deadline."""

    def __init__(
        self,
        name: str,
        *,
        score_trace_key: str,
        phase_deadline_key: str,
        max_runtime_key: str,
        fold_count_key: str,
        stack_count_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        bindings = {
            "score":       (score_trace_key,    py_trees.common.Access.WRITE),
            "deadline":    (phase_deadline_key, py_trees.common.Access.WRITE),
            "max_runtime": (max_runtime_key,    py_trees.common.Access.READ),
            "fold_count":  (fold_count_key,     py_trees.common.Access.WRITE),
            "stack_count": (stack_count_key,    py_trees.common.Access.WRITE),
        }
        for local, (bb_key, access) in bindings.items():
            self.blackboard.register_key(key=local, access=access, remap_to=_abs_key(bb_key))

    def update(self) -> py_trees.common.Status:
        max_runtime = self.blackboard.max_runtime
        if not isinstance(max_runtime, (int, float)):
            max_runtime = 390.0
        self.blackboard.score = []
        self.blackboard.deadline = time.time() + float(max_runtime)
        self.blackboard.fold_count = 0
        self.blackboard.stack_count = 0
        return py_trees.common.Status.SUCCESS


class BtNode_FoldClothing(py_trees.behaviour.Behaviour):
    """Integration seam for `fold_clothing_action`.

    Real mode returns FAILURE while the action server is absent, so the
    surrounding Selector falls back to the operator-assisted branch.
    Mock mode succeeds after 2 ticks.
    """

    def __init__(self, name: str, *, bb_key_target_point: str, fold_cycles: int = 3):
        super().__init__(name=name)
        self.fold_cycles = fold_cycles
        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="target_point",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(bb_key_target_point),
        )
        self._mock_ticks = 0

    def initialise(self) -> None:
        self._mock_ticks = 0

    def update(self) -> py_trees.common.Status:
        if self.mock_mode:
            self._mock_ticks += 1
            if self._mock_ticks >= 2:
                target = self.blackboard.target_point
                print(f"🧺 MOCK FOLD: target {getattr(target, 'point', target)}")
                self.feedback_message = "MOCK fold succeeded"
                return py_trees.common.Status.SUCCESS
            self.feedback_message = "MOCK fold running..."
            return py_trees.common.Status.RUNNING
        # Real mode: action server does not exist yet → FAIL so Selector falls
        # through to the operator-assisted fold path.
        self.feedback_message = "fold_clothing_action server not available; deferring to operator help"
        return py_trees.common.Status.FAILURE


class BtNode_RecordCompletion(py_trees.behaviour.Behaviour):
    """Append `{action, points, success, timestamp}` to the score trace."""

    def __init__(
        self,
        name: str,
        *,
        score_trace_key: str,
        action_label: str,
        points: int,
        success: bool,
    ):
        super().__init__(name=name)
        self.action_label = str(action_label)
        self.points = int(points)
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

    def update(self) -> py_trees.common.Status:
        score = list(self.blackboard.score or [])
        score.append(
            {
                "action": self.action_label,
                "points": self.points,
                "success": self.success,
                "timestamp": time.time(),
            }
        )
        self.blackboard.score_write = score
        self.feedback_message = f"+{self.points} ({self.action_label})" if self.success else f"0 ({self.action_label} skipped)"
        return py_trees.common.Status.SUCCESS


class BtNode_IncrementCounter(py_trees.behaviour.Behaviour):
    """Increment a single integer blackboard counter by 1."""

    def __init__(self, name: str, *, counter_key: str):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="value",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(counter_key),
        )
        self.blackboard.register_key(
            key="value_write",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(counter_key),
        )

    def update(self) -> py_trees.common.Status:
        cur = self.blackboard.value
        try:
            cur_int = int(cur)
        except (TypeError, ValueError):
            cur_int = 0
        self.blackboard.value_write = cur_int + 1
        self.feedback_message = f"counter -> {cur_int + 1}"
        return py_trees.common.Status.SUCCESS


class BtNode_TimeoutCutoverChecker(py_trees.behaviour.Behaviour):
    """Condition: deadline not yet exceeded (FAILURE = stop the loop)."""

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
    """Build a concise completion message from the score trace + counters."""

    def __init__(
        self,
        name: str,
        *,
        score_trace_key: str,
        fold_count_key: str,
        stack_count_key: str,
        summary_key: str,
    ):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="score",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(score_trace_key),
        )
        self.blackboard.register_key(
            key="folds",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(fold_count_key),
        )
        self.blackboard.register_key(
            key="stacks",
            access=py_trees.common.Access.READ,
            remap_to=_abs_key(stack_count_key),
        )
        self.blackboard.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=_abs_key(summary_key),
        )

    def update(self) -> py_trees.common.Status:
        score = list(self.blackboard.score or [])
        total = sum(int(item.get("points", 0)) for item in score if item.get("success"))
        folds = int(self.blackboard.folds or 0)
        stacks = int(self.blackboard.stacks or 0)
        if not score:
            self.blackboard.summary = "Doing laundry complete. No actions recorded."
        else:
            self.blackboard.summary = (
                f"Doing laundry complete. Estimated {total} points. "
                f"Folded {folds} item(s), stacked {stacks}."
            )
        return py_trees.common.Status.SUCCESS
