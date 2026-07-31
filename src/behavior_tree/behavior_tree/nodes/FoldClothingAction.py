# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# FoldClothing Action Node Module
# ===============================
#
# Behavior-tree node that drives the *real* garment-folding action,
# ``tinker_arm_msgs/action/FoldClothing`` served on ``fold_clothing_action`` by
# ``arm_api/fold/fold_clothing_server.py`` (perceive -> plan folds -> execute).
#
# This is the manipulation "newest action" the Doing-Laundry task should use.
# It is a NEW node, separate from the legacy ``BtNode_FoldClothing``
# (``Manipulation.py``), which — despite its name — sends an EMPTY ``Fold.Goal()``
# to the older garment-agnostic ``fold_action`` and therefore cannot drive the
# real folder.
#
# Action contract (FoldClothing.action)
# --------------------------------------
#   Goal     : string garment_label, uint8 bottom_fold_mode (0 default/1 half/
#              2 third), bool return_to_scan
#   Result   : bool success, uint8 folds_completed, string message
#   Feedback : uint8 fold_index, uint8 total_folds, string stage
#
# The feedback is NON-CONFORMANT to the BT action protocol (no
# ``delay_limit``/``status`` fields) and the server's ``perceive`` stage can run
# for several seconds before the first feedback. This node therefore overrides
# ``feedback_callback`` (like ``BtNode_GotoAction`` / the audio action nodes) to
# force ``action_status=0`` and keep a wide ``feedback_timeout`` so the
# ``ActionHandler`` never aborts on the long perceive stage.
#
# Mock Mode
# ---------
# Registered under the ``manipulation`` subsystem in ``mock_config.json``
# (KEYPRESS). In mock mode no action client is created and no goal is sent — the
# node advances on the manipulation success key (or auto-completes when
# keyboard_control is disabled).
#

import time
from typing import Any, Optional

import py_trees
from py_trees.common import Status

from behavior_tree.interfaces.common import action_msgs
from behavior_tree.nodes.ActionBase import ActionHandler


def _import_fold_clothing_type():
    """Return the real ``FoldClothing`` action type, or ``None`` if unavailable.

    Imported locally (rather than via ``behavior_tree.interfaces.messages``) so this node
    file stays self-contained; ``messages.py`` does not export ``FoldClothing``.
    """
    try:
        from tinker_arm_msgs.action import FoldClothing
        return FoldClothing
    except Exception:
        return None


class BtNode_FoldClothingAction(ActionHandler):
    """Fold a garment via the real ``fold_clothing_action`` server.

    Args:
        name: Behaviour name.
        bb_key_garment_label: Optional blackboard key (root-scoped) to READ the
            garment label hint from at goal time. When present and truthy it
            overrides ``garment_label``.
        garment_label: Static fallback garment hint (e.g. ``"shirt"``);
            ``""`` means generic.
        bottom_fold_mode: 0 = server default, 1 = half, 2 = third.
        return_to_scan: End the action at the scan pose.
        action_name: Action server name (default ``"fold_clothing_action"``).
        wait_for_server_timeout_sec: Passed through to ``ActionHandler``.
    """

    def __init__(
        self,
        name: str,
        bb_key_garment_label: Optional[str] = None,
        garment_label: str = "",
        bottom_fold_mode: int = 0,
        return_to_scan: bool = True,
        action_name: str = "fold_clothing_action",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(
            name=name,
            action_type=None,            # resolved in setup() (real mode only)
            action_name=action_name,
            key=None,                    # we manage the blackboard ourselves
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        self.bb_key_garment_label = bb_key_garment_label
        self.garment_label = garment_label
        self.bottom_fold_mode = int(bottom_fold_mode)
        self.return_to_scan = bool(return_to_scan)

        self._fold_clothing_type = None
        self._bb_reader = None

        # Latest feedback snapshot (for the feedback message / debugging).
        self._fold_index = 0
        self._total_folds = 0
        self._stage = ""

    def setup(self, **kwargs):
        """Resolve the action type (real mode), then set up client + blackboard."""
        if not self.mock_mode:
            self._fold_clothing_type = _import_fold_clothing_type()
            if self._fold_clothing_type is None:
                raise RuntimeError(
                    "FoldClothing action type unavailable: build and source "
                    "tinker_arm_msgs, or run this node in mock mode "
                    "(manipulation subsystem mocked)."
                )
            self.action_type = self._fold_clothing_type

        super().setup(**kwargs)

        if self.bb_key_garment_label is not None:
            self._bb_reader = self.attach_blackboard_client(name=f"{self.name}_bb")
            self._bb_reader.register_key(
                key="garment_label",
                access=py_trees.common.Access.READ,
                remap_to=py_trees.blackboard.Blackboard.absolute_name(
                    "/", self.bb_key_garment_label
                ),
            )

    def _resolve_label(self) -> str:
        if self._bb_reader is not None:
            try:
                if self._bb_reader.exists("garment_label"):
                    value = self._bb_reader.garment_label
                    if value:
                        return str(value)
            except Exception:
                pass
        return self.garment_label or ""

    def send_goal(self):
        """Construct and dispatch a ``FoldClothing`` goal (mock-aware)."""
        if self.mock_mode:
            self.feedback_message = "MOCK: FoldClothing goal sent"

            class MockFuture:
                def done(self):
                    return True

            self.send_goal_future = MockFuture()
            return

        label = self._resolve_label()
        goal = self._fold_clothing_type.Goal()
        goal.garment_label = label
        goal.bottom_fold_mode = self.bottom_fold_mode
        goal.return_to_scan = self.return_to_scan
        self.send_goal_request(goal)
        self.feedback_message = (
            f"Sent FoldClothing goal (garment='{label or 'generic'}', "
            f"mode={self.bottom_fold_mode}, return_to_scan={self.return_to_scan})"
        )

    def feedback_callback(self, msg: Any):
        """Handle FoldClothing's non-conformant feedback (no delay_limit/status).

        Keeps the action 'healthy' (status OK, wide feedback timeout) so the
        ``ActionHandler`` never trips its missed-``delay_limit`` abort while the
        server perceives/plans. Does NOT call ``super()`` (which reads the
        missing ``delay_limit``/``status`` fields).
        """
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.action_status = 0
        self.feedback_timeout = max(self.feedback_timeout, 120.0)

        self._fold_index = int(getattr(feedback, "fold_index", 0))
        self._total_folds = int(getattr(feedback, "total_folds", 0))
        self._stage = str(getattr(feedback, "stage", ""))
        self.feedback_message = (
            f"folding [{self._stage}] {self._fold_index}/{self._total_folds}"
        )

    def process_result(self) -> Status:
        """Map the FoldClothing result using stable status with getattr fallbacks."""
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            result = getattr(self.result_message, "result", None)
            leg_s = bool(getattr(result, "success", True))
            st = int(getattr(result, "status", 0 if leg_s else 9))
            sg = int(getattr(result, "stage", 0))
            err = str(getattr(result, "error_msg", "missing error detail"))
            self.feedback_message = (
                f"FoldClothing failed (action status {self.result_status}): "
                f"result(status={st}, stage={sg}, error={err})"
            )
            return py_trees.common.Status.FAILURE

        result = getattr(self.result_message, "result", None)
        legacy_success = bool(getattr(result, "success", True))
        status = int(getattr(result, "status", 0 if legacy_success else 9))
        stage = int(getattr(result, "stage", 0))
        error = str(getattr(result, "error_msg", "missing error detail"))
        folds = int(getattr(result, "folds_completed", 0))
        message = getattr(result, "message", "") or ""
        succeeded = legacy_success and status == 0
        self.feedback_message = (
            f"FoldClothing {'succeeded' if succeeded else 'reported failure'}: "
            f"status={status}, stage={stage}, error={error}, "
            f"folds_completed={folds} {message}"
        ).rstrip()
        return (
            py_trees.common.Status.SUCCESS
            if succeeded
            else py_trees.common.Status.FAILURE
        )
