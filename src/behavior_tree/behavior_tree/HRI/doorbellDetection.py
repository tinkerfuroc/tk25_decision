"""Doorbell-detection BT leaf wrapping tk_24_audio's `doorbell_action`."""

import time

from py_trees.common import Status

from behavior_tree.TemplateNodes.ActionBase import ActionHandler
from behavior_tree.TemplateNodes.Audio import _MockFuture
from behavior_tree.messages import Doorbell, action_msgs


class BtNode_DoorbellDetection(ActionHandler):
    """
    Action-client wrapper for tk_24_audio's `doorbell_action`
    (`tinker_audio_msgs/action/Doorbell`). SUCCESS only when the action
    completes with `result.status == 0` (doorbell heard).

    Server-side semantics (`audio_pakage/doorbell_ac.py`):
      * Goal is empty; server uses `LISTEN_TIMEOUT = 10.0` s.
      * Result codes: 0 = heard, 1 = timeout, 2 = error/cancelled.
      * Feedback is `{elapsed, partial_transcription}` — non-canonical
        (no `{stage, stage_name, status, delay_limit}`) — and is published
        only ONCE, immediately before `goal_handle.succeed()`. The timeout
        and error paths publish no feedback at all.

    Because feedback is single-shot, the BT-side `feedback_timeout` must
    cover the full server listen window with margin so we don't abort the
    silent-listening period as a stalled action. Same override pattern as
    `BtNode_ListenAction` / `BtNode_GetConfirmationAction`.
    """

    def __init__(
        self,
        name: str = "Detect doorbell",
        action_name: str = "doorbell_action",
        wait_for_server_timeout_sec: float = -3.0,
    ):
        super().__init__(
            name,
            Doorbell,
            action_name,
            key=None,
            wait_for_server_timeout_sec=wait_for_server_timeout_sec,
        )
        self._feedback_timeout_secs = 30.0

    def send_goal(self):
        if self.mock_mode:
            self.feedback_message = "MOCK: Doorbell goal sent"
            self.send_goal_future = _MockFuture()
            return
        goal = Doorbell.Goal()
        self.send_goal_request(goal)
        self.feedback_message = "sent Doorbell goal (server timeout=10s)"

    def feedback_callback(self, msg):
        feedback = msg.feedback
        self.last_feedback_time = time.time()
        self.feedback_timeout = self._feedback_timeout_secs
        self.action_status = 0
        elapsed = getattr(feedback, "elapsed", 0.0)
        partial = getattr(feedback, "partial_transcription", "")
        self.feedback_message = f"doorbell elapsed={elapsed:.2f}" + (
            f" [partial: '{partial}']" if partial else ""
        )

    def process_result(self):
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            err = getattr(
                getattr(self.result_message, "result", None), "error_message", ""
            )
            self.feedback_message = (
                f"Doorbell action did not succeed "
                f"({self.result_status_string}): {err}"
            )
            return Status.FAILURE
        result = self.result_message.result
        if getattr(result, "status", 0) != 0:
            self.feedback_message = (
                f"Doorbell failed code={result.status}: "
                f"{getattr(result, 'error_message', '')}"
            )
            return Status.FAILURE
        self.feedback_message = (
            f"Heard doorbell: {getattr(result, 'transcription', '')}"
        )
        return Status.SUCCESS
