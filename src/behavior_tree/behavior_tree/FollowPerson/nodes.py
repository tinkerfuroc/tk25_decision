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
# Follow-person BT nodes
# ======================
#
# Non-blocking ``py_trees`` behaviour used by the follow-person tree:
#
# - ``BtNode_ReacqAnnounce``: reads the tracker's reacquisition state from the
#   blackboard and speaks the right phrase (slow-down while PASSIVE, raise-hand
#   while NEEDS_HELP) on state transitions and on a throttled repeat, using a
#   CoalescingTTS so speech never overlaps and never blocks the tick.
#
# It always returns SUCCESS so it never gates the tree, and exposes an injection
# hook (``inject_coalescer``) so unit tests can substitute a fake without a live
# ROS graph.
#
# Navigation is no longer driven from here: the follow executive
# (``BtNode_FollowAction`` -> ``follow_server``) consumes the tracker's
# ``/target_points`` topic directly, so the former ``BtNode_PublishFollowGoal``
# (which republished to ``/follow_target``) has been removed.
#

import time

import py_trees
from py_trees.common import Status

from behavior_tree.config import is_node_mocked

# Reacquisition-state codes (mirror TrackPerson action feedback).
REACQ_TRACKING = 0
REACQ_PASSIVE = 1
REACQ_NEEDS_HELP = 2

DEFAULT_PASSIVE_TEXT = "Please slow down so I can keep up."
DEFAULT_NEEDS_HELP_TEXT = "I've lost you. Please raise your hand."


class _DoneHandle:
    """Already-done completion handle used by the mock TTS start callback."""

    @staticmethod
    def done():
        return True


class BtNode_ReacqAnnounce(py_trees.behaviour.Behaviour):
    """Announce reacquisition guidance based on the tracker's state.

    Reads ``bb_key`` (an int reacquisition state) from the blackboard each tick
    and submits an utterance to a non-blocking CoalescingTTS:

    - PASSIVE (1): ``passive_text`` (ask the person to slow down).
    - NEEDS_HELP (2): ``needs_help_text`` (ask the person to raise their hand).
    - TRACKING (0): nothing; resets the last-announced state so re-entering a
      help state announces immediately.

    Fires on a state transition OR on a throttled repeat (``throttle_s``) while
    still in a help state. Always returns SUCCESS (never gates the tree).
    """

    def __init__(
        self,
        name: str,
        passive_text: str = DEFAULT_PASSIVE_TEXT,
        needs_help_text: str = DEFAULT_NEEDS_HELP_TEXT,
        throttle_s: float = 5.0,
        bb_key: str = "track/reacquisition_state",
        service_name: str = "announce",
        clock=time.monotonic,
    ):
        """Initialize the announcer.

        Args:
            name: Behaviour tree node name.
            passive_text: Phrase spoken while PASSIVE-reacquiring.
            needs_help_text: Phrase spoken while NEEDS_HELP.
            throttle_s: Minimum seconds between repeat announcements in a state.
            bb_key: Blackboard key holding the reacquisition state (int).
            service_name: TextToSpeech service name.
            clock: Monotonic clock callable returning seconds (injectable).
        """
        super().__init__(name=name)
        self.passive_text = passive_text
        self.needs_help_text = needs_help_text
        self.throttle_s = throttle_s
        self.bb_key = bb_key
        self.service_name = service_name
        self._clock = clock

        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self._tts = None

        # Last-announced help state (None = nothing announced / reset) and the
        # clock time of that announcement.
        self._last_state = None
        self._last_announce_time = 0.0

        # Register the reacq-state key for READ so update() can read it even
        # when setup() has not run (unit tests).
        self._bb = self.attach_blackboard_client(name=f"{self.name}_reacq")
        self._bb.register_key(
            key=self.bb_key, access=py_trees.common.Access.READ
        )

    def inject_coalescer(self, coalescer):
        """Inject a CoalescingTTS (or fake) directly, bypassing ROS setup."""
        self._tts = coalescer

    def setup(self, **kwargs):
        """Obtain the shared rclpy node and build the CoalescingTTS speaker."""
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        # If a coalescer was already injected (tests), keep it.
        if self._tts is not None:
            return

        from behavior_tree.FollowPerson.coalescing_tts import CoalescingTTS
        from behavior_tree.messages import TextToSpeech

        if self.mock_mode:
            def start(text):
                self.logger.info(f"MOCK TTS: '{text}'")
                print(f"🔊 MOCK REACQ ANNOUNCE: {text}")
                return _DoneHandle()

            self._tts = CoalescingTTS(start=start, is_done=lambda h: h.done())
            return

        client = self.node.create_client(TextToSpeech, self.service_name)

        def start(text):
            req = TextToSpeech.Request()
            req.text = text
            return client.call_async(req)

        self._client = client
        self._tts = CoalescingTTS(start=start, is_done=lambda f: f.done())

    def update(self) -> Status:
        """Advance speech and announce on transition / throttled repeat."""
        if self._tts is not None:
            self._tts.poll()

        try:
            state = int(self._bb.get(self.bb_key))
        except Exception:
            state = REACQ_TRACKING

        now = self._clock()

        if state == REACQ_PASSIVE:
            text = self.passive_text
        elif state == REACQ_NEEDS_HELP:
            text = self.needs_help_text
        else:
            text = None

        if text is None:
            # TRACKING (or unknown): reset so re-entry announces immediately.
            self._last_state = None
            self.feedback_message = "Tracking; no announcement"
            return Status.SUCCESS

        transitioned = state != self._last_state
        throttled = (now - self._last_announce_time) >= self.throttle_s

        if transitioned or throttled:
            if self._tts is not None:
                self._tts.submit(text)
            self._last_state = state
            self._last_announce_time = now
            self.feedback_message = f"Announced: '{text}'"
        else:
            self.feedback_message = f"Holding announcement for state {state}"

        return Status.SUCCESS
