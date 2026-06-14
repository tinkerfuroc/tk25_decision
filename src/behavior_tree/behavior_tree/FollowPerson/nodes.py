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


class _WaveReseedBridge:
    """Live ROS bridge for ``BtNode_WaveReseed``: async DetectWaving + ReseedTarget.

    Hides the tk26 message types from the node so the node logic stays ROS-free
    and unit-testable (a fake bridge is injected in tests). ``detect_async``
    returns an rclpy Future of a ``DetectWaving.Response`` (read ``.status`` +
    ``.waving_boxes``); ``reseed_async(box)`` returns a Future of a
    ``ReseedTarget.Response`` (read ``.success``). ``box`` is ``(x0, y0, x1, y1)``
    in the current color frame.
    """

    def __init__(self, node, waving_service, reseed_service):
        # Imported lazily (live path only) so mock/test never need tk26 msgs.
        from behavior_tree.messages import DetectWaving, ReseedTarget
        self._DetectWaving = DetectWaving
        self._ReseedTarget = ReseedTarget
        self._wave_cli = node.create_client(DetectWaving, waving_service)
        self._reseed_cli = node.create_client(ReseedTarget, reseed_service)

    def detect_async(self):
        # min_waving_persons=0 (default) -> fast MediaPipe-only path on the waving
        # server (no VLM fallback), keeping the call well inside the vision budget.
        return self._wave_cli.call_async(self._DetectWaving.Request())

    def reseed_async(self, box):
        req = self._ReseedTarget.Request()
        x0, y0, x1, y1 = box
        req.bbox.x_offset = max(0, int(x0))
        req.bbox.y_offset = max(0, int(y0))
        req.bbox.width = max(0, int(x1 - x0))
        req.bbox.height = max(0, int(y1 - y0))
        req.frame_id = ""
        return self._reseed_cli.call_async(req)


class BtNode_WaveReseed(py_trees.behaviour.Behaviour):
    """Wave-to-reseed recovery while the tracker is latched in NEEDS_HELP.

    When the tracker can no longer auto re-acquire it escalates to NEEDS_HELP
    (``reacquisition_state == 2``) and ``BtNode_ReacqAnnounce`` asks the operator
    to raise a hand. This node closes that loop: while in NEEDS_HELP it polls the
    waving detector (``detect_waving_persons``) and, on an UNAMBIGUOUS single
    waver, reseeds the tracker onto that person's box (``~/reseed_target``) — the
    manual escape from the indefinite NEEDS_HELP hold. Zero or multiple wavers ->
    NO reseed (precision: never re-lock onto an ambiguous candidate; keep
    announcing). Mirrors the proven ``track_web`` dashboard wave-to-resume policy.

    Non-blocking: at most one service call is in flight, driven across ticks by a
    small IDLE -> WAVE_PENDING -> RESEED_PENDING state machine, with DetectWaving
    throttled to ``throttle_s``. ALWAYS returns SUCCESS (never gates the tree), so
    it composes with the never-mid-abort ``SuccessOnAll`` Parallel. Leaving
    NEEDS_HELP cancels any in-flight cycle. In mock mode (or when the waving
    bridge is unavailable) it is an inert no-op.
    """

    _IDLE, _WAVE_PENDING, _RESEED_PENDING = "idle", "wave_pending", "reseed_pending"

    def __init__(
        self,
        name: str,
        throttle_s: float = 3.0,
        bb_key: str = "track/reacquisition_state",
        waving_service: str = "detect_waving_persons",
        reseed_service: str = "/person_track_node/reseed_target",
        clock=time.monotonic,
    ):
        """Initialize the wave-reseed reaction.

        Args:
            name: Behaviour tree node name.
            throttle_s: Minimum seconds between DetectWaving scans while in
                NEEDS_HELP (measured from the start of the previous scan).
            bb_key: Blackboard key holding the reacquisition state (int); the
                same key ``BtNode_ReacqAnnounce`` reads.
            waving_service: DetectWaving service name.
            reseed_service: ReseedTarget service name (the tracker's
                ``~/reseed_target``; default assumes node name ``person_track_node``).
            clock: Monotonic clock callable returning seconds (injectable).
        """
        super().__init__(name=name)
        self.throttle_s = throttle_s
        self.bb_key = bb_key
        self.waving_service = waving_service
        self.reseed_service = reseed_service
        self._clock = clock

        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self._bridge = None

        # Async state machine.
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._last_detect_time = -1e9

        self._bb = self.attach_blackboard_client(name=f"{self.name}_reacq")
        self._bb.register_key(
            key=self.bb_key, access=py_trees.common.Access.READ
        )

    def inject_bridge(self, bridge):
        """Inject a wave/reseed bridge (or fake), bypassing ROS setup."""
        self._bridge = bridge

    def setup(self, **kwargs):
        """Obtain the shared rclpy node and build the live wave/reseed bridge."""
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        # A bridge already injected (tests) wins; mock mode stays bridge-less so
        # update() no-ops the service calls.
        if self._bridge is not None or self.mock_mode:
            return
        self._bridge = _WaveReseedBridge(
            self.node, self.waving_service, self.reseed_service
        )

    def update(self) -> Status:
        """Drive the wave->reseed cycle while in NEEDS_HELP; always SUCCESS."""
        state = self._read_state()

        # Only act in NEEDS_HELP. Leaving it cancels any in-flight cycle so a
        # later loss starts fresh.
        if state != REACQ_NEEDS_HELP:
            self._phase = self._IDLE
            self._future = None
            self._pending_box = None
            self.feedback_message = "Not in NEEDS_HELP; idle"
            return Status.SUCCESS

        if self._bridge is None:
            self.feedback_message = "NEEDS_HELP but waving bridge unavailable (mock)"
            return Status.SUCCESS

        self._advance()
        return Status.SUCCESS

    def _read_state(self) -> int:
        try:
            return int(self._bb.get(self.bb_key))
        except Exception:
            return REACQ_TRACKING

    def _advance(self) -> None:
        """One step of the IDLE -> WAVE_PENDING -> RESEED_PENDING machine."""
        now = self._clock()

        if self._phase == self._IDLE:
            if (now - self._last_detect_time) >= self.throttle_s:
                self._future = self._bridge.detect_async()
                self._last_detect_time = now
                self._phase = self._WAVE_PENDING
                self.feedback_message = "Scanning for a waving operator"
            else:
                self.feedback_message = "NEEDS_HELP; waiting to re-scan"
            return

        if self._phase == self._WAVE_PENDING:
            if self._future is None or not self._future.done():
                self.feedback_message = "Awaiting waving result"
                return
            resp = self._future.result()
            self._future = None
            box = self._single_waver_box(resp)
            if box is None:
                # 0 or >1 wavers: do not reseed; retry after the throttle.
                self._phase = self._IDLE
                self.feedback_message = "No unambiguous single waver"
                return
            self._future = self._bridge.reseed_async(box)
            self._pending_box = box
            self._phase = self._RESEED_PENDING
            self.feedback_message = "Single waver found; reseeding tracker"
            return

        if self._phase == self._RESEED_PENDING:
            if self._future is None or not self._future.done():
                self.feedback_message = "Awaiting reseed result"
                return
            resp = self._future.result()
            ok = bool(getattr(resp, "success", False))
            self._future = None
            self._pending_box = None
            self._phase = self._IDLE
            self.feedback_message = (
                "Reseed succeeded" if ok else "Reseed did not match a person"
            )
            return

    @staticmethod
    def _single_waver_box(resp):
        """Return ``(x0, y0, x1, y1)`` iff exactly one waver with status OK.

        Zero or multiple wavers (or a non-OK status) -> None, so the caller does
        NOT reseed onto an ambiguous candidate.
        """
        if resp is None or int(getattr(resp, "status", -1)) != 0:
            return None
        boxes = list(getattr(resp, "waving_boxes", []) or [])
        if len(boxes) != 1:
            return None
        b = boxes[0]
        x0 = int(b.x_offset)
        y0 = int(b.y_offset)
        return (x0, y0, x0 + int(b.width), y0 + int(b.height))
