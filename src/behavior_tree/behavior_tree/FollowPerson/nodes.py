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

import math
import time

import py_trees
from py_trees.common import Status

from behavior_tree.config import is_node_mocked
from behavior_tree.FollowPerson.wave_reseed_cycle import (
    WaveReseedCycle,
    _WaveReseedBridge,
)
from behavior_tree.FollowPerson.recovery_scan import RecoveryScanFSM

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


class BtNode_WaveReseed(py_trees.behaviour.Behaviour):
    """Wave-to-reseed recovery while the tracker is latched in NEEDS_HELP.

    Thin py_trees wrapper around the shared ``WaveReseedCycle``: while in
    NEEDS_HELP it advances the cycle (throttled DetectWaving -> ReseedTarget on an
    unambiguous single waver); leaving NEEDS_HELP cancels any in-flight cycle.
    ALWAYS returns SUCCESS (never gates the never-mid-abort Parallel). Inert in
    mock mode / when the bridge is unavailable.
    """

    _IDLE = WaveReseedCycle._IDLE

    def __init__(
        self,
        name: str,
        throttle_s: float = 3.0,
        bb_key: str = "track/reacquisition_state",
        waving_service: str = "detect_waving_persons",
        reseed_service: str = "/person_track_node/reseed_target",
        clock=time.monotonic,
    ):
        super().__init__(name=name)
        self.bb_key = bb_key
        self.waving_service = waving_service
        self.reseed_service = reseed_service

        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self._cycle = WaveReseedCycle(throttle_s=throttle_s, clock=clock)

        self._bb = self.attach_blackboard_client(name=f"{self.name}_reacq")
        self._bb.register_key(key=self.bb_key, access=py_trees.common.Access.READ)

    @property
    def _phase(self):
        return self._cycle._phase

    def inject_bridge(self, bridge):
        """Inject a wave/reseed bridge (or fake), bypassing ROS setup."""
        self._cycle.set_bridge(bridge)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e
        if self._cycle.has_bridge or self.mock_mode:
            return
        self._cycle.set_bridge(_WaveReseedBridge(
            self.node, self.waving_service, self.reseed_service))

    def update(self) -> Status:
        try:
            state = int(self._bb.get(self.bb_key))
        except Exception:
            state = REACQ_TRACKING
        if state != REACQ_NEEDS_HELP:
            self._cycle.reset()
            self.feedback_message = "Not in NEEDS_HELP; idle"
            return Status.SUCCESS
        if not self._cycle.has_bridge:
            self.feedback_message = "NEEDS_HELP but waving bridge unavailable (mock)"
            return Status.SUCCESS
        self._cycle.step()
        return Status.SUCCESS


class BtNode_RecoveryScan(py_trees.behaviour.Behaviour):
    """Two-pass head-scan recovery while the tracker is latched in NEEDS_HELP.

    Owns the active recovery the announcer/wave-reseed used to do passively:
    drives ``RecoveryScanFSM`` and executes its per-tick actions — speak each
    pass's line (Pass 1 "please stop", Pass 2 "please raise your hand") via a
    CoalescingTTS, command the head to ABSOLUTE scan angles (tilt fixed) on
    ``/pan_tilt_controller/cmd``, and (Pass 2 only) advance the shared
    ``WaveReseedCycle``. Returns RUNNING while scanning, SUCCESS when idle. All
    ROS I/O is injectable (``inject_coalescer`` / ``inject_pan_sink`` /
    ``inject_wave_bridge``) so the node unit-tests without a ROS graph; lazy msg
    imports keep ``behavior_tree.messages`` off the import path until live setup.
    """

    def __init__(
        self,
        name: str,
        dwell_sec: float = 4.0,
        scan_angles_deg=(-60.0, 0.0, 60.0),
        tilt_deg: float = 40.0,
        include_current_first: bool = True,
        bb_key: str = "track/reacquisition_state",
        announce_service: str = "announce",
        waving_service: str = "detect_waving_persons",
        reseed_service: str = "/person_track_node/reseed_target",
        pan_cmd_topic: str = "/pan_tilt_controller/cmd",
        wave_throttle_s: float = 3.0,
        clock=time.monotonic,
    ):
        super().__init__(name=name)
        self.bb_key = bb_key
        self.announce_service = announce_service
        self.waving_service = waving_service
        self.reseed_service = reseed_service
        self.pan_cmd_topic = pan_cmd_topic
        self._tilt_rad = math.radians(float(tilt_deg))
        self._clock = clock

        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self._tts = None
        self._pan_sink = None          # callable(pan_rad, tilt_rad) -> None
        self._wave = WaveReseedCycle(throttle_s=wave_throttle_s, clock=clock)
        self._fsm = RecoveryScanFSM(
            dwell_sec=dwell_sec,
            scan_angles_rad=[math.radians(a) for a in scan_angles_deg],
            include_current_first=include_current_first,
        )

        self._bb = self.attach_blackboard_client(name=f"{self.name}_reacq")
        self._bb.register_key(key=self.bb_key, access=py_trees.common.Access.READ)

    def inject_coalescer(self, coalescer):
        self._tts = coalescer

    def inject_pan_sink(self, sink):
        self._pan_sink = sink

    def inject_wave_bridge(self, bridge):
        self._wave.set_bridge(bridge)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        if self._tts is None:
            from behavior_tree.FollowPerson.coalescing_tts import CoalescingTTS
            if self.mock_mode:
                def start(text):
                    print(f"🔊 MOCK RECOVERY ANNOUNCE: {text}")
                    return _DoneHandle()
                self._tts = CoalescingTTS(start=start, is_done=lambda h: h.done())
            else:
                from behavior_tree.messages import TextToSpeech
                client = self.node.create_client(TextToSpeech, self.announce_service)
                self._client = client

                def start(text):
                    req = TextToSpeech.Request()
                    req.text = text
                    return client.call_async(req)
                self._tts = CoalescingTTS(start=start, is_done=lambda f: f.done())

        if self._pan_sink is None and not self.mock_mode:
            from behavior_tree.messages import PanTiltCommand
            pub = self.node.create_publisher(PanTiltCommand, self.pan_cmd_topic, 10)

            def _sink(pan_rad, tilt_rad):
                cmd = PanTiltCommand()
                cmd.mode = PanTiltCommand.ABSOLUTE
                cmd.pan_rad = float(pan_rad)
                cmd.tilt_rad = float(tilt_rad)
                cmd.speed_raw = 0
                cmd.accel_raw = 0
                pub.publish(cmd)
            self._pan_sink = _sink

        if not self._wave.has_bridge and not self.mock_mode:
            self._wave.set_bridge(_WaveReseedBridge(
                self.node, self.waving_service, self.reseed_service))

    def update(self) -> Status:
        try:
            state = int(self._bb.get(self.bb_key))
        except Exception:
            state = REACQ_TRACKING
        action = self._fsm.tick(state, self._clock())

        if action.speak and self._tts is not None:
            self._tts.submit(action.speak)
        if self._tts is not None:
            self._tts.poll()

        if action.pan_target_rad is not None and self._pan_sink is not None:
            self._pan_sink(action.pan_target_rad, self._tilt_rad)

        if action.allow_wave and self._wave.has_bridge:
            self._wave.step()
        else:
            self._wave.reset()

        if action.active:
            self.feedback_message = "Recovery scanning"
            return Status.RUNNING
        self.feedback_message = "Idle (not in NEEDS_HELP)"
        return Status.SUCCESS
