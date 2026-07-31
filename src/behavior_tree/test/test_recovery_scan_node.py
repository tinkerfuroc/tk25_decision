# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for BtNode_RecoveryScan (executes RecoveryScanFSM actions)."""

import math
from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.components.following.nodes import BtNode_RecoveryScan

TRACKING, NEEDS_HELP = 0, 2
PASS1 = "Please stop walking so I can find you."
PASS2 = "I've lost you. Please raise your hand."


class _FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, s):
        self.now += s


class _FakeCoalescer:
    def __init__(self):
        self.submitted = []
        self.polls = 0

    def submit(self, text):
        self.submitted.append(text)

    def poll(self):
        self.polls += 1


class _FakeFuture:
    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeBridge:
    def __init__(self):
        self.detect_calls = 0
        self.detect_thresholds = []
        self.reseed_boxes = []
        self.next_detect = SimpleNamespace(status=0, waving_boxes=[])

    def detect_async(self, threshold_m):
        self.detect_calls += 1
        self.detect_thresholds.append(threshold_m)
        return _FakeFuture(self.next_detect)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _FakeFuture(SimpleNamespace(success=True))


class _PendingFuture:
    """Reports not-done for the first ``pending`` done() calls, then done."""

    def __init__(self, result, pending):
        self._result = result
        self._pending = pending

    def done(self):
        if self._pending > 0:
            self._pending -= 1
            return False
        return True

    def result(self):
        return self._result


class _PendingBridge:
    """Detect future stays pending for ``pending`` step()s, then resolves."""

    def __init__(self, detect_result, pending):
        self.detect_calls = 0
        self.detect_thresholds = []
        self.reseed_boxes = []
        self._detect_result = detect_result
        self._pending = pending

    def detect_async(self, threshold_m):
        self.detect_calls += 1
        self.detect_thresholds.append(threshold_m)
        return _PendingFuture(self._detect_result, self._pending)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _PendingFuture(SimpleNamespace(success=True), 0)


class _StuckBridge:
    """No-server simulation: the detect future never resolves."""

    def __init__(self):
        self.detect_calls = 0
        self.detect_thresholds = []

    def detect_async(self, threshold_m):
        self.detect_calls += 1
        self.detect_thresholds.append(threshold_m)
        return _PendingFuture(None, 10 ** 9)

    def reseed_async(self, box):                       # pragma: no cover
        raise AssertionError("stuck detect must not reach reseed")


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _writer(bb_key):
    c = py_trees.blackboard.Client(name="reacq_writer")
    c.register_key(key=bb_key, access=py_trees.common.Access.WRITE)
    return c


def _make(clock, tts, pan_calls, bridge=None):
    bb = "track/reacquisition_state"
    node = BtNode_RecoveryScan(name="RecoveryScan", dwell_sec=7.0, settle_sec=2.0,
                               wave_max_distance_m=3.5, bb_key=bb, clock=clock)
    node.inject_coalescer(tts)
    node.inject_pan_sink(lambda pan, tilt: pan_calls.append((pan, tilt)))
    if bridge is not None:
        node.inject_wave_bridge(bridge)
    return node, _writer(bb), bb


def test_idle_when_tracking_returns_success():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    node, writer, bb = _make(clock, tts, pan)
    writer.set(bb, TRACKING, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert tts.submitted == [] and pan == []


def test_entry_speaks_pass1_holds_and_runs():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    node, writer, bb = _make(clock, tts, pan)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.RUNNING
    assert tts.submitted == [PASS1]
    assert pan == []                       # HOLD-current: no pan command
    assert tts.polls == 1


def test_sweep_publishes_absolute_pan_with_fixed_tilt():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    node, writer, bb = _make(clock, tts, pan)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()                       # HOLD at t=0
    clock.advance(7.0)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()                       # -> -60deg
    assert pan == [(math.radians(-60.0), math.radians(35.0))]


def test_pass1_never_calls_wave_bridge():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _FakeBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    for t in [0.0, 7.0, 14.0, 21.0]:        # all of Pass 1 (dwell 7s)
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
    assert bridge.detect_calls == 0


def test_pass2_detects_after_settle_with_threshold():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _FakeBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    # Walk through Pass 1 into Pass 2 entry (4 angles * 7s).
    for t in [0.0, 7.0, 14.0, 21.0, 28.0]:
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
    assert PASS2 in tts.submitted
    # At Pass-2 entry + <settle: no detect yet.
    clock.now = 28.0 + 1.9
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert bridge.detect_calls == 0
    # >= settle: one detect with the 3.5 m threshold.
    clock.now = 28.0 + 2.0
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert bridge.detect_calls == 1
    assert bridge.detect_thresholds == [3.5]


def test_no_pan_sink_no_crash():
    # Mock/headless: no pan sink injected -> must not raise.
    clock = _FakeClock()
    bb = "track/reacquisition_state"
    node = BtNode_RecoveryScan(name="RecoveryScan", dwell_sec=7.0, bb_key=bb,
                               clock=clock)
    node.inject_coalescer(_FakeCoalescer())
    writer = _writer(bb)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.RUNNING


def _walk_into_pass2_entry(node, writer, bb, clock):
    """Tick Pass 1 (4 angles * 7s dwell) up to the Pass-2 entry at t=28."""
    writer.set(bb, NEEDS_HELP, overwrite=True)
    for t in [0.0, 7.0, 14.0, 21.0, 28.0]:
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()


def test_pass2_holds_while_detect_in_flight_then_advances():
    # The load-bearing wiring: wave_busy = has_bridge and not is_idle, computed
    # BEFORE the FSM tick, must hold the angle (no advance, no second detect)
    # for as long as the detect future is in flight, then advance once it resolves.
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _PendingBridge(SimpleNamespace(status=0, waving_boxes=[]), pending=2)
    node, writer, bb = _make(clock, tts, pan, bridge)
    _walk_into_pass2_entry(node, writer, bb, clock)
    n_at_entry = len(pan)
    # Settle tick: exactly one detect; the detect tick itself does not advance.
    clock.now = 30.0
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert bridge.detect_calls == 1
    assert bridge.detect_thresholds == [3.5]
    assert len(pan) == n_at_entry
    # Hold while pending, then advance once the future resolves.
    advanced = False
    for k in range(6):
        clock.now = 30.1 + 0.1 * k
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
        assert bridge.detect_calls == 1              # never double-fires
        if len(pan) > n_at_entry:
            advanced = True
            break
        assert len(pan) == n_at_entry                # still held at the angle
    assert advanced
    assert pan[-1] == (math.radians(-60.0), math.radians(35.0))


def test_pass2_stuck_detect_advances_on_timeout():
    # No server: the detect future never resolves -> the node must NOT hang at the
    # angle; the FSM's detect_timeout_sec (5s default) advances it.
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _StuckBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    _walk_into_pass2_entry(node, writer, bb, clock)
    n_at_entry = len(pan)
    clock.now = 30.0                                 # settle -> detect fires (stuck)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert bridge.detect_calls == 1
    # Within the timeout window: held despite the future never resolving.
    clock.now = 34.0                                 # 4s after detect (< 5)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert len(pan) == n_at_entry
    # Past the timeout: advance even though the detect is still in flight.
    clock.now = 35.1                                 # > 5s after detect at 30.0
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert pan[-1] == (math.radians(-60.0), math.radians(35.0))
