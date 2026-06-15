# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for BtNode_RecoveryScan (executes RecoveryScanFSM actions)."""

import math
from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.FollowPerson.nodes import BtNode_RecoveryScan

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
    assert pan == [(math.radians(-60.0), math.radians(37.0))]


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
