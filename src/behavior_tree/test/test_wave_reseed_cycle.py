# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for WaveReseedCycle (trigger-driven wave->reseed machine)."""

from types import SimpleNamespace

from behavior_tree.FollowPerson.wave_reseed_cycle import WaveReseedCycle


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
        self.next_reseed_success = True

    def detect_async(self, threshold_m):
        self.detect_calls += 1
        self.detect_thresholds.append(threshold_m)
        return _FakeFuture(self.next_detect)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _FakeFuture(SimpleNamespace(success=self.next_reseed_success))


def _roi(x, y, w, h):
    return SimpleNamespace(x_offset=x, y_offset=y, width=w, height=h)


def _detect(status=0, boxes=()):
    return SimpleNamespace(status=status, waving_boxes=list(boxes))


def _cycle(bridge, dist=3.5):
    return WaveReseedCycle(bridge=bridge, distance_threshold_m=dist)


def test_step_without_trigger_never_fires():
    bridge = _FakeBridge()
    c = _cycle(bridge)
    c.step()
    c.step()
    c.step()
    assert bridge.detect_calls == 0
    assert c.is_idle is True


def test_trigger_then_step_fires_once_with_threshold():
    bridge = _FakeBridge()
    bridge.next_detect = _detect(0, [_roi(10, 20, 30, 40)])
    c = _cycle(bridge, dist=3.5)
    c.trigger()
    assert c.is_idle is False           # pending -> busy
    c.step()                            # fires DetectWaving(threshold=3.5)
    assert bridge.detect_calls == 1
    assert bridge.detect_thresholds == [3.5]
    assert c.is_idle is False           # WAVE_PENDING
    c.step()                            # response -> reseed
    assert bridge.reseed_boxes == [(10, 20, 40, 60)]
    c.step()                            # reseed done -> idle
    assert c.is_idle is True
    # A second step does NOT re-fire (trigger is one-shot).
    c.step()
    assert bridge.detect_calls == 1


def test_zero_or_multiple_or_bad_status_does_not_reseed():
    for det in (_detect(0, []),
                _detect(0, [_roi(0, 0, 5, 5), _roi(9, 9, 5, 5)]),
                _detect(1, [_roi(10, 20, 30, 40)])):
        bridge = _FakeBridge()
        bridge.next_detect = det
        c = _cycle(bridge)
        c.trigger()
        c.step()
        c.step()
        assert bridge.reseed_boxes == []
        assert c.is_idle is True


def test_reset_clears_pending_and_phase():
    bridge = _FakeBridge()
    bridge.next_detect = _detect(0, [_roi(10, 20, 30, 40)])
    c = _cycle(bridge)
    c.trigger()
    c.step()                            # WAVE_PENDING
    assert c.is_idle is False
    c.reset()
    assert c.is_idle is True
    # A pending trigger that was reset before firing does not fire later.
    c.trigger()
    c.reset()
    c.step()
    assert bridge.detect_calls == 1     # only the first (pre-reset) detect


def test_no_bridge_trigger_step_is_inert():
    c = WaveReseedCycle(bridge=None, distance_threshold_m=3.5)
    c.trigger()
    c.step()
    assert c.is_idle is True            # nothing to fire; pending consumed
    assert c.has_bridge is False
