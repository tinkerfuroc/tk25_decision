# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for WaveReseedCycle (shared wave->reseed async machine)."""

from types import SimpleNamespace

from behavior_tree.FollowPerson.wave_reseed_cycle import WaveReseedCycle


class _FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, s):
        self.now += s


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
        self.reseed_boxes = []
        self.next_detect = SimpleNamespace(status=0, waving_boxes=[])
        self.next_reseed_success = True

    def detect_async(self):
        self.detect_calls += 1
        return _FakeFuture(self.next_detect)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _FakeFuture(SimpleNamespace(success=self.next_reseed_success))


def _roi(x, y, w, h):
    return SimpleNamespace(x_offset=x, y_offset=y, width=w, height=h)


def _detect(status=0, boxes=()):
    return SimpleNamespace(status=status, waving_boxes=list(boxes))


def _cycle(bridge, clock):
    return WaveReseedCycle(bridge=bridge, throttle_s=3.0, clock=clock)


def test_has_bridge_reflects_injection():
    c = WaveReseedCycle(throttle_s=3.0, clock=_FakeClock())
    assert c.has_bridge is False
    c.set_bridge(_FakeBridge())
    assert c.has_bridge is True


def test_single_waver_reseeds_with_correct_box():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(status=0, boxes=[_roi(10, 20, 30, 40)])
    c = _cycle(bridge, clock)
    c.step()                                  # IDLE -> start detect
    assert bridge.detect_calls == 1
    c.step()                                  # detect done -> reseed (x0,y0,x1,y1)
    assert bridge.reseed_boxes == [(10, 20, 40, 60)]
    c.step()                                  # reseed done -> IDLE
    assert c._phase == WaveReseedCycle._IDLE


def test_zero_or_multiple_or_bad_status_does_not_reseed():
    clock = _FakeClock()
    for det in (_detect(0, []),
                _detect(0, [_roi(0, 0, 5, 5), _roi(9, 9, 5, 5)]),
                _detect(1, [_roi(10, 20, 30, 40)])):
        bridge = _FakeBridge()
        bridge.next_detect = det
        c = _cycle(bridge, clock)
        c.step()
        c.step()
        assert bridge.reseed_boxes == []


def test_detect_is_throttled():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(0, [])       # no waver -> cycles fast
    c = _cycle(bridge, clock)
    c.step()                                  # detect #1 at t=0
    c.step()                                  # done -> IDLE
    c.step()                                  # within throttle -> no new detect
    assert bridge.detect_calls == 1
    clock.advance(3.0)
    c.step()                                  # detect #2
    assert bridge.detect_calls == 2


def test_reset_returns_to_idle_and_drops_future():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(0, [_roi(10, 20, 30, 40)])
    c = _cycle(bridge, clock)
    c.step()                                  # WAVE_PENDING
    assert c._phase == WaveReseedCycle._WAVE_PENDING
    c.reset()
    assert c._phase == WaveReseedCycle._IDLE
    assert c._future is None
