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

"""Unit tests for BtNode_WaveReseed (NEEDS_HELP wave->reseed recovery)."""

from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.FollowPerson.nodes import BtNode_WaveReseed

REACQ_TRACKING = 0
REACQ_PASSIVE = 1
REACQ_NEEDS_HELP = 2


class _FakeClock:
    """Mutable monotonic clock for deterministic throttle tests."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class _FakeFuture:
    """Immediately-done future stand-in (rclpy Future protocol subset)."""

    def __init__(self, result):
        self._result = result

    def done(self):
        return True

    def result(self):
        return self._result


class _FakeBridge:
    """Scriptable wave/reseed bridge: records calls, returns scripted results."""

    def __init__(self):
        self.detect_calls = 0
        self.reseed_boxes = []
        # Next DetectWaving response (SimpleNamespace status/waving_boxes).
        self.next_detect = SimpleNamespace(status=0, waving_boxes=[])
        self.next_reseed_success = True

    def detect_async(self):
        self.detect_calls += 1
        return _FakeFuture(self.next_detect)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _FakeFuture(
            SimpleNamespace(success=self.next_reseed_success, target_track_id=9))


def _roi(x, y, w, h):
    return SimpleNamespace(x_offset=x, y_offset=y, width=w, height=h)


def _detect(status=0, boxes=()):
    return SimpleNamespace(status=status, waving_boxes=list(boxes))


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _writer(bb_key):
    client = py_trees.blackboard.Client(name="reacq_writer")
    client.register_key(key=bb_key, access=py_trees.common.Access.WRITE)
    return client


def _make_node(clock, bridge, bb_key="track/reacquisition_state"):
    node = BtNode_WaveReseed(name="WaveReseed", throttle_s=3.0,
                             bb_key=bb_key, clock=clock)
    node.inject_bridge(bridge)
    return node


def test_no_action_outside_needs_help():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)

    for state in (REACQ_TRACKING, REACQ_PASSIVE):
        writer.set(bb, state, overwrite=True)
        node.tick_once()
        assert node.status == py_trees.common.Status.SUCCESS
    assert bridge.detect_calls == 0
    assert bridge.reseed_boxes == []


def test_single_waver_reseeds_with_correct_box():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(status=0, boxes=[_roi(10, 20, 30, 40)])
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)
    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)

    # Tick 1: IDLE -> start DetectWaving.
    node.tick_once()
    assert bridge.detect_calls == 1
    assert bridge.reseed_boxes == []
    # Tick 2: WAVE_PENDING done, one waver -> start reseed with (x0,y0,x1,y1).
    node.tick_once()
    assert bridge.reseed_boxes == [(10, 20, 40, 60)]   # (10,20,10+30,20+40)
    # Tick 3: RESEED_PENDING done -> back to IDLE, all SUCCESS.
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS


def test_zero_wavers_does_not_reseed():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(status=0, boxes=[])
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)
    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)

    node.tick_once()          # start detect
    node.tick_once()          # detect done: no wavers -> no reseed
    assert bridge.detect_calls == 1
    assert bridge.reseed_boxes == []


def test_multiple_wavers_stays_ambiguous_no_reseed():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(
        status=0, boxes=[_roi(0, 0, 10, 10), _roi(50, 50, 10, 10)])
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)
    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)

    node.tick_once()          # start detect
    node.tick_once()          # detect done: 2 wavers -> ambiguous, no reseed
    assert bridge.reseed_boxes == []


def test_nonzero_status_does_not_reseed():
    clock = _FakeClock()
    bridge = _FakeBridge()
    # A single box but a failed status must NOT reseed.
    bridge.next_detect = _detect(status=1, boxes=[_roi(10, 20, 30, 40)])
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)
    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)

    node.tick_once()
    node.tick_once()
    assert bridge.reseed_boxes == []


def test_detect_is_throttled():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(status=0, boxes=[])   # no waver -> cycles fast
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)
    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)

    node.tick_once()          # detect #1 at t=0
    node.tick_once()          # detect done (no waver) -> IDLE
    node.tick_once()          # still within throttle (t=0): no new detect
    assert bridge.detect_calls == 1
    clock.advance(3.0)        # cross throttle_s
    node.tick_once()          # detect #2
    assert bridge.detect_calls == 2


def test_leaving_needs_help_cancels_cycle():
    clock = _FakeClock()
    bridge = _FakeBridge()
    bridge.next_detect = _detect(status=0, boxes=[_roi(10, 20, 30, 40)])
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = _make_node(clock, bridge, bb)

    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)
    node.tick_once()          # start detect (WAVE_PENDING)
    assert bridge.detect_calls == 1
    # Operator re-acquired before the cycle finished.
    writer.set(bb, REACQ_TRACKING, overwrite=True)
    node.tick_once()          # resets to IDLE, drops the in-flight future
    assert bridge.reseed_boxes == []
    assert node._phase == BtNode_WaveReseed._IDLE


def test_no_bridge_is_inert_noop():
    clock = _FakeClock()
    bb = "track/reacquisition_state"
    writer = _writer(bb)
    node = BtNode_WaveReseed(name="WaveReseed", bb_key=bb, clock=clock)
    # No bridge injected (mock / unavailable): NEEDS_HELP must still SUCCESS, no raise.
    writer.set(bb, REACQ_NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS


if __name__ == "__main__":  # pragma: no cover
    pytest.main([__file__, "-v"])
