"""Regression: an already-aligned head still needs feedback after its new command."""
import math
from types import SimpleNamespace as NS
from behavior_tree.HomeServiceHangzhou.ros_nodes import SettledPanTilt
from behavior_tree.HomeServiceHangzhou.state import S,get

def head(monkeypatch):
    monkeypatch.setattr("time.time",lambda:10.)
    monkeypatch.setattr("time.monotonic",lambda:100.)
    h=SettledPanTilt("head",x=0.,y=30.)
    h.mock_mode=False
    h.node=NS(get_clock=lambda:NS(now=lambda:NS(nanoseconds=123000000000)))
    h._arm_settle(0.,30.)
    h._command_at=100.
    h._feedback_received=99.
    h._latest_state=NS(pan_rad=0.,tilt_rad=math.radians(30),feedback_ok=True)
    return h

def test_aligned_old_feedback_waits_for_new_sample(monkeypatch):
    h=head(monkeypatch)
    assert h.update()==S.RUNNING
    h._state_cb(h._latest_state)
    assert h.update()==S.SUCCESS
    assert get("capture_after")==123.

def test_old_feedback_still_fails_at_deadline(monkeypatch):
    h=head(monkeypatch);h._settle_deadline=9.
    assert h.update()==S.FAILURE
