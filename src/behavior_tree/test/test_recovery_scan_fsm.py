# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for RecoveryScanFSM (pure two-pass head-scan recovery)."""

import math

from behavior_tree.components.following.recovery_scan import RecoveryScanFSM

TRACKING, PASSIVE, NEEDS_HELP = 0, 1, 2
PASS1 = "Please stop walking so I can find you."
PASS2 = "I've lost you. Please raise your hand."


def _fsm():
    # dwell 4s, default angles [-60,0,60] with a leading HOLD-current step.
    return RecoveryScanFSM(dwell_sec=4.0)


def test_idle_outside_needs_help():
    f = _fsm()
    a = f.tick(TRACKING, now=0.0)
    assert a.active is False
    assert a.speak is None and a.pan_target_rad is None and a.allow_wave is False


def test_entry_speaks_pass1_and_holds_current():
    f = _fsm()
    a = f.tick(NEEDS_HELP, now=0.0)
    assert a.active is True
    assert a.speak == PASS1
    assert a.pan_target_rad is None        # HOLD-current: no pan command
    assert a.allow_wave is False           # Pass 1: no wave


def test_dwell_then_sweep_minus60_zero_plus60():
    f = _fsm()
    f.tick(NEEDS_HELP, now=0.0)             # step 0 HOLD
    # Within dwell: no new command.
    a = f.tick(NEEDS_HELP, now=3.9)
    assert a.pan_target_rad is None and a.speak is None
    # Cross dwell -> step 1 = -60deg.
    a = f.tick(NEEDS_HELP, now=4.0)
    assert a.pan_target_rad == math.radians(-60.0)
    assert a.speak is None and a.allow_wave is False
    # -> step 2 = 0deg.
    a = f.tick(NEEDS_HELP, now=8.0)
    assert a.pan_target_rad == math.radians(0.0)
    # -> step 3 = +60deg.
    a = f.tick(NEEDS_HELP, now=12.0)
    assert a.pan_target_rad == math.radians(60.0)


def test_pass1_complete_escalates_to_pass2_with_wave():
    f = _fsm()
    f.tick(NEEDS_HELP, now=0.0)             # HOLD
    f.tick(NEEDS_HELP, now=4.0)             # -60
    f.tick(NEEDS_HELP, now=8.0)             # 0
    f.tick(NEEDS_HELP, now=12.0)            # +60 (last step of pass 1)
    # After +60's dwell -> pass 2 starts: speak raise-hand, HOLD, wave allowed.
    a = f.tick(NEEDS_HELP, now=16.0)
    assert a.speak == PASS2
    assert a.pan_target_rad is None        # pass 2 also starts with HOLD-current
    assert a.allow_wave is True


def test_pass1_steps_never_allow_wave_pass2_steps_always():
    f = _fsm()
    # Walk all of pass 1: allow_wave stays False.
    times = [0.0, 4.0, 8.0, 12.0]
    for t in times:
        assert f.tick(NEEDS_HELP, now=t).allow_wave is False
    # Enter + walk pass 2: allow_wave stays True.
    for t in [16.0, 20.0, 24.0, 28.0]:
        assert f.tick(NEEDS_HELP, now=t).allow_wave is True


def test_pass2_repeats_after_completion():
    # New Pass-2 cadence: each angle takes two ticks (settle -> detect_now, then
    # advance-when-not-busy). Walk all four Pass-2 angles to the wrap.
    f = _fsm()                              # dwell 4s, settle 2s (defaults)
    for t in [0.0, 4.0, 8.0, 12.0]:         # pass 1
        f.tick(NEEDS_HELP, now=t)
    a = f.tick(NEEDS_HELP, now=16.0)        # -> pass 2 entry (HOLD, speak PASS2)
    assert a.speak == PASS2
    base = 16.0
    for _ in range(4):                      # HOLD, -60, 0, +60
        f.tick(NEEDS_HELP, now=base + 2.0)              # settle -> detect_now
        a = f.tick(NEEDS_HELP, now=base + 2.1,          # response in -> advance
                   wave_busy=False)
        base += 2.1
    # The 4th angle's advance wraps pass 2 -> re-announce raise-hand.
    assert a.speak == PASS2
    assert a.allow_wave is True


def test_relock_resets_to_pass1():
    f = _fsm()
    f.tick(NEEDS_HELP, now=0.0)            # HOLD
    f.tick(NEEDS_HELP, now=4.0)            # -60
    f.tick(NEEDS_HELP, now=8.0)            # 0  (mid-recovery)
    # Re-lock.
    a = f.tick(TRACKING, now=9.0)
    assert a.active is False
    # Fresh loss -> back to Pass 1 from the top.
    a = f.tick(NEEDS_HELP, now=10.0)
    assert a.speak == PASS1
    assert a.pan_target_rad is None
    assert a.allow_wave is False


def test_pass2_waits_settle_before_detect():
    # Pass 2 angle entry must NOT detect until settle_sec elapses since the turn.
    f = RecoveryScanFSM(dwell_sec=7.0, settle_sec=2.0)
    # Drive to the start of Pass 2 (Pass 1 = 4 steps of 7s).
    t = 0.0
    f.tick(NEEDS_HELP, now=t)                       # HOLD (pass1)
    for _ in range(3):
        t += 7.0
        f.tick(NEEDS_HELP, now=t)                   # -60,0,+60 (pass1)
    t += 7.0
    a = f.tick(NEEDS_HELP, now=t)                   # -> Pass 2 entry (HOLD)
    assert a.speak == PASS2 and a.allow_wave is True
    pass2_start = t
    # < settle: no detect.
    a = f.tick(NEEDS_HELP, now=pass2_start + 1.9)
    assert a.detect_now is False
    # >= settle: one-shot detect_now.
    a = f.tick(NEEDS_HELP, now=pass2_start + 2.0)
    assert a.detect_now is True
    # next tick: not re-fired.
    a = f.tick(NEEDS_HELP, now=pass2_start + 2.1, wave_busy=True)
    assert a.detect_now is False


def test_pass2_holds_until_wave_not_busy():
    f = RecoveryScanFSM(dwell_sec=7.0, settle_sec=2.0, detect_timeout_sec=5.0)
    t = 0.0
    f.tick(NEEDS_HELP, now=t)
    for _ in range(3):
        t += 7.0
        f.tick(NEEDS_HELP, now=t)
    t += 7.0
    f.tick(NEEDS_HELP, now=t)                       # Pass 2 entry, step 0 (HOLD)
    s = t
    f.tick(NEEDS_HELP, now=s + 2.0)                 # detect_now (fired)
    # Busy: stays on the same angle (pan stays None, no advance).
    a = f.tick(NEEDS_HELP, now=s + 2.5, wave_busy=True)
    assert a.pan_target_rad is None and a.detect_now is False
    a = f.tick(NEEDS_HELP, now=s + 6.0, wave_busy=True)
    assert a.pan_target_rad is None                 # still held (busy), pre-timeout window
    # Response arrives (not busy): advance to the next angle (-60).
    a = f.tick(NEEDS_HELP, now=s + 6.1, wave_busy=False)
    assert a.pan_target_rad == math.radians(-60.0)


def test_pass2_advances_on_detect_timeout_when_stuck_busy():
    # No-server: wave_busy never clears -> advance after detect_timeout_sec so the
    # scan never hangs at one angle.
    f = RecoveryScanFSM(dwell_sec=7.0, settle_sec=2.0, detect_timeout_sec=5.0)
    t = 0.0
    f.tick(NEEDS_HELP, now=t)
    for _ in range(3):
        t += 7.0
        f.tick(NEEDS_HELP, now=t)
    t += 7.0
    f.tick(NEEDS_HELP, now=t)                       # Pass 2 entry
    s = t
    f.tick(NEEDS_HELP, now=s + 2.0)                 # detect fired at s+2.0
    a = f.tick(NEEDS_HELP, now=s + 6.9, wave_busy=True)
    assert a.pan_target_rad is None                 # within timeout (5s from s+2.0)
    a = f.tick(NEEDS_HELP, now=s + 7.1, wave_busy=True)   # > s+2+5
    assert a.pan_target_rad == math.radians(-60.0)  # timed out -> advance


def test_pass1_never_emits_detect_now():
    f = RecoveryScanFSM(dwell_sec=7.0, settle_sec=2.0)
    t = 0.0
    a = f.tick(NEEDS_HELP, now=t)
    assert a.detect_now is False
    for _ in range(3):
        t += 7.0
        a = f.tick(NEEDS_HELP, now=t)
        assert a.detect_now is False
