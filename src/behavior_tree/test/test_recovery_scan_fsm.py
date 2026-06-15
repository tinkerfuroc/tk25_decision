# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for RecoveryScanFSM (pure two-pass head-scan recovery)."""

import math

from behavior_tree.FollowPerson.recovery_scan import RecoveryScanFSM

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
    f = _fsm()
    for t in [0.0, 4.0, 8.0, 12.0]:        # pass 1
        f.tick(NEEDS_HELP, now=t)
    for t in [16.0, 20.0, 24.0, 28.0]:     # pass 2 (HOLD,-60,0,+60)
        f.tick(NEEDS_HELP, now=t)
    # +60 of pass 2 done -> repeat pass 2: re-announce raise-hand.
    a = f.tick(NEEDS_HELP, now=32.0)
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
