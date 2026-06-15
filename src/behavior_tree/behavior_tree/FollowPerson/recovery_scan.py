# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Pure two-pass head-scan recovery FSM (no ROS).

Ticked each BT tick with ``(reacq_state, now, wave_busy)``; emits a
``RecoveryAction`` the node executes (speak / command pan / arm wave detect).
Pass 1 asks the person to stop and sweeps the head, dwelling ``dwell_sec`` at
each angle for passive re-lock. Pass 2 asks the operator to raise a hand and, at
each angle, waits ``settle_sec`` for the head to fully stop, fires ONE wave
detect (``detect_now``), and holds the angle until the detect round-trip
finishes (``wave_busy`` falls) or ``detect_timeout_sec`` elapses — then advances.
Pass 2 repeats until re-lock. Scan steps are a leading HOLD-current dwell then
the absolute pan angles.
"""

import math
from dataclasses import dataclass

REACQ_NEEDS_HELP = 2

# Sentinel scan step: hold the head where it is (issue no pan command).
_HOLD = None


@dataclass(frozen=True)
class RecoveryAction:
    """One tick's outputs. ``speak``/``pan_target_rad``/``detect_now`` are
    one-shot (set only on the entry tick of a pass / scan step / detect)."""

    active: bool
    speak: str | None = None
    pan_target_rad: float | None = None
    allow_wave: bool = False
    detect_now: bool = False


class RecoveryScanFSM:
    def __init__(
        self,
        dwell_sec: float = 7.0,
        settle_sec: float = 2.0,
        detect_timeout_sec: float = 5.0,
        scan_angles_rad: list[float] | None = None,
        include_current_first: bool = True,
        pass1_text: str = "Please stop walking so I can find you.",
        pass2_text: str = "I've lost you. Please raise your hand.",
        needs_help_state: int = REACQ_NEEDS_HELP,
    ):
        self.dwell_sec = float(dwell_sec)
        self.settle_sec = float(settle_sec)
        self.detect_timeout_sec = float(detect_timeout_sec)
        if scan_angles_rad is None:
            scan_angles_rad = [math.radians(-60.0), math.radians(0.0),
                               math.radians(60.0)]
        self._steps = ([_HOLD] if include_current_first else []) + \
            [float(a) for a in scan_angles_rad]
        if not self._steps:
            raise ValueError("RecoveryScanFSM needs at least one scan step")
        self.pass1_text = pass1_text
        self.pass2_text = pass2_text
        self.needs_help_state = int(needs_help_state)
        self._reset_state()

    def _reset_state(self):
        self._pass = 0
        self._step = 0
        self._step_start = None
        self._prev_state = None
        self._detect_fired = False
        self._detect_t = None

    def reset(self):
        self._reset_state()

    def _pass_text(self, pass_idx):
        return self.pass1_text if pass_idx == 1 else self.pass2_text

    @staticmethod
    def _pan_for(step_value):
        return None if step_value is _HOLD else float(step_value)

    def _enter_step(self, now, *, speak=None):
        """Return the entry action for the current ``self._step`` and clear the
        per-angle detect state."""
        self._step_start = now
        self._detect_fired = False
        self._detect_t = None
        return RecoveryAction(
            active=True,
            speak=speak,
            pan_target_rad=self._pan_for(self._steps[self._step]),
            allow_wave=(self._pass >= 2),
        )

    def _advance(self, now):
        """Move to the next scan step (or wrap a completed pass -> Pass 2)."""
        self._step += 1
        if self._step >= len(self._steps):
            self._pass = 2          # 1 -> 2, then repeats 2
            self._step = 0
            return self._enter_step(now, speak=self._pass_text(self._pass))
        return self._enter_step(now)

    def tick(self, reacq_state, now, wave_busy=False) -> RecoveryAction:
        state = int(reacq_state)

        # Outside NEEDS_HELP -> idle; reset so a fresh loss restarts at Pass 1.
        if state != self.needs_help_state:
            self._reset_state()
            self._prev_state = state
            return RecoveryAction(active=False)

        entering = (self._prev_state != self.needs_help_state) or \
            (self._step_start is None)
        self._prev_state = state

        if entering:
            self._pass = 1
            self._step = 0
            return self._enter_step(now, speak=self._pass_text(self._pass))

        elapsed = now - self._step_start

        # Pass 1: fixed dwell, no detection.
        if self._pass < 2:
            if elapsed >= self.dwell_sec:
                return self._advance(now)
            return RecoveryAction(active=True, allow_wave=False)

        # Pass 2: settle -> one detect -> wait for response -> advance.
        if elapsed < self.settle_sec:
            return RecoveryAction(active=True, allow_wave=True)   # settling
        if not self._detect_fired:
            self._detect_fired = True
            self._detect_t = now
            return RecoveryAction(active=True, allow_wave=True, detect_now=True)
        timed_out = (now - self._detect_t) >= self.detect_timeout_sec
        if (not wave_busy) or timed_out:
            return self._advance(now)
        return RecoveryAction(active=True, allow_wave=True)        # awaiting
