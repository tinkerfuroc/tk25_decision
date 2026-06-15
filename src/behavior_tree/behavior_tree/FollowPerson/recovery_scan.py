# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Pure two-pass head-scan recovery FSM (no ROS).

Ticked each BT tick with ``(reacq_state, now)``; emits a ``RecoveryAction`` the
node executes (speak / command pan / allow wave). Pass 1 asks the person to stop
and sweeps the head; Pass 2 asks the operator to raise a hand and sweeps with
wave-reseed enabled; Pass 2 repeats until re-lock. Scan steps are a leading
HOLD-current dwell (head stays put) then the absolute pan angles. Each step
dwells ``dwell_sec`` so the camera settles and the tracker's relaxed
in-NEEDS_HELP re-lock can commit.
"""

import math
from dataclasses import dataclass

REACQ_NEEDS_HELP = 2

# Sentinel scan step: hold the head where it is (issue no pan command).
_HOLD = None


@dataclass(frozen=True)
class RecoveryAction:
    """One tick's outputs. ``speak``/``pan_target_rad`` are one-shot (non-None
    only on the entry tick of a pass / scan step)."""

    active: bool
    speak: str | None = None
    pan_target_rad: float | None = None
    allow_wave: bool = False


class RecoveryScanFSM:
    def __init__(
        self,
        dwell_sec: float = 4.0,
        scan_angles_rad: list[float] | None = None,
        include_current_first: bool = True,
        pass1_text: str = "Please stop walking so I can find you.",
        pass2_text: str = "I've lost you. Please raise your hand.",
        needs_help_state: int = REACQ_NEEDS_HELP,
    ):
        self.dwell_sec = float(dwell_sec)
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

    def reset(self):
        self._reset_state()

    def _pass_text(self, pass_idx):
        return self.pass1_text if pass_idx == 1 else self.pass2_text

    @staticmethod
    def _pan_for(step_value):
        return None if step_value is _HOLD else float(step_value)

    def tick(self, reacq_state, now) -> RecoveryAction:
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
            self._step_start = now
            return RecoveryAction(
                active=True,
                speak=self._pass_text(self._pass),
                pan_target_rad=self._pan_for(self._steps[0]),
                allow_wave=(self._pass >= 2),
            )

        if (now - self._step_start) >= self.dwell_sec:
            self._step += 1
            if self._step >= len(self._steps):
                # Pass complete -> 1->2, then repeat 2. Re-announce + restart steps.
                self._pass = 2
                self._step = 0
                self._step_start = now
                return RecoveryAction(
                    active=True,
                    speak=self._pass_text(self._pass),
                    pan_target_rad=self._pan_for(self._steps[0]),
                    allow_wave=(self._pass >= 2),
                )
            self._step_start = now
            return RecoveryAction(
                active=True,
                speak=None,
                pan_target_rad=self._pan_for(self._steps[self._step]),
                allow_wave=(self._pass >= 2),
            )

        # Still dwelling: keep scanning, no new command.
        return RecoveryAction(active=True, speak=None, pan_target_rad=None,
                              allow_wave=(self._pass >= 2))
