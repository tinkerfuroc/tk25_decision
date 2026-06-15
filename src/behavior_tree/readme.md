# behavior_tree

ROS2 `py_trees` behavior-tree package for the Tinker service robot
(RoboCup@Home). Implements the decision-making layer for the competition tasks
(Receptionist, GPSR/EGPSR, Storing Groceries, Help Me Carry, Restaurant, Serve
Breakfast, Inspection) and the follow-person pipeline. Nodes inherit from the
`ServiceHandler` / `ActionHandler` base classes and ship a mock-mode system so
behaviors can be exercised without hardware or Tinker-specific ROS packages.

See `CLAUDE.md` for the full architecture, mock-mode controls, and per-task
layout.

## Changelog

Newest first.

- **2026-06-15 — NEEDS_HELP two-pass head-scan recovery.** Replaced the passive
  NEEDS_HELP reaction (announce "raise your hand" + wave-reseed) with an active
  two-pass recovery: Pass 1 asks the person to stop and sweeps the head across
  `[current, -60deg, 0, +60deg]` (ABSOLUTE pan, tilt 40deg, ~4 s dwell each) for
  re-lock; Pass 2 asks the operator to raise a hand and sweeps again with
  wave-reseed active; Pass 2 repeats until re-lock or cancel. `BtNode_RecoveryScan`
  (pure `RecoveryScanFSM` core) owns it; the wave->reseed machine was extracted
  to the shared `WaveReseedCycle`; `BtNode_ReacqAnnounce` is now PASSIVE-only.
  Paired with a tracker guard (`pan_follow_suppressed`) so the head hands off
  cleanly. Spec: `docs/superpowers/specs/2026-06-15-needs-help-recovery-scan-design.md`.
