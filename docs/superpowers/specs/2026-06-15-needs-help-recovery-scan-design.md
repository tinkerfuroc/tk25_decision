# NEEDS_HELP Two-Pass Head-Scan Recovery — Design

**Date:** 2026-06-15
**Status:** Approved (design phase)
**Repos touched:** `tk25_decision` (primary), `tk26_vision` (one tracker guard)

## Problem

When the person tracker loses the operator and the loss exceeds
`active_help_after_sec` (5 s), it latches into **NEEDS_HELP**. Today the
behavior tree's reaction to NEEDS_HELP is passive: `BtNode_ReacqAnnounce` speaks
*"I've lost you. Please raise your hand."* and `BtNode_WaveReseed` watches for a
raise-hand to reseed the tracker. The head does not move — it holds wherever
pan-follow left it — so if the operator has stepped out of the camera's
~120° horizontal FOV, re-lock is impossible no matter how long the robot waits.

We want an **active** recovery: when NEEDS_HELP engages, the robot asks the
person to **stop**, then sweeps its head across a set of pan angles, dwelling at
each long enough for the camera to settle and the existing re-lock machinery to
engage. Only if that whole sweep fails does it escalate to asking the operator
to **raise their hand**, sweeping again with wave-reseed active. If that also
fails, it repeats the raise-hand sweep indefinitely until re-lock or the follow
goal is cancelled.

## Goals

1. On NEEDS_HELP entry, speak *"please stop"* and scan the head across
   `[current, -60°, 0°, +60°]`, dwelling **3–5 s** (default 4 s) at each angle so
   the camera settles and the relaxed in-NEEDS_HELP re-lock path can commit.
2. **Pass 1** uses only passive re-lock as the camera scans — **no** wave
   detection.
3. If Pass 1's full sweep fails, escalate to **Pass 2**: speak *"please raise
   your hand"* and run the same sweep **with wave-reseed active**.
4. If Pass 2's sweep also fails, **repeat Pass 2** (re-announce + re-sweep)
   until re-lock or cancel.
5. Re-lock at any point (`reacq_state → TRACKING`) aborts the scan immediately
   and hands the head back to the tracker's normal pan-follow.
6. ABSOLUTE pan commands only (no accumulated error), tilt held at the same
   fixed 40° as follow.

## Non-goals

- No change to the ReID / lock thresholds or the tracker's re-lock logic. The
  scan only improves **FOV coverage** and **settle time**; the existing relaxed
  in-NEEDS_HELP commit path does the actual re-lock.
- No explicit base-halt command. When lost the tracker publishes no
  `/target_points`, so `follow_server` already has no goal and the base is
  stopped; *"please stop"* is addressed to the human.
- No servo arrival-detection in v1 (dwell is timed from command-publish; 4 s
  generously covers a ≤120° servo move plus settle). Arrival-gated dwell is a
  possible future refinement.

## Architecture & ownership

The head, the voice, and wave-reseed are all driven **from the behavior tree**
during recovery. The contract is a single mutual-exclusion invariant keyed on
`reacq_state` (the `track/reacquisition_state` blackboard key, already populated
by `BtNode_TrackPersonAction` from the `TrackPerson` action feedback):

> **TRACKING (0)** → the *tracker* owns the head (pan-follow centers the bbox).
> **NEEDS_HELP (2)** → the *BT recovery node* owns the head (scan), the voice,
> and wave-reseed.
> **PASSIVE (1) / INACTIVE (255)** → neither scans; head holds; the gentle
> PASSIVE "slow down" announce still fires.

Why BT-centric (vs. tracker-centric): recovery is **coarse-grained** —
multi-second dwells and spoken lines — so the BT tick latency that rules the BT
out for *per-frame* pan-follow is irrelevant here. BT-centric reuses the
existing `/announce` + wave-reseed machinery, keeps human-facing wording in the
decision layer, and needs no new tracker interface (the BT publishes
`PanTiltCommand` straight to the controller, exactly as the tracker does).

### The one tracker-side change

To make the exclusion airtight and avoid head jitter when the tracker gets a
*provisional, uncommitted* detection mid-dwell, the tracker's pan-follow tick is
**suppressed while `_help_latched` is set**. `_help_latched` is True exactly
during NEEDS_HELP and clears the instant a re-lock commits, so pan ownership
hands back to the tracker seamlessly (ABSOLUTE mode → no re-homing). This is the
*only* change in `tk26_vision`.

## Components

### 1. Pure FSM core — `tk25_decision/.../FollowPerson/recovery_scan.py`

A ROS-free `RecoveryScanFSM`, mirroring the codebase's pure-core pattern
(`vision_track/.../core/pan_follow.py`, `.../core/reacq_state.py`). Ticked by the
BT node each tick with `(reacq_state, now)`; returns a small action struct.

**Construction params** (defaults; injectable for tests):
- `dwell_sec: float = 4.0`
- `scan_angles_rad: list[float]` — absolute servo angles for the moving steps,
  default `[radians(-60), radians(0), radians(60)]`.
- `include_current_first: bool = True` — prepend a HOLD-current dwell step.
- enum constant for `REACQ_NEEDS_HELP = 2` (shared with the announce/wave nodes).

**Per-tick output struct** (`RecoveryAction`):
- `speak: str | None` — non-`None` only on a pass-entry tick (the line to speak).
- `pan_target_rad: float | None` — non-`None` only on a moving-step entry tick
  (the absolute pan to command). `None` on HOLD-current steps and dwell ticks.
- `allow_wave: bool` — True iff the current pass ≥ 2.
- `active: bool` — True while a scan is in progress (drives node RUNNING vs
  SUCCESS).

**State** (internal): `pass_idx` (1-based), `step_idx` (0-based over the scan
step list), `step_start: float | None`, `prev_state: int`.

**Spoken lines** (constructor params, defaults):
- Pass 1: `"Please stop walking so I can find you."`
- Pass 2+: `"I've lost you. Please raise your hand."`

**Tick logic:**

```
def tick(reacq_state, now) -> RecoveryAction:
    if reacq_state != REACQ_NEEDS_HELP:
        reset()                       # pass_idx, step_idx, step_start cleared
        return RecoveryAction(active=False)   # idle / re-lock → hand back

    steps = ([HOLD] if include_current_first else []) + scan_angles_rad

    if just entered NEEDS_HELP (prev_state != NEEDS_HELP) or step_start is None:
        pass_idx = pass_idx or 1
        step_idx = 0
        step_start = now
        speak = pass_line(pass_idx)               # pass-entry announce
        pan = pan_for(steps[0])                    # None if HOLD
        return RecoveryAction(speak, pan, allow_wave=(pass_idx>=2), active=True)

    if now - step_start >= dwell_sec:              # advance
        step_idx += 1
        if step_idx >= len(steps):                 # pass complete → next pass
            pass_idx = 2 if pass_idx == 1 else 2   # 1→2, then repeat 2
            step_idx = 0
            step_start = now
            return RecoveryAction(speak=pass_line(pass_idx),
                                  pan=pan_for(steps[0]),
                                  allow_wave=(pass_idx>=2), active=True)
        step_start = now
        return RecoveryAction(speak=None, pan=pan_for(steps[step_idx]),
                              allow_wave=(pass_idx>=2), active=True)

    # mid-dwell: no new command
    return RecoveryAction(speak=None, pan=None,
                          allow_wave=(pass_idx>=2), active=True)
```

`pan_for(HOLD)` → `None`; `pan_for(angle)` → `angle`. `prev_state` is updated at
the end of each tick.

**Dwell rationale:** 4 s ≥ the tracker's 2 s post-camera-motion penalty window,
so each settle fully clears the head-motion handicap before the relaxed
in-NEEDS_HELP commit path tries to lock.

### 2. BT node — `BtNode_RecoveryScan` in `FollowPerson/nodes.py`

A thin adapter (a py_trees `Behaviour`), following the `BtNode_WaveReseed`
single-FSM-node precedent. Holds: a `RecoveryScanFSM`, a `PanTiltCommand`
publisher on `/pan_tilt_controller/cmd`, a `CoalescingTTS` over the `/announce`
`TextToSpeech` client (same construction as `BtNode_ReacqAnnounce`), and the
`detect_waving_persons` + `reseed_target` clients + throttle lifted from
`BtNode_WaveReseed`.

Params (declared on the node, all with the defaults from §E): `recovery_dwell_sec`,
`recovery_scan_angles_deg`, `recovery_scan_tilt_deg`,
`recovery_scan_include_current_first`.

**`update()` per tick:**
1. Read `reacq_state` from `track/reacquisition_state` (default to INACTIVE if
   unset).
2. `action = self._fsm.tick(reacq_state, now())`.
3. If `action.speak`: submit it to the `CoalescingTTS` (non-blocking).
4. If `action.pan_target_rad is not None`: publish
   `PanTiltCommand(mode=ABSOLUTE, pan_rad=target, tilt_rad=radians(tilt_deg),
   speed_raw=0, accel_raw=0)`.
5. If `action.allow_wave`: run the throttled `detect_waving_persons → reseed_target`
   dance (unchanged from `BtNode_WaveReseed`). Skip entirely when the waving
   service is unavailable.
6. Always poll the `CoalescingTTS`.
7. Return `RUNNING` if `action.active` else `SUCCESS` (matches the always-alive
   reaction-branch semantics).

### 3. Trim `BtNode_ReacqAnnounce`

Drop its NEEDS_HELP "raise your hand" branch — `BtNode_RecoveryScan` now owns
all NEEDS_HELP speech, so leaving it in would double-announce. Keep the PASSIVE
"slow down" nudge.

### 4. Tree wiring — `FollowPerson/follow_person.py`

Reactions branch becomes `Sequence(memory=False, [announce, recovery_scan])`
where `announce = BtNode_ReacqAnnounce` (now PASSIVE-only) and
`recovery_scan = BtNode_RecoveryScan`. The standalone `BtNode_WaveReseed` is
removed from the tree (its detect+reseed logic is absorbed into RecoveryScan).
The `FailureIsRunning`-wrapped `Parallel(SuccessOnAll, synchronise=False)` root
is unchanged; RecoveryScan returning RUNNING-while-scanning keeps the parallel
RUNNING (never SUCCESS) during recovery, which is correct — follow must not
"succeed" mid-recovery.

### 5. Tracker guard — `tk26_vision/.../person_track_node.py`

In the pan-follow tick path, skip issuing any pan command while
`self._help_latched` is True (NEEDS_HELP active). One guard; documents the
ownership contract in code.

## Data flow

```
tracker (person_track_node)
  ├─ TrackPerson action feedback.reacquisition_state ──▶ BtNode_TrackPersonAction
  │                                                        └─ writes track/reacquisition_state (blackboard)
  └─ pan-follow tick: SUPPRESSED while _help_latched  (guard)

BtNode_RecoveryScan (ticks each BT tick)
  ├─ reads track/reacquisition_state ─▶ RecoveryScanFSM.tick(state, now)
  ├─ speak  ─▶ /announce (TextToSpeech, CoalescingTTS)
  ├─ pan    ─▶ /pan_tilt_controller/cmd (PanTiltCommand ABSOLUTE, tilt 40°)
  └─ wave   ─▶ /detect_waving_persons ─▶ /person_track_node/reseed_target  (Pass 2 only)
                                              └─ tracker re-locks ─▶ reacq_state → TRACKING ─▶ FSM idle, guard releases
```

## Error handling & degradation

- **No `/pan_tilt_controller/cmd` subscriber** (sim/bench, no servo): publish is
  a harmless no-op.
- **No `/announce` server**: `client.call_async` future never completes; the
  `CoalescingTTS` simply never promotes — no crash, no block.
- **No `detect_waving_persons` server** (sim: launched `UnlessCondition`):
  RecoveryScan skips the wave step and keeps scanning (existing WaveReseed
  tolerance preserved).
- **`reacq_state` blackboard key unset**: treated as INACTIVE → FSM idle.
- **Goal cancel / permanent loss → INACTIVE**: FSM resets to idle; node returns
  SUCCESS; tree stays alive via the existing `FailureIsRunning` wrapping.

## Parameters (new, on `BtNode_RecoveryScan`)

| Param | Default | Meaning |
|---|---|---|
| `recovery_dwell_sec` | `4.0` | Seconds dwelt at each scan angle (3–5 s range). |
| `recovery_scan_angles_deg` | `[-60.0, 0.0, 60.0]` | Absolute servo pan angles for the moving steps. |
| `recovery_scan_tilt_deg` | `40.0` | Fixed tilt during scan; **must match** tracker `fixed_tilt_deg`. |
| `recovery_scan_include_current_first` | `true` | Prepend a HOLD-current dwell as scan step 0. |

## Testing

**Pure FSM (`recovery_scan.py`)** — ROS-free, inject `now`:
- Non-NEEDS_HELP state → idle (`active=False`), no pan/speak.
- NEEDS_HELP entry → Pass-1 line spoken, step 0 = HOLD (pan `None`).
- Dwell timing: pan target emitted only after `now` advances ≥ `dwell_sec`; the
  4-step sequence (`HOLD, -60, 0, +60`) over time.
- Pass-1 sweep complete → Pass-2 line spoken, `allow_wave` flips True at Pass 2.
- Pass-2 sweep complete → repeats Pass 2 (line re-spoken, `allow_wave` stays
  True).
- `allow_wave` is False for all of Pass 1, True for all of Pass 2+.
- Re-lock mid-scan (`state → TRACKING`) → idle next tick, internal state reset
  (fresh loss restarts at Pass 1).

**BT node integration** — mock the publisher + service clients:
- Verifies pan publish on step-entry ticks (ABSOLUTE mode, tilt 40°), `/announce`
  submit on pass entry, wave dance only when `allow_wave`, RUNNING vs SUCCESS.

**Tracker guard** — `tk26_vision`:
- Assert the pan-follow tick issues no `PanTiltCommand` while `_help_latched` is
  True, and resumes once it clears.

## Files

- **Create** `src/tk25_decision/src/behavior_tree/behavior_tree/FollowPerson/recovery_scan.py`
- **Modify** `.../FollowPerson/nodes.py` (add `BtNode_RecoveryScan`; trim
  `BtNode_ReacqAnnounce`)
- **Modify** `.../FollowPerson/follow_person.py` (rewire reactions branch)
- **Create** `.../test/` tests for the FSM + node
- **Modify** `src/tk26_vision/src/vision_track/vision_track/person_track_node.py`
  (pan-follow guard)
- **Modify** `.../vision_track/test/` (guard test)
- **Modify** READMEs/changelogs in the same commits as their code.
```

