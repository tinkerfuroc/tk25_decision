# Recovery Wave: Settle-Gated, Sequenced Detection + 3.5 m Gate — Plan

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development. One commit per phase. Steps use `- [ ]`.

**Goal:** In the NEEDS_HELP recovery scan's Pass 2, the head must fully stop at each angle (≥ `settle_sec` since the turn) **before** firing one `DetectWaving`, then **wait for the response** before advancing; and only wavers ≤ `wave_max_distance_m` (3.5 m) count.

**Approach:** "Fully stopped" = a pure FSM timer (≥2 s since the angle's pan command) — no servo feedback. The `RecoveryScanFSM` gains a `settle_sec` gate, a one-shot `detect_now` output, a `wave_busy` input, and a `detect_timeout_sec` safety cap; it fires the detect once after settle and holds the angle until the round-trip completes (or times out). `WaveReseedCycle` becomes trigger-driven (no throttle) and passes `threshold_meters = wave_max_distance_m` to the waving server (which already drops wavers beyond it). The now-dead `BtNode_WaveReseed` (+ its test) is deleted.

**Repo:** all changes in `tk25_decision` (`/home/tinker/tk25_ws/src/tk25_decision`, branch `dev`). **Concurrent committers active** — pathspec-only staging, NEVER `git add -A`/`.`, NEVER `--amend`/rebase. Verify `git rev-parse HEAD` before, parent==pre-HEAD after. Commit trailer: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.

**Test cwd:** `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/<f>.py -v`. Lint `python -m flake8 <f> --max-line-length=100` (changed lines clean). `behavior_tree.messages` is unimportable on this host → message imports stay lazy inside `setup()`/bridge.

**Deploy (after R4, NOT plain colcon):** `cd /home/tinker/tk25_ws && tkbuild tk25_decision --packages-select behavior_tree` (NEVER `colcon --symlink-install` — breaks ament_python entry-point metadata).

**Design note — node `update()` order is load-bearing.** `wave_busy` is computed BEFORE ticking the FSM (so a detect fired last tick reads as busy this tick), and the cycle is reset whenever a new angle's pan is published. This avoids a premature "not busy → advance" on the trigger tick.

---

## Phase R1: Delete the dead `BtNode_WaveReseed`

`BtNode_WaveReseed` left the follow tree in P4 and is instantiated nowhere; its wave→reseed logic now lives in `WaveReseedCycle`. Remove it so the cycle can be reshaped freely.

**Files:**
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/nodes.py`
- Delete: `src/behavior_tree/test/test_wave_reseed.py`
- Modify: `src/behavior_tree/README.md` (changelog)

- [ ] **Step 1: Confirm nothing imports it**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision && grep -rn "BtNode_WaveReseed" src/behavior_tree --include=*.py | grep -v test_wave_reseed.py`
Expected: only the `class BtNode_WaveReseed` definition line in `nodes.py` (no importers — `follow_person.py` imports `BtNode_RecoveryScan`, not this).

- [ ] **Step 2: Delete the class from `nodes.py`**

Remove the entire `class BtNode_WaveReseed(py_trees.behaviour.Behaviour):` block (its docstring through the end of its `update()` method). KEEP the top-of-file import line `from behavior_tree.FollowPerson.wave_reseed_cycle import (WaveReseedCycle, _WaveReseedBridge)` — both are still used by `BtNode_RecoveryScan`. KEEP `REACQ_TRACKING`/`REACQ_PASSIVE`/`REACQ_NEEDS_HELP` module constants (used elsewhere).

- [ ] **Step 3: Delete the test file**

Run: `git rm src/behavior_tree/test/test_wave_reseed.py`

- [ ] **Step 4: Verify the rest of the suite is green**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_node.py test/test_recovery_scan_fsm.py test/test_wave_reseed_cycle.py test/test_reacq_announce.py test/test_follow_tree_build.py -v`
Expected: all PASS (no reference to the deleted class).

- [ ] **Step 5: Changelog + commit**

Add to `src/behavior_tree/README.md` `## 📜 Changelog` (newest on top):
```markdown
- **2026-06-15** — Removed the dead `BtNode_WaveReseed` node + its test. It left
  the follow tree in the recovery-scan rewire and was instantiated nowhere; the
  wave→reseed cycle now lives only in `WaveReseedCycle`, driven by
  `BtNode_RecoveryScan` Pass 2.
```
Commit (pathspec-only): `nodes.py`, the test deletion, `README.md`.

```bash
git rev-parse HEAD
git add -- src/behavior_tree/behavior_tree/FollowPerson/nodes.py src/behavior_tree/README.md
git rm --cached -- src/behavior_tree/test/test_wave_reseed.py 2>/dev/null; git add -- src/behavior_tree/test/test_wave_reseed.py
git diff --cached --name-only   # exactly these 3 paths (test shows as deleted)
git commit -m "refactor(behavior_tree): delete dead BtNode_WaveReseed

Unused since the recovery-scan rewire (left the follow tree in P4,
instantiated nowhere). The wave->reseed cycle lives in WaveReseedCycle,
driven by BtNode_RecoveryScan Pass 2. Frees the cycle to be reshaped.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

---

## Phase R2: Reshape `WaveReseedCycle` — trigger-driven + distance threshold

Replace the free-running throttle with an explicit `trigger()` (fire one DetectWaving on the next `step()`), pass `threshold_meters`, and expose `is_idle`.

**Files:**
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/wave_reseed_cycle.py`
- Modify: `src/behavior_tree/test/test_wave_reseed_cycle.py`

- [ ] **Step 1: Rewrite the test** — `src/behavior_tree/test/test_wave_reseed_cycle.py`

```python
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
    c.step(); c.step(); c.step()
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
        c.trigger(); c.step(); c.step()
        assert bridge.reseed_boxes == []
        assert c.is_idle is True


def test_reset_clears_pending_and_phase():
    bridge = _FakeBridge()
    bridge.next_detect = _detect(0, [_roi(10, 20, 30, 40)])
    c = _cycle(bridge)
    c.trigger(); c.step()               # WAVE_PENDING
    assert c.is_idle is False
    c.reset()
    assert c.is_idle is True
    # A pending trigger that was reset before firing does not fire later.
    c.trigger(); c.reset(); c.step()
    assert bridge.detect_calls == 1     # only the first (pre-reset) detect


def test_no_bridge_trigger_step_is_inert():
    c = WaveReseedCycle(bridge=None, distance_threshold_m=3.5)
    c.trigger(); c.step()
    assert c.is_idle is True            # nothing to fire; pending consumed
    assert c.has_bridge is False
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_wave_reseed_cycle.py -v`
Expected: FAIL (no `trigger`/`is_idle`; `detect_async` takes no threshold; throttle-based old code).

- [ ] **Step 3: Rewrite `wave_reseed_cycle.py`**

```python
# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Trigger-driven wave->reseed async machine for NEEDS_HELP recovery.

``WaveReseedCycle`` fires ONE ``DetectWaving`` (gated to ``distance_threshold_m``)
when ``trigger()`` has been called, then walks IDLE -> WAVE_PENDING ->
RESEED_PENDING, at most one service call in flight, one transition per ``step()``.
It does NOT free-run: without a ``trigger()`` it stays idle. ROS-free (injected
``bridge``) so it unit-tests with a fake. ``_WaveReseedBridge`` is the live ROS
adapter (lazy tk26 msg import). The recovery FSM fires one detect per scan angle
AFTER the head has settled and waits for the response (``is_idle``) before
advancing.
"""


class _WaveReseedBridge:
    """Live ROS bridge: async DetectWaving + ReseedTarget.

    ``detect_async(threshold_m)`` requests wavers within ``threshold_m`` of the
    camera (the server drops any waver whose centroid depth exceeds it) and
    returns an rclpy Future of a ``DetectWaving.Response`` (read ``.status`` +
    ``.waving_boxes``). ``reseed_async(box)`` returns a Future of a
    ``ReseedTarget.Response`` (read ``.success``). ``box`` is ``(x0,y0,x1,y1)``.
    """

    def __init__(self, node, waving_service, reseed_service):
        # Lazy import (live path only) so mock/test never need tk26 msgs.
        from behavior_tree.messages import DetectWaving, ReseedTarget
        self._DetectWaving = DetectWaving
        self._ReseedTarget = ReseedTarget
        self._wave_cli = node.create_client(DetectWaving, waving_service)
        self._reseed_cli = node.create_client(ReseedTarget, reseed_service)

    def detect_async(self, threshold_m):
        req = self._DetectWaving.Request()
        req.threshold_meters = float(threshold_m)
        # min_waving_persons=0 -> fast MediaPipe-only path (no VLM fallback).
        req.min_waving_persons = 0
        return self._wave_cli.call_async(req)

    def reseed_async(self, box):
        req = self._ReseedTarget.Request()
        x0, y0, x1, y1 = box
        req.bbox.x_offset = max(0, int(x0))
        req.bbox.y_offset = max(0, int(y0))
        req.bbox.width = max(0, int(x1 - x0))
        req.bbox.height = max(0, int(y1 - y0))
        req.frame_id = ""
        return self._reseed_cli.call_async(req)


class WaveReseedCycle:
    """One-shot, trigger-driven DetectWaving -> ReseedTarget cycle.

    ``trigger()`` arms one detect; the next ``step()`` while IDLE fires it (gated
    to ``distance_threshold_m``). On an UNAMBIGUOUS single waver (status OK,
    exactly one box) it reseeds the tracker; zero/multiple wavers or a non-OK
    status -> NO reseed. ``is_idle`` is False from ``trigger()`` until the
    round-trip completes, so the caller can wait on the response.
    """

    _IDLE, _WAVE_PENDING, _RESEED_PENDING = "idle", "wave_pending", "reseed_pending"

    def __init__(self, bridge=None, distance_threshold_m: float = 3.5):
        self._bridge = bridge
        self.distance_threshold_m = float(distance_threshold_m)
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._armed = False

    def set_bridge(self, bridge):
        self._bridge = bridge

    @property
    def has_bridge(self) -> bool:
        return self._bridge is not None

    @property
    def is_idle(self) -> bool:
        """True iff no detect is armed and no request is in flight."""
        return self._phase == self._IDLE and not self._armed

    def trigger(self) -> None:
        """Arm one detect; the next IDLE ``step()`` fires it."""
        self._armed = True

    def reset(self) -> None:
        """Cancel any armed/in-flight cycle."""
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._armed = False

    def step(self) -> str:
        """Advance one transition of IDLE -> WAVE_PENDING -> RESEED_PENDING."""
        if self._phase == self._IDLE:
            if self._armed:
                self._armed = False
                if self._bridge is not None:
                    self._future = self._bridge.detect_async(
                        self.distance_threshold_m)
                    self._phase = self._WAVE_PENDING
                    return f"Scanning for a waver (<= {self.distance_threshold_m:.1f} m)"
            return "Idle"

        if self._phase == self._WAVE_PENDING:
            if self._future is None or not self._future.done():
                return "Awaiting waving result"
            resp = self._future.result()
            self._future = None
            box = self._single_waver_box(resp)
            if box is None:
                self._phase = self._IDLE
                return "No unambiguous single waver"
            self._future = self._bridge.reseed_async(box)
            self._pending_box = box
            self._phase = self._RESEED_PENDING
            return "Single waver found; reseeding tracker"

        if self._phase == self._RESEED_PENDING:
            if self._future is None or not self._future.done():
                return "Awaiting reseed result"
            resp = self._future.result()
            ok = bool(getattr(resp, "success", False))
            self._future = None
            self._pending_box = None
            self._phase = self._IDLE
            return "Reseed succeeded" if ok else "Reseed did not match a person"

        return ""

    @staticmethod
    def _single_waver_box(resp):
        """``(x0, y0, x1, y1)`` iff exactly one waver with status OK, else None."""
        if resp is None or int(getattr(resp, "status", -1)) != 0:
            return None
        boxes = list(getattr(resp, "waving_boxes", []) or [])
        if len(boxes) != 1:
            return None
        b = boxes[0]
        x0 = int(b.x_offset)
        y0 = int(b.y_offset)
        return (x0, y0, x0 + int(b.width), y0 + int(b.height))
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_wave_reseed_cycle.py -v`
Expected: PASS (5 tests). Lint both files clean.

- [ ] **Step 5: Commit (R2)** — pathspec `wave_reseed_cycle.py` + `test_wave_reseed_cycle.py`.
```bash
git rev-parse HEAD
git add -- src/behavior_tree/behavior_tree/FollowPerson/wave_reseed_cycle.py src/behavior_tree/test/test_wave_reseed_cycle.py
git diff --cached --name-only   # exactly these 2
git commit -m "feat(behavior_tree): WaveReseedCycle trigger-driven + 3.5m gate

Replace the free-running throttle with one-shot trigger(): fire one
DetectWaving (threshold_meters=distance_threshold_m, default 3.5) only when
armed, expose is_idle so the recovery FSM can wait for the response before
advancing. detect_async now passes the distance threshold the waving server
already honors.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

---

## Phase R3: `RecoveryScanFSM` — settle gate + sequenced detect

Pass 2 per angle: command pan → wait `settle_sec` → one-shot `detect_now` → hold while `wave_busy` (response in flight) → advance when not busy OR `detect_timeout_sec` elapsed. Pass 1 unchanged (fixed `dwell_sec`).

**Files:**
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/recovery_scan.py`
- Modify: `src/behavior_tree/test/test_recovery_scan_fsm.py`

- [ ] **Step 1: Add failing tests** — append to `src/behavior_tree/test/test_recovery_scan_fsm.py` (keep the existing tests; the Pass-1 ones still hold). Add at the top of the file (after the existing imports) nothing new; add these tests:

```python
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
    a = f.tick(NEEDS_HELP, now=s + 9.0, wave_busy=True)
    assert a.pan_target_rad is None                 # still held (busy), pre-timeout window
    # Response arrives (not busy): advance to the next angle (-60).
    a = f.tick(NEEDS_HELP, now=s + 9.1, wave_busy=False)
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
```

(The existing Pass-1 tests must be updated for the new `dwell_sec` only if they relied on 4.0; they construct their own FSM with explicit `dwell_sec`, so re-check each `_fsm()` helper / explicit dwell value still matches the sweep timings. If the module-level `_fsm()` used `dwell_sec=4.0`, leave it — those tests still pass with 4.0; the new tests pass explicit 7.0.)

- [ ] **Step 2: Run to verify the new tests fail**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_fsm.py -v`
Expected: the 4 new tests FAIL (`detect_now`/`settle_sec`/`wave_busy` not present), existing tests still PASS.

- [ ] **Step 3: Rewrite `recovery_scan.py`**

```python
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
```

- [ ] **Step 4: Run all FSM tests**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_fsm.py -v`
Expected: ALL pass (existing + 4 new). Lint clean.

- [ ] **Step 5: Commit (R3)** — pathspec `recovery_scan.py` + `test_recovery_scan_fsm.py`.
```bash
git rev-parse HEAD
git add -- src/behavior_tree/behavior_tree/FollowPerson/recovery_scan.py src/behavior_tree/test/test_recovery_scan_fsm.py
git diff --cached --name-only
git commit -m "feat(behavior_tree): RecoveryScanFSM settle-gated sequenced detect

Pass 2 per angle now: wait settle_sec (2s, 'head fully stopped' = time since
the turn) -> fire ONE detect (detect_now one-shot) -> hold the angle while
wave_busy (response in flight) -> advance when not busy or detect_timeout_sec
elapsed (no-server safety). Pass 1 unchanged (fixed dwell_sec). Adds settle_sec/
detect_timeout_sec params, detect_now output, wave_busy input.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

---

## Phase R4: Wire `BtNode_RecoveryScan` + params + changelog + deploy

**Files:**
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/nodes.py`
- Modify: `src/behavior_tree/test/test_recovery_scan_node.py`
- Modify: `src/behavior_tree/README.md`

- [ ] **Step 1: Update the node test** — `test/test_recovery_scan_node.py`. Update the `_FakeBridge.detect_async` to take `threshold_m`, and add tests that detect fires only after settle and that the threshold is passed. Replace the existing `_FakeBridge` with:

```python
class _FakeBridge:
    def __init__(self):
        self.detect_calls = 0
        self.detect_thresholds = []
        self.reseed_boxes = []
        self.next_detect = SimpleNamespace(status=0, waving_boxes=[])

    def detect_async(self, threshold_m):
        self.detect_calls += 1
        self.detect_thresholds.append(threshold_m)
        return _FakeFuture(self.next_detect)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _FakeFuture(SimpleNamespace(success=True))
```

Replace `test_pass1_never_calls_wave_bridge` and `test_pass2_drives_wave_bridge` with these (the node now uses `dwell_sec=7.0` Pass-1 and `settle_sec` Pass-2; the helper `_make` must pass `dwell_sec=7.0, settle_sec=2.0, wave_max_distance_m=3.5`):

```python
def test_pass1_never_calls_wave_bridge():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _FakeBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    for t in [0.0, 7.0, 14.0, 21.0]:        # all of Pass 1 (dwell 7s)
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
    assert bridge.detect_calls == 0


def test_pass2_detects_after_settle_with_threshold():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _FakeBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    # Walk through Pass 1 into Pass 2 entry (4 angles * 7s).
    for t in [0.0, 7.0, 14.0, 21.0, 28.0]:
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
    assert PASS2 in tts.submitted
    # At Pass-2 entry + <settle: no detect yet.
    clock.now = 28.0 + 1.9
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert bridge.detect_calls == 0
    # >= settle: one detect with the 3.5 m threshold.
    clock.now = 28.0 + 2.0
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert bridge.detect_calls == 1
    assert bridge.detect_thresholds == [3.5]
```

Ensure the test file imports `_FakeFuture` (add the small class if not already present) and `SimpleNamespace`. Update `_make` to:
```python
def _make(clock, tts, pan_calls, bridge=None):
    bb = "track/reacquisition_state"
    node = BtNode_RecoveryScan(name="RecoveryScan", dwell_sec=7.0, settle_sec=2.0,
                               wave_max_distance_m=3.5, bb_key=bb, clock=clock)
    node.inject_coalescer(tts)
    node.inject_pan_sink(lambda pan, tilt: pan_calls.append((pan, tilt)))
    if bridge is not None:
        node.inject_wave_bridge(bridge)
    return node, _writer(bb), bb
```
Keep the other existing node tests; update any that assumed `dwell_sec=4.0` timing to 7.0.

- [ ] **Step 2: Run to verify failures**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_node.py -v`
Expected: FAIL (node lacks `settle_sec`/`wave_max_distance_m`; `detect_async` arity; old throttled wiring).

- [ ] **Step 3: Update `BtNode_RecoveryScan` in `nodes.py`**

In `__init__`, the signature becomes (PRESERVE the committed `dwell_sec=7.0`, `tilt_deg=37.0`; DROP `wave_throttle_s`; ADD `settle_sec`, `detect_timeout_sec`, `wave_max_distance_m`):
```python
    def __init__(
        self,
        name: str,
        dwell_sec: float = 7.0,
        settle_sec: float = 2.0,
        detect_timeout_sec: float = 5.0,
        scan_angles_deg=(-60.0, 0.0, 60.0),
        tilt_deg: float = 37.0,
        include_current_first: bool = True,
        bb_key: str = "track/reacquisition_state",
        announce_service: str = "announce",
        waving_service: str = "detect_waving_persons",
        reseed_service: str = "/person_track_node/reseed_target",
        pan_cmd_topic: str = "/pan_tilt_controller/cmd",
        wave_max_distance_m: float = 3.5,
        clock=time.monotonic,
    ):
```
Construct the cycle and FSM with the new params (replace the old `WaveReseedCycle(throttle_s=...)` and `RecoveryScanFSM(dwell_sec=..., scan_angles_rad=..., include_current_first=...)`):
```python
        self._wave = WaveReseedCycle(distance_threshold_m=wave_max_distance_m)
        self._fsm = RecoveryScanFSM(
            dwell_sec=dwell_sec,
            settle_sec=settle_sec,
            detect_timeout_sec=detect_timeout_sec,
            scan_angles_rad=[math.radians(a) for a in scan_angles_deg],
            include_current_first=include_current_first,
        )
```
Rewrite `update()` (the load-bearing order — compute `wave_busy` BEFORE the tick; reset the cycle on a new angle; trigger on `detect_now`):
```python
    def update(self) -> Status:
        try:
            state = int(self._bb.get(self.bb_key))
        except Exception:
            state = REACQ_TRACKING
        # Busy iff a detect is armed/in flight (computed BEFORE ticking so a
        # detect fired last tick reads as busy this tick).
        wave_busy = self._wave.has_bridge and not self._wave.is_idle
        action = self._fsm.tick(state, self._clock(), wave_busy)

        if action.speak and self._tts is not None:
            self._tts.submit(action.speak)
        if self._tts is not None:
            self._tts.poll()

        # New scan angle: command the head and cancel any stale detect cycle.
        if action.pan_target_rad is not None:
            if self._pan_sink is not None:
                self._pan_sink(action.pan_target_rad, self._tilt_rad)
            self._wave.reset()

        # Settled at the angle: arm one detect (Pass 2 only).
        if action.detect_now:
            self._wave.trigger()

        # Advance the wave cycle while wave is allowed (Pass 2); else keep it idle.
        if action.allow_wave and self._wave.has_bridge:
            self.feedback_message = self._wave.step()
        else:
            self._wave.reset()

        if action.active:
            return Status.RUNNING
        self.feedback_message = "Idle (not in NEEDS_HELP)"
        return Status.SUCCESS
```
(The `_DoneHandle`, `setup()`, and injection hooks are unchanged.)

- [ ] **Step 4: Run node tests + full follow suite**

Run:
```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
python -m pytest test/test_recovery_scan_node.py test/test_recovery_scan_fsm.py \
  test/test_wave_reseed_cycle.py test/test_reacq_announce.py test/test_follow_tree_build.py -v
```
Expected: ALL pass. Lint `nodes.py test/test_recovery_scan_node.py` clean on changed lines.

- [ ] **Step 5: Changelog**

Add to `README.md` `## 📜 Changelog` (newest on top):
```markdown
- **2026-06-15** — Recovery Pass-2 wave detection is now settle-gated and
  sequenced. At each scan angle the head must be fully stopped (≥ `settle_sec`,
  2 s, since the turn) before ONE `DetectWaving` fires; the scan then waits for
  the response (or `detect_timeout_sec`, 5 s) before advancing — no more firing
  mid-slew. Wavers are gated to `wave_max_distance_m` (3.5 m) via the request's
  `threshold_meters` (the waving server drops farther wavers). `WaveReseedCycle`
  is now trigger-driven (throttle removed). New `BtNode_RecoveryScan` params:
  `recovery`/`settle_sec=2.0`, `detect_timeout_sec=5.0`, `wave_max_distance_m=3.5`.
```

- [ ] **Step 6: Commit (R4)** — pathspec `nodes.py`, `test_recovery_scan_node.py`, `README.md`.
```bash
git rev-parse HEAD
git add -- src/behavior_tree/behavior_tree/FollowPerson/nodes.py src/behavior_tree/test/test_recovery_scan_node.py src/behavior_tree/README.md
git diff --cached --name-only
git commit -m "feat(behavior_tree): wire settle-gated wave detect into RecoveryScan

update() computes wave_busy before the FSM tick, resets the cycle on each new
angle, arms one detect on detect_now (after 2s settle), and holds the angle
until the response (or 5s timeout). New params settle_sec=2.0,
detect_timeout_sec=5.0, wave_max_distance_m=3.5 (DetectWaving threshold_meters);
dropped wave_throttle_s. Preserves live tuning dwell 7s / tilt 37deg.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

- [ ] **Step 7: Deploy via tkbuild**
```bash
cd /home/tinker/tk25_ws && tkbuild tk25_decision --packages-select behavior_tree
```
Verify `behavior-tree` metadata still resolves (real egg-info), e.g. the follow-person entry point is present.

---

## Self-review
- Settle gate (≥2 s since the turn) before detect — R3 FSM `elapsed < settle_sec`; node tests R4.
- One detect per angle, wait for response — R3 `detect_now` one-shot + hold-while-busy; R2 cycle `trigger()`/`is_idle`.
- No-server safety — R3 `detect_timeout_sec` advance.
- 3.5 m gate — R2 `detect_async(threshold_m)` → `threshold_meters`; default `wave_max_distance_m=3.5`.
- Dead node removed — R1.
- Live tuning preserved — `dwell_sec=7.0`, `tilt_deg=37.0` kept in R4 `__init__`.
- Types consistent: `RecoveryAction.detect_now`, FSM `tick(state, now, wave_busy)`, cycle `trigger()`/`is_idle`/`distance_threshold_m`/`detect_async(threshold_m)` used identically across R2/R3/R4.
