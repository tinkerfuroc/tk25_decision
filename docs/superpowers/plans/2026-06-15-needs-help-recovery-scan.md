# NEEDS_HELP Two-Pass Head-Scan Recovery — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When the person tracker latches into NEEDS_HELP, the behavior tree actively recovers — ask the person to stop, sweep the head across `[current, -60°, 0°, +60°]` dwelling ~4 s at each angle for re-lock; if that fails, ask the operator to raise a hand and sweep again with wave-reseed active, repeating until re-lock or cancel.

**Architecture:** BT-centric. A pure `RecoveryScanFSM` (no ROS) owns the pass/scan/dwell logic; a thin `BtNode_RecoveryScan` executes its per-tick actions — speech via the existing `/announce` `CoalescingTTS`, ABSOLUTE `PanTiltCommand` to `/pan_tilt_controller/cmd`, and (Pass 2 only) the wave→reseed cycle. The wave→reseed async machine is extracted from `BtNode_WaveReseed` into a shared `WaveReseedCycle` so both nodes reuse it. One mutual-exclusion invariant keyed on `reacq_state` governs head ownership; a single tracker-side guard (`pan_follow_suppressed`) enforces it.

**Tech Stack:** Python 3.10, `py_trees`, ROS2 Humble (`rclpy`), `tinker_vision_msgs_26` (PanTiltCommand/DetectWaving/ReseedTarget), `tinker_audio_msgs` (TextToSpeech). Tests: `pytest`.

**Spec:** `src/tk25_decision/docs/superpowers/specs/2026-06-15-needs-help-recovery-scan-design.md`

---

## Repos & git discipline

Two independent git repos, each its own root:
- `tk25_decision` → root `/home/tinker/tk25_ws/src/tk25_decision`, branch `dev`. **Concurrent committers active** (GPSR + FollowAction work in the working tree). **Stage ONLY the exact pathspecs listed per phase. NEVER `git add -A`/`.`** Verify HEAD before/after each commit; never `--amend`/rebase.
- `tk26_vision` → root `/home/tinker/tk25_ws/src/tk26_vision`, branch `dev`. Same pathspec-only rule.

Every commit message ends with the trailer:
```
Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
```

**One commit per phase.** Phases P1–P4 land in `tk25_decision`; P5 lands in `tk26_vision`.

## Test invocation

- **tk25_decision** (no build needed — pure pytest on source):
  ```bash
  cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
  python -m pytest test/<file>.py -v
  ```
  (`behavior_tree/` is importable because it is a subdir of the cwd. `behavior_tree.messages` is NOT importable on this host — the TTSCnRequest gotcha — so production code MUST import message types lazily inside `setup()`/bridge constructors, never at module top. All tests inject fakes and never hit those paths.)
- **tk26_vision** (pure pytest on source, under the vision venv):
  ```bash
  source /home/tinker/tk25_ws/src/tk26_vision/.venv-vision-main/bin/activate
  cd /home/tinker/tk25_ws/src/tk26_vision/src/vision_track
  python -m pytest test/test_pan_follow.py -v
  ```
- **Lint** (changed lines must be clean; pre-existing E501s in untouched lines are out of scope):
  ```bash
  python -m flake8 <changed_file> --max-line-length=100
  ```

## Deploy (after all phases; not a per-phase gate)

```bash
# ALWAYS tkbuild, NEVER plain `colcon build --symlink-install`: symlink/develop
# install leaves an ament_python pkg with an .egg-link and no resolvable
# metadata, so `ros2 run` dies with PackageNotFoundError. tkbuild strips
# --symlink-install and writes a real .egg-info.
# tk25_decision (behavior_tree)
tkbuild tk25_decision --packages-select behavior_tree
# tk26_vision (root install tree the bench resolves)
tkbuild tk26_vision --packages-select vision_track
```

---

## File Structure

**tk25_decision** (`src/behavior_tree/behavior_tree/FollowPerson/`):
- **Create** `wave_reseed_cycle.py` — `WaveReseedCycle` (shared wave→reseed async state machine) + `_WaveReseedBridge` (moved here). One responsibility: drive a throttled DetectWaving→ReseedTarget cycle from a bridge.
- **Create** `recovery_scan.py` — `RecoveryAction` (dataclass) + `RecoveryScanFSM` (pure pass/scan/dwell machine). No ROS.
- **Modify** `nodes.py` — refactor `BtNode_WaveReseed` to delegate to `WaveReseedCycle`; trim `BtNode_ReacqAnnounce` to PASSIVE-only; add `BtNode_RecoveryScan`.
- **Modify** `follow_person.py` — reactions branch `[announce, recovery_scan]`.

**tk25_decision** (`src/behavior_tree/test/`):
- **Create** `test_wave_reseed_cycle.py`, `test_recovery_scan_fsm.py`, `test_recovery_scan_node.py`.
- **Modify** `test_reacq_announce.py` (NEEDS_HELP no longer announces), `test_follow_tree_build.py` (reactions now hold RecoveryScan).
- **Unchanged & must stay green:** `test_wave_reseed.py` (interface preserved).

**tk26_vision** (`src/vision_track/`):
- **Modify** `vision_track/core/pan_follow.py` — add `pan_follow_suppressed` predicate.
- **Modify** `vision_track/person_track_node.py` — `_pan_follow_tick` uses the predicate (adds `_help_latched` gate).
- **Modify** `test/test_pan_follow.py` — predicate test.

**Docs:** `src/behavior_tree/readme.md` (or the package's changelog doc) updated in P4; `src/vision_track/readme.md` changelog in P5.

---

## Phase P1: Extract `WaveReseedCycle` (behavior-preserving refactor)

**Goal:** Move the wave→reseed async machine out of `BtNode_WaveReseed` into a reusable `WaveReseedCycle`, so Pass 2 of the recovery scan can reuse it. `test_wave_reseed.py` must pass **unchanged**.

**Files:**
- Create: `src/behavior_tree/behavior_tree/FollowPerson/wave_reseed_cycle.py`
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/nodes.py`
- Create: `src/behavior_tree/test/test_wave_reseed_cycle.py`

### Task P1.1: Create the `WaveReseedCycle` module

- [ ] **Step 1: Write the failing test** — `src/behavior_tree/test/test_wave_reseed_cycle.py`

```python
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
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_wave_reseed_cycle.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'behavior_tree.FollowPerson.wave_reseed_cycle'`.

- [ ] **Step 3: Create the module** — `src/behavior_tree/behavior_tree/FollowPerson/wave_reseed_cycle.py`

```python
# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Shared wave->reseed async machine used by the NEEDS_HELP reactions.

``WaveReseedCycle`` drives a throttled ``DetectWaving`` -> ``ReseedTarget`` cycle
through a small IDLE -> WAVE_PENDING -> RESEED_PENDING state machine, at most one
service call in flight, advanced one step per ``step()`` call. It is ROS-free
(driven by an injected ``bridge``) so it unit-tests with a fake bridge.
``_WaveReseedBridge`` is the live ROS adapter (lazy tk26 msg import).
"""

import time


class _WaveReseedBridge:
    """Live ROS bridge: async DetectWaving + ReseedTarget.

    ``detect_async()`` returns an rclpy Future of a ``DetectWaving.Response``
    (read ``.status`` + ``.waving_boxes``); ``reseed_async(box)`` returns a Future
    of a ``ReseedTarget.Response`` (read ``.success``). ``box`` is
    ``(x0, y0, x1, y1)`` in the current color frame.
    """

    def __init__(self, node, waving_service, reseed_service):
        # Lazy import (live path only) so mock/test never need tk26 msgs.
        from behavior_tree.messages import DetectWaving, ReseedTarget
        self._DetectWaving = DetectWaving
        self._ReseedTarget = ReseedTarget
        self._wave_cli = node.create_client(DetectWaving, waving_service)
        self._reseed_cli = node.create_client(ReseedTarget, reseed_service)

    def detect_async(self):
        # min_waving_persons=0 (default) -> fast MediaPipe-only path (no VLM
        # fallback), well inside the vision budget.
        return self._wave_cli.call_async(self._DetectWaving.Request())

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
    """Throttled DetectWaving -> ReseedTarget cycle, advanced one step per tick.

    On an UNAMBIGUOUS single waver (status OK, exactly one box) it reseeds the
    tracker onto that box; zero/multiple wavers or a non-OK status -> NO reseed
    (precision: never re-lock onto an ambiguous candidate). DetectWaving is
    throttled to ``throttle_s`` (measured from the start of the previous scan).
    """

    _IDLE, _WAVE_PENDING, _RESEED_PENDING = "idle", "wave_pending", "reseed_pending"

    def __init__(self, bridge=None, throttle_s: float = 3.0, clock=time.monotonic):
        self._bridge = bridge
        self.throttle_s = throttle_s
        self._clock = clock
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._last_detect_time = -1e9

    def set_bridge(self, bridge):
        self._bridge = bridge

    @property
    def has_bridge(self) -> bool:
        return self._bridge is not None

    def reset(self) -> None:
        """Cancel any in-flight cycle (does NOT reset the throttle clock)."""
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None

    def step(self) -> str:
        """Advance one step of IDLE -> WAVE_PENDING -> RESEED_PENDING."""
        now = self._clock()

        if self._phase == self._IDLE:
            if (now - self._last_detect_time) >= self.throttle_s:
                self._future = self._bridge.detect_async()
                self._last_detect_time = now
                self._phase = self._WAVE_PENDING
                return "Scanning for a waving operator"
            return "Waiting to re-scan"

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
Expected: PASS (5 tests).

### Task P1.2: Refactor `BtNode_WaveReseed` to delegate

- [ ] **Step 1: Edit `nodes.py`** — replace the `_WaveReseedBridge` class (lines 200–232) and the body of `BtNode_WaveReseed` so it delegates to `WaveReseedCycle`, preserving its public interface.

At the top of `nodes.py`, after `from behavior_tree.config import is_node_mocked`, add:
```python
from behavior_tree.FollowPerson.wave_reseed_cycle import (
    WaveReseedCycle,
    _WaveReseedBridge,
)
```

**Delete** the `_WaveReseedBridge` class definition now living in `wave_reseed_cycle.py` (old `nodes.py:200–232`).

**Replace** the `BtNode_WaveReseed` class (old `nodes.py:235–410`) with this delegating version (same docstring kept):
```python
class BtNode_WaveReseed(py_trees.behaviour.Behaviour):
    """Wave-to-reseed recovery while the tracker is latched in NEEDS_HELP.

    Thin py_trees wrapper around the shared ``WaveReseedCycle``: while in
    NEEDS_HELP it advances the cycle (throttled DetectWaving -> ReseedTarget on an
    unambiguous single waver); leaving NEEDS_HELP cancels any in-flight cycle.
    ALWAYS returns SUCCESS (never gates the never-mid-abort Parallel). Inert in
    mock mode / when the bridge is unavailable.
    """

    _IDLE = WaveReseedCycle._IDLE

    def __init__(
        self,
        name: str,
        throttle_s: float = 3.0,
        bb_key: str = "track/reacquisition_state",
        waving_service: str = "detect_waving_persons",
        reseed_service: str = "/person_track_node/reseed_target",
        clock=time.monotonic,
    ):
        super().__init__(name=name)
        self.bb_key = bb_key
        self.waving_service = waving_service
        self.reseed_service = reseed_service

        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self._cycle = WaveReseedCycle(throttle_s=throttle_s, clock=clock)

        self._bb = self.attach_blackboard_client(name=f"{self.name}_reacq")
        self._bb.register_key(key=self.bb_key, access=py_trees.common.Access.READ)

    @property
    def _phase(self):
        return self._cycle._phase

    def inject_bridge(self, bridge):
        """Inject a wave/reseed bridge (or fake), bypassing ROS setup."""
        self._cycle.set_bridge(bridge)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e
        if self._cycle.has_bridge or self.mock_mode:
            return
        self._cycle.set_bridge(_WaveReseedBridge(
            self.node, self.waving_service, self.reseed_service))

    def update(self) -> Status:
        try:
            state = int(self._bb.get(self.bb_key))
        except Exception:
            state = REACQ_TRACKING
        if state != REACQ_NEEDS_HELP:
            self._cycle.reset()
            self.feedback_message = "Not in NEEDS_HELP; idle"
            return Status.SUCCESS
        if not self._cycle.has_bridge:
            self.feedback_message = "NEEDS_HELP but waving bridge unavailable (mock)"
            return Status.SUCCESS
        self._cycle.step()
        return Status.SUCCESS
```

- [ ] **Step 2: Run the wave-reseed tests (both must pass)**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_wave_reseed.py test/test_wave_reseed_cycle.py -v`
Expected: PASS — `test_wave_reseed.py` (8 tests) **unchanged & green** (the `_phase` property + `_IDLE` constant preserve it), `test_wave_reseed_cycle.py` (5 tests) green.

- [ ] **Step 3: Lint**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m flake8 behavior_tree/FollowPerson/wave_reseed_cycle.py behavior_tree/FollowPerson/nodes.py --max-line-length=100`
Expected: no new errors on changed lines.

- [ ] **Step 4: Commit (P1)**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git rev-parse HEAD                     # note pre-commit HEAD
git add -- \
  src/behavior_tree/behavior_tree/FollowPerson/wave_reseed_cycle.py \
  src/behavior_tree/behavior_tree/FollowPerson/nodes.py \
  src/behavior_tree/test/test_wave_reseed_cycle.py
git diff --cached --name-only          # MUST be exactly these 3 files
git commit -m "refactor(behavior_tree): extract WaveReseedCycle from BtNode_WaveReseed

Move the wave->reseed async machine (+ _WaveReseedBridge) into a shared
WaveReseedCycle so the upcoming NEEDS_HELP recovery scan can reuse it for
Pass 2. BtNode_WaveReseed now delegates; its public interface (_phase,
_IDLE, inject_bridge, always-SUCCESS) is preserved so test_wave_reseed.py
passes unchanged. Behavior-preserving.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1                   # verify new HEAD is your commit
```

---

## Phase P2: `RecoveryScanFSM` pure core

**Goal:** A ROS-free FSM that, ticked with `(reacq_state, now)`, drives the two-pass scan and emits per-tick actions.

**Files:**
- Create: `src/behavior_tree/behavior_tree/FollowPerson/recovery_scan.py`
- Create: `src/behavior_tree/test/test_recovery_scan_fsm.py`

### Task P2.1: The FSM

- [ ] **Step 1: Write the failing test** — `src/behavior_tree/test/test_recovery_scan_fsm.py`

```python
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
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_fsm.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'behavior_tree.FollowPerson.recovery_scan'`.

- [ ] **Step 3: Create the module** — `src/behavior_tree/behavior_tree/FollowPerson/recovery_scan.py`

```python
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
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_fsm.py -v`
Expected: PASS (7 tests).

- [ ] **Step 5: Lint**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m flake8 behavior_tree/FollowPerson/recovery_scan.py test/test_recovery_scan_fsm.py --max-line-length=100`
Expected: clean.

- [ ] **Step 6: Commit (P2)**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git rev-parse HEAD
git add -- \
  src/behavior_tree/behavior_tree/FollowPerson/recovery_scan.py \
  src/behavior_tree/test/test_recovery_scan_fsm.py
git diff --cached --name-only          # exactly these 2 files
git commit -m "feat(behavior_tree): RecoveryScanFSM pure two-pass recovery core

ROS-free FSM driving the NEEDS_HELP head-scan: Pass 1 (stop + sweep
[current,-60,0,+60], no wave), Pass 2 (raise-hand + same sweep, wave
allowed), Pass 2 repeats; re-lock resets to Pass 1. Dwell-timed via an
injected clock; emits a per-tick RecoveryAction. Pure unit tests.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

---

## Phase P3: `BtNode_RecoveryScan` node

**Goal:** A thin py_trees node that executes the FSM's actions via injectable sinks (TTS, pan, wave bridge) — fully testable without a ROS graph.

**Files:**
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/nodes.py`
- Create: `src/behavior_tree/test/test_recovery_scan_node.py`

### Task P3.1: The node

- [ ] **Step 1: Write the failing test** — `src/behavior_tree/test/test_recovery_scan_node.py`

```python
# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Unit tests for BtNode_RecoveryScan (executes RecoveryScanFSM actions)."""

import math
from types import SimpleNamespace

import py_trees
import pytest

from behavior_tree.FollowPerson.nodes import BtNode_RecoveryScan

TRACKING, NEEDS_HELP = 0, 2
PASS1 = "Please stop walking so I can find you."
PASS2 = "I've lost you. Please raise your hand."


class _FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, s):
        self.now += s


class _FakeCoalescer:
    def __init__(self):
        self.submitted = []
        self.polls = 0

    def submit(self, text):
        self.submitted.append(text)

    def poll(self):
        self.polls += 1


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

    def detect_async(self):
        self.detect_calls += 1
        return _FakeFuture(self.next_detect)

    def reseed_async(self, box):
        self.reseed_boxes.append(box)
        return _FakeFuture(SimpleNamespace(success=True))


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _writer(bb_key):
    c = py_trees.blackboard.Client(name="reacq_writer")
    c.register_key(key=bb_key, access=py_trees.common.Access.WRITE)
    return c


def _make(clock, tts, pan_calls, bridge=None):
    bb = "track/reacquisition_state"
    node = BtNode_RecoveryScan(name="RecoveryScan", dwell_sec=4.0, bb_key=bb,
                               clock=clock)
    node.inject_coalescer(tts)
    node.inject_pan_sink(lambda pan, tilt: pan_calls.append((pan, tilt)))
    if bridge is not None:
        node.inject_wave_bridge(bridge)
    return node, _writer(bb), bb


def test_idle_when_tracking_returns_success():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    node, writer, bb = _make(clock, tts, pan)
    writer.set(bb, TRACKING, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert tts.submitted == [] and pan == []


def test_entry_speaks_pass1_holds_and_runs():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    node, writer, bb = _make(clock, tts, pan)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.RUNNING
    assert tts.submitted == [PASS1]
    assert pan == []                       # HOLD-current: no pan command
    assert tts.polls == 1


def test_sweep_publishes_absolute_pan_with_fixed_tilt():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    node, writer, bb = _make(clock, tts, pan)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()                       # HOLD at t=0
    clock.advance(4.0)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()                       # -> -60deg
    assert pan == [(math.radians(-60.0), math.radians(40.0))]


def test_pass1_never_calls_wave_bridge():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _FakeBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    for t in [0.0, 4.0, 8.0, 12.0]:        # all of pass 1
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
    assert bridge.detect_calls == 0


def test_pass2_drives_wave_bridge():
    clock, tts, pan = _FakeClock(), _FakeCoalescer(), []
    bridge = _FakeBridge()
    node, writer, bb = _make(clock, tts, pan, bridge)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    for t in [0.0, 4.0, 8.0, 12.0, 16.0]:  # walk into pass 2 (raise-hand at 16)
        clock.now = t
        writer.set(bb, NEEDS_HELP, overwrite=True)
        node.tick_once()
    assert PASS2 in tts.submitted
    assert bridge.detect_calls >= 1        # wave scanning engaged in pass 2


def test_no_pan_sink_no_crash():
    # Mock/headless: no pan sink injected -> must not raise.
    clock = _FakeClock()
    bb = "track/reacquisition_state"
    node = BtNode_RecoveryScan(name="RecoveryScan", dwell_sec=4.0, bb_key=bb,
                               clock=clock)
    node.inject_coalescer(_FakeCoalescer())
    writer = _writer(bb)
    writer.set(bb, NEEDS_HELP, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.RUNNING
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_node.py -v`
Expected: FAIL — `ImportError: cannot import name 'BtNode_RecoveryScan'`.

- [ ] **Step 3: Add imports + the node to `nodes.py`**

At the top of `nodes.py`, after the `wave_reseed_cycle` import added in P1, add:
```python
import math

from behavior_tree.FollowPerson.recovery_scan import RecoveryScanFSM
```
(If `import time` is already present, leave it; add `import math` only if absent.)

Append this class to the end of `nodes.py`:
```python
class BtNode_RecoveryScan(py_trees.behaviour.Behaviour):
    """Two-pass head-scan recovery while the tracker is latched in NEEDS_HELP.

    Owns the active recovery the announcer/wave-reseed used to do passively:
    drives ``RecoveryScanFSM`` and executes its per-tick actions — speak each
    pass's line (Pass 1 "please stop", Pass 2 "please raise your hand") via a
    CoalescingTTS, command the head to ABSOLUTE scan angles (tilt fixed) on
    ``/pan_tilt_controller/cmd``, and (Pass 2 only) advance the shared
    ``WaveReseedCycle``. Returns RUNNING while scanning, SUCCESS when idle. All
    ROS I/O is injectable (``inject_coalescer`` / ``inject_pan_sink`` /
    ``inject_wave_bridge``) so the node unit-tests without a ROS graph; lazy msg
    imports keep ``behavior_tree.messages`` off the import path until live setup.
    """

    def __init__(
        self,
        name: str,
        dwell_sec: float = 4.0,
        scan_angles_deg=(-60.0, 0.0, 60.0),
        tilt_deg: float = 40.0,
        include_current_first: bool = True,
        bb_key: str = "track/reacquisition_state",
        announce_service: str = "announce",
        waving_service: str = "detect_waving_persons",
        reseed_service: str = "/person_track_node/reseed_target",
        pan_cmd_topic: str = "/pan_tilt_controller/cmd",
        wave_throttle_s: float = 3.0,
        clock=time.monotonic,
    ):
        super().__init__(name=name)
        self.bb_key = bb_key
        self.announce_service = announce_service
        self.waving_service = waving_service
        self.reseed_service = reseed_service
        self.pan_cmd_topic = pan_cmd_topic
        self._tilt_rad = math.radians(float(tilt_deg))
        self._clock = clock

        self.mock_mode = is_node_mocked(self.__class__.__name__)
        self.node = None
        self._tts = None
        self._pan_sink = None          # callable(pan_rad, tilt_rad) -> None
        self._wave = WaveReseedCycle(throttle_s=wave_throttle_s, clock=clock)
        self._fsm = RecoveryScanFSM(
            dwell_sec=dwell_sec,
            scan_angles_rad=[math.radians(a) for a in scan_angles_deg],
            include_current_first=include_current_first,
        )

        self._bb = self.attach_blackboard_client(name=f"{self.name}_reacq")
        self._bb.register_key(key=self.bb_key, access=py_trees.common.Access.READ)

    def inject_coalescer(self, coalescer):
        self._tts = coalescer

    def inject_pan_sink(self, sink):
        self._pan_sink = sink

    def inject_wave_bridge(self, bridge):
        self._wave.set_bridge(bridge)

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            error_message = "didn't find 'node' in setup's kwargs [{}][{}]".format(
                self.name, self.__class__.__name__
            )
            raise KeyError(error_message) from e

        if self._tts is None:
            from behavior_tree.FollowPerson.coalescing_tts import CoalescingTTS
            if self.mock_mode:
                def start(text):
                    print(f"🔊 MOCK RECOVERY ANNOUNCE: {text}")
                    return _DoneHandle()
                self._tts = CoalescingTTS(start=start, is_done=lambda h: h.done())
            else:
                from behavior_tree.messages import TextToSpeech
                client = self.node.create_client(TextToSpeech, self.announce_service)
                self._client = client

                def start(text):
                    req = TextToSpeech.Request()
                    req.text = text
                    return client.call_async(req)
                self._tts = CoalescingTTS(start=start, is_done=lambda f: f.done())

        if self._pan_sink is None and not self.mock_mode:
            from behavior_tree.messages import PanTiltCommand
            pub = self.node.create_publisher(PanTiltCommand, self.pan_cmd_topic, 10)

            def _sink(pan_rad, tilt_rad):
                cmd = PanTiltCommand()
                cmd.mode = PanTiltCommand.ABSOLUTE
                cmd.pan_rad = float(pan_rad)
                cmd.tilt_rad = float(tilt_rad)
                cmd.speed_raw = 0
                cmd.accel_raw = 0
                pub.publish(cmd)
            self._pan_sink = _sink

        if not self._wave.has_bridge and not self.mock_mode:
            self._wave.set_bridge(_WaveReseedBridge(
                self.node, self.waving_service, self.reseed_service))

    def update(self) -> Status:
        try:
            state = int(self._bb.get(self.bb_key))
        except Exception:
            state = REACQ_TRACKING
        action = self._fsm.tick(state, self._clock())

        if action.speak and self._tts is not None:
            self._tts.submit(action.speak)
        if self._tts is not None:
            self._tts.poll()

        if action.pan_target_rad is not None and self._pan_sink is not None:
            self._pan_sink(action.pan_target_rad, self._tilt_rad)

        if action.allow_wave and self._wave.has_bridge:
            self._wave.step()
        else:
            self._wave.reset()

        if action.active:
            self.feedback_message = "Recovery scanning"
            return Status.RUNNING
        self.feedback_message = "Idle (not in NEEDS_HELP)"
        return Status.SUCCESS
```

- [ ] **Step 4: Run the node + regression tests**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_recovery_scan_node.py test/test_wave_reseed.py test/test_recovery_scan_fsm.py -v`
Expected: PASS (6 + 8 + 7).

- [ ] **Step 5: Lint**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m flake8 behavior_tree/FollowPerson/nodes.py test/test_recovery_scan_node.py --max-line-length=100`
Expected: clean on changed lines.

- [ ] **Step 6: Commit (P3)**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git rev-parse HEAD
git add -- \
  src/behavior_tree/behavior_tree/FollowPerson/nodes.py \
  src/behavior_tree/test/test_recovery_scan_node.py
git diff --cached --name-only          # exactly these 2 files
git commit -m "feat(behavior_tree): BtNode_RecoveryScan executes the recovery FSM

Thin py_trees node: drives RecoveryScanFSM and runs its actions — speak
via CoalescingTTS (/announce), ABSOLUTE PanTiltCommand to
/pan_tilt_controller/cmd (tilt fixed 40deg), and the shared WaveReseedCycle
in Pass 2. All ROS I/O injectable; lazy msg imports keep tests ROS-free.
Not yet wired into the follow tree.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

---

## Phase P4: Swap the NEEDS_HELP reaction (feature goes live)

**Goal:** Trim `BtNode_ReacqAnnounce` to PASSIVE-only and rewire the follow tree's reactions to `[announce, recovery_scan]`. One cohesive commit — no double-speak, no voice gap.

**Files:**
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/nodes.py` (trim ReacqAnnounce)
- Modify: `src/behavior_tree/behavior_tree/FollowPerson/follow_person.py` (rewire)
- Modify: `src/behavior_tree/test/test_reacq_announce.py`
- Modify: `src/behavior_tree/test/test_follow_tree_build.py`
- Modify: `src/behavior_tree/readme.md` (changelog)

### Task P4.1: Trim ReacqAnnounce to PASSIVE-only

- [ ] **Step 1: Update `test_reacq_announce.py`** — drop the `needs_help_text` kwarg and assert NEEDS_HELP no longer announces.

Replace `_make_node` (lines 68–79) with:
```python
def _make_node(clock, fake_tts, bb_key="track/reacquisition_state"):
    node = BtNode_ReacqAnnounce(
        name="ReacqAnnounce",
        passive_text=PASSIVE_TEXT,
        throttle_s=5.0,
        bb_key=bb_key,
        clock=clock,
    )
    node.inject_coalescer(fake_tts)
    return node
```

Replace the NEEDS_HELP block in `test_full_cadence_sequence` (lines 113–131) with:
```python
    # Transition to NEEDS_HELP (2): the announcer is now PASSIVE-only and stays
    # silent here — BtNode_RecoveryScan owns all NEEDS_HELP speech.
    writer.set(bb_key, 2, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT]

    # Back to TRACKING (0): no announcement, resets last-announced state.
    writer.set(bb_key, 0, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT]

    # Re-enter PASSIVE immediately (no clock advance): announces right away.
    writer.set(bb_key, 1, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT, PASSIVE_TEXT]
    assert node.status == py_trees.common.Status.SUCCESS
```
Also delete the now-unused `NEEDS_HELP_TEXT` constant (line 23).

- [ ] **Step 2: Run it to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_reacq_announce.py -v`
Expected: FAIL — the old code still submits `needs_help_text` on NEEDS_HELP, and `_make_node` still passes a `needs_help_text` kwarg the trimmed `__init__` will reject after Step 3 (run order: this proves the behavior change).

- [ ] **Step 3: Trim `BtNode_ReacqAnnounce` in `nodes.py`**

Delete the module constant `DEFAULT_NEEDS_HELP_TEXT = "I've lost you. Please raise your hand."` (line 49) — leave `DEFAULT_PASSIVE_TEXT`.

In `BtNode_ReacqAnnounce.__init__`, remove the `needs_help_text` parameter and its assignment. The signature becomes:
```python
    def __init__(
        self,
        name: str,
        passive_text: str = DEFAULT_PASSIVE_TEXT,
        throttle_s: float = 5.0,
        bb_key: str = "track/reacquisition_state",
        service_name: str = "announce",
        clock=time.monotonic,
    ):
```
Delete the line `self.needs_help_text = needs_help_text`.

In `update()`, replace the state→text mapping (lines 172–177) with PASSIVE-only:
```python
        if state == REACQ_PASSIVE:
            text = self.passive_text
        else:
            # NEEDS_HELP/TRACKING/unknown: BtNode_RecoveryScan owns NEEDS_HELP
            # speech; PASSIVE is the only line this node speaks.
            text = None
```
Update the class docstring's NEEDS_HELP bullet to note it no longer announces (PASSIVE-only).

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_reacq_announce.py -v`
Expected: PASS.

### Task P4.2: Rewire the follow tree

- [ ] **Step 1: Update `test_follow_tree_build.py`**

Change the import (line 20) to:
```python
from behavior_tree.FollowPerson.nodes import BtNode_ReacqAnnounce, BtNode_RecoveryScan
```
Replace `test_reactions_sequence_holds_announce_and_wave_reseed` (lines 73–82) with:
```python
def test_reactions_sequence_holds_announce_and_recovery_scan():
    # Two per-tick reactions: the PASSIVE voice announcer + the NEEDS_HELP
    # two-pass head-scan recovery. The announcer always returns SUCCESS;
    # RecoveryScan returns RUNNING only while actively scanning.
    root = create_follow_person_tree()
    reactions = _inner(root.children[-1])
    assert isinstance(reactions, py_trees.composites.Sequence)
    assert reactions.memory is False
    announce, recovery = reactions.children
    assert isinstance(announce, BtNode_ReacqAnnounce)
    assert isinstance(recovery, BtNode_RecoveryScan)
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_follow_tree_build.py -v`
Expected: FAIL — `ImportError` (BtNode_RecoveryScan not imported in follow_person) / the reactions still hold BtNode_WaveReseed.

- [ ] **Step 3: Rewire `follow_person.py`**

Change the import block (lines 52–55) to:
```python
from behavior_tree.FollowPerson.nodes import (
    BtNode_ReacqAnnounce,
    BtNode_RecoveryScan,
)
```
Replace the `wave_reseed` construction + reactions Sequence (lines 119–131) with:
```python
    announce = BtNode_ReacqAnnounce(name="Reacq Announce")
    # Two-pass head-scan recovery owns the NEEDS_HELP loop the announcer's
    # PASSIVE nudge precedes: ask the person to stop and sweep the head for
    # re-lock (Pass 1), then ask the operator to raise a hand and sweep again
    # with wave-reseed active (Pass 2), repeating Pass 2 until re-lock — the
    # active escape from the indefinite NEEDS_HELP hold. SUCCESS when idle,
    # RUNNING while scanning; NEEDS_HELP-gated, inert otherwise.
    recovery_scan = BtNode_RecoveryScan(name="Recovery Scan")
    reactions = py_trees.composites.Sequence(
        name="Follow Reactions",
        memory=False,
        children=[announce, recovery_scan],
    )
```
Update the module-header docstring (lines 25 and 30–33) to describe the third child as `Sequence[ReacqAnnounce, RecoveryScan]` and the recovery as the two-pass head scan (replace the `BtNode_WaveReseed` mention).

- [ ] **Step 4: Run the full follow-suite to verify everything passes**

Run:
```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
python -m pytest test/test_follow_tree_build.py test/test_reacq_announce.py \
  test/test_wave_reseed.py test/test_recovery_scan_node.py \
  test/test_recovery_scan_fsm.py test/test_wave_reseed_cycle.py -v
```
Expected: ALL PASS. (`test_wave_reseed.py` still green — `BtNode_WaveReseed` remains a valid standalone node, just no longer in this tree.)

- [ ] **Step 5: Lint**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m flake8 behavior_tree/FollowPerson/nodes.py behavior_tree/FollowPerson/follow_person.py test/test_reacq_announce.py test/test_follow_tree_build.py --max-line-length=100`
Expected: clean on changed lines.

### Task P4.3: Changelog

- [ ] **Step 1: Append a changelog entry** to `src/behavior_tree/readme.md`

Add under the changelog section (create a `## Changelog` section at the top of the body if none exists; append-only, newest first):
```markdown
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
```

- [ ] **Step 2: Commit (P4)**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git rev-parse HEAD
git add -- \
  src/behavior_tree/behavior_tree/FollowPerson/nodes.py \
  src/behavior_tree/behavior_tree/FollowPerson/follow_person.py \
  src/behavior_tree/test/test_reacq_announce.py \
  src/behavior_tree/test/test_follow_tree_build.py \
  src/behavior_tree/readme.md
git diff --cached --name-only          # exactly these 5 files
git commit -m "feat(behavior_tree): wire two-pass recovery into the follow tree

Reactions branch becomes [ReacqAnnounce (now PASSIVE-only), RecoveryScan].
RecoveryScan owns all NEEDS_HELP speech + head scan + Pass-2 wave-reseed;
ReacqAnnounce keeps only the PASSIVE 'slow down' nudge. BtNode_WaveReseed
stays as a standalone node (no longer in this tree). Tests + changelog.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

> **Note:** if `src/behavior_tree/readme.md` does not exist, create it with a short package blurb + the `## Changelog` section, and include it in the same commit.

---

## Phase P5: Tracker pan-follow guard (tk26_vision)

**Goal:** Suppress the tracker's pan-follow while `_help_latched` (NEEDS_HELP) so the BT owns the head during recovery — clean mutual exclusion, no jitter on provisional detections.

**Files:**
- Modify: `src/vision_track/vision_track/core/pan_follow.py` (add predicate)
- Modify: `src/vision_track/vision_track/person_track_node.py` (`_pan_follow_tick` guard)
- Modify: `src/vision_track/test/test_pan_follow.py` (predicate test)
- Modify: `src/vision_track/readme.md` (changelog)

### Task P5.1: The predicate

- [ ] **Step 1: Add the failing test** to `src/vision_track/test/test_pan_follow.py`

Add this import at the top (alongside the existing `from vision_track.core.pan_follow import PanFollower`):
```python
from vision_track.core.pan_follow import pan_follow_suppressed
```
Append:
```python
def test_pan_follow_suppressed_during_needs_help():
    # Tracker must NOT command pan while NEEDS_HELP is latched — the behavior
    # tree owns the head (two-pass recovery scan) until re-lock clears the latch.
    assert pan_follow_suppressed(True, True, help_latched=True) is True
    # Normal tracking: pan-follow runs.
    assert pan_follow_suppressed(True, True, help_latched=False) is False
    # Existing enable gates still suppress.
    assert pan_follow_suppressed(False, True, help_latched=False) is True
    assert pan_follow_suppressed(True, False, help_latched=False) is True
```

- [ ] **Step 2: Run it to verify it fails**

Run:
```bash
source /home/tinker/tk25_ws/src/tk26_vision/.venv-vision-main/bin/activate
cd /home/tinker/tk25_ws/src/tk26_vision/src/vision_track
python -m pytest test/test_pan_follow.py -v
```
Expected: FAIL — `ImportError: cannot import name 'pan_follow_suppressed'`.

- [ ] **Step 3: Add the predicate** to `src/vision_track/vision_track/core/pan_follow.py` (module scope, e.g. just below the imports / above `class PanFollower`):
```python
def pan_follow_suppressed(enable_follow: bool, has_publisher: bool,
                          help_latched: bool) -> bool:
    """True when the tracker must NOT issue a pan command this tick.

    Suppress when pan-follow is disabled, when there is no command publisher, or
    when NEEDS_HELP is latched — in NEEDS_HELP the behavior tree owns the head
    (the two-pass recovery scan), so the tracker holds off until re-lock clears
    the latch and hands head control back.
    """
    return (not enable_follow) or (not has_publisher) or bool(help_latched)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/vision_track && python -m pytest test/test_pan_follow.py -v`
Expected: PASS (10 tests: 9 existing + 1 new).

### Task P5.2: Apply the guard in `_pan_follow_tick`

- [ ] **Step 1: Edit `person_track_node.py`** — make `_pan_follow_tick` use the predicate.

Add the import near the top-of-file imports (where `from vision_track.core.pan_follow import PanFollower` appears):
```python
from vision_track.core.pan_follow import PanFollower, pan_follow_suppressed
```
(If `PanFollower` is imported on its own line, extend that line.)

Replace the opening guard of `_pan_follow_tick` (current `person_track_node.py:2003-2004`):
```python
        if not self.enable_pan_tilt_follow or self._pan_cmd_pub is None:
            return
```
with:
```python
        if pan_follow_suppressed(self.enable_pan_tilt_follow,
                                 self._pan_cmd_pub is not None,
                                 getattr(self, '_help_latched', False)):
            return
```
This adds the `_help_latched` gate while preserving the existing enable/publisher gates. The existing HOLD-comment block below (lines ~2026–2032) stays.

- [ ] **Step 2: Verify the pan-follow suite still passes** (the guard test already covers the predicate; the node method has no standalone unit test, so the predicate test is the contract):

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/vision_track && python -m pytest test/test_pan_follow.py -v`
Expected: PASS (10).

- [ ] **Step 3: Lint**

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/vision_track && python -m flake8 vision_track/core/pan_follow.py vision_track/person_track_node.py test/test_pan_follow.py --max-line-length=100`
Expected: no new errors on changed lines (pre-existing E501s on untouched lines are out of scope).

### Task P5.3: Changelog + commit

- [ ] **Step 1: Append a changelog entry** to `src/vision_track/readme.md` (append-only, newest first):
```markdown
- **2026-06-15 — Pan-follow suppressed during NEEDS_HELP.** `_pan_follow_tick`
  now holds off issuing any pan command while `_help_latched` (NEEDS_HELP), via
  the new pure `pan_follow_suppressed(enable, has_pub, help_latched)` predicate.
  This hands head control to the behavior tree's two-pass recovery scan during
  NEEDS_HELP (clean mutual exclusion keyed on the help latch; the tracker
  resumes centering the instant a re-lock clears the latch). Pairs with
  tk25_decision `BtNode_RecoveryScan`.
```

- [ ] **Step 2: Commit (P5)**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
git rev-parse HEAD
git add -- \
  src/vision_track/vision_track/core/pan_follow.py \
  src/vision_track/vision_track/person_track_node.py \
  src/vision_track/test/test_pan_follow.py \
  src/vision_track/readme.md
git diff --cached --name-only          # exactly these 4 files
git commit -m "feat(vision_track): suppress pan-follow during NEEDS_HELP

_pan_follow_tick holds off any pan command while _help_latched, via the new
pure pan_follow_suppressed() predicate, so the behavior tree owns the head
during its two-pass recovery scan (mutual exclusion keyed on the help latch;
tracker resumes centering the instant re-lock clears it). Pairs with
tk25_decision BtNode_RecoveryScan.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log --oneline -1
```

- [ ] **Step 3: Deploy (after P5)** — build both packages via tkbuild so the live trees pick up the change. NEVER `colcon build --symlink-install` here — it breaks ament_python entry-point metadata (PackageNotFoundError at `ros2 run`):
```bash
cd /home/tinker/tk25_ws
tkbuild tk25_decision --packages-select behavior_tree
tkbuild tk26_vision --packages-select vision_track
```

---

## Self-Review

**1. Spec coverage** — every spec section maps to a task:
- Goals 1–4 (pass structure, Pass-1-no-wave, Pass-2-wave, repeat-Pass-2) → P2 (`RecoveryScanFSM`) + P3 (node wiring) + tests `test_recovery_scan_fsm.py`/`test_recovery_scan_node.py`.
- Goal 5 (re-lock aborts → hand back) → FSM `tick` idle branch (P2) + the tracker guard (P5).
- Goal 6 (ABSOLUTE pan, fixed tilt) → P3 `_sink` builds `PanTiltCommand.ABSOLUTE` with `_tilt_rad`; test `test_sweep_publishes_absolute_pan_with_fixed_tilt`.
- Architecture / ownership invariant + the one tracker change → P5.
- Components: WaveReseedCycle (§Components 1 extraction) → P1; RecoveryScanFSM (§1) → P2; BtNode_RecoveryScan (§2) → P3; trim ReacqAnnounce (§3) + tree rewire (§4) → P4; tracker guard (§5) → P5.
- Degradation (no pan sub / no announce / no waving / unset bb) → P3 `test_no_pan_sink_no_crash` + `has_bridge`/mock guards + `state` default to TRACKING.
- Params (§Parameters) → P3 `__init__` defaults (`dwell_sec`, `scan_angles_deg`, `tilt_deg`, `include_current_first`).
- Testing (§Testing) → all three new test files + the two updated.

**2. Placeholder scan** — no TBD/TODO; every code step shows complete code; every test step shows assertions.

**3. Type consistency** — `RecoveryAction` fields (`active`, `speak`, `pan_target_rad`, `allow_wave`) are used identically in P2 and P3. `WaveReseedCycle` API (`set_bridge`, `has_bridge`, `reset`, `step`, `_phase`, `_IDLE`) is consistent across P1 (definition + delegation) and P3 (node use). `pan_follow_suppressed(enable_follow, has_publisher, help_latched)` signature matches between P5.1 (def + test) and P5.2 (call site). `BtNode_RecoveryScan` injection hooks (`inject_coalescer`/`inject_pan_sink`/`inject_wave_bridge`) match between P3 def and `test_recovery_scan_node.py`.

**4. Ordering safety** — each phase leaves the package importable and green: P1 refactor preserves `test_wave_reseed.py`; P2/P3 add unused-but-tested modules; P4 swaps in one cohesive commit (no double-speak / no voice gap); P5 is an isolated tracker guard. No phase depends on a later phase to build.
