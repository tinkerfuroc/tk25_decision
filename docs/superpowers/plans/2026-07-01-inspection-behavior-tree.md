# Inspection Behavior Tree Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rework the `Inspection` behavior tree so one `ros2 run behavior_tree inspection` waits for the door to open → announces "door open" → navigates to InspectionPoint → waits for Enter → navigates to ExitPoint, and fix the Enter-keypress node so a stray keystroke can't skip the checkpoint.

**Architecture:** Two isolated changes in `tk25_decision`'s `behavior_tree` package. (1) Harden `BtNode_PressEnterToSucceed` (`Inspection/customNodes.py`) into a robust, self-draining, input-consuming Enter detector, and decouple it from the `messages.py` import hazard by dropping a dead import. (2) Restructure `createInspection()` (`Inspection/inspection.py`) to the exact 9-node sequence, inserting the missing "door open" announcement and collapsing the verbose self-introduction to a single line. All four building-block nodes already exist and are reused verbatim; only `createInspection()` and the keypress node change.

**Tech Stack:** Python 3.10, `py_trees` / `py_trees_ros`, `rclpy`, ROS2, `pytest`. Runs on the Tinker robot; poses come from `Inspection/constants.json` (`map` frame).

**Related spec:** `docs/superpowers/specs/2026-07-01-inspection-behavior-tree-design.md`

## Global Constraints

- **Package:** `behavior_tree` under `src/tk25_decision/src/behavior_tree/`; repo `tk25_decision`, branch `dev` (shared — commit new only, verify `HEAD`, never `--amend`/rebase/force-push).
- **Build policy:** never raw `colcon`; use `tkbuild tk25_decision --packages-select behavior_tree` if a rebuild is needed. Pure-Python edits are live under `--symlink-install`; `pytest` runs against source and needs **no** build. The **user drives builds and on-robot `ros2 run` verification**; Claude may run `pytest`/read-only checks.
- **Commit policy:** commit only on the user's explicit go-ahead (workspace rule). Plan includes commit steps; honor the user's timing at execution.
- **Test env:** `pytest` requires the sourced `tk25_decision` env where `py_trees` is importable (the `.venv_decision`). Set `BT_MOCK_MODE=true` at the top of tree-structural tests (established pattern, `test_rulebook_structure.py`).
- **No changes** to `cli.py`, `setup.py`, `runtime.py`, or `constants.json` **values**. The `inspection` entry point (`setup.py:61`) already exists.
- **Known host hazard (pre-flight):** `messages.py:43` `TTSCnRequest` import can fail on some builds, breaking every BT import that transitively touches `behavior_tree.messages` (documented in `tk25_decision/CLAUDE.md`). Task 1's node/test are deliberately decoupled from it; Task 2's structural test shares the same import chain as the existing (passing) `test_rulebook_structure.py`, so if it errors at *import* rather than *assert*, that is the known hazard — resolve it before proceeding.

---

### Task 1: Harden the Enter-keypress node (`BtNode_PressEnterToSucceed`)

**Files:**
- Modify: `src/tk25_decision/src/behavior_tree/behavior_tree/Inspection/customNodes.py` (full rewrite — currently 63 lines)
- Test: `src/tk25_decision/src/behavior_tree/test/test_press_enter_node.py` (create)

**Interfaces:**
- Consumes: nothing from other tasks.
- Produces: `BtNode_PressEnterToSucceed(name: str = "Press Enter to Succeed")` — a plain `py_trees.behaviour.Behaviour`. `update()` returns `RUNNING` until a newline-terminated line is read on `sys.stdin`, then `SUCCESS` (consuming that line). `initialise()` drains stale buffered stdin (EOF-guarded). Module imports only `select`, `sys`, `py_trees` (no `behavior_tree.messages`). Task 2 reuses this class by name at index 6 of the tree.

**Behavior being fixed:** today `is_enter_pressed()` (`customNodes.py:13`) returns true on *any* pending stdin and never consumes it, so a keystroke buffered during the preceding door-wait/navigation makes the node return `SUCCESS` immediately, skipping the human checkpoint.

- [ ] **Step 1: Write the failing test**

Create `src/tk25_decision/src/behavior_tree/test/test_press_enter_node.py`:

```python
import py_trees
from behavior_tree.Inspection import customNodes as C


class FakeStdin:
    """Minimal stdin stand-in: readline() pops a queued line, '' at EOF."""

    def __init__(self, lines=()):
        self._lines = list(lines)

    def feed(self, *lines):
        self._lines.extend(lines)

    def has_data(self):
        return len(self._lines) > 0

    def readline(self):
        return self._lines.pop(0) if self._lines else ""


def _install(monkeypatch, fake, ready):
    """Route the node's module-level select/sys at the fake. `ready` -> bool."""
    monkeypatch.setattr(C.sys, "stdin", fake)
    monkeypatch.setattr(
        C.select, "select",
        lambda r, w, x, t: (([fake] if ready() else []), [], []),
    )


def test_running_when_no_input(monkeypatch):
    fake = FakeStdin()
    _install(monkeypatch, fake, fake.has_data)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()
    assert node.update() == py_trees.common.Status.RUNNING


def test_success_and_consumes_on_enter(monkeypatch):
    fake = FakeStdin()
    _install(monkeypatch, fake, fake.has_data)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()                                    # nothing buffered
    assert node.update() == py_trees.common.Status.RUNNING
    fake.feed("\n")                                      # user presses Enter
    assert node.update() == py_trees.common.Status.SUCCESS
    assert fake.has_data() is False                      # the newline was consumed


def test_initialise_drains_stale_input(monkeypatch):
    fake = FakeStdin(["garbage\n"])                      # buffered before checkpoint
    _install(monkeypatch, fake, fake.has_data)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()                                    # must drain the stale line
    assert fake.has_data() is False
    assert node.update() == py_trees.common.Status.RUNNING  # waits for a fresh Enter


def test_initialise_drain_stops_on_eof(monkeypatch):
    # A closed/EOF stdin selects readable but readline() -> "". The drain loop
    # must break on EOF, not spin. `ready` is capped so a missing guard fails
    # the assertion instead of hanging the suite.
    fake = FakeStdin()                                   # readline always "" (EOF)
    calls = {"n": 0}

    def ready():
        calls["n"] += 1
        return calls["n"] <= 5

    _install(monkeypatch, fake, ready)
    node = C.BtNode_PressEnterToSucceed()
    node.initialise()
    assert calls["n"] <= 2                               # broke on first EOF read
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_press_enter_node.py -v`
Expected: `test_success_and_consumes_on_enter` FAILS (old code never consumes → `has_data()` still True), `test_initialise_drains_stale_input` FAILS (old `initialise` doesn't drain → old `update` returns SUCCESS on the stale byte), `test_initialise_drain_stops_on_eof` FAILS (no EOF-guarded drain in old `initialise`). `test_running_when_no_input` may already pass.

- [ ] **Step 3: Write the implementation**

Replace the **entire** contents of `Inspection/customNodes.py` with:

```python
import select
import sys

import py_trees


class BtNode_PressEnterToSucceed(py_trees.behaviour.Behaviour):
    """
    Wait for the user to press Enter, then return SUCCESS.

    Ticks RUNNING until a deliberate Enter (a newline-terminated line) is read
    on stdin, then returns SUCCESS on that tick and consumes the line so no
    stray newline leaks to later readers. On ``initialise`` it drains any stale
    buffered input, so a keystroke pressed earlier (e.g. during the preceding
    door-wait or navigation) cannot instantly satisfy the wait.

    Assumes an interactive, line-buffered (cooked) terminal, as provided by
    ``ros2 run``: stdin only selects readable once a full Enter-terminated line
    is available.
    """

    def __init__(self, name: str = "Press Enter to Succeed"):
        super().__init__(name=name)

    def initialise(self) -> None:
        # Drain stale/buffered stdin lines so a stray earlier keystroke can't
        # instantly satisfy the Enter-wait. Guard EOF (piped/closed stdin) so
        # the loop can't spin — a closed fd reads select-ready but readline()->"".
        while select.select([sys.stdin], [], [], 0)[0]:
            if sys.stdin.readline() == "":  # EOF
                break
        self.logger.info(f"'{self.name}': Press ENTER to continue...")

    def update(self) -> py_trees.common.Status:
        if select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()  # consume the line incl. the trailing '\n'
            self.feedback_message = "Enter detected"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "Waiting for user to press Enter..."
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.logger.info(f"'{self.name}': Terminating with status {new_status}.")
```

This drops the dead `is_enter_pressed()` helper and the dead imports (`copy`, `typing.Any`, `rclpy.node.Node`, `time`, and crucially `from behavior_tree.messages import PointStamped` — none are used, and the last coupled the module to the `messages.py` hazard). `inspection.py` imports `PointStamped` from `geometry_msgs` directly, so nothing downstream breaks.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_press_enter_node.py -v`
Expected: 4 passed.

- [ ] **Step 5: Commit** (on user's go-ahead)

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/Inspection/customNodes.py src/behavior_tree/test/test_press_enter_node.py
git commit -m "fix(Inspection): robust Enter-keypress node (drain stale stdin, consume on success)"
```

---

### Task 2: Restructure `createInspection()`

**Files:**
- Modify: `src/tk25_decision/src/behavior_tree/behavior_tree/Inspection/inspection.py` (rewrite `createInspection()`; delete `createQandA()`; drop newly-dead imports/keys)
- Test: `src/tk25_decision/src/behavior_tree/test/test_inspection_tree.py` (create)

**Interfaces:**
- Consumes: `BtNode_PressEnterToSucceed` from Task 1 (imported at `inspection.py:9` as `from .customNodes import BtNode_PressEnterToSucceed`). Reused verbatim node classes: `BtNode_WriteToBlackboard`, `BtNode_GotoAction`, `BtNode_Announce` (stores `given_msg`), `BtNode_MoveArmSingle`, `BtNode_DoorDetection`; helpers `createConstantWriter()`, `createToIspection()`, `createToExit()`; keys `KEY_INSPECTION_POSE`, `KEY_EXIT_POSE`, `KEY_ARM_NAVIGATING`, `KEY_DOOR_STATUS`.
- Produces: `createInspection() -> py_trees.composites.Sequence` named `"Inspection Root"`, `memory=True`, with 9 children in the fixed order asserted below. Unchanged entry: `cli.py` → `run_tree(createInspection, period_ms=500.0, title="Inspection")`.

- [ ] **Step 1: Write the failing test**

Create `src/tk25_decision/src/behavior_tree/test/test_inspection_tree.py`:

```python
import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.Inspection import inspection as I  # noqa: E402


def _announce_messages(root):
    return [
        c.given_msg
        for c in root.children
        if c.__class__.__name__ == "BtNode_Announce"
    ]


def test_root_is_memory_sequence_named_inspection_root():
    root = I.createInspection()
    assert isinstance(root, py_trees.composites.Sequence)
    assert root.name == "Inspection Root"
    assert root.memory is True
    assert len(root.children) == 9


def test_child_order_and_types():
    kids = I.createInspection().children
    assert isinstance(kids[0], py_trees.composites.Parallel)              # constants
    assert isinstance(kids[1], py_trees.decorators.Retry)                 # arm nav-pose
    assert isinstance(kids[2], py_trees.decorators.Retry)                 # door detection
    assert kids[2].decorated.__class__.__name__ == "BtNode_DoorDetection"
    assert kids[3].__class__.__name__ == "BtNode_Announce"               # NEW
    assert kids[3].given_msg == "door open"
    assert isinstance(kids[4], py_trees.composites.Sequence)             # to inspection
    assert kids[5].__class__.__name__ == "BtNode_Announce"              # self-intro
    assert kids[5].given_msg == "Dear referees, I am Tinker."
    assert kids[6].__class__.__name__ == "BtNode_PressEnterToSucceed"   # wait Enter
    assert kids[7].__class__.__name__ == "BtNode_Announce"              # leaving
    assert kids[7].given_msg == "Heading to the exit."
    assert isinstance(kids[8], py_trees.composites.Sequence)            # to exit


def test_door_open_announced_immediately_after_detection():
    kids = I.createInspection().children
    assert kids[2].decorated.__class__.__name__ == "BtNode_DoorDetection"
    assert kids[3].__class__.__name__ == "BtNode_Announce"
    assert kids[3].given_msg == "door open"


def test_verbose_self_intro_removed():
    msgs = _announce_messages(I.createInspection())
    assert msgs == ["door open", "Dear referees, I am Tinker.", "Heading to the exit."]
    assert not any("at home service robot" in (m or "") for m in msgs)
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_inspection_tree.py -v`
Expected: `test_child_order_and_types` FAILS (current `kids[3]` is the go-to-inspection Sequence, not the "door open" announce), `test_door_open_announced_immediately_after_detection` FAILS (no announce after door detection today), `test_verbose_self_intro_removed` FAILS (current announces are `["I am Tinker, I am ready for inspection…", <long tinker_description>, "Heading to the exit."]`).
If it instead ERRORS at import with `ImportError: cannot import name 'TTSCnRequest'`, that is the known `messages.py` host hazard (Global Constraints) — resolve it first, then re-run.

- [ ] **Step 3: Rewrite `createInspection()` and drop dead code**

In `Inspection/inspection.py`:

(a) Replace the `createInspection()` function (currently `:91-108`) with:

```python
def createInspection():
    root = py_trees.composites.Sequence(name="Inspection Root", memory=True)

    # write all the constants to blackboard first
    root.add_child(createConstantWriter())

    # tuck the arm into the navigating pose
    root.add_child(py_trees.decorators.Retry("retry", BtNode_MoveArmSingle(name="Move arm to nav", action_name=arm_action_name, arm_pose_bb_key=KEY_ARM_NAVIGATING, add_octomap=False), 3))

    # wait until the door is detected open (Retry keeps polling on closed/error)
    root.add_child(py_trees.decorators.Retry(name="retry", child=BtNode_DoorDetection(name="Door detection", bb_door_state_key=KEY_DOOR_STATUS), num_failures=999))

    # announce as soon as the door is open
    root.add_child(BtNode_Announce(name="Announce door open", bb_source=None, message="door open"))

    # navigate to the inspection point
    root.add_child(createToIspection())

    # brief self-introduction for the referees
    root.add_child(BtNode_Announce(name="introduce self", bb_source=None, message="Dear referees, I am Tinker."))

    # wait for the operator to press Enter, then head out
    root.add_child(BtNode_PressEnterToSucceed())

    root.add_child(BtNode_Announce(name="announce leaving", bb_source=None, message="Heading to the exit."))
    root.add_child(createToExit())
    return root
```

(b) Delete the now-dead `createQandA()` function (currently `:79-89`).

(c) Drop the newly-dead symbols so the file stays tidy:
- On the Audio import line (`:5`), remove `BtNode_ListenAction`: change
  `from behavior_tree.TemplateNodes.Audio import BtNode_Announce, BtNode_ListenAction`
  → `from behavior_tree.TemplateNodes.Audio import BtNode_Announce`
- On the Manipulation import line (`:6`), remove the unused `BtNode_PointTo`: change
  `from behavior_tree.TemplateNodes.Manipulation import BtNode_PointTo, BtNode_MoveArmSingle`
  → `from behavior_tree.TemplateNodes.Manipulation import BtNode_MoveArmSingle`
- Remove the `KEY_LISTEN_RESULT = "listen_result"` constant (`:54`).
- Remove the `tinker_description = constants["tinker_description"]` line (`:46`). (The `tinker_description` key stays in `constants.json`, now dormant — no JSON change.)

Leave everything else (`createConstantWriter`, `createToIspection`, `createToExit`, pose loading, `KEY_*` for poses/arm/door, `arm_action_name`) untouched.

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -m pytest test/test_inspection_tree.py test/test_press_enter_node.py -v`
Expected: all passed (Task 1 + Task 2 together).

- [ ] **Step 5: User-driven mock-mode smoke verification**

The user runs (rebuild first only if not symlink-installed: `tkbuild tk25_decision --packages-select behavior_tree`):

```bash
BT_MOCK_MODE=true ros2 run behavior_tree inspection
```

Expected observable order: constants written → arm mock → door mock-opens → prints/says **"door open"** → nav mock (inspection) → says **"Dear referees, I am Tinker."** → **blocks until you press Enter** (tap other keys first — it must keep waiting, proving the drain/consume fix) → says **"Heading to the exit."** → nav mock (exit) → tree returns SUCCESS. No exceptions.

- [ ] **Step 6: Commit** (on user's go-ahead)

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/Inspection/inspection.py src/behavior_tree/test/test_inspection_tree.py
git commit -m "feat(Inspection): trim tree to door-open→announce→inspect→enter→exit; announce door open"
```

---

## Self-Review

**1. Spec coverage** (against `2026-07-01-inspection-behavior-tree-design.md`):
- §3 tree (arm kept, door-open announce inserted, single self-intro, Enter, leaving announce, exits) → Task 2 Step 3 + `test_child_order_and_types`. ✓
- §4 keypress fix (drain-on-init, EOF guard, consume-on-success) → Task 1 (all four tests). ✓
- §1.1 remove Q&A + dead imports/keys → Task 2 Step 3(b)(c). ✓
- §5 mock-mode end-to-end + Enter not mocked → Task 2 Step 5. ✓
- §5 `messages.py` pre-flight hazard → Global Constraints + Task 2 Step 2 note. ✓
- §6 no change to `cli.py`/`setup.py`/`runtime.py`/`constants.json` values → not touched by any task. ✓
- §1.2 path de-hardcode explicitly out of scope → no task. ✓

**2. Placeholder scan:** No TBD/TODO/"handle edge cases"/"similar to". Every code step shows complete code; every run step shows an exact command + expected result. ✓

**3. Type consistency:** `BtNode_PressEnterToSucceed` (Task 1) is the exact class Task 2 asserts at `kids[6]`. `given_msg` is the real `BtNode_Announce` attribute (`Audio.py:230`). `.decorated` is the py_trees `Retry` child accessor (as used in `test_rulebook_structure.py`). Child count 9 matches the enumerated indices 0–8. ✓
