# DoingLaundry Entry Gate + Door Detection + Fold-Prompt Update — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Gate the `doing-laundry` task start on an operator Enter press, wait for the arena door to open (Inspection-style door detection), only then navigate to the folding/laundry table; update the fold prompt to reference the on-screen shirt layout and extend the lay-out wait from 5 s to 10 s.

**Architecture:** All tree changes live in `behavior_tree/DoingLaundry/laundry.py` (`createDoingLaundry()` + `foldClothingOnce()`). The Enter gate reuses the canonical `BtNode_PressEnterToSucceed` from `TemplateNodes/OperatorGate.py` (added in commit `771a87b`, already used by GPSR + HRI). Door detection mirrors `Inspection/inspection.py:87-96`: announce-ready parallel with a pan-tilt door aim, then `Retry(999) × BtNode_DoorDetection`, then a door-open announcement — all inserted **before** the existing arm-tuck + navigate-to-folding-table steps, so navigation only happens after the door opens.

**Tech Stack:** Python 3.10, `py_trees` / `py_trees_ros`, ROS 2, pytest (structural tests in `src/behavior_tree/test/`).

## Global Constraints

- **A committed test already specifies the target shape.** `src/behavior_tree/test/test_doing_laundry_start_gate.py` (commit `771a87b`, teammate grange) requires: `root.children[0].__class__.__name__ == "BtNode_PressEnterToSucceed"`, `len(root.children) == 10`, and `root.children[1]` is a `Parallel` (the constants writer). It currently FAILS. Do not edit that file; make it pass.
- Announcement copy (verbatim from the request): the fold prompt must ask the person to **"lay the shirt out in the manner as shown on my screen"**.
- Fold lay-out wait: exactly **10.0 s** (5 s longer than the current 5.0 s).
- Edit ONLY the outer `src/behavior_tree/behavior_tree/DoingLaundry/laundry.py` (the `doing-laundry` entry point per `setup.py:92`). Do NOT touch the stale nested duplicate `DoingLaundry/DoingLaundry/`, `laundry-opus.py`, or `laundry_v2.py`.
- `laundry.py` already carries pre-existing uncommitted working-tree changes (fold-complete message tweak, `BtNode_TurnPanTilt("look at desktop")` in Setup, commented-out pickup phases). Keep them; they ride along in the commit. Leave `hri.py` and `mock_config.json` (also modified, not ours) UNSTAGED.
- Concurrent-committer repo: before committing, re-check `git log --oneline -1` (HEAD moves under you). Never `--amend`/rebase; commit new only.
- Build with `tkbuild tk25_decision --packages-select behavior_tree` — never raw `colcon build`.
- Blackboard key for door state: `KEY_DOOR_STATUS` = `"dl_door_status"`, already defined at `config.py:217`; its import in `laundry.py` is commented out at line 74 and must be re-enabled.
- README discipline: append a Changelog entry to `src/behavior_tree/README.md` in the same commit.

---

### Task 1: Structural tests for door gating + fold prompt (failing first)

**Files:**
- Create: `src/behavior_tree/test/test_doing_laundry_entry_and_fold.py`
- (Reference, do not edit: `src/behavior_tree/test/test_doing_laundry_start_gate.py`)

**Interfaces:**
- Consumes: `createDoingLaundry()`, `foldClothingOnce()` from `behavior_tree.DoingLaundry.laundry`.
- Produces: test names Task 2 must satisfy. Node names asserted: `"Wait for operator to start"`, `"Door detection"`, `"Retry door detection"`, `"Announce door open"`, `"Navigate to laundry area"`, `"Announce folding start"`, `"Wait for clothing to be laid flat"`. Attributes asserted: `BtNode_Announce.given_msg` (str), `py_trees.timers.Timer.duration` (float).

- [ ] **Step 1: Write the failing test file**

```python
import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.DoingLaundry.laundry import (  # noqa: E402
    createDoingLaundry,
    foldClothingOnce,
)


def _names(root):
    return [b.name for b in root.iterate()]


def test_door_gate_ordering():
    """Enter gate -> door detection -> door-open announce -> nav to table."""
    names = _names(createDoingLaundry())
    gate = names.index("Wait for operator to start")
    door = names.index("Door detection")
    announce_open = names.index("Announce door open")
    nav = names.index("Navigate to laundry area")
    assert gate < door < announce_open < nav


def test_door_detection_is_retried():
    root = createDoingLaundry()
    retry = next(b for b in root.iterate() if b.name == "Retry door detection")
    assert isinstance(retry, py_trees.decorators.Retry)
    assert retry.children[0].name == "Door detection"


def test_fold_announce_references_screen():
    fold = foldClothingOnce()
    announce = next(
        b for b in fold.iterate() if b.name == "Announce folding start"
    )
    assert "as shown on my screen" in announce.given_msg


def test_fold_wait_extended_to_ten_seconds():
    fold = foldClothingOnce()
    timer = next(
        b for b in fold.iterate() if b.name == "Wait for clothing to be laid flat"
    )
    assert timer.duration == 10.0
```

- [ ] **Step 2: Run the new tests plus grange's committed gate test — all must FAIL**

Run:
```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
  ../../.venv_decision/bin/python -m pytest \
  test/test_doing_laundry_entry_and_fold.py \
  test/test_doing_laundry_start_gate.py -v
```
Expected: the 4 new tests FAIL (ValueError from `names.index("Wait for operator to start")` / `StopIteration` / message-content assert), and `test_root_starts_with_operator_gate` FAILS (`len(root.children)` is 6, no gate). `test_constant_writer_still_second` may pass — fine.
(If collection fails on ROS message imports, the `BT_MOCK_MODE=true` line makes `behavior_tree.messages` fall back to mocks; run from the `.venv_decision` interpreter as shown.)

- [ ] **Step 3: Commit the failing tests**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && git log --oneline -1 && \
git add src/behavior_tree/test/test_doing_laundry_entry_and_fold.py && \
git commit -m "test(DoingLaundry): specify entry gate, door-detect ordering, screen fold prompt, 10s wait"
```

---

### Task 2: Implement gate + door detection + prompt/wait changes in laundry.py

**Files:**
- Modify: `src/behavior_tree/behavior_tree/DoingLaundry/laundry.py` (imports ~line 41-74; `foldClothingOnce()` lines 365-396; `createDoingLaundry()` lines 399-447)
- Modify: `src/behavior_tree/README.md` (Changelog section, ~line 675)

**Interfaces:**
- Consumes: `BtNode_PressEnterToSucceed(name: str = "Press Enter to Succeed")` from `behavior_tree.TemplateNodes.OperatorGate`; `BtNode_DoorDetection(name, bb_door_state_key, service_name="door_detection_srv")` and `BtNode_TurnPanTilt(name, x=0.0, y=45.0)` (already imported at lines 36, 39); `KEY_DOOR_STATUS` from `.config`.
- Produces: `createDoingLaundry()` returning a Sequence with EXACTLY these 10 children, in order: ① `BtNode_PressEnterToSucceed("Wait for operator to start")`, ② constants-writer Parallel, ③ "Setup" Parallel, ④ "Announce ready + aim pan-tilt" Parallel, ⑤ `Retry("Retry door detection")`, ⑥ `Announce("Announce door open")`, ⑦ arm-tuck Retry, ⑧ nav-to-table Retry, ⑨ `Announce("Announce folding start")`, ⑩ fold Repeat.

- [ ] **Step 1: Add the OperatorGate import and re-enable KEY_DOOR_STATUS**

After line 42 (`from behavior_tree.TemplateNodes.manipulation_new import BtNode_JointMoveAction`) add:

```python
from behavior_tree.TemplateNodes.OperatorGate import BtNode_PressEnterToSucceed
```

In the `from .config import (...)` block, replace the commented line 74:

```python
    # KEY_DOOR_STATUS,
```
with:
```python
    KEY_DOOR_STATUS,
```

- [ ] **Step 2: Update `foldClothingOnce()` — prompt text + 10 s wait**

Replace the "Announce folding start" child (lines 373-379) message and the Timer (line 381):

```python
    root.add_child(
        BtNode_Announce(
            name="Announce folding start",
            bb_source=None,
            message="Start folding, please lay the shirt out in the manner as shown on my screen.",
        )
    )
    root.add_child(
        py_trees.timers.Timer(name="Wait for clothing to be laid flat", duration=10.0)
    )
```

- [ ] **Step 3: Rebuild `createDoingLaundry()` with gate + door wait before nav**

Replace the whole function (lines 399-447) with:

```python
def createDoingLaundry():
    root = py_trees.composites.Sequence(name="Doing Laundry", memory=True)

    # Operator gate: nothing runs until a deliberate Enter (same gate as GPSR/HRI).
    root.add_child(BtNode_PressEnterToSucceed(name="Wait for operator to start"))

    root.add_child(createConstantWriter())

    start_parallel = py_trees.composites.Parallel(
        "Setup", policy=py_trees.common.ParallelPolicy.SuccessOnAll()
    )
    start_parallel.add_child(_gripperOpenSafe(name="Ensure gripper starts open"))
    start_parallel.add_child(
        BtNode_Announce(
            name="Announce task start",
            bb_source=None,
            message="Starting laundry task!",
        )
    )
    start_parallel.add_child(BtNode_TurnPanTilt("look at desktop"))
    root.add_child(start_parallel)

    # Door wait, mirroring Inspection: announce ready + aim head, then poll the
    # door_detection_srv until it reports open (FAILURE on closed keeps Retry going).
    ready = py_trees.composites.Parallel(
        name="Announce ready + aim pan-tilt",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
    )
    ready.add_child(
        BtNode_Announce(
            name="Announce ready for laundry",
            bb_source=None,
            message="I am ready for the laundry task, please open the door.",
        )
    )
    ready.add_child(BtNode_TurnPanTilt(name="Aim pan-tilt at door", x=0.0, y=45.0))
    root.add_child(ready)

    root.add_child(
        py_trees.decorators.Retry(
            name="Retry door detection",
            child=BtNode_DoorDetection(
                name="Door detection", bb_door_state_key=KEY_DOOR_STATUS
            ),
            num_failures=999,
        )
    )

    root.add_child(
        BtNode_Announce(
            name="Announce door open",
            bb_source=None,
            message="The door is open. Heading to the folding table.",
        )
    )

    # root.add_child(pickupOneClothing())
    # root.add_child(pickupLaundryBasket())
    # root.add_child(goAndPlaceBasket())

    root.add_child(
        _moveArmRetry(
            name="Move arm to base moving",
            arm_pose_key=KEY_ARM_NAVIGATING,
            add_octomap=False,
        )
    )
    root.add_child(
        _gotoRetry(name="Navigate to laundry area", pose_key=KEY_POSE_FOLDING_TABLE)
    )

    root.add_child(
        BtNode_Announce(
            name="Announce folding start",
            bb_source=None,
            message="Starting to fold clothing!",
        )
    )

    root.add_child(
        py_trees.decorators.Repeat(
            name="Repeat fold action",
            child=foldClothingOnce(),
            num_success=999,
        )
    )
    return root
```

Notes: the Setup announcement drops "Navigating to laundry area." because navigation now only happens after the door opens. The pan-tilt door aim `(x=0.0, y=45.0)` copies `Inspection/inspection.py:89` — the proven aim `BtNode_DoorDetection` (orbbec head camera) works with. Child count = 10, gate at index 0, constants writer at index 1 — exactly what the committed gate test requires.

- [ ] **Step 4: Run the full DoingLaundry test set — all PASS**

Run:
```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
  ../../.venv_decision/bin/python -m pytest \
  test/test_doing_laundry_entry_and_fold.py \
  test/test_doing_laundry_start_gate.py \
  test/test_press_enter_node.py test/test_inspection_tree.py -v
```
Expected: ALL PASS (including grange's previously-failing `test_root_starts_with_operator_gate`; `test_press_enter_node.py` + `test_inspection_tree.py` guard the shared gate/door nodes against regressions).

- [ ] **Step 5: Append the README changelog entry**

In `src/behavior_tree/README.md`, add under the `## 📜 Changelog` heading (line ~675), matching the existing entry format, dated 2026-07-02:

```markdown
- **2026-07-02** — DoingLaundry: task now starts behind an operator Enter gate
  (`BtNode_PressEnterToSucceed`), then waits for the arena door via
  `door_detection_srv` (Inspection-style Retry) before navigating to the
  folding table. Fold prompt now asks to "lay the shirt out in the manner as
  shown on my screen"; lay-out wait extended 5 s → 10 s.
```

- [ ] **Step 6: Commit (laundry.py + README only)**

Pre-existing uncommitted `laundry.py` tweaks (fold-complete message, Setup pan-tilt, commented pickup phases) ride along — mention them in the body. Do NOT stage `hri.py` / `mock_config.json`.

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && git log --oneline -1 && \
git add src/behavior_tree/behavior_tree/DoingLaundry/laundry.py src/behavior_tree/README.md && \
git commit -m "feat(DoingLaundry): Enter gate + door-detect before nav; screen fold prompt; 10s wait

Satisfies test_doing_laundry_start_gate.py (771a87b). Also carries the
pre-existing working-tree tweaks to laundry.py (fold-complete message,
Setup pan-tilt, pickup phases commented out)."
```

---

### Task 3: Build + mock-mode smoke run

**Files:** none created/modified (build + verification only).

**Interfaces:**
- Consumes: the `doing-laundry` console script (`behavior_tree.DoingLaundry.laundry:main`, `setup.py:92`).
- Produces: verified installed entry point; live on-robot verification handed to the operator.

- [ ] **Step 1: Rebuild behavior_tree via tkbuild**

```bash
tkbuild tk25_decision --packages-select behavior_tree
```
Expected: `Summary: 1 package finished`. (If the user prefers to drive builds, hand this command over instead of running it.)

- [ ] **Step 2: Mock-mode smoke run of the installed entry point**

```bash
source /home/tinker/tk25_ws/src/tk25_decision/install/setup.zsh 2>/dev/null || source /home/tinker/tk25_ws/install/setup.zsh
BT_MOCK_MODE=true ros2 run behavior_tree doing-laundry
```
Expected interaction: tree idles at `Wait for operator to start` until Enter is pressed → Setup runs → "Announce ready + aim pan-tilt" → `Door detection` waits for 's' keypress (mock KEYPRESS mode per `mock_config.json:33`) → "Announce door open" → arm/nav mocks → fold loop announces the new screen prompt and sits in the 10 s timer. Ctrl+C to exit.

- [ ] **Step 3: Hand live verification to the operator**

On-robot check (not automatable from here): `door_detection_srv` (vision_util) must be up; confirm the robot stays put until Enter + physical door opening, then drives to the folding table, and the on-screen shirt-layout image is actually being displayed when the announcement plays (the prompt references "my screen" — the screen content itself is outside this tree change; flag to the user if nothing currently renders it).
