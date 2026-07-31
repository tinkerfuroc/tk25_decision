# Restaurant-2026 Referee-View Announcement Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** After each successful waving-person detection in restaurant-2026, the robot prompts the referee to view the on-screen detection boxes — the full four-line spiel on the first detection of the run, a one-line reminder on every subsequent detection.

**Architecture:** A task-level blackboard latch (`KEY_REFEREE_ANNOUNCED`) plus a best-effort guard `Selector` composed from existing nodes (`BtNode_CheckIfEmpty` / `BtNode_Announce` / `BtNode_WriteToBlackboard`). The `Selector`'s guarded reminder branch is first and the unguarded full-spiel branch second, so a falsy flag falls through to the full spiel (which latches the flag) and a truthy flag speaks the reminder. No new node class. Scope is naturally isolated to restaurant-2026 because `_create_one_sweep` / `createScanForUpToNCustomers` are used only by that path.

**Tech Stack:** Python, `py_trees`, ROS2 `behavior_tree` package. Tests: `pytest`.

## Global Constraints

- Change touches `restaurant-2026` only. Do NOT modify `restaurant_v2.py`; do NOT touch the canonical `restaurant` detection path (`createDetectAndArbitrateCustomers`).
- No new `BtNode_*` class — compose from existing nodes.
- Exact first-fire lines, in order: `"Dear referee, please move behind me."`, `"The bounding boxes are shown on my screen."`, `"Please take a moment to view it."`, `"Thank you."`
- Exact subsequent-fire line: `"Look at my screen again to view the detection results with bounding boxes."`
- Latch key name: `KEY_REFEREE_ANNOUNCED = "referee_view_announced"`.
- `Selector` child order is load-bearing: guarded reminder branch FIRST, unguarded full-spiel branch SECOND.
- Run tests with the package venv: `.venv_decision/bin/python -m pytest ...` (run from repo root `/home/tinker/tk25_ws/src/tk25_decision`). Some pre-existing tests fail at baseline; only require that the NEW tests pass and no previously-passing test regresses.

---

### Task 1: Latch key + referee-view announcement factory

Adds the config key and the self-contained `_create_referee_view_announcement()` factory with its module-level line constants, verified by unit tests on the factory's structure and the guard's truthy/falsy semantics.

**Files:**
- Modify: `src/behavior_tree/behavior_tree/Restaurant/config.py` (append one key near the other waving keys, ~line 59)
- Modify: `src/behavior_tree/behavior_tree/Restaurant/restaurants.py` (import `BtNode_CheckIfEmpty`; add `KEY_REFEREE_ANNOUNCED` to the `.config` import; add constants + factory just above `_create_one_sweep`, ~line 346)
- Test: `src/behavior_tree/test/test_scan_for_n_customers.py` (append tests)

**Interfaces:**
- Produces:
  - `config.KEY_REFEREE_ANNOUNCED: str` = `"referee_view_announced"`
  - `restaurants.REFEREE_VIEW_LINES: tuple[str, ...]` (4 lines) and `restaurants.REFEREE_VIEW_REMINDER: str`
  - `restaurants._create_referee_view_announcement() -> py_trees.decorators.FailureIsSuccess` wrapping a `Selector` `[reminder-branch, full-spiel-branch]`

- [ ] **Step 1: Write the failing tests**

Append to `src/behavior_tree/test/test_scan_for_n_customers.py`:

```python
# --------------------------------------------------------------------------- #
# Referee-view announcement (first = full spiel, subsequent = short reminder)
# --------------------------------------------------------------------------- #
def test_referee_guard_checkifempty_semantics():
    """The CheckIfEmpty guard drives first-vs-subsequent: falsy flag -> FAILURE
    (fall through to full spiel), truthy flag -> SUCCESS (reminder wins)."""
    from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_CheckIfEmpty
    from behavior_tree.Restaurant.config import KEY_REFEREE_ANNOUNCED

    _set_bb(**{KEY_REFEREE_ANNOUNCED: False})
    falsy = BtNode_CheckIfEmpty(name="g_false", bb_source=KEY_REFEREE_ANNOUNCED)
    falsy.setup(node=object())
    assert _tick_once(falsy) == py_trees.common.Status.FAILURE

    _set_bb(**{KEY_REFEREE_ANNOUNCED: True})
    truthy = BtNode_CheckIfEmpty(name="g_true", bb_source=KEY_REFEREE_ANNOUNCED)
    truthy.setup(node=object())
    assert _tick_once(truthy) == py_trees.common.Status.SUCCESS


def test_referee_factory_has_guard_lines_and_latch():
    """Factory subtree contains the flag guard, all four spiel lines, the short
    reminder, and a latch write of True to the flag key."""
    from behavior_tree.Restaurant.restaurants import (
        _create_referee_view_announcement,
        REFEREE_VIEW_LINES,
        REFEREE_VIEW_REMINDER,
    )
    from behavior_tree.Restaurant.config import KEY_REFEREE_ANNOUNCED
    from behavior_tree.TemplateNodes.BaseBehaviors import (
        BtNode_CheckIfEmpty,
        BtNode_WriteToBlackboard,
    )

    nodes = list(_create_referee_view_announcement().iterate())

    guards = [n for n in nodes
              if isinstance(n, BtNode_CheckIfEmpty)
              and n.bb_source == KEY_REFEREE_ANNOUNCED]
    assert len(guards) == 1

    latches = [n for n in nodes
               if isinstance(n, BtNode_WriteToBlackboard)
               and n.bb_key == KEY_REFEREE_ANNOUNCED and n.object is True]
    assert len(latches) == 1

    msgs = [getattr(n, "given_msg", None) for n in nodes]
    for line in REFEREE_VIEW_LINES:
        assert line in msgs
    assert REFEREE_VIEW_REMINDER in msgs


def test_referee_reminder_branch_is_first():
    """The guarded reminder branch must be the Selector's FIRST child; the
    unguarded full spiel SECOND. (Reversed, the spiel would fire every time.)"""
    from behavior_tree.Restaurant.restaurants import _create_referee_view_announcement
    from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_CheckIfEmpty

    selector = _create_referee_view_announcement().children[0]
    first_branch, second_branch = selector.children[0], selector.children[1]
    assert any(isinstance(n, BtNode_CheckIfEmpty) for n in first_branch.iterate())
    assert not any(isinstance(n, BtNode_CheckIfEmpty) for n in second_branch.iterate())
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv_decision/bin/python -m pytest src/behavior_tree/test/test_scan_for_n_customers.py -k referee -v`
Expected: FAIL — `ImportError: cannot import name '_create_referee_view_announcement'` (and `REFEREE_VIEW_LINES`, `KEY_REFEREE_ANNOUNCED`).

- [ ] **Step 3: Add the config key**

In `src/behavior_tree/behavior_tree/Restaurant/config.py`, after the `KEY_WAVING_DETECT_SUMMARY` block (~line 59), add:

```python
# Latched True after the first referee-view spiel of the run (BtNode_CheckIfEmpty
# guard in restaurants._create_referee_view_announcement), so later detections
# speak the short reminder instead of the full four-line spiel. Task-level;
# initialized once by createCollectOrdersPhaseItems and never reset.
KEY_REFEREE_ANNOUNCED = "referee_view_announced"
```

- [ ] **Step 4: Wire imports in `restaurants.py`**

In `src/behavior_tree/behavior_tree/Restaurant/restaurants.py`:

Change line 34 from:

```python
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
```

to:

```python
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_CheckIfEmpty, BtNode_WriteToBlackboard
```

In the `from .config import (` block (starts ~line 60), add `KEY_REFEREE_ANNOUNCED,` in alphabetical position (between `KEY_PICKUP_VERIFIED,` and `KEY_TRAY_LOCATION,`).

- [ ] **Step 5: Add the constants + factory**

In `restaurants.py`, immediately above `def _create_one_sweep(` (~line 346), insert:

```python
# Referee-view prompt spoken after each on-screen detection count (restaurant-2026
# only). First fire of the run speaks REFEREE_VIEW_LINES; every later fire speaks
# REFEREE_VIEW_REMINDER. User-facing referee-interaction copy — edit here.
REFEREE_VIEW_LINES = (
    "Dear referee, please move behind me.",
    "The bounding boxes are shown on my screen.",
    "Please take a moment to view it.",
    "Thank you.",
)
REFEREE_VIEW_REMINDER = (
    "Look at my screen again to view the detection results with bounding boxes."
)


def _create_referee_view_announcement():
    """Referee-view prompt spoken after each on-screen detection count.

    First fire of the run speaks the full ``REFEREE_VIEW_LINES`` spiel and latches
    ``KEY_REFEREE_ANNOUNCED``; every later fire speaks ``REFEREE_VIEW_REMINDER``.
    Wrapped ``FailureIsSuccess`` so a TTS hiccup can't fail the sweep and drop the
    just-detected caller (mirrors the adjacent "Announce detected count").

    Child order is load-bearing: the guarded reminder branch MUST be the
    Selector's FIRST child, the unguarded full-spiel branch SECOND.
    ``BtNode_CheckIfEmpty`` returns SUCCESS when the flag is truthy (already
    fired) -> reminder wins; FAILURE when falsy (first fire) -> the Selector falls
    through to the full spiel, which latches the flag. The latch write is last, so
    a mid-spiel TTS failure re-arms the spiel for the next detection rather than
    silently swallowing it.
    """
    reminder = py_trees.composites.Sequence(
        name="Referee reminder (subsequent)", memory=True
    )
    reminder.add_child(
        BtNode_CheckIfEmpty(
            name="Already gave full referee spiel?",
            bb_source=KEY_REFEREE_ANNOUNCED,
        )
    )
    reminder.add_child(
        BtNode_Announce(
            name="Announce referee reminder",
            bb_source=None,
            message=REFEREE_VIEW_REMINDER,
        )
    )

    full = py_trees.composites.Sequence(name="Referee spiel (first)", memory=True)
    for idx, line in enumerate(REFEREE_VIEW_LINES):
        full.add_child(
            BtNode_Announce(
                name=f"Announce referee line {idx + 1}",
                bb_source=None,
                message=line,
            )
        )
    full.add_child(
        BtNode_WriteToBlackboard(
            name="Latch referee spiel done",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_REFEREE_ANNOUNCED,
            object=True,
        )
    )

    selector = py_trees.composites.Selector(
        name="Referee view announcement", memory=True
    )
    selector.add_child(reminder)
    selector.add_child(full)
    return py_trees.decorators.FailureIsSuccess(
        name="Referee announce best-effort", child=selector
    )
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `.venv_decision/bin/python -m pytest src/behavior_tree/test/test_scan_for_n_customers.py -k referee -v`
Expected: PASS (3 tests).

- [ ] **Step 7: Commit**

```bash
git add src/behavior_tree/behavior_tree/Restaurant/config.py \
        src/behavior_tree/behavior_tree/Restaurant/restaurants.py \
        src/behavior_tree/test/test_scan_for_n_customers.py
git commit -m "feat(restaurant): referee-view announcement factory + latch key"
```

---

### Task 2: Wire the factory into the sweep + one-time flag init

Inserts the factory after the detection-count announce in `_create_one_sweep` (so it fires per successful detection) and initializes the latch to `False` once at Phase-1 start, verified by structural tests on the scan tree and the collect phase.

**Files:**
- Modify: `src/behavior_tree/behavior_tree/Restaurant/restaurants.py` (`_create_one_sweep`, right after the "Announce detected count" child, ~line 403)
- Modify: `src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py` (add `KEY_REFEREE_ANNOUNCED` to the `.config` import; prepend the init write in `createCollectOrdersPhaseItems`, ~line 303)
- Test: `src/behavior_tree/test/test_scan_for_n_customers.py` (append tests)

**Interfaces:**
- Consumes (from Task 1): `restaurants._create_referee_view_announcement`, `restaurants.REFEREE_VIEW_LINES`, `config.KEY_REFEREE_ANNOUNCED`.
- Produces: no new symbols — `createScanForUpToNCustomers` now contains one referee factory per scan position; `createCollectOrdersPhaseItems`'s first child is a `BtNode_WriteToBlackboard` writing `False` to `KEY_REFEREE_ANNOUNCED`.

- [ ] **Step 1: Write the failing tests**

Append to `src/behavior_tree/test/test_scan_for_n_customers.py` (the `scan_tree` module fixture already builds a 3-position scan):

```python
def test_scan_has_referee_spiel_once_per_position(scan_tree):
    """One referee-view factory per scan position: the first spiel line and the
    reminder each appear once per position (3 in the fixture)."""
    from behavior_tree.Restaurant.restaurants import (
        REFEREE_VIEW_LINES,
        REFEREE_VIEW_REMINDER,
    )
    msgs = [getattr(n, "given_msg", None) for n in scan_tree.iterate()]
    assert msgs.count(REFEREE_VIEW_LINES[0]) == 3
    assert msgs.count(REFEREE_VIEW_REMINDER) == 3


def test_collect_phase_inits_referee_flag_once_first():
    """createCollectOrdersPhaseItems initializes the latch to False exactly once,
    as the phase's first child (before the order-collect retries)."""
    from behavior_tree.Restaurant.order_intake_items import createCollectOrdersPhaseItems
    from behavior_tree.Restaurant.config import KEY_REFEREE_ANNOUNCED
    from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard

    phase = createCollectOrdersPhaseItems()
    inits = [n for n in phase.iterate()
             if isinstance(n, BtNode_WriteToBlackboard)
             and n.bb_key == KEY_REFEREE_ANNOUNCED and n.object is False]
    assert len(inits) == 1
    assert phase.children[0] is inits[0]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `.venv_decision/bin/python -m pytest src/behavior_tree/test/test_scan_for_n_customers.py -k "referee_spiel_once or inits_referee" -v`
Expected: FAIL — `test_scan_has_referee_spiel_once_per_position` gets `count == 0` (factory not wired into the sweep); `test_collect_phase_inits_referee_flag_once_first` gets `len(inits) == 0` (init not added).

- [ ] **Step 3: Insert the factory into `_create_one_sweep`**

In `restaurants.py`, in `_create_one_sweep`, immediately after the existing best-effort "Announce detected count" child (the `detect_at_pos.add_child(py_trees.decorators.FailureIsSuccess(name="Announce is best-effort", ...))` block, ~line 403), add:

```python
        # Referee-view prompt after the on-screen count: full spiel on the first
        # detection of the run, a short reminder on every later detection.
        detect_at_pos.add_child(_create_referee_view_announcement())
```

- [ ] **Step 4: Add the one-time flag init in `order_intake_items.py`**

In `src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py`, add `KEY_REFEREE_ANNOUNCED,` to the `from .config import (` block (alphabetical, before `KEY_CUSTOMER_LOCATION,`).

Then in `createCollectOrdersPhaseItems` (~line 303), prepend the init as the first child, before the `for i in range(2):` loop:

```python
    root = py_trees.composites.Sequence(name="Collect orders (2x, items)", memory=True)
    # Task-level init of the referee-spiel latch: the first detection of the run
    # speaks the full spiel, every later detection the short reminder. Runs once,
    # before either order's scan; never reset.
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Init referee-spiel flag",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_REFEREE_ANNOUNCED,
            object=False,
        )
    )
    for i in range(2):
        root.add_child(
            py_trees.decorators.Retry(
                name=f"retry collect order {i + 1}",
                child=createCollectOneOrderItems(seed_customer_id=i + 1),
                num_failures=2,
            )
        )
    return root
```

(`BtNode_WriteToBlackboard` is already imported in `order_intake_items.py` at line 52.)

- [ ] **Step 5: Run tests to verify they pass**

Run: `.venv_decision/bin/python -m pytest src/behavior_tree/test/test_scan_for_n_customers.py -k "referee_spiel_once or inits_referee" -v`
Expected: PASS (2 tests).

- [ ] **Step 6: Run the full scan test file (no regressions)**

Run: `.venv_decision/bin/python -m pytest src/behavior_tree/test/test_scan_for_n_customers.py -v`
Expected: all tests PASS (5 pre-existing + 5 new referee tests).

- [ ] **Step 7: Commit**

```bash
git add src/behavior_tree/behavior_tree/Restaurant/restaurants.py \
        src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py \
        src/behavior_tree/test/test_scan_for_n_customers.py
git commit -m "feat(restaurant): fire referee-view prompt after each waving detection (restaurant-2026)"
```

---

### Task 3: Build + import smoke verification

Confirms the package builds and the restaurant-2026 tree still constructs with the change (catches import/typo errors the structural tests can't, since they import modules directly).

**Files:** none (verification only).

- [ ] **Step 1: Byte-compile the three changed modules**

Run from repo root:
`.venv_decision/bin/python -m py_compile src/behavior_tree/behavior_tree/Restaurant/config.py src/behavior_tree/behavior_tree/Restaurant/restaurants.py src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py`
Expected: no output (success).

- [ ] **Step 2: Construct the full restaurant-2026 tree offline**

Run:
```bash
BT_MOCK_MODE=true .venv_decision/bin/python -c "
from behavior_tree.Restaurant.restaurant_v2 import createRestaurantTask2026
from behavior_tree.Restaurant.restaurants import REFEREE_VIEW_LINES, REFEREE_VIEW_REMINDER
root = createRestaurantTask2026()
msgs = [getattr(n, 'given_msg', None) for n in root.iterate()]
assert msgs.count(REFEREE_VIEW_LINES[0]) >= 1, 'referee spiel not present in 2026 tree'
assert REFEREE_VIEW_REMINDER in msgs, 'reminder line not present in 2026 tree'
print('OK: restaurant-2026 tree constructs; referee lines present:', msgs.count(REFEREE_VIEW_LINES[0]))
"
```
Expected: `OK: restaurant-2026 tree constructs; referee lines present: <N>` (N = number of scan positions in Phase 1, currently 7 per order × 2 orders paths — any value ≥ 1 passes).

If this step fails with the pre-existing `messages.py` / `TTSCnRequest` import error documented in `src/tk25_decision/CLAUDE.md`, that is a host-baseline issue unrelated to this change; note it and rely on Task 1/2 structural tests (which import the same modules and must pass).

- [ ] **Step 3: No commit** (verification only).

---

## Self-Review

**Spec coverage:**
- First-fire full 4-line spiel → Task 1 factory (`REFEREE_VIEW_LINES`, full branch) + `test_referee_factory_has_guard_lines_and_latch`. ✅
- Subsequent-fire single reminder → Task 1 factory (`REFEREE_VIEW_REMINDER`, reminder branch) + guard-semantic test. ✅
- Fires every time / no suppression → factory always announces (reminder or spiel); Task 2 wires it per position. ✅
- Task-level latch, never reset → `KEY_REFEREE_ANNOUNCED` (Task 1) + one-time init in `createCollectOrdersPhaseItems` (Task 2) + `test_collect_phase_inits_referee_flag_once_first`. ✅
- Guard via `BtNode_CheckIfEmpty`, no new node class → Task 1 factory, all existing nodes. ✅
- Reminder-branch-first ordering → `test_referee_reminder_branch_is_first`. ✅
- Best-effort wrapping → `FailureIsSuccess` in factory. ✅
- Insertion after count-announce → Task 2 Step 3. ✅
- Scope isolated to restaurant-2026 (no `restaurant_v2.py`/canonical edits) → only `config.py`, `restaurants.py`, `order_intake_items.py` touched; canonical path untouched. ✅
- Namespace/`setup(node=)` correctness → verified empirically in the spec's Adversarial Review; guard-semantic test re-checks at runtime. ✅

**Placeholder scan:** No TBD/TODO/"handle edge cases"; every code step shows full code; every test step shows the assertion and the exact command + expected output.

**Type consistency:** `KEY_REFEREE_ANNOUNCED`, `REFEREE_VIEW_LINES`, `REFEREE_VIEW_REMINDER`, `_create_referee_view_announcement` used identically across Tasks 1–3. `BtNode_Announce` message read via `given_msg`, `BtNode_WriteToBlackboard` fields `bb_key`/`object`, `BtNode_CheckIfEmpty` field `bb_source` — all match the actual class definitions.
