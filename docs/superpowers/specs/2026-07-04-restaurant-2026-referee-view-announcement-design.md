# Restaurant-2026: referee-view announcement after waving detection

**Date:** 2026-07-04
**Entry point affected:** `restaurant-2026` (`behavior_tree.Restaurant.restaurant_v2:main`) only.
**Status:** design approved, pending implementation plan.

## Goal

After each successful waving-person detection — i.e. right after the robot
speaks the on-screen count ("I detected N waving customers.") — it should also
prompt the referee to view the detection result on its screen.

The content depends on whether it is the **first** such announcement of the run:

- **First fire (once per run):** the full four-line spiel
  1. "Dear referee, please move behind me."
  2. "The bounding boxes are shown on my screen."
  3. "Please take a moment to view it."
  4. "Thank you."
- **Every subsequent fire:** a single short reminder
  - "Look at my screen again to view the detection results with bounding boxes."

The announcement **fires every time** a detection queues a new caller (no
frequency suppression). Only the *content* changes between the first fire and
later fires.

## Where this hooks in

The "I detected N waving customers." line is spoken by the
`BtNode_Announce(name="Announce detected count", bb_source=KEY_WAVING_DETECT_SUMMARY)`
node inside `_create_one_sweep` (`restaurants.py`), once per scan position that
queues a *new* caller, wrapped in `FailureIsSuccess` (best-effort so a TTS
hiccup can't drop the just-detected caller).

**Scope is naturally isolated to restaurant-2026.** `_create_one_sweep` and its
caller `createScanForUpToNCustomers` are used *exclusively* by the
restaurant-2026 path (`restaurant_v2.py` → `order_intake_items.createCollectOrdersPhaseItems`
→ `createCollectOneOrderItems` → `createScanForUpToNCustomers`). The canonical
`restaurant` task uses a different detection path
(`createDetectAndArbitrateCustomers` → `_create_waving_detection_pass`) that
never even speaks the count line. Editing this block therefore touches only
restaurant-2026; no parameterization or scope plumbing is needed.

## Design

Composed entirely from existing nodes — **no new node class**.

### 1. Task-level latch flag

Add one blackboard key in `config.py`:

```python
# Latched True after the first referee-view spiel of the run, so later
# detections speak the short reminder instead of the full four-line spiel.
KEY_REFEREE_ANNOUNCED = "referee_view_announced"
```

The flag is **task-level and never reset** — "the first fire" means the first
detection in the entire run, not per-scan or per-order.

### 2. One-time initialization

`BtNode_CheckIfEmpty` raises if it reads an unset blackboard key, so the flag
must exist before the first detection. Initialize it **once** by prepending a
write as the first child of `createCollectOrdersPhaseItems`
(`order_intake_items.py`):

```python
root.add_child(
    BtNode_WriteToBlackboard(
        name="Init referee-spiel flag",
        bb_namespace="",
        bb_source=None,
        bb_key=KEY_REFEREE_ANNOUNCED,
        object=False,
    )
)
```

This runs once at Phase-1 start, before either order's scan. Placing it here
(rather than in `restaurant_v2._createConstantWriterCaptured`) keeps
`restaurant_v2.py` untouched, preserving its "changes exactly two things"
design, and keeps the init local to the module that owns the scan.

### 3. Per-detection announcement factory

New helper in `restaurants.py`, next to `_create_one_sweep`:

```python
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

    First fire of the run speaks the full REFEREE_VIEW_LINES spiel and latches
    KEY_REFEREE_ANNOUNCED; every later fire speaks REFEREE_VIEW_REMINDER.
    Wrapped FailureIsSuccess so a TTS hiccup can't fail the sweep and drop the
    just-detected caller (mirrors the adjacent "Announce detected count").
    """
```

Tree shape:

```
FailureIsSuccess "Referee announce best-effort"
└─ Selector "Referee view announcement" (memory)
   ├─ Sequence "Referee reminder (subsequent)" (memory)
   │  ├─ BtNode_CheckIfEmpty(bb_source=KEY_REFEREE_ANNOUNCED)   # truthy → already fired → SUCCESS
   │  └─ BtNode_Announce(message=REFEREE_VIEW_REMINDER)
   └─ Sequence "Referee spiel (first)" (memory)                 # only reached when flag still falsy
      ├─ BtNode_Announce(message=REFEREE_VIEW_LINES[0])
      ├─ BtNode_Announce(message=REFEREE_VIEW_LINES[1])
      ├─ BtNode_Announce(message=REFEREE_VIEW_LINES[2])
      ├─ BtNode_Announce(message=REFEREE_VIEW_LINES[3])
      └─ BtNode_WriteToBlackboard(KEY_REFEREE_ANNOUNCED, True)   # latch, after speaking
```

**Guard logic.** `BtNode_CheckIfEmpty` returns SUCCESS when the flag is truthy
and FAILURE when falsy/empty (`BaseBehaviors.py:586`). So:

- **Subsequent fire** (flag `True`): the reminder branch's `CheckIfEmpty`
  succeeds, the short line is spoken, the Selector is done — the full spiel is
  never reached.
- **First fire** (flag `False`): `CheckIfEmpty` fails → the reminder branch
  fails → the Selector falls through to the full spiel, which speaks all four
  lines and latches the flag `True`.

No `Inverter` and no new node class are required. The latch write is placed
*after* the four announces, so a total TTS failure re-arms the spiel for the
next detection rather than silently swallowing it (consistent with the existing
best-effort tolerance).

### 4. Insertion into `_create_one_sweep`

Immediately after the existing best-effort "Announce detected count" child:

```python
detect_at_pos.add_child(
    py_trees.decorators.FailureIsSuccess(
        name="Announce is best-effort",
        child=BtNode_Announce(name="Announce detected count",
                              bb_source=KEY_WAVING_DETECT_SUMMARY),
    )
)
# NEW: referee-view prompt (first fire = full spiel, later = short reminder).
detect_at_pos.add_child(_create_referee_view_announcement())
```

## Rejected alternatives

- **Per-scan reset / once-per-scan suppression.** Earlier iteration; the user
  chose "fire every time" with content that varies by first-vs-subsequent, so
  no reset is added and the flag is task-level.
- **Reuse the per-order checklist (`KEY_ORDER_CHECKLIST`).** Would need a new
  checklist-flag reader node and couples referee-announce to unrelated order
  state; also per-order not per-run. More code, more coupling.
- **Speak once after the whole scan completes.** Detaches the prompt from the
  "I detected N" line; the requirement is that it follows each detection.

## Testing

Extend `test/test_scan_for_n_customers.py` (already exercises the scan tree
structurally via `scan_tree.iterate()` and ticks blackboard nodes directly):

- **Structural (scan tree):**
  - The full-spiel branch contains all four `REFEREE_VIEW_LINES`, present once
    per scan position.
  - The subsequent branch's `BtNode_CheckIfEmpty` guards on
    `KEY_REFEREE_ANNOUNCED` and precedes the `REFEREE_VIEW_REMINDER` announce.
  - The first-fire branch latches the flag via a `BtNode_WriteToBlackboard`
    writing `True` to `KEY_REFEREE_ANNOUNCED`.
- **Structural (init):** `createCollectOrdersPhaseItems` writes
  `KEY_REFEREE_ANNOUNCED = False` exactly once, before the order-collect
  retries.
- **Semantic (guard):** tick `BtNode_CheckIfEmpty` against the flag — `True` →
  SUCCESS (reminder branch wins), `False`/empty → FAILURE (falls through to full
  spiel) — locking in the first-vs-subsequent behavior. (May pass a dummy `node`
  in `setup()` since `BtNode_CheckIfEmpty.setup` requires it.)

Run with the package venv:

```bash
.venv_decision/bin/python -m pytest src/behavior_tree/test/test_scan_for_n_customers.py
```

## Adversarial review (verified 2026-07-04)

Stress-tested against the actual code; no runtime bugs found. Verified facts:

- **Blackboard namespace match.** `BtNode_WriteToBlackboard(bb_namespace="")`
  attaches its client with `namespace=""` and registers the bare key;
  `BtNode_CheckIfEmpty` attaches with the default (root) namespace and registers
  the bare key. Empirically both resolve to the same absolute key
  (`/referee_view_announced`) — the read returns the written value, no unset-key
  raise. `False`→`FAILURE`→full spiel, `True`→`SUCCESS`→reminder, as designed.
- **`setup(node=...)` is provisioned.** `BtNode_CheckIfEmpty.setup` requires
  `kwargs["node"]`. `run_tree` calls `tree.setup(...)`, which passes `node` to
  every behaviour (the same requirement in `BtNode_WriteToBlackboard.setup`
  already works live). `BtNode_CheckIfEmpty` is also used live in
  `DoingLaundry/laundry.py:468`, confirming the pattern.
- **Child order is load-bearing.** The reminder branch (guarded by
  `CheckIfEmpty`) MUST be the Selector's *first* child and the unguarded
  full-spiel branch *second*. Reversed, the spiel would fire every time and the
  reminder never. A comment must state this.
- **First fire speaks only the spiel.** The reminder Sequence's `CheckIfEmpty`
  guard fails *before* its announce runs, so the first fire is not
  reminder+spiel.
- **Best-effort trade-off documented.** A mid-spiel TTS failure leaves the flag
  un-latched, so the full spiel is re-attempted on the next detection. This is
  intentional (matches the adjacent count-announce's best-effort wrapping);
  the latch write is deliberately last.
- **No impact on sweep/gate semantics.** The new node is wrapped
  `FailureIsSuccess` and appended after the count-announce inside the memory
  `detect_at_pos` Sequence, which only runs after a *successful* detection;
  it cannot change whether `detect_at_pos` succeeds or the `n_gate` gates.

## Files touched

| File | Change |
|---|---|
| `behavior_tree/Restaurant/config.py` | `+KEY_REFEREE_ANNOUNCED` |
| `behavior_tree/Restaurant/restaurants.py` | `REFEREE_VIEW_LINES`/`REFEREE_VIEW_REMINDER` + `_create_referee_view_announcement()` + one insert in `_create_one_sweep` |
| `behavior_tree/Restaurant/order_intake_items.py` | one-time flag init in `createCollectOrdersPhaseItems` |
| `test/test_scan_for_n_customers.py` | referee-spiel structural + guard-semantic tests |
