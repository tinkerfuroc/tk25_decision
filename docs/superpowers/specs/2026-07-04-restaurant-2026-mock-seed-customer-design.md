# restaurant-2026 mock-seed-customer bypass — design

- **Date:** 2026-07-04
- **Status:** Approved (design), pending implementation plan
- **Scope:** `src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py` (restaurant-2026 only)
- **Entry point affected:** `restaurant-2026` (`restaurant_v2.py`). The canonical
  `restaurant` (`restaurants.py`) is **not** touched.

## Problem

Running `restaurant-2026` with everything mocked except audio (the intended
"test the real order-extraction / confirmation audio offline" configuration:
`vision`, `manipulation`, `navigation` mocked; `audio_input` + `announcement`
real) is impractical because Phase 1 blocks on person-scanning.

`createCollectOrdersPhaseItems` → `createCollectOneOrderItems` calls the shared
`createScanForUpToNCustomers([...7 positions...], n_gate=2)` once per order.
Every scan position instantiates `BtNode_TurnPanTilt` and
`BtNode_ScanForWavingPerson`, both `wait_keypress` in mock — each returns
`RUNNING` until a key is pressed. With 7 positions × 2 orders, plus 2.5 s
settle timers and real TTS announcements per position, the operator must
hand-crank ~14 keypress-gated stops before ever reaching the audio pipeline
under test. (It is not an infinite hang — the mock scan writes a synthetic
person — but it is a long, noisy manual slog that defeats offline audio
testing.)

## Goal

A single, off-by-default module constant that, when enabled, replaces Phase 1's
person-scan with a blackboard seed of a synthetic active customer, so the tree
flows straight into the **real-audio** order extraction and confirmation. No
keypresses, no pan-tilt sweep, no scan TTS.

## Non-goals

- Phase 2 (barman) and Phase 3 (delivery) are **out of scope**. They keep their
  mocked-nav keypresses. This bypass targets only "stuck on scanning".
- Not a general mock-mode feature; scoped to `restaurant-2026`.
- The seed provides only the *customer*, never a canned order — the order must
  come from real audio (that is the point of the test).

## Design

### 1. The toggle

A module-level boolean at the top of `order_intake_items.py`, off by default:

```python
# Offline audio-test affordance (restaurant-2026 only). When True, Phase 1
# skips the person-scan sweep and seeds a synthetic active customer so the tree
# jumps straight to the real-audio order extraction + confirmation. Leave False
# for production. Mirrors GPSR/EGPSR's construction-time USE_NEW_SCAN_WAVING
# toggle — a compile-time subtree swap, NOT a runtime `if MOCK_MODE` branch.
MOCK_SEED_CUSTOMER = False
```

Placement rationale: `order_intake_items.py` is the only module where
restaurant-2026's Phase-1 scan is invoked, and (per its own docstring) it is
consumed *exclusively* by `restaurant_v2.py`. Keeping the toggle and the swap
there leaves the shared `restaurants.py` scan factory — and thus the canonical
`restaurant` task — completely unaffected. `restaurant_v2.py`'s module docstring
will point to the toggle for discoverability.

### 2. The seed subtree (composed from existing nodes — no new node class)

`BtNode_WriteToBlackboard` is a plain `py_trees.behaviour.Behaviour`: no
`mock_mode` gate, `update()` writes and returns `SUCCESS`, and it is absent from
`mock_config.json`. So the seed can be built entirely from it.

```python
def _createSeedCustomerSubtree(customer_id: int) -> py_trees.composites.Sequence:
    """Seed the post-SelectNextQueuedCustomer blackboard state for one synthetic
    active customer, replacing the Phase-1 scan under MOCK_SEED_CUSTOMER."""
    pose = PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(position=Point(x=1.0, y=0.0, z=0.0),
                  orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
    )
    entry = {"id": customer_id, "pose": pose, "picture_path": "",
             "timestamp": 0.0, "confidence": 1.0, "status": "active"}
    seq = py_trees.composites.Sequence(f"Seed mock customer {customer_id}", memory=True)
    seq.add_child(_write(KEY_CUSTOMER_QUEUE, [entry]))
    seq.add_child(_write(KEY_ACTIVE_CUSTOMER_ID, customer_id))
    seq.add_child(_write(KEY_CUSTOMER_LOCATION, pose))
    seq.add_child(_write(KEY_ACTIVE_CUSTOMER_PICTURE, ""))
    return seq
```

where `_write(key, obj)` is
`BtNode_WriteToBlackboard(name=..., bb_namespace="", bb_source=None, bb_key=key, object=obj)`.

These four writes reproduce exactly what a real scan + `SelectNextQueuedCustomer`
leaves behind (verified against that node's registered keys: `queue_write`
marks the selected entry `"active"`, plus `active_id` / `selected_pose` /
`picture_path`). The placeholder `PoseStamped` is never navigated to — mocked
`BtNode_Approach` is `immediate` — but `BtNode_RecordOrder` stores it and Phase-3
delivery (mocked) reads it back without validating contents. `rclpy.time` +
`geometry_msgs` follow the existing `restaurant_v2._origin_bar_pose` pattern.

### 3. Swap point and distinct IDs

`createCollectOneOrderItems` gains an optional `seed_customer_id: int = 1` and
swaps only the scan child at construction time:

```python
def createCollectOneOrderItems(seed_customer_id: int = 1):
    root = ...
    root.add_child(BtNode_InitOrderChecklist(...))
    if MOCK_SEED_CUSTOMER:
        root.add_child(_createSeedCustomerSubtree(seed_customer_id))
    else:
        root.add_child(createScanForUpToNCustomers(scan_positions=[...], n_gate=2))
    root.add_child(createApproachCustomer())
    root.add_child(BtNode_RequireActiveCustomer(...))
    root.add_child(createTakeAndConfirmOrderItems())
    root.add_child(BtNode_RecordOrder(...))
    root.add_child(BtNode_CloseActiveCustomer(...))
    return root
```

`createCollectOrdersPhaseItems` passes the loop index so the two seeded
customers get ids `1` and `2` (deterministic, avoids collisions across the two
Phase-1 iterations; `CloseActiveCustomer` marks each `"done"` in turn):

```python
for i in range(2):
    root.add_child(py_trees.decorators.Retry(
        name=f"retry collect order {i + 1}",
        child=createCollectOneOrderItems(seed_customer_id=i + 1),
        num_failures=2))
```

### 4. What stays untouched

- **Toggle off (production):** byte-for-byte today's behavior — the `else`
  branch is the current `createScanForUpToNCustomers(...)` call verbatim, and
  the new `seed_customer_id` param is ignored on that path.
- **Shared `restaurants.py`** and the canonical `restaurant` task.
- **Phase 2/3.** Barman/delivery keep mocked-nav keypresses (per scope).
- **The order-taking `Parallel`.** `createTakeAndConfirmOrderItems` still wraps
  the order loop in `Parallel(SuccessOnSelected([order_loop]))` alongside
  `BtNode_MaintainEyeContact`.

## Relied-upon invariants (verified during adversarial review)

1. `BtNode_MaintainEyeContact` in mock returns `RUNNING` (never `FAILURE`) via
   `wait_for_keypress_in_mock`; the order-taking `Parallel` is
   `SuccessOnSelected([order_loop])`, so eye-contact cannot fail the parallel
   and the parallel returns `SUCCESS` as soon as the order loop succeeds. The
   seeded path therefore reaches order extraction without a keypress. (This is
   pre-existing behavior, unchanged by the seed.)
2. `BtNode_WriteToBlackboard` is not mock-gated and is unconditional.
3. Construction-time boolean subtree swaps are an established pattern here
   (`egpsr.py` `USE_NEW_SCAN_WAVING`), so this does not reintroduce a runtime
   `if MOCK_MODE` branch in a task tree.

## Testing

New `test/test_restaurant_2026_mock_seed.py`, following the ROS-free stub
pattern in `test_restaurant_state_machine.py` / the mock-import pattern in
`test_restaurant_audio_announcements.py`:

1. **Toggle on removes the scan.** With `MOCK_SEED_CUSTOMER` monkeypatched
   `True`, build `createCollectOrdersPhaseItems()` and assert no
   `BtNode_ScanForWavingPerson` and no `BtNode_TurnPanTilt` appear in the tree,
   and that two seed subtrees (ids 1 and 2) are present.
2. **Seed populates the blackboard.** Tick `_createSeedCustomerSubtree(7)` in
   isolation (calling `setup(node=<dummy>)` first, since
   `BtNode_WriteToBlackboard.setup` requires a `node` kwarg) and assert
   `KEY_ACTIVE_CUSTOMER_ID == 7`, `KEY_CUSTOMER_LOCATION` is the placeholder
   pose, `KEY_ACTIVE_CUSTOMER_PICTURE == ""`, and `KEY_CUSTOMER_QUEUE` holds a
   single entry with `id == 7`, `status == "active"`.
3. **Toggle off is unchanged.** With `MOCK_SEED_CUSTOMER` `False`, assert the
   scan nodes are present and no seed writes exist.

Test-first (TDD): the three tests are written and watched to fail before the
constant / seed factory / swap exist.

## Risks and trade-offs

- **Adjacent to the "no `if MOCK_MODE` in trees" rule.** Mitigated: this is a
  named developer toggle (not `BT_MOCK_MODE`), evaluated at construction time,
  matching the existing `USE_NEW_SCAN_WAVING` pattern. Documented at the swap.
- **Seed drift.** If `SelectNextQueuedCustomer`'s output contract changes, the
  seed must track it. Mitigated by test #2 asserting the exact seeded shape.
- **Left-on-in-production.** A committed `MOCK_SEED_CUSTOMER = True` would skip
  real scanning on the robot. Mitigated by default `False`, a loud module
  comment, and the toggle-off regression test. (Optional future hardening:
  assert-False at import unless an env flag is set — deferred, YAGNI.)
```
