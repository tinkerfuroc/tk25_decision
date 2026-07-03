# restaurant-2026 mock-seed-customer bypass — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an off-by-default `MOCK_SEED_CUSTOMER` toggle to `order_intake_items.py` that swaps restaurant-2026's Phase-1 person-scan for a blackboard seed of a synthetic active customer, so the tree flows straight into real-audio order extraction/confirmation when everything but audio is mocked.

**Architecture:** A module-level boolean, evaluated at tree-construction time (mirroring GPSR/EGPSR's `USE_NEW_SCAN_WAVING`), selects between the existing `createScanForUpToNCustomers(...)` subtree and a new `_createSeedCustomerSubtree(id)` built from plain `BtNode_WriteToBlackboard` writes. The seed reproduces exactly what a real scan + `BtNode_SelectNextQueuedCustomer` leaves on the blackboard. The shared `restaurants.py` and the canonical `restaurant` task are untouched.

**Tech Stack:** Python 3.10, py_trees, ROS 2 Humble (`geometry_msgs`, `std_msgs`, `rclpy`), pytest.

**Spec:** `docs/superpowers/specs/2026-07-04-restaurant-2026-mock-seed-customer-design.md`

## Global Constraints

- Python 3.10 only (matches Humble); do not target 3.11+.
- Commit author MUST be `Ccindy0171 <cindy.w0135@gmail.com>` (use `--author`), not the default.
- Never hand-copy into `install/`; rebuild via `./tkbuild tk25_decision --packages-select behavior_tree` from the workspace root.
- Do NOT reintroduce a runtime `if MOCK_MODE` branch inside a node. The toggle is a construction-time subtree swap only.
- `MOCK_SEED_CUSTOMER` MUST default to `False` (production-safe).
- Toggle OFF must leave the Phase-1 path byte-for-byte identical to today.
- Test runs on this host require `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` (a user-site `anyio` plugin clashes with system pytest) and a sourced workspace.

**Standard test-run command** (used verbatim in every task):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
source /opt/ros/humble/setup.zsh 2>/dev/null; source /home/tinker/tk25_ws/install/setup.zsh 2>/dev/null
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_restaurant_2026_mock_seed.py -q
```

---

### Task 1: Constant, imports, and the seed-customer subtree factory

**Files:**
- Modify: `src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py` (imports block ~line 46-53; new constant + factory after `ORDER_ITEMS_TIMEOUT_S` at line 79)
- Test: `src/behavior_tree/test/test_restaurant_2026_mock_seed.py`

**Interfaces:**
- Produces:
  - `MOCK_SEED_CUSTOMER: bool` (module global, default `False`)
  - `_seed_write(key: str, obj) -> BtNode_WriteToBlackboard`
  - `_createSeedCustomerSubtree(customer_id: int) -> py_trees.composites.Sequence` — on tick, writes `KEY_CUSTOMER_QUEUE=[{"id":customer_id,"pose":PoseStamped,"picture_path":"","timestamp":0.0,"confidence":1.0,"status":"active"}]`, `KEY_ACTIVE_CUSTOMER_ID=customer_id`, `KEY_CUSTOMER_LOCATION=PoseStamped(frame_id="map")`, `KEY_ACTIVE_CUSTOMER_PICTURE=""`.

- [ ] **Step 1: Write the failing test**

Create `src/behavior_tree/test/test_restaurant_2026_mock_seed.py`:

```python
# Copyright 2026 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""restaurant-2026 MOCK_SEED_CUSTOMER Phase-1 bypass.

Verifies the offline audio-test toggle: when on, Phase 1 seeds a synthetic
active customer instead of scanning; when off, the real scan is unchanged.
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
import pytest  # noqa: E402

import behavior_tree.Restaurant.order_intake_items as oii  # noqa: E402


class _DummyNode:
    """Stand-in for the rclpy node BtNode_WriteToBlackboard.setup() requires."""


def _clear_bb():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()


def _node_class_names(root):
    return {n.__class__.__name__ for n in root.iterate()}


def _setup_and_tick(subtree):
    for node in subtree.iterate():
        node.setup(node=_DummyNode())
    for _ in subtree.tick():
        pass
    return subtree.status


def test_seed_subtree_populates_active_customer_state():
    _clear_bb()
    subtree = oii._createSeedCustomerSubtree(7)
    assert _setup_and_tick(subtree) == py_trees.common.Status.SUCCESS

    reader = py_trees.blackboard.Client(name="reader")
    for key in (
        oii.KEY_ACTIVE_CUSTOMER_ID,
        oii.KEY_CUSTOMER_LOCATION,
        oii.KEY_ACTIVE_CUSTOMER_PICTURE,
        oii.KEY_CUSTOMER_QUEUE,
    ):
        reader.register_key(
            key=key,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
    assert getattr(reader, oii.KEY_ACTIVE_CUSTOMER_ID) == 7
    assert getattr(reader, oii.KEY_ACTIVE_CUSTOMER_PICTURE) == ""
    queue = getattr(reader, oii.KEY_CUSTOMER_QUEUE)
    assert len(queue) == 1
    assert queue[0]["id"] == 7
    assert queue[0]["status"] == "active"
    loc = getattr(reader, oii.KEY_CUSTOMER_LOCATION)
    assert loc.header.frame_id == "map"
```

- [ ] **Step 2: Run test to verify it fails**

Run the standard test-run command.
Expected: FAIL — `AttributeError: module 'behavior_tree.Restaurant.order_intake_items' has no attribute '_createSeedCustomerSubtree'`.

- [ ] **Step 3: Add imports**

In `order_intake_items.py`, extend the imports. After the existing
`from behavior_tree.TemplateNodes.Vision import BtNode_MaintainEyeContact` line, add:

```python
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
```

- [ ] **Step 4: Add the constant and the seed factory**

Immediately after the `ORDER_ITEMS_TIMEOUT_S = 7.0` line (line 79), add:

```python

# Offline audio-test affordance (restaurant-2026 only). When True, Phase 1
# skips the person-scan sweep and seeds a synthetic active customer so the tree
# jumps straight to the real-audio order extraction + confirmation. Leave False
# for production. Mirrors GPSR/EGPSR's construction-time USE_NEW_SCAN_WAVING
# toggle -- a compile-time subtree swap, NOT a runtime `if MOCK_MODE` branch.
MOCK_SEED_CUSTOMER = False


def _seed_write(key: str, obj):
    """One root-namespace blackboard write for the mock-customer seed."""
    return BtNode_WriteToBlackboard(
        name=f"Seed {key}",
        bb_namespace="",
        bb_source=None,
        bb_key=key,
        object=obj,
    )


def _createSeedCustomerSubtree(customer_id: int) -> py_trees.composites.Sequence:
    """Seed the post-SelectNextQueuedCustomer blackboard state for one synthetic
    active customer, replacing the Phase-1 scan under MOCK_SEED_CUSTOMER.

    Writes exactly what a real scan + BtNode_SelectNextQueuedCustomer leaves
    behind: a queue entry marked "active", the active id, the customer location
    (a placeholder PoseStamped -- mocked BtNode_Approach is immediate and never
    navigates to it), and an empty picture path.
    """
    pose = PoseStamped(
        header=Header(stamp=rclpy.time.Time().to_msg(), frame_id="map"),
        pose=Pose(
            position=Point(x=1.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    entry = {
        "id": customer_id,
        "pose": pose,
        "picture_path": "",
        "timestamp": 0.0,
        "confidence": 1.0,
        "status": "active",
    }
    seq = py_trees.composites.Sequence(
        name=f"Seed mock customer {customer_id}", memory=True
    )
    seq.add_child(_seed_write(KEY_CUSTOMER_QUEUE, [entry]))
    seq.add_child(_seed_write(KEY_ACTIVE_CUSTOMER_ID, customer_id))
    seq.add_child(_seed_write(KEY_CUSTOMER_LOCATION, pose))
    seq.add_child(_seed_write(KEY_ACTIVE_CUSTOMER_PICTURE, ""))
    return seq
```

- [ ] **Step 5: Run test to verify it passes**

Run the standard test-run command.
Expected: PASS (`1 passed`).

- [ ] **Step 6: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py \
        src/behavior_tree/test/test_restaurant_2026_mock_seed.py
git commit --author="Ccindy0171 <cindy.w0135@gmail.com>" -m "feat(restaurant): add MOCK_SEED_CUSTOMER seed subtree for restaurant-2026

Off-by-default toggle + _createSeedCustomerSubtree that writes the
post-SelectNextQueuedCustomer blackboard state (queue/active-id/location/
picture) from plain BtNode_WriteToBlackboard nodes, for offline audio testing.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: Wire the toggle into Phase 1 and point the docstring at it

**Files:**
- Modify: `src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py` (`createCollectOneOrderItems` line 170; its scan `add_child` block lines ~185-201; `createCollectOrdersPhaseItems` loop lines ~237-244)
- Modify: `src/behavior_tree/behavior_tree/Restaurant/restaurant_v2.py` (module docstring)
- Test: `src/behavior_tree/test/test_restaurant_2026_mock_seed.py` (add two tests)

**Interfaces:**
- Consumes: `MOCK_SEED_CUSTOMER`, `_createSeedCustomerSubtree` (Task 1); existing `createScanForUpToNCustomers`, `createApproachCustomer`, `BtNode_InitOrderChecklist`, `BtNode_RequireActiveCustomer`, `BtNode_RecordOrder`, `BtNode_CloseActiveCustomer`.
- Produces: `createCollectOneOrderItems(seed_customer_id: int = 1)` (new optional param); `createCollectOrdersPhaseItems()` now passes `seed_customer_id=i + 1`.

- [ ] **Step 1: Write the failing tests**

Append to `src/behavior_tree/test/test_restaurant_2026_mock_seed.py`:

```python
def test_toggle_on_replaces_scan_with_two_seeds(monkeypatch):
    monkeypatch.setattr(oii, "MOCK_SEED_CUSTOMER", True)
    phase = oii.createCollectOrdersPhaseItems()
    names = _node_class_names(phase)
    assert "BtNode_ScanForWavingPerson" not in names
    assert "BtNode_TurnPanTilt" not in names
    seed_names = [
        n.name for n in phase.iterate() if n.name.startswith("Seed mock customer")
    ]
    assert "Seed mock customer 1" in seed_names
    assert "Seed mock customer 2" in seed_names


def test_toggle_off_keeps_scan_and_no_seed(monkeypatch):
    # Regression guard for the production path: passes before and after wiring.
    monkeypatch.setattr(oii, "MOCK_SEED_CUSTOMER", False)
    phase = oii.createCollectOrdersPhaseItems()
    names = _node_class_names(phase)
    assert "BtNode_ScanForWavingPerson" in names
    assert "BtNode_TurnPanTilt" in names
    seed_names = [
        n.name for n in phase.iterate() if n.name.startswith("Seed mock customer")
    ]
    assert seed_names == []
```

- [ ] **Step 2: Run tests to verify the toggle-on test fails**

Run the standard test-run command.
Expected: `test_toggle_on_replaces_scan_with_two_seeds` FAILS on `assert "BtNode_ScanForWavingPerson" not in names` (scan is still built because the swap isn't wired). `test_toggle_off_keeps_scan_and_no_seed` passes (regression guard). `test_seed_subtree_populates_active_customer_state` still passes.

- [ ] **Step 3: Add the `seed_customer_id` param and the swap**

In `order_intake_items.py`, change the `createCollectOneOrderItems` signature:

```python
def createCollectOneOrderItems(seed_customer_id: int = 1) -> py_trees.composites.Sequence:
```

Then replace the scan `add_child` block (the `root.add_child(createScanForUpToNCustomers(...))` call with its `scan_positions=[...]`, `n_gate=2`) with the toggle:

```python
    if MOCK_SEED_CUSTOMER:
        root.add_child(_createSeedCustomerSubtree(seed_customer_id))
    else:
        root.add_child(
            createScanForUpToNCustomers(
                # BtNode_TurnPanTilt takes (x=pan, y=tilt) in degrees -- tilt
                # fixed at 35 deg for every position (a consistent
                # look-for-a-person angle), pan sweeps the room.
                scan_positions=[
                    (0.0, 35.0),
                    (30.0, 35.0),
                    (60.0, 35.0),
                    (-30.0, 35.0),
                    (-60.0, 35.0),
                    (-120.0, 35.0),
                    (120.0, 35.0),
                ],
                n_gate=2,
            )
        )
```

- [ ] **Step 4: Thread the id through the Phase-1 loop**

In `createCollectOrdersPhaseItems`, change the child construction to pass the loop index as the seed id:

```python
    for i in range(2):
        root.add_child(
            py_trees.decorators.Retry(
                name=f"retry collect order {i + 1}",
                child=createCollectOneOrderItems(seed_customer_id=i + 1),
                num_failures=2,
            )
        )
```

- [ ] **Step 5: Point the restaurant_v2 docstring at the toggle**

In `restaurant_v2.py`, inside the module docstring's "Fully offline" section (the block just before the closing `"""`), add a sentence:

```
To skip Phase-1 person-scanning entirely and jump straight to the real-audio
order extraction/confirmation (everything-but-audio mocked), set
``order_intake_items.MOCK_SEED_CUSTOMER = True`` -- it seeds a synthetic active
customer per order instead of sweeping for waving persons.
```

- [ ] **Step 6: Run tests to verify all pass**

Run the standard test-run command.
Expected: PASS (`3 passed`).

- [ ] **Step 7: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/Restaurant/order_intake_items.py \
        src/behavior_tree/behavior_tree/Restaurant/restaurant_v2.py \
        src/behavior_tree/test/test_restaurant_2026_mock_seed.py
git commit --author="Ccindy0171 <cindy.w0135@gmail.com>" -m "feat(restaurant): wire MOCK_SEED_CUSTOMER into restaurant-2026 Phase 1

createCollectOneOrderItems swaps the person-scan for the seed subtree when the
toggle is on; createCollectOrdersPhaseItems seeds customers 1 and 2. Production
path (toggle off) unchanged. restaurant_v2 docstring points at the toggle.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: Regression pass and root-install rebuild

**Files:** none modified — verification only.

- [ ] **Step 1: Run the full restaurant + gate suites**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
source /opt/ros/humble/setup.zsh 2>/dev/null; source /home/tinker/tk25_ws/install/setup.zsh 2>/dev/null
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  test/test_restaurant_2026_mock_seed.py \
  test/test_restaurant_state_machine.py \
  test/test_restaurant_simplified.py \
  test/test_restaurant_timeouts.py \
  test/test_restaurant_audio_announcements.py -q
```

Expected: all pass. (Pre-existing unrelated failures in `test_rulebook_structure`, `test_entrypoints_migration`, `test_restaurant_nav_test::test_sweep_positions_are_centre_left_right` stem from separate uncommitted WIP and `TILT_DEG` drift — not from this change. Do not attempt to fix them here.)

- [ ] **Step 2: Verify the toggle-on tree builds offline end-to-end**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree
source /opt/ros/humble/setup.zsh 2>/dev/null; source /home/tinker/tk25_ws/install/setup.zsh 2>/dev/null
BT_MOCK_MODE=true python3 -c "
import behavior_tree.Restaurant.order_intake_items as oii
oii.MOCK_SEED_CUSTOMER = True
from behavior_tree.Restaurant.restaurant_v2 import createRestaurantTask2026
root = createRestaurantTask2026()
names = {n.__class__.__name__ for n in root.iterate()}
assert 'BtNode_ScanForWavingPerson' not in names, 'scan should be bypassed'
assert 'BtNode_TurnPanTilt' not in names, 'pan-tilt should be bypassed'
seeds = [n.name for n in root.iterate() if n.name.startswith('Seed mock customer')]
assert seeds == ['Seed mock customer 1', 'Seed mock customer 2'], seeds
print('restaurant-2026 builds with scan bypassed; seeds:', seeds)
"
```

Expected: prints the confirmation line; no assertion error.

- [ ] **Step 3: Rebuild the root install**

```bash
cd /home/tinker/tk25_ws
./tkbuild tk25_decision --packages-select behavior_tree
```

Expected: `Summary: 1 package finished`. (The tkbuild warning about a stale duplicate install at `src/tk25_decision/install/behavior_tree` is pre-existing and unrelated.)

- [ ] **Step 4: No commit** (verification-only task; nothing changed).

---

## Notes for the implementer

- The seed writes are order-independent among themselves but must all precede `createApproachCustomer`; keep them inside the `if MOCK_SEED_CUSTOMER` branch that replaces the scan `add_child`, before the existing `createApproachCustomer()` add_child.
- `MOCK_SEED_CUSTOMER` is read as a module global inside the factory, so tests monkeypatch `oii.MOCK_SEED_CUSTOMER`; do not capture it into a default argument or a local at import time.
- Do not touch `restaurants.py` (`createScanForUpToNCustomers`) — the shared factory must keep working for the canonical `restaurant` task.
