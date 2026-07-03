# Pick-and-Place Incheon Staged Tree Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A new staged competition tree (`pick-and-place-incheon`) implementing the team's confirmed run plan: enter → inside-door chair request → dining-table scan + announce → grasp mug/cup → wash-staging with a referee (deus-ex-machina) dishwasher door/rack request → fixed-arm-pose rack place → cabinet scan + announce.

**Architecture:** One new module `PickAndPlace/pick_and_place_incheon.py` that composes existing, individually-tested subtrees (`enterArena`, `navigateToTable`, `createTableObjectRecognition`, `createTableGrasp`, `createCabinetCategorization`, `_gotoRetryWith_Announcement`, `_moveArmRetry`) plus two small new factories (chair-removal request, rack place). One new pose constant (`pose_inside_door`, copied from DoingLaundry's `pose_arena_entry`). No existing tree is modified.

**Tech Stack:** ROS2 Humble, `py_trees` / `py_trees_ros`, `rclpy`, `pytest`; `behavior_tree` (ament_python).

## Spec (user-provided, verbatim intent)

> go to inside door position (which doing laundry uses) and announce "please remove the chair between me and the dining table", goto the dining table, lower orbbec pan tilt, scan for all objects (using list of objects) and announce which ones are on the table ("I see ....... on the table"), try grasping the mug/cup, go to washing stage, instruct deus ex machina to open door of washing machine and pull out the rack, place the mug (use move arm single to fixed arm pose then open gripper), then goto cabinet and announce everything you see as well.

## Interpretation decisions (locked in)

1. **"washing machine" = the dishwasher.** The P&P kitchen appliance is a dishwasher; the rulebook (Additional Rules 7–8) explicitly allows requesting help for the dishwasher door/rack with **zero penalty** ("Human assistance: moving dishwasher door or rack: –0"). Announce text says "dishwasher".
2. **"washing stage" = `KEY_POSE_WASH_STAGING`** (the user's literal words; `constants.json:pose_wash_staging` is documented as "Safe surface near the dishwasher"). The factory takes `wash_pose_key=KEY_POSE_WASH_STAGING` as a kwarg — if at the Incheon arena the rack-place stop must be directly at the dishwasher, flip the caller to `KEY_POSE_DISH_WASHER` (one line). **All poses in `constants.json` were captured in the China arena and MUST be re-surveyed at Incheon Setup Days** (use `ros2 run behavior_tree`'s nav-test sweep in `nav_test.py`/`dev_cli.py`).
3. **"inside door position (which doing laundry uses)"** = DoingLaundry's `pose_arena_entry` (`DoingLaundry/constants.json`, value `point (2.879, -0.196, 0.0)`, `quat (0, 0, -0.017086100664103192, 0.9998540219272493)`). The value is **copied** into `PickAndPlace/constants.json` as `pose_inside_door` (PickAndPlace convention: each task dir owns its constants; no cross-task config import).
4. **"scan for all objects (using list of objects)"** = the existing `TABLE_SCAN_PROMPT` (`constants.json:table_scan_prompt`: `fork . knife . spoon . plate . mug . cup . bowl . paper cup . napkin . bottle . can . wrapper . snack bag . pringles . chocolate . ketchup . oats . tuna`) via `createTableObjectRecognition()`, which already does: pan-tilt down (tilt 20°) → arm clear of view → generalist scan → `BtNode_WriteFoundItems(place_seen="on the table")` → announce "I see one X, one Y … on the table." Exactly the requested behavior; reused wholesale.
5. **"try grasping"** = `createTableGrasp(prompt="mug . cup")`, whose built-in failure-cleanup Selector makes it best-effort (grasp failure does not abort the mission — the run continues to the dishwasher/cabinet phases). Announce "I will now pick up the mug or cup." first (rulebook "Communicating Perception": attempting to pick + stating it is sufficient).
6. **"move arm single"** = `BtNode_MoveArmSingle` (via the canonical `_moveArmRetry` helper in `pick_and_place.py`, which wraps exactly that node), to the existing `KEY_ARM_WASH_DROP` fixed joint pose (`arm_pos_wash_drop = [0, 6, 0, 36, 0, -60, 1]` deg), then `BtNode_GripperAction(open_gripper=True)`.
7. **Waits after announcements**: `py_trees.timers.Timer` (same pattern as `Receptionist/receptionist_2ndcall.py:316`). Durations are factory kwargs (`chair_wait_sec=8.0`, `rack_wait_sec=12.0`) so tests pass tiny values and Setup Days can tune.
8. **Rack-place failure cleanup does NOT open the gripper** (mirrors `drop_trash.py`): if the arm never reached the drop pose, opening the gripper would drop the mug mid-air (–40 "objects thrown or dropped"). On failure we keep holding it and reset the arm to nav.
9. **New module + new entry point; nothing existing is modified** except `setup.py` (add entry) and docs. `pick-and-place-2026` (v2), `pick-and-place` (rulebook tree), and `pick-and-place-demo` stay as they are.

## Global Constraints

- **NEVER raw `colcon build`** — use `tkbuild tk25_decision`. Pure-python tests run with `python3 -m pytest` (no `pytest` console script installed) — no build needed for tests.
- Branch: `dev` in `/home/tinker/tk25_ws/src/tk25_decision`. Concurrent committers → **selective `git add <path>` only**, commit new files, NEVER `-A` / `--amend` / rebase.
- Every commit ends with the two trailers:
  ```
  Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_01Rx8tJj9fdP1weQMsEB9Dmx
  ```
- All tests must pass under `BT_MOCK_MODE=true` with **no** robot, servers, camera, or network.
- Announcement text is English (competition language). The two verbatim-required lines:
  - `"Please remove the chair between me and the dining table."`
  - `"Please open the dishwasher door and pull out the rack."`
- Package root for all relative paths below: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/`.

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `behavior_tree/PickAndPlace/constants.json` | Modify (append 1 key) | `pose_inside_door` value (copied from DoingLaundry `pose_arena_entry`) |
| `behavior_tree/PickAndPlace/config.py` | Modify (append 3 lines) | `KEY_POSE_INSIDE_DOOR`, `POSE_INSIDE_DOOR` exports |
| `behavior_tree/PickAndPlace/pick_and_place_incheon.py` | Create | The staged mission tree: helpers + `createPickAndPlaceIncheon()` + `main()` |
| `test/test_pp_incheon_structure.py` | Create | Pure structure tests (config contract + tree shape), no ROS spin |
| `test/test_pp_incheon_mock.py` | Create | Layer-B whole-tree mock smoke: ticks to SUCCESS under full mock |
| `setup.py` | Modify (1 line) | Entry point `pick-and-place-incheon` |
| `behavior_tree/PickAndPlace/RULEBOOK_PLAN.md` | Modify (status note) | Record the new staged tree + its scope |

Frozen interface (used across tasks):
- `createPickAndPlaceIncheon(*, chair_wait_sec: float = 8.0, rack_wait_sec: float = 12.0, wash_pose_key: str = KEY_POSE_WASH_STAGING) -> py_trees.composites.Sequence` — root name `"Pick and Place Incheon"`.
- `createChairRemovalRequest(chair_wait_sec: float) -> py_trees.composites.Sequence` — name `"Chair removal request"`.
- `createRackPlace() -> py_trees.composites.Selector` — name `"Rack place with failure cleanup"`.
- `create_tree() -> py_trees.behaviour.Behaviour` — no-arg alias for smoke harnesses.
- Config additions: `KEY_POSE_INSIDE_DOOR = "pp_pose_inside_door"`, `POSE_INSIDE_DOOR` (PoseStamped via `_pose_reader`).

---

### Task 1: Inside-door pose constant (`constants.json` + `config.py`)

**Files:**
- Modify: `behavior_tree/PickAndPlace/constants.json` (append key after `pose_kitchen_entry`)
- Modify: `behavior_tree/PickAndPlace/config.py` (pose readers ~line 113, key block ~line 173)
- Test: `test/test_pp_incheon_structure.py` (created here with the config-contract test; extended in Task 2)

**Interfaces:**
- Consumes: existing `_pose_reader` in `config.py`.
- Produces: `from behavior_tree.PickAndPlace.config import KEY_POSE_INSIDE_DOOR, POSE_INSIDE_DOOR` — `KEY_POSE_INSIDE_DOOR == "pp_pose_inside_door"`, `POSE_INSIDE_DOOR` is a `PoseStamped` at x≈2.879. Task 2's tree factory imports both.

- [ ] **Step 1: Write the failing test.** Create `test/test_pp_incheon_structure.py`:

```python
"""Structure tests for the Incheon staged pick-and-place tree (no ROS spin)."""
import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 — must precede behavior_tree imports

import pytest  # noqa: E402


def test_config_exposes_inside_door_pose():
    from behavior_tree.PickAndPlace.config import (
        KEY_POSE_INSIDE_DOOR,
        POSE_INSIDE_DOOR,
    )

    assert KEY_POSE_INSIDE_DOOR == "pp_pose_inside_door"
    # Value copied from DoingLaundry pose_arena_entry (see plan §Interpretation 3).
    assert POSE_INSIDE_DOOR.pose.position.x == pytest.approx(2.879)
    assert POSE_INSIDE_DOOR.pose.position.y == pytest.approx(-0.196)
    assert POSE_INSIDE_DOOR.pose.orientation.w == pytest.approx(0.99985, abs=1e-4)
```

- [ ] **Step 2: Run it, expect FAIL** (`ImportError: cannot import name 'KEY_POSE_INSIDE_DOOR'`):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
python3 -m pytest test/test_pp_incheon_structure.py -q
```

- [ ] **Step 3: Append to `behavior_tree/PickAndPlace/constants.json`**, immediately after the `pose_kitchen_entry` object (keep valid JSON — add a comma to the preceding entry):

```json
  "pose_inside_door": {
    "_comment": "Inside-door stop (copied from DoingLaundry pose_arena_entry). Announce chair-removal here. Re-survey at Incheon Setup Days.",
    "point": {"x": 2.879, "y": -0.196, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": -0.017086100664103192, "w": 0.9998540219272493}
  },
```

- [ ] **Step 4: Append to `behavior_tree/PickAndPlace/config.py`.** Next to the other pose readers (after `POSE_DISH_WASHER = _pose_reader(...)`, line ~118):

```python
POSE_INSIDE_DOOR = _pose_reader(constants["pose_inside_door"])
```

and next to the other pose keys (after `KEY_POSE_DISH_WASHER`, line ~178):

```python
KEY_POSE_INSIDE_DOOR = "pp_pose_inside_door"
```

- [ ] **Step 5: Run the test, expect PASS:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
python3 -m pytest test/test_pp_incheon_structure.py -q
```

Expected: `1 passed`.

- [ ] **Step 6: Commit:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/PickAndPlace/constants.json \
        src/behavior_tree/behavior_tree/PickAndPlace/config.py \
        src/behavior_tree/test/test_pp_incheon_structure.py
git commit -m "$(cat <<'EOF'
feat(PickAndPlace): inside-door pose constant for the Incheon staged tree

pose_inside_door copied from DoingLaundry pose_arena_entry; exported as
KEY_POSE_INSIDE_DOOR / POSE_INSIDE_DOOR. Re-survey at Setup Days.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Rx8tJj9fdP1weQMsEB9Dmx
EOF
)"
```

---

### Task 2: The staged mission module `pick_and_place_incheon.py`

**Files:**
- Create: `behavior_tree/PickAndPlace/pick_and_place_incheon.py`
- Modify: `test/test_pp_incheon_structure.py` (append tree-shape tests)

**Interfaces:**
- Consumes (all existing, source-verified):
  - `from .pick_and_place import createConstantWriter, enterArena, navigateToTable, _gotoRetryWith_Announcement, _moveArmRetry` — `_moveArmRetry(name, arm_pose_key, *, add_octomap=False, retries=2)` wraps `BtNode_MoveArmSingle`; `_gotoRetryWith_Announcement(location_name, pose_key)` announces + arm-to-nav + retried goto. (Cross-module import of these `_`-helpers is the established convention — `pick_and_place_v2.py` already does it.)
  - `from .table_object_recognition import createTableObjectRecognition` — pan-tilt 20° down + `TABLE_SCAN_PROMPT` scan + "I see … on the table" announce.
  - `from .table_grasping import createTableGrasp` — best-effort grasp with cleanup Selector.
  - `from .cabinet_categorization import createCabinetCategorization` — per-shelf pan-tilt scan + "I see … on the top/bottom shelf" announce; assumes robot already at the cabinet.
  - `from behavior_tree.TemplateNodes.Audio import BtNode_Announce`; `from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction`; `from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard`; `from behavior_tree.runtime import run_tree`.
  - Config: `KEY_POSE_INSIDE_DOOR`, `POSE_INSIDE_DOOR` (Task 1), `KEY_POSE_WASH_STAGING`, `KEY_POSE_DISH_WASHER`, `KEY_POSE_CABINET`, `KEY_ARM_WASH_DROP`, `KEY_ARM_NAVIGATING`.
- Produces: `createPickAndPlaceIncheon`, `createChairRemovalRequest`, `createRackPlace`, `create_tree`, `main` (exact signatures in the Frozen interface block above). Task 3's mock test and Task 4's entry point rely on these.

- [ ] **Step 1: Append the failing tree-shape tests** to `test/test_pp_incheon_structure.py`:

```python
def _iter_names(root):
    return [b.name for b in root.iterate()]


def test_incheon_tree_builds_and_orders_phases():
    from behavior_tree.PickAndPlace.pick_and_place_incheon import (
        createPickAndPlaceIncheon,
    )

    root = createPickAndPlaceIncheon(chair_wait_sec=0.05, rack_wait_sec=0.05)
    names = _iter_names(root)

    # The stage anchors, in mission order.
    anchors = [
        "Write PickAndPlace constants",        # canonical constant writer
        "Write inside door pose",              # Task-1 pose seeded
        "Enter arena (best effort)",           # door detection (reused)
        "Go to inside door",                   # goto inside-door stop
        "announce chair removal",              # verbatim chair line
        "wait for chair removal",              # Timer
        "Go to table",                         # navigateToTable (reused)
        "Table object recognition",            # scan + announce subtree
        "announce grasp intent",
        "Table grasp (mug . cup)",             # grasp subtree, mug/cup prompt
        "Go to wash staging",                  # goto wash pose
        "request dishwasher open",             # verbatim deus-ex-machina line
        "wait for dishwasher rack",            # Timer
        "Rack place with failure cleanup",     # MoveArmSingle + gripper open
        "Go to cabinet",
        "Cabinet categorization",              # shelf scan + announce subtree
    ]
    idx = -1
    for anchor in anchors:
        assert anchor in names, f"missing node: {anchor}"
        nxt = names.index(anchor)
        assert nxt > idx, f"{anchor} out of order"
        idx = nxt


def test_chair_and_dishwasher_announcement_texts_are_verbatim():
    from behavior_tree.PickAndPlace.pick_and_place_incheon import (
        createPickAndPlaceIncheon,
    )

    root = createPickAndPlaceIncheon(chair_wait_sec=0.05, rack_wait_sec=0.05)
    by_name = {b.name: b for b in root.iterate()}
    assert (
        by_name["announce chair removal"].message
        == "Please remove the chair between me and the dining table."
    )
    assert (
        by_name["request dishwasher open"].message
        == "Please open the dishwasher door and pull out the rack."
    )


def test_rack_place_cleanup_does_not_open_gripper():
    """On rack-place failure we must keep holding the mug (no mid-air drop)."""
    from behavior_tree.PickAndPlace.pick_and_place_incheon import createRackPlace

    root = createRackPlace()
    cleanup = [c for c in root.children if "cleanup" in c.name.lower()
               or "failure" in c.name.lower()][-1]
    cleanup_names = [b.name for b in cleanup.iterate()]
    assert not any("gripper" in n.lower() and "open" in n.lower()
                   for n in cleanup_names), cleanup_names


def test_wash_pose_key_is_switchable():
    from behavior_tree.PickAndPlace.config import KEY_POSE_DISH_WASHER
    from behavior_tree.PickAndPlace.pick_and_place_incheon import (
        createPickAndPlaceIncheon,
    )

    root = createPickAndPlaceIncheon(
        chair_wait_sec=0.05, rack_wait_sec=0.05,
        wash_pose_key=KEY_POSE_DISH_WASHER,
    )
    by_name = {b.name: b for b in root.iterate()}
    goto = by_name["Go to wash staging"]
    # BtNode_GotoAction stores its blackboard key as .key ("goal" registration
    # remaps to the given key); assert via the node's repr of registered keys.
    assert KEY_POSE_DISH_WASHER in repr(goto.blackboard.remappings) or \
        getattr(goto, "key", None) == KEY_POSE_DISH_WASHER
```

Note for the implementer: `BtNode_Announce` stores its fixed message on `self.message` (constructor arg `message=`); `_gotoRetryWith_Announcement` names its goto leaf `f"Go to {location_name}"` — the anchors above rely on the `location_name` values passed in the module below. If an anchor assertion fails, print `names` and match the module's actual node names — the contract is the module code in Step 3; fix the TEST to match it, never rename production nodes to satisfy a guess. The last test's fallback tolerates `BtNode_GotoAction` internals; if both branches fail, inspect `vars(goto)` for where the key landed and assert on that attribute.

- [ ] **Step 2: Run, expect FAIL** (`ModuleNotFoundError: ...pick_and_place_incheon`):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
python3 -m pytest test/test_pp_incheon_structure.py -q
```

- [ ] **Step 3: Create `behavior_tree/PickAndPlace/pick_and_place_incheon.py`:**

```python
from __future__ import annotations

"""Pick and Place — Incheon 2026 staged competition tree.

Entry: ``pick-and-place-incheon``. Implements the team's confirmed run plan:

  1. Enter arena (door detection, best effort) and stop at the INSIDE-DOOR
     position (same spot DoingLaundry uses); ask the referee to remove the
     chair between the robot and the dining table, and wait briefly.
  2. Go to the dining table, lower the orbbec pan-tilt, scan with the full
     official object list (TABLE_SCAN_PROMPT) and announce
     "I see ... on the table" (rulebook: Communicating Perception).
  3. Try grasping the mug/cup (best effort — a failed grasp does not abort).
  4. Go to the wash-staging stop, ask the referee (deus ex machina) to open
     the dishwasher door and pull out the rack — penalty-free assistance per
     rulebook ("Human assistance: moving dishwasher door or rack: -0") —
     wait, then place the mug with a fixed-joint-pose MoveArmSingle
     (KEY_ARM_WASH_DROP) + gripper open. Failure cleanup does NOT open the
     gripper (never drop the mug mid-air); it resets the arm and keeps hold.
  5. Go to the cabinet and announce everything seen on each shelf
     (cabinet_categorization subtree).

All destination poses come from constants.json and were captured in the
CHINA arena — they MUST be re-surveyed at Incheon Setup Days (nav_test.py
sweep). ``wash_pose_key`` selects the rack-place stop: default is the
wash-staging pose; pass KEY_POSE_DISH_WASHER if the robot must stand at the
dishwasher itself.

Run::

    ros2 run behavior_tree pick-and-place-incheon

Fully offline (auto-advance, no servers)::

    BT_MOCK_MODE=true ros2 run behavior_tree pick-and-place-incheon
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import BtNode_GripperAction
from behavior_tree.runtime import run_tree

# Canonical mission helpers, reused unchanged (same convention as v2).
from .pick_and_place import (
    createConstantWriter,
    enterArena,
    navigateToTable,
    _gotoRetryWith_Announcement,
    _moveArmRetry,
)
from .table_object_recognition import createTableObjectRecognition
from .table_grasping import createTableGrasp
from .cabinet_categorization import createCabinetCategorization

from .config import (
    KEY_ARM_NAVIGATING,
    KEY_ARM_WASH_DROP,
    KEY_POSE_CABINET,
    KEY_POSE_INSIDE_DOOR,
    KEY_POSE_WASH_STAGING,
    POSE_INSIDE_DOOR,
)

# Open-vocab prompt for the single table pick of this staged run.
MUG_PROMPT = "mug . cup"

# Referee-action waits (seconds). Tuned at Setup Days; tests pass tiny values.
DEFAULT_CHAIR_WAIT_SEC = 8.0
DEFAULT_RACK_WAIT_SEC = 12.0


def _writeInsideDoorPose() -> BtNode_WriteToBlackboard:
    """Seed the one pose the canonical constant writer does not know about."""
    return BtNode_WriteToBlackboard(
        name="Write inside door pose",
        bb_namespace="",
        bb_source=None,
        bb_key=KEY_POSE_INSIDE_DOOR,
        object=POSE_INSIDE_DOOR,
    )


def createChairRemovalRequest(
    chair_wait_sec: float = DEFAULT_CHAIR_WAIT_SEC,
) -> py_trees.composites.Sequence:
    """At the inside-door stop: ask the referee to clear the chair, then wait.

    NOTE: per the current rulebook this is a scored assistance event
    ("Human assistance: environment changes (per item)", -40 per chair) —
    the team accepts the cost for a reliable table approach.
    """
    seq = py_trees.composites.Sequence(name="Chair removal request", memory=True)
    seq.add_child(
        BtNode_Announce(
            name="announce chair removal",
            bb_source=None,
            message="Please remove the chair between me and the dining table.",
        )
    )
    seq.add_child(
        py_trees.timers.Timer(
            name="wait for chair removal", duration=float(chair_wait_sec)
        )
    )
    return seq


def createRackPlace() -> py_trees.composites.Selector:
    """Place the held mug on the pulled-out rack: fixed-joint-pose MoveArmSingle
    (KEY_ARM_WASH_DROP) then gripper open, then arm back to nav.

    Mirrors drop_trash's shape, WITHOUT its nav step (we are already at the
    wash stop) and WITHOUT a gripper-open in the failure branch: if the arm
    never reached the drop pose, opening the gripper would drop the mug
    mid-air (rulebook -40 "objects thrown or dropped"). Keep holding instead.
    """
    place_seq = py_trees.composites.Sequence(name="Rack place", memory=True)
    place_seq.add_child(
        _moveArmRetry(
            name="move arm to rack drop pose",
            arm_pose_key=KEY_ARM_WASH_DROP,
            retries=3,
        )
    )
    place_seq.add_child(
        BtNode_GripperAction(
            name="release mug on rack", open_gripper=True
        )
    )
    place_seq.add_child(
        _moveArmRetry(
            name="move arm to nav after rack place",
            arm_pose_key=KEY_ARM_NAVIGATING,
            retries=3,
        )
    )

    failure_cleanup = py_trees.composites.Sequence(
        name="Reset arm on rack place failure (keep holding)",
        memory=True,
        children=[
            _moveArmRetry(
                name="move arm to nav on rack place failure",
                arm_pose_key=KEY_ARM_NAVIGATING,
                retries=3,
            ),
        ],
    )

    root = py_trees.composites.Selector(
        name="Rack place with failure cleanup", memory=True
    )
    root.add_child(place_seq)
    root.add_child(failure_cleanup)
    return root


def createPickAndPlaceIncheon(
    *,
    chair_wait_sec: float = DEFAULT_CHAIR_WAIT_SEC,
    rack_wait_sec: float = DEFAULT_RACK_WAIT_SEC,
    wash_pose_key: str = KEY_POSE_WASH_STAGING,
) -> py_trees.composites.Sequence:
    """Full staged Incheon mission (see module docstring for the run plan)."""
    root = py_trees.composites.Sequence(name="Pick and Place Incheon", memory=True)

    # Phase 0 — constants (canonical writer seeds every pose/arm key except
    # the inside-door pose, which we add) + arena entry.
    root.add_child(createConstantWriter())
    root.add_child(_writeInsideDoorPose())
    root.add_child(enterArena())

    # Phase 1 — inside-door stop + chair request.
    root.add_child(
        _gotoRetryWith_Announcement("inside door", KEY_POSE_INSIDE_DOOR)
    )
    root.add_child(createChairRemovalRequest(chair_wait_sec))

    # Phase 2 — table: scan with the official object list and announce
    # "I see ... on the table" (pan-tilt lowering is inside the subtree).
    root.add_child(navigateToTable())
    root.add_child(createTableObjectRecognition())

    # Phase 3 — try grasping the mug/cup (best effort by construction).
    root.add_child(
        BtNode_Announce(
            name="announce grasp intent",
            bb_source=None,
            message="I will now pick up the mug or cup.",
        )
    )
    root.add_child(createTableGrasp(prompt=MUG_PROMPT))

    # Phase 4 — wash stop: deus-ex-machina dishwasher request, wait, place.
    root.add_child(_gotoRetryWith_Announcement("wash staging", wash_pose_key))
    root.add_child(
        BtNode_Announce(
            name="request dishwasher open",
            bb_source=None,
            message="Please open the dishwasher door and pull out the rack.",
        )
    )
    root.add_child(
        py_trees.timers.Timer(
            name="wait for dishwasher rack", duration=float(rack_wait_sec)
        )
    )
    root.add_child(
        BtNode_Announce(
            name="thank referee",
            bb_source=None,
            message="Thank you. Placing the mug now.",
        )
    )
    root.add_child(createRackPlace())

    # Phase 5 — cabinet: announce everything seen, per shelf.
    root.add_child(_gotoRetryWith_Announcement("cabinet", KEY_POSE_CABINET))
    root.add_child(createCabinetCategorization())

    root.add_child(
        BtNode_Announce(
            name="announce mission complete",
            bb_source=None,
            message="Pick and place run complete.",
        )
    )
    return root


def create_tree() -> py_trees.behaviour.Behaviour:
    """No-arg alias for offline smoke harnesses."""
    return createPickAndPlaceIncheon()


def main():
    run_tree(
        create_tree,
        period_ms=300.0,
        title="Pick And Place (Incheon staged)",
        node_name="pick_and_place_incheon",
    )


if __name__ == "__main__":
    main()
```

Implementer notes:
- `_gotoRetryWith_Announcement("inside door", ...)` names its goto leaf `"Go to inside door"` and announces "Going to inside door." — matching the Step-1 anchors.
- `navigateToTable()` produces the `"Go to table"` leaf; `createTableObjectRecognition()`'s root is `"Table object recognition"`; `createTableGrasp(prompt="mug . cup")`'s inner sequence is `"Table grasp (mug . cup)"`; `createCabinetCategorization()`'s root is `"Cabinet categorization"` — all verified against current source. If any anchor mismatches at test time, adjust the TEST anchor to the actual name (production names are the source of truth).
- `run_tree` signature verified: `run_tree(root_factory, *, period_ms, title, node_name="root_node")`.

- [ ] **Step 4: Run the structure tests, expect PASS:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
python3 -m pytest test/test_pp_incheon_structure.py -q
```

Expected: `5 passed` (1 from Task 1 + 4 new).

- [ ] **Step 5: Commit:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/PickAndPlace/pick_and_place_incheon.py \
        src/behavior_tree/test/test_pp_incheon_structure.py
git commit -m "$(cat <<'EOF'
feat(PickAndPlace): Incheon staged mission tree

Enter -> inside-door chair request -> table scan+announce -> mug/cup
grasp (best effort) -> wash stop with penalty-free dishwasher door/rack
referee request -> fixed-pose MoveArmSingle rack place -> cabinet
scan+announce. Composes existing subtrees; nothing existing modified.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Rx8tJj9fdP1weQMsEB9Dmx
EOF
)"
```

---

### Task 3: Layer-B whole-tree mock smoke test

**Files:**
- Create: `test/test_pp_incheon_mock.py`

**Interfaces:**
- Consumes: `createPickAndPlaceIncheon` (Task 2). Mock-forcing pattern copied from `test/test_rulebook_tree_mock.py` (`_force_full_mock` — do not import across test modules; the helper is replicated here so this file stands alone).
- Produces: proof the tree `setup()`s and ticks to `SUCCESS` with zero servers.

- [ ] **Step 1: Write the failing test.** Create `test/test_pp_incheon_mock.py`:

```python
"""Layer-B whole-tree mock smoke for createPickAndPlaceIncheon().

Runs the real tree end-to-end under BT_MOCK_MODE=true with every subsystem
mocked and keyboard control off (KEYPRESS -> IMMEDIATE), so tree.setup()
contacts NO real service and every Handler auto-succeeds. Timers use tiny
durations. A tick cap asserts FAIL (not hang) if the tree never terminates.

The mock-forcing helper is a standalone copy of the one in
test_rulebook_tree_mock.py (kept local so this file has no cross-test import).
"""

import json
import os
import tempfile

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 must precede config import

import py_trees  # noqa: E402
import py_trees_ros  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402

import behavior_tree.config as btcfg  # noqa: E402

TICK_CAP = 2000

_ALL_SUBSYSTEMS = (
    "vision",
    "manipulation",
    "navigation",
    "audio_input",
    "announcement",
    "mock_controls",
)


def _all_mock_config():
    return {
        "force_mock_nodes": {},
        "mock_mode": {
            "enabled": True,
            "auto_detect": True,
            "subsystems": {s: {"enabled": True, "nodes": {}} for s in _ALL_SUBSYSTEMS},
        },
        "keyboard_control": {"enabled": False},
        "mock_keyboard": {"subsystem_start_keys": {}, "success_key": "ENTER"},
        "teleop": {"params": {}},
        "logging": {"print_mock_operations": False, "use_emoji": False},
    }


@pytest.fixture(scope="module")
def full_mock_env():
    fd, path = tempfile.mkstemp(prefix="pp_incheon_mock_", suffix=".json")
    with os.fdopen(fd, "w") as f:
        json.dump(_all_mock_config(), f)
    saved_cfg = os.environ.get("BT_MOCK_CONFIG")
    os.environ["BT_MOCK_CONFIG"] = path
    btcfg._config._load_mock_config(force=True)
    mc = btcfg._config._mock_config
    mc.setdefault("mock_mode", {})["enabled"] = True
    for sub in mc["mock_mode"].setdefault("subsystems", {}).values():
        sub["enabled"] = True
    mc.setdefault("keyboard_control", {})["enabled"] = False

    rclpy.init()
    yield
    rclpy.try_shutdown()

    if saved_cfg is None:
        os.environ.pop("BT_MOCK_CONFIG", None)
    else:
        os.environ["BT_MOCK_CONFIG"] = saved_cfg
    if os.path.exists(path):
        os.remove(path)
    try:
        btcfg._config._load_mock_config(force=True)
    except Exception:
        pass


def test_incheon_tree_ticks_to_success_in_full_mock(full_mock_env):
    from behavior_tree.PickAndPlace.pick_and_place_incheon import (
        createPickAndPlaceIncheon,
    )

    root = createPickAndPlaceIncheon(chair_wait_sec=0.05, rack_wait_sec=0.05)
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name="pp_incheon_mock_test", timeout=15)
    ticks = 0
    try:
        while ticks < TICK_CAP:
            tree.tick()
            ticks += 1
            if root.status != py_trees.common.Status.RUNNING:
                break
    finally:
        try:
            tree.shutdown()
        except Exception:
            pass

    assert ticks < TICK_CAP, f"tree did not terminate within {TICK_CAP} ticks"
    assert root.status == py_trees.common.Status.SUCCESS, root.status
```

- [ ] **Step 2: Run it — it should PASS immediately if Task 2 was correct.** The failing-first state was covered by Task 2 Step 2; here the value is catching setup()-blocking or non-terminating composition bugs:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
python3 -m pytest test/test_pp_incheon_mock.py -q
```

Expected: `1 passed` in well under a minute. If it hangs in `setup()`, a used node class is registered under a subsystem that the forced config failed to mock — inspect `btcfg._config._mock_config` in a debugger and compare with `test_rulebook_tree_mock.py`'s belt-and-suspenders block. If it hits TICK_CAP with the root still RUNNING at a Timer node, the Timer durations weren't threaded through the kwargs.

- [ ] **Step 3: Run the FULL behavior-tree test suite to check for cross-test config leakage:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
python3 -m pytest test/ -q 2>&1 | tail -5
```

Expected: same pass/fail set as before this plan (pre-existing failures, if any, unchanged; no NEW failures).

- [ ] **Step 4: Commit:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/test/test_pp_incheon_mock.py
git commit -m "$(cat <<'EOF'
test(PickAndPlace): whole-tree mock smoke for the Incheon staged tree

Forces all subsystems mocked (private temp config, keyboard off), ticks
createPickAndPlaceIncheon() to SUCCESS under a tick cap with no servers.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Rx8tJj9fdP1weQMsEB9Dmx
EOF
)"
```

---

### Task 4: Entry point, build, run-book, docs

**Files:**
- Modify: `setup.py` (console_scripts, next to the other `pick-and-place*` entries at lines ~79-84)
- Modify: `behavior_tree/PickAndPlace/RULEBOOK_PLAN.md` (append status)

**Interfaces:**
- Consumes: `behavior_tree.PickAndPlace.pick_and_place_incheon:main` (Task 2).
- Produces: `ros2 run behavior_tree pick-and-place-incheon`.

- [ ] **Step 1: Add the entry point.** In `setup.py`, after the line `"pick-and-place-demo = behavior_tree.PickAndPlace.cli_demo:main",`:

```python
            "pick-and-place-incheon = behavior_tree.PickAndPlace.pick_and_place_incheon:main",
```

- [ ] **Step 2: Verify the entry-point target resolves without a build** (pure import check):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
BT_MOCK_MODE=true python3 -c \
"from behavior_tree.PickAndPlace.pick_and_place_incheon import main; print('entry OK')"
```

Expected: `entry OK`.

- [ ] **Step 3: Build (user-approved; NEVER raw colcon) and smoke via ros2 run:**

```bash
cd /home/tinker/tk25_ws && tkbuild tk25_decision --packages-select behavior_tree
source /home/tinker/tk25_ws/install/setup.zsh
BT_MOCK_MODE=true timeout 90 ros2 run behavior_tree pick-and-place-incheon
```

Expected: the tree visualizer shows all phases advancing to SUCCESS ("Pick and Place Incheon" root goes green; final announce "Pick and place run complete."), then idles; Ctrl+C / timeout exits cleanly. (In mock, announcements print instead of speaking if `announcement` is mocked; with the on-disk `mock_config.json` announcement=real-TTS setting, the robot workstation will actually speak — both are fine for the smoke.)

- [ ] **Step 4: Append to `behavior_tree/PickAndPlace/RULEBOOK_PLAN.md`:**

```markdown
## Status (2026-07-02) — Incheon staged tree

`pick_and_place_incheon.py` (`ros2 run behavior_tree pick-and-place-incheon`) is the
staged competition run for Incheon 2026: enter → inside-door chair-removal request
(accepted -40 assistance) → table scan + "I see ... on the table" → mug/cup grasp
(best effort) → wash stop + penalty-free dishwasher door/rack referee request →
fixed-pose rack place (MoveArmSingle KEY_ARM_WASH_DROP + gripper open; failure
branch keeps holding) → cabinet per-shelf scan + announce.
Composes existing subtrees; `pick-and-place-2026` (v2) and the older trees are
untouched. ALL constants.json poses are China-arena captures — re-survey at
Setup Days (nav_test sweep); `wash_pose_key=KEY_POSE_DISH_WASHER` if the rack
place must happen at the dishwasher itself.
```

- [ ] **Step 5: Commit:**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/setup.py \
        src/behavior_tree/behavior_tree/PickAndPlace/RULEBOOK_PLAN.md
git commit -m "$(cat <<'EOF'
feat(PickAndPlace): register pick-and-place-incheon entry point + status doc

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_01Rx8tJj9fdP1weQMsEB9Dmx
EOF
)"
```

---

## On-robot bring-up checklist (post-implementation, operator-driven — NOT part of this plan's execution)

1. Re-capture every pose at the Incheon arena (`/amcl_pose` while driving): `pose_inside_door`, `pose_table_1`, `pose_wash_staging` (or decide to stand at `pose_dish_washer` and flip `wash_pose_key`), `pose_cabinet`. Verify with the existing nav sweep (`dev_cli.py` nav-test entries).
2. Verify `arm_pos_wash_drop` clears the pulled-out rack height with a held mug (teleop first, then `BtNode_MoveArmSingle` dry move without an object).
3. Tune `DEFAULT_CHAIR_WAIT_SEC` / `DEFAULT_RACK_WAIT_SEC` to real referee walking time.
4. Confirm `DEFAULT_CABINET_LAYERS` pan/tilt in `cabinet_categorization.py` actually frame the real cabinet shelves.
5. Confirm the trash category announced at Setup Days (map note says "Fruits will be the trash for this task" — irrelevant to this staged tree since it only picks the mug, but keep prompts consistent if the tree grows).

## Self-review notes

- Spec coverage: every clause of the user's spec maps to a Phase in `createPickAndPlaceIncheon` (chair line → Phase 1; table scan/announce with object list → Phase 2 via `createTableObjectRecognition`/`TABLE_SCAN_PROMPT`; mug grasp → Phase 3; wash stop + deus-ex-machina + MoveArmSingle place → Phase 4; cabinet announce → Phase 5).
- Names/types cross-checked against current source: `run_tree(root_factory, *, period_ms, title, node_name)`; `_moveArmRetry(name, arm_pose_key, *, add_octomap, retries)` wraps `BtNode_MoveArmSingle`; `BtNode_Announce(name, bb_source, message=...)`; `BtNode_WriteToBlackboard(name, bb_namespace, bb_source, bb_key, object)`; `py_trees.timers.Timer(name, duration)`; mock registration confirmed for every Handler used (`BtNode_ScanForGeneralist`, `BtNode_DoorDetection`, `BtNode_TurnPanTilt` → vision; `BtNode_Grasp`, `BtNode_MoveArmSingle`, `BtNode_GripperAction`, `BtNode_JointMoveAction` → manipulation; `BtNode_GotoAction` → navigation; `BtNode_Announce` → announcement).
- Known accepted trade-offs recorded inline: chair request costs −40 assistance; grasp is best-effort (may reach the rack empty-handed — the announce+place still runs; acceptable for the staged run); rack-place failure keeps holding the mug.
