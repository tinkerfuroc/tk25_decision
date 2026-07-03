# HRI Turn-Around Pose + Host/Guest Scripting Updates Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Three modifications to the production HRI task (`hri-2026`): (1) after the bag handover and right before the follow announcements, drive to a new pose that is the sofa waypoint with heading flipped 180°; (2) at task start, ask the host to sit on the sofa and not move around the room; (3) extend the "make yourself comfortable" prompt to the seated guests with "please look at my head camera".

**Architecture:** The production entry point is `ros2 run behavior_tree hri-2026` → `hri_2026.py:createHRITask2026()`, which reuses the phase factories in `hri.py` and swaps in the real bag flow `createBagFlowReal2026()`. The reversed pose is *computed* in `HRI/config.py` from `POSE_SOFA` (not duplicated in `constants.json`), so re-teaching `pose_sofa` automatically keeps the pair consistent. It is seeded onto the blackboard by the shared `createConstantWriter()` and consumed by a new best-effort `BtNode_GotoAction` in the bag flow. Modification 2 is a new `BtNode_Announce` leaf in `createHRITask2026` only (the canonical `createHRITask` is a frozen reference tree per the `hri_2026.py` docstring). Modification 3 edits the one live "make yourself comfortable" announcement, which is in the shared `hri.py:createTwoWayIntroduction()` — it addresses both seated guests and runs immediately before the seated-guest `BtNode_FeatureMatching` scan (which is exactly why looking at the head camera helps).

**Tech Stack:** Python 3, `py_trees` behavior trees, pytest. No ROS runtime needed for any test (mock mode + construction-only tree-shape tests, the established pattern in `test/test_hri_2026_start_gate.py` and `test/test_bar_return_projection.py`).

## Global Constraints

- All work happens in `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/`. Run all pytest commands from that directory.
- Every new test file MUST start with `os.environ.setdefault("BT_MOCK_MODE", "true")` **before** any `behavior_tree` import (nodes check mock mode at construction).
- Every new test file MUST carry the Apache-2.0 license header (the `test/test_copyright.py` gate) and a module docstring (the `test/test_pep257.py` gate), and pass flake8 (the `test/test_flake8.py` gate).
- `BtNode_GotoAction` does NOT retain its blackboard key as a plain attribute — it remaps the client key `"goal"`. Verified on this machine: `goto.blackboard.remappings == {"/goal": "/<key>"}`. Tests use that, plus node names, to pin wiring (same approach as `test/test_bar_return_projection.py`).
- Announcement text lives in `BtNode_Announce(message=...)` and is exposed as the `given_msg` attribute (see `TemplateNodes/Audio.py:231`).
- **Dirty working tree caution:** `HRI/hri.py` (~96 changed lines), `HRI/hri_2026.py` (~27), and `HRI/config.py` (~2) already carry *uncommitted* modifications from concurrent sessions. Before each commit, run `git diff <file>` — unrelated hunks in a file you must commit get acknowledged in the commit body with an `HONESTY NOTE:` paragraph (repo precedent: CHANGELOG 2.2.16 / commit `3d1c431`). Do NOT stash, revert, or checkout these files.
- Commit messages follow the repo's conventional-commit style (`feat(HRI): ...`, `test(...)`, `docs(behavior_tree): ...`).
- Do not touch `behavior_tree/HRI/HRI/` (a stale nested duplicate directory) or `hriwithfollow.py` (retired variant with its own copy of the messages).

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `behavior_tree/HRI/config.py` | Modify | `_flip_pose_180()` helper, `POSE_SOFA_REVERSED`, `KEY_SOFA_POSE_REVERSED` |
| `behavior_tree/HRI/hri.py` | Modify | `createConstantWriter()` seeds the reversed pose; `createTwoWayIntroduction()` message gains the head-camera line |
| `behavior_tree/HRI/hri_2026.py` | Modify | Turn-around Goto in `createBagFlowReal2026()`; host seating announcement in `createHRITask2026()` |
| `test/test_hri_sofa_reversed_pose.py` | Create | Pins the reversed-pose math (point unchanged, yaw exactly +180°) |
| `test/test_hri_turnaround_goto.py` | Create | Pins constant-writer seeding + Goto placement/key in the bag flow |
| `test/test_hri_2026_start_gate.py` | Modify | Child count 15→16; pins the host seating announcement |
| `test/test_hri_intro_camera_prompt.py` | Create | Pins the head-camera line in the two-way-intro prompt |
| `CHANGELOG.md` | Modify | 2.2.17 entry |

---

### Task 1: Reversed sofa pose in `HRI/config.py`

**Files:**
- Modify: `behavior_tree/HRI/config.py` (helper after `_arm_pose_reader` ~line 97; constants after `POSE_SOFA` at line 103 and after `KEY_SOFA_POSE` at line 125)
- Test: `test/test_hri_sofa_reversed_pose.py` (create)

**Interfaces:**
- Consumes: existing `POSE_SOFA` (`PoseStamped`), the module's `PoseStamped/Pose/Point/Quaternion/Header` classes (real geometry_msgs or the module's non-ROS fallbacks — the helper only reads attributes, so it works with both).
- Produces: `POSE_SOFA_REVERSED: PoseStamped` and `KEY_SOFA_POSE_REVERSED = "hri_sofa_pose_reversed"`, imported by Task 2.

- [ ] **Step 1: Write the failing test**

Create `test/test_hri_sofa_reversed_pose.py`:

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

"""POSE_SOFA_REVERSED: same map point as POSE_SOFA, yaw rotated exactly 180 deg.

The reversed pose is computed from POSE_SOFA at module load (not stored in
constants.json), so re-teaching pose_sofa keeps the pair consistent. Used by
the hri-2026 bag flow to turn the robot around in place before the follow.
"""

import math
import os

os.environ.setdefault("BT_MOCK_MODE", "true")

from behavior_tree.HRI.config import (  # noqa: E402
    KEY_SOFA_POSE,
    KEY_SOFA_POSE_REVERSED,
    POSE_SOFA,
    POSE_SOFA_REVERSED,
)


def _yaw(quaternion):
    """Yaw of a planar (x=y=0) quaternion."""
    return 2.0 * math.atan2(quaternion.z, quaternion.w)


def test_reversed_key_is_distinct():
    assert KEY_SOFA_POSE_REVERSED == "hri_sofa_pose_reversed"
    assert KEY_SOFA_POSE_REVERSED != KEY_SOFA_POSE


def test_point_unchanged():
    original = POSE_SOFA.pose.position
    flipped = POSE_SOFA_REVERSED.pose.position
    assert (flipped.x, flipped.y, flipped.z) == (original.x, original.y, original.z)


def test_frame_unchanged():
    assert POSE_SOFA_REVERSED.header.frame_id == POSE_SOFA.header.frame_id == "map"


def test_yaw_flipped_exactly_180_degrees():
    delta = math.remainder(
        _yaw(POSE_SOFA_REVERSED.pose.orientation)
        - _yaw(POSE_SOFA.pose.orientation),
        2.0 * math.pi,
    )
    assert abs(abs(delta) - math.pi) < 1e-9


def test_orientation_stays_unit_quaternion():
    q = POSE_SOFA_REVERSED.pose.orientation
    norm = math.sqrt(q.x ** 2 + q.y ** 2 + q.z ** 2 + q.w ** 2)
    assert abs(norm - 1.0) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_sofa_reversed_pose.py -v`
Expected: collection error — `ImportError: cannot import name 'KEY_SOFA_POSE_REVERSED' from 'behavior_tree.HRI.config'`

- [ ] **Step 3: Implement in `behavior_tree/HRI/config.py`**

Insert the helper directly after the `_arm_pose_reader` function (after line 97, before `constants = _load_constants()`):

```python
def _flip_pose_180(pose_stamped):
    """Same point as ``pose_stamped``, orientation rotated 180 deg about z.

    Quaternion body-z half-turn: (x, y, z, w) -> (y, -x, w, -z); for the
    planar (x=y=0) poses used here that is simply yaw + pi. Used by the
    bag flow to turn the robot around in place at the sofa waypoint so the
    host can stand in front of it before the follow starts.
    """
    q = pose_stamped.pose.orientation
    p = pose_stamped.pose.position
    return PoseStamped(
        header=Header(
            stamp=pose_stamped.header.stamp,
            frame_id=pose_stamped.header.frame_id,
        ),
        pose=Pose(
            position=Point(x=p.x, y=p.y, z=p.z),
            orientation=Quaternion(x=q.y, y=-q.x, z=q.w, w=-q.z),
        ),
    )
```

Directly after the existing `POSE_SOFA = _pose_reader(constants["pose_sofa"])` line (line 103), add:

```python
POSE_SOFA_REVERSED = _flip_pose_180(POSE_SOFA)
```

Directly after the existing `KEY_SOFA_POSE = "hri_sofa_pose"` line (line 125), add:

```python
KEY_SOFA_POSE_REVERSED = "hri_sofa_pose_reversed"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_sofa_reversed_pose.py -v`
Expected: 5 passed

- [ ] **Step 5: Commit**

Check `git diff src/behavior_tree/behavior_tree/HRI/config.py` first — config.py carries ~2 unrelated uncommitted lines; if they are still present, mention them in an `HONESTY NOTE:` paragraph in the commit body.

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/HRI/config.py src/behavior_tree/test/test_hri_sofa_reversed_pose.py
git commit -m "feat(HRI): add POSE_SOFA_REVERSED 180-degree turn-around pose to config"
```

---

### Task 2: Seed reversed pose + turn-around Goto in the 2026 bag flow

**Files:**
- Modify: `behavior_tree/HRI/hri.py` (`.config` import block at lines 50–91; `createConstantWriter()` at line 156)
- Modify: `behavior_tree/HRI/hri_2026.py` (imports at lines 40–46; `createBagFlowReal2026()` between the arm-to-nav block ending at line 108 and the `Look at host` leaf at line 111)
- Test: `test/test_hri_turnaround_goto.py` (create)

**Interfaces:**
- Consumes: `POSE_SOFA_REVERSED` / `KEY_SOFA_POSE_REVERSED` from Task 1; existing `BtNode_WriteToBlackboard(name, bb_namespace, bb_key, bb_source, object)` and `BtNode_GotoAction(name, key)`.
- Produces: blackboard key `hri_sofa_pose_reversed` holding a `PoseStamped`; a direct child of the bag-flow Sequence named `"Turn around at sofa (best effort)"` (a `FailureIsSuccess` wrapping `Retry` wrapping `BtNode_GotoAction` named `"Turn around at sofa"`), placed immediately before the `"Look at host"` leaf.

- [ ] **Step 1: Write the failing test**

Create `test/test_hri_turnaround_goto.py`:

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

"""Bag-flow turn-around: after handover, before the follow announcements.

The hri-2026 bag flow must drive to the reversed sofa pose (same point,
heading flipped 180 deg) after the bag is grasped and the arm is back at
the nav pose, and before the ``Look at host`` + follow announcements block.
``BtNode_GotoAction`` does not retain its key as an attribute, so the key is
pinned via the ``goal`` blackboard remap (see test_bar_return_projection.py).
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import behavior_tree.HRI.hri as hri  # noqa: E402
from behavior_tree.HRI.config import (  # noqa: E402
    KEY_SOFA_POSE_REVERSED,
    POSE_SOFA_REVERSED,
)
from behavior_tree.HRI.hri_2026 import createBagFlowReal2026  # noqa: E402


def test_constant_writer_seeds_reversed_sofa_pose():
    writer = hri.createConstantWriter()
    writes = [
        b
        for b in writer.iterate()
        if getattr(b, "bb_key", None) == KEY_SOFA_POSE_REVERSED
    ]
    assert len(writes) == 1, "exactly one writer for the reversed sofa pose"
    assert writes[0].object is POSE_SOFA_REVERSED


def test_turnaround_sits_between_handover_and_follow_announcements():
    root = createBagFlowReal2026()
    names = [child.name for child in root.children]
    turn_idx = names.index("Turn around at sofa (best effort)")
    close_idx = names.index("Close gripper with bag")
    look_idx = names.index("Look at host")
    assert close_idx < turn_idx < look_idx
    assert turn_idx == look_idx - 1, (
        "turn-around must be the last step before the follow "
        "announcements block (Look at host + host announcements)"
    )


def test_turnaround_goto_targets_reversed_sofa_key():
    root = createBagFlowReal2026()
    gotos = [
        b for b in root.iterate() if b.__class__.__name__ == "BtNode_GotoAction"
    ]
    assert len(gotos) == 1, "the turn-around is the bag flow's only base nav"
    assert gotos[0].name == "Turn around at sofa"
    assert gotos[0].blackboard.remappings == {
        "/goal": "/" + KEY_SOFA_POSE_REVERSED
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_turnaround_goto.py -v`
Expected: 3 failed — `test_constant_writer_seeds_reversed_sofa_pose` with `assert 0 == 1`, the other two with `ValueError: 'Turn around at sofa (best effort)' is not in list` / `assert 0 == 1`

- [ ] **Step 3: Seed the pose in `behavior_tree/HRI/hri.py`**

In the `from .config import (...)` block (lines 50–91), add two names, keeping alphabetical order — `KEY_SOFA_POSE_REVERSED` right after the existing `KEY_SOFA_POSE,` (line 86) and `POSE_SOFA_REVERSED,` right after the existing `POSE_SOFA,` (line 89):

```python
    KEY_SOFA_POSE,
    KEY_SOFA_POSE_REVERSED,
    NAMES,
    POSE_DOOR,
    POSE_SOFA,
    POSE_SOFA_REVERSED,
    SEAT_CATALOG,
```

In `createConstantWriter()`, directly after the `"Write sofa pose"` child (the `root.add_child(...)` ending at line 179), add:

```python
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write reversed sofa pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_SOFA_POSE_REVERSED,
            object=POSE_SOFA_REVERSED,
        )
    )
```

- [ ] **Step 4: Add the turn-around Goto in `behavior_tree/HRI/hri_2026.py`**

Extend the config import (line 40) and add the Navigation import after it:

```python
from behavior_tree.HRI.config import (
    KEY_ARM_DROP,
    KEY_ARM_NAVIGATING,
    KEY_SOFA_POSE_REVERSED,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_GotoAction
```

In `createBagFlowReal2026()`, between the `arm to nav pose with bag (best effort)` block (`root.add_child(...)` ending at line 108) and the `# --- real follow host until the host signals to stop ---` comment (line 110), insert:

```python
    # Turn 180 deg in place: same map point as the sofa waypoint, heading
    # flipped (POSE_SOFA_REVERSED), so the robot faces away from the sofa
    # and the host can stand in front of it for the follow. Best-effort:
    # a nav refusal must not forfeit the follow-to-drop (200) scoring.
    root.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="Turn around at sofa (best effort)",
            child=py_trees.decorators.Retry(
                name="Retry turn around at sofa",
                child=BtNode_GotoAction(
                    name="Turn around at sofa",
                    key=KEY_SOFA_POSE_REVERSED,
                ),
                num_failures=3,
            ),
        )
    )
```

- [ ] **Step 5: Run tests to verify they pass (plus no regression in the start-gate suite)**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_turnaround_goto.py test/test_hri_2026_start_gate.py test/test_hri_host_trim_wiring.py -v`
Expected: all passed (the start-gate count test still sees 15 root children — Task 3 changes that, not this task)

- [ ] **Step 6: Commit**

Check `git diff` on both modified sources first; acknowledge unrelated pre-existing hunks with an `HONESTY NOTE:` paragraph if present (hri.py carries ~96 uncommitted lines from concurrent sessions).

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/HRI/hri.py src/behavior_tree/behavior_tree/HRI/hri_2026.py src/behavior_tree/test/test_hri_turnaround_goto.py
git commit -m "feat(HRI): drive to reversed sofa pose after bag handover, before follow announcements"
```

---

### Task 3: Host seating instruction at task start

**Files:**
- Modify: `behavior_tree/HRI/hri_2026.py` (`createHRITask2026()`, after the `HRI start announcement` child ending at line 158)
- Modify: `test/test_hri_2026_start_gate.py` (count 15→16 at line 13; new test)

**Interfaces:**
- Consumes: existing `BtNode_Announce(name, bb_source, message)` (already imported in `hri_2026.py`).
- Produces: root child at index 3 of `createHRITask2026()` — a `BtNode_Announce` with `given_msg == "Dear host, please sit down on the sofa and remain seated. Please do not walk around the room during the task."`. All later children shift by one (root count 15→16).

- [ ] **Step 1: Update the start-gate test (fails first)**

In `test/test_hri_2026_start_gate.py`, change the count assertion in `test_root_starts_with_operator_gate` (line 13):

```python
    assert len(root.children) == 16
```

And append this test at the end of the file:

```python
def test_host_seating_instruction_is_fourth():
    root = createHRITask2026()
    announce = root.children[3]
    assert announce.__class__.__name__ == "BtNode_Announce"
    assert announce.given_msg == (
        "Dear host, please sit down on the sofa and remain seated. "
        "Please do not walk around the room during the task."
    )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_2026_start_gate.py -v`
Expected: `test_root_starts_with_operator_gate` FAILS (`assert 15 == 16`) and `test_host_seating_instruction_is_fourth` FAILS (child 3 is the arm-nav Retry, not an Announce); the other tests pass

- [ ] **Step 3: Implement in `behavior_tree/HRI/hri_2026.py`**

In `createHRITask2026()`, directly after the `HRI start announcement` child (the `root.add_child(...)` ending at line 158) and before the `Retry arm nav pose at start` child, insert:

```python
    root.add_child(
        BtNode_Announce(
            name="Host seating instruction",
            bb_source=None,
            message=(
                "Dear host, please sit down on the sofa and remain seated. "
                "Please do not walk around the room during the task."
            ),
        )
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_2026_start_gate.py -v`
Expected: all passed (including the two updated/new tests)

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/HRI/hri_2026.py src/behavior_tree/test/test_hri_2026_start_gate.py
git commit -m "feat(HRI): ask host to sit on sofa and stay put at task start"
```

---

### Task 4: Guests' comfort prompt asks for a look at the head camera

**Files:**
- Modify: `behavior_tree/HRI/hri.py` (the `Complete escort announcement` message at line 959 in `createTwoWayIntroduction()`)
- Test: `test/test_hri_intro_camera_prompt.py` (create)

**Interfaces:**
- Consumes: existing `hri.createTwoWayIntroduction()`; the announce leaf named `"Complete escort announcement"`.
- Produces: that leaf's `given_msg` becomes `"Please sit down and make yourself comfortable. Please look at my head camera and remain seated."` This is the single live "make yourself comfortable" prompt; it addresses both seated guests and runs right before the seated-guest `BtNode_FeatureMatching` scan, which the camera-look request directly helps. (The shared factory means the canonical `hri` entry inherits the same wording — intended.)

- [ ] **Step 1: Write the failing test**

Create `test/test_hri_intro_camera_prompt.py`:

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

"""The comfort prompt to both seated guests asks for a head-camera look.

The prompt opens createTwoWayIntroduction() and runs immediately before the
seated-guest BtNode_FeatureMatching scan; guests looking at the head camera
is what makes that scan reliable.
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import behavior_tree.HRI.hri as hri  # noqa: E402


def test_comfort_prompt_includes_head_camera_request():
    root = hri.createTwoWayIntroduction()
    announces = [
        b for b in root.iterate() if b.name == "Complete escort announcement"
    ]
    assert len(announces) == 1
    assert announces[0].given_msg == (
        "Please sit down and make yourself comfortable. "
        "Please look at my head camera and remain seated."
    )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_intro_camera_prompt.py -v`
Expected: 1 failed — `given_msg` is still `"Please sit down and make yourself comfortable. Remain seated please."`

- [ ] **Step 3: Implement in `behavior_tree/HRI/hri.py`**

In `createTwoWayIntroduction()` (line 956–960), change the message of the `Complete escort announcement` leaf:

```python
    root.add_child(BtNode_Announce(
        name=f"Complete escort announcement",
        bb_source=None,
        message=(
            "Please sit down and make yourself comfortable. "
            "Please look at my head camera and remain seated."
        )
    ))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest test/test_hri_intro_camera_prompt.py -v`
Expected: 1 passed

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/behavior_tree/HRI/hri.py src/behavior_tree/test/test_hri_intro_camera_prompt.py
git commit -m "feat(HRI): ask seated guests to look at head camera before intro scan"
```

---

### Task 5: Changelog + full verification

**Files:**
- Modify: `CHANGELOG.md` (new 2.2.17 entry at the top, above `## [2.2.16]`)

**Interfaces:**
- Consumes: everything from Tasks 1–4.
- Produces: documented, lint-clean change-set.

- [ ] **Step 1: Add the CHANGELOG entry**

Insert at the top of `CHANGELOG.md` (directly under the `# Changelog` heading, above `## [2.2.16]`):

```markdown
## [2.2.17] - 2026-07-03

### 🛋️ HRI: turn-around before follow + host/guest scripting updates

- `HRI/config.py`: new `POSE_SOFA_REVERSED` / `KEY_SOFA_POSE_REVERSED` —
  same map point as `pose_sofa`, yaw rotated exactly 180°. Computed from
  `POSE_SOFA` at load time (not stored in `constants.json`), so re-teaching
  the sofa waypoint keeps the pair consistent. Seeded onto the blackboard
  by `createConstantWriter`.
- `hri_2026.py:createBagFlowReal2026`: after the bag handover (arm back at
  nav pose) and right before the follow announcements, the robot now drives
  to the reversed sofa pose — an in-place 180° turn so it faces away from
  the sofa and the host can step in front of it for the follow.
  Best-effort (`FailureIsSuccess(Retry×3)`): a nav refusal must not forfeit
  the follow-to-drop scoring.
- `hri_2026.py:createHRITask2026`: new start-of-task announcement asking
  the host to sit on the sofa and not walk around the room (root children
  15→16; start-gate test updated).
- `hri.py:createTwoWayIntroduction`: the "make yourself comfortable" prompt
  to both seated guests now also asks them to look at the head camera —
  spoken immediately before the seated-guest `BtNode_FeatureMatching` scan
  it helps.
- New tests: `test_hri_sofa_reversed_pose.py` (5),
  `test_hri_turnaround_goto.py` (3), `test_hri_intro_camera_prompt.py` (1);
  `test_hri_2026_start_gate.py` updated (16 children + host instruction).
```

- [ ] **Step 2: Run the full verification suite (new + neighboring + lint gates)**

Run:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python3 -m pytest \
  test/test_hri_sofa_reversed_pose.py \
  test/test_hri_turnaround_goto.py \
  test/test_hri_2026_start_gate.py \
  test/test_hri_intro_camera_prompt.py \
  test/test_hri_host_trim_wiring.py \
  test/test_hri_behavior.py \
  test/test_copyright.py test/test_flake8.py test/test_pep257.py -v
```

Expected: all passed. If a lint gate fails on files this plan did NOT touch, that is a pre-existing failure — report it, don't fix it here.

- [ ] **Step 3: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git add src/behavior_tree/CHANGELOG.md
git commit -m "docs(behavior_tree): changelog 2.2.17 for HRI turn-around + scripting updates"
```

---

## Design decisions locked in (for reviewers)

1. **Reversed pose is derived, not configured.** Computing `POSE_SOFA_REVERSED` from `POSE_SOFA` in `config.py` (rather than a new `constants.json` entry) means the "same point" invariant can never drift when the sofa waypoint is re-taught per arena. Quaternion math verified on this machine: sofa yaw −94.54° → reversed +85.46°, delta exactly 180.0°.
2. **Turn-around is best-effort.** Wrapped `FailureIsSuccess(Retry×3)` like the bag flow's arm moves: if Nav2 refuses the in-place rotation, the follow can still proceed (the host simply stands wherever the robot faces) — failing the whole bag flow would forfeit the follow-to-drop (200) + drop (50) points for a cosmetic step.
3. **Placement before `Look at host`.** The requirement says "right before the announcements start"; the `Look at host` pan-tilt leaf is part of the follow-intro block, so the drive goes before it — move base first, then aim the head, then speak.
4. **Modification 2 targets `hri-2026` only.** `createHRITask2026` is the production tree; `hri.py:createHRITask` is a frozen reference per the `hri_2026.py` docstring. Modification 3 lives in the shared factory `createTwoWayIntroduction`, so both trees inherit it — intended, since it's the only live "make yourself comfortable" prompt.
5. **The head-camera prompt site.** The only live "make yourself comfortable" ask is the `Complete escort announcement` at the start of `createTwoWayIntroduction` — it addresses both seated guests at once and immediately precedes the `BtNode_FeatureMatching` scan of the seated guests, which is exactly the moment a head-camera look helps. (Alternative reading — a per-guest prompt after each seating in `createEscortAndSeat` — was rejected: that message is commented out today, and no scan happens at those moments.)
