# GPSR `approach_person` Real Navigation Wiring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **Revision 2 (2026-07-02):** hardened after a 3-agent adversarial review.
> See "Adversarial-review findings incorporated" at the bottom for what
> changed and why. Key deltas: explicit `min/max_distance` + `timeout_sec`
> on the goal (the original single-`desired_distance` call disabled the
> server's entire attempt-2 recompute via `STATUS_INVALID_REQUEST`), a new
> Task 2 fixing pre-existing duplicate-key timeouts in
> `approach_planner.yaml` (currently 300 s at runtime, red CI invariant),
> venv-correct pytest commands, build-before-green-test ordering, mock
> registration, and honest regression/verification expectations.

**Goal:** Replace the current no-op mock of GPSR's `approach_person` action
(`behavior_tree.GPSR.small_trees.create_approach_person`) with a real
navigation call through the `approach_planner` package's `go_to_approach`
action, and make sure `approach_planner` is actually running when GPSR
launches. Standoff: `desired_distance=1.3` m center-to-person
(approach_planner's tuned 1.0 m default + the requested 0.3 m; ≈1.05 m
bumper-to-person given the 0.25 m footprint front extent, ±5 cm Nav2
`xy_goal_tolerance`). Note the baseline honestly: the pre-mock GPSR code
drove Nav2 straight onto the raw person point, so vs *that* behavior the
delta is the entire 1.3 m standoff; "+0.3 m" is relative to
approach_planner's own default that other tasks (Restaurant) use.

**Architecture:** `create_approach_person()` currently returns a bare
`py_trees.behaviours.Success` (mocked 2026-07-02). This plan swaps that for
`BtNode_Approach` (defined in `TemplateNodes/Navigation.py:578`, used by the
Restaurant task), which calls the `go_to_approach` action server hosted by
the `approach_planner` ROS node (`src/tk26_navigation/src/approach_planner`).
The server runs two attempts: (1) geometric standoff projection sent to
Nav2 — this honors `desired_distance` unconditionally; (2) on nav
abort/stall, a costmap ring search with reachability gating — this one
validates `min_distance <= desired <= max_distance` and hard-rejects the
goal (`STATUS_INVALID_REQUEST`, `algorithm.py:184`) if the bounds are left
at defaults (min 0.5 / max 1.0) with desired 1.3. Hence the goal passes all
three explicitly. The `approach_planner` node is a *standalone* launch file
not included by any `navigation_bringup` launch; `master_gpsr.sh` currently
only brings up `driver.launch.py` + `bringup_dwb_launch.py`, so Task 3 adds
a pane for it (same pattern `tmux_slam_navigation.sh` already uses for
Restaurant).

**Tech Stack:** ROS2 (rclpy, py_trees, py_trees_ros), Python 3.10, pytest
(via `.venv_decision`), tmux launcher scripts (bash).

## Global Constraints

- The goal passes **all three** distance fields explicitly:
  `desired_distance=1.3`, `min_distance=1.0`, `max_distance=1.6`, plus
  `timeout_sec=45.0`. Never send `desired_distance` alone: the server falls
  back to `min=0.5/max=1.0` defaults and the attempt-2 solve rejects
  `desired > max` with `STATUS_INVALID_REQUEST` — killing the recompute
  safety net. `min=1.0` also prevents the ring search from "succeeding"
  0.5 m from the person (~0.25 m bumper-to-person); `BtNode_Approach.
  process_result` accepts any `status==0` without checking achieved
  distance, so the floor must be enforced goal-side.
- Do NOT change `desired_distance_default` (or any *distance* default) in
  `src/tk26_navigation/src/approach_planner/config/approach_planner.yaml` —
  those are shared by every other caller (Restaurant etc.). Task 2's
  deletion of the *duplicate* timeout keys (lines 76-79) is the only edit
  to that file: it restores the intended first-occurrence values that a
  currently-RED CI invariant (`test_config_invariants.py::
  test_timeout_stack_is_bounded`) already pins.
- All pytest commands in the behavior_tree package run under
  `/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest`
  from a shell that has sourced `~/tk25_ws/install/setup.zsh`. System
  `python3` lacks `openai` (imported transitively via
  `GPSR/custom_nodes.py:11`) and fails at collection.
- The behavior_tree install tree is a plain COPY (tkbuild strips
  `--symlink-install`), and pytest imports the INSTALL copy via PYTHONPATH.
  Therefore: red test runs may happen any time, but the green test run must
  come AFTER `tkbuild`.
- Build/verify per repo convention: `tkbuild` (never raw `colcon`);
  `ros2 run` for node-level tests.
- The full behavior_tree test suite has a PRE-EXISTING collection error
  unrelated to this change: `test_gpsr_start_gate.py` → `KeyError:
  'pose_command'` (`gpsr_new.py:46` reads the installed `constants.json`,
  which is stale — the known behavior_tree package_data bug). Regression
  gates in this plan are "no NEW failures vs recorded baseline", not "all
  green".
- tk26_navigation has a concurrent committer on main: before committing
  there, re-check `git status`/HEAD; commit new only, never amend/rebase.
- Every package touched gets its README + CHANGELOG updated in the same
  commit as the code change (Task 4).
- Known latent hazard (documented, no action): if `two_stage_approach` is
  ever flipped ON in approach_planner, Stage B stops at its own
  `final_standoff` param (0.7 m) and IGNORES the goal's `desired_distance`
  — GPSR's standoff would silently regress. Default-off today, pinned by
  `test_two_stage_approach_default_off`. The factory docstring carries a
  CAUTION for whoever flips it.

---

### Task 1: Un-mock `create_approach_person` onto `BtNode_Approach`

**Files:**
- Modify: `src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/small_trees.py`
  - Import block (currently lines 25-29)
  - Constants block (after `WAVING_THRESHOLD_METERS`, currently line 138)
  - `bb_keys` class (remove `PERSON_NAV_POSE`, currently line 85)
  - `BtNode_PointToPoseStamped` class (currently lines 328-369) — delete, now dead
  - `create_approach_person()` (currently lines 776-788, the mocked body)
- Modify: `src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.json`
  (and every other config that enumerates navigation nodes — find them with
  the grep in Step 8) — register `BtNode_Approach`.
- Test: `src/tk25_decision/src/behavior_tree/test/test_gpsr_approach_person.py` (new)

**Interfaces:**
- Consumes: `behavior_tree.TemplateNodes.Navigation.BtNode_Approach.__init__(self, name, bb_target_key, desired_distance=0.0, min_distance=0.0, max_distance=0.0, num_angles=0, preferred_yaw_rad=nan, facing_yaw_offset_rad=0.0, timeout_sec=0.0, debug=False, action_name="go_to_approach", wait_for_server_timeout_sec=-3.0, action_timeout_ticks=0)` — already exists, do not modify. All of `desired_distance`/`min_distance`/`max_distance`/`timeout_sec` are stored as instance attributes (`Navigation.py:623-629`) and copied into the goal (`Navigation.py:666-672`).
- Produces: `create_approach_person() -> py_trees.composites.Sequence` whose
  children are `[_tuck_arm_for_nav(...), py_trees.decorators.Retry(child=BtNode_Approach(...), num_failures=3)]`,
  plus module constants `PERSON_APPROACH_DESIRED_DISTANCE_M / _MIN_DISTANCE_M / _MAX_DISTANCE_M / _TIMEOUT_SEC`.
  `ACTION_FACTORIES["approach_person"]` (line ~1395, unchanged) still points
  at this factory — no caller-side changes needed.

- [ ] **Step 1: Record the pre-change test-suite baseline**

Run: `/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/ -q 2>&1 | tail -20`
Save the tail (pass/fail/error counts and which files error) — Step 11
compares against it. Expected today: a collection ERROR from
`test_gpsr_start_gate.py` (`KeyError: 'pose_command'`, pre-existing).

- [ ] **Step 2: Write the failing structural test**

Create `src/tk25_decision/src/behavior_tree/test/test_gpsr_approach_person.py`:

```python
import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
from behavior_tree.GPSR.small_trees import (  # noqa: E402
    PERSON_APPROACH_DESIRED_DISTANCE_M,
    PERSON_APPROACH_MAX_DISTANCE_M,
    PERSON_APPROACH_MIN_DISTANCE_M,
    create_approach_person,
)
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach  # noqa: E402


def _find_approach_node(root):
    for child in root.iterate():
        if isinstance(child, BtNode_Approach):
            return child
    return None


def test_approach_person_calls_real_navigation():
    root = create_approach_person()
    assert isinstance(root, py_trees.composites.Sequence)
    node = _find_approach_node(root)
    assert node is not None, "create_approach_person no longer drives BtNode_Approach"
    assert node.desired_distance == PERSON_APPROACH_DESIRED_DISTANCE_M
    assert node.min_distance == PERSON_APPROACH_MIN_DISTANCE_M
    assert node.max_distance == PERSON_APPROACH_MAX_DISTANCE_M
    # NOTE: bb_target_key is NOT a readable attribute on BtNode_Approach —
    # ActionHandler.__init__ only uses it to wire a blackboard-client remap
    # to "goal"; asserting on it would raise AttributeError. The blackboard
    # wiring is exercised by the `gpsr-test-approach-person` ros2 run dev
    # runner (test_uncovered_actions.py), not this pure-Python test.


def test_approach_person_distance_bounds_satisfy_planner_guard():
    # approach_planner's solve rejects any goal where
    # not (min <= desired <= max) with STATUS_INVALID_REQUEST
    # (approach_planner/algorithm.py:184), which would disable its entire
    # attempt-2 costmap recompute. Pin the ordering so nobody re-breaks it
    # by "simplifying" the goal back to desired-only.
    assert (
        PERSON_APPROACH_MIN_DISTANCE_M
        <= PERSON_APPROACH_DESIRED_DISTANCE_M
        <= PERSON_APPROACH_MAX_DISTANCE_M
    )
    # 1.3 = approach_planner's tuned 1.0 m default + the requested 0.3 m.
    assert PERSON_APPROACH_DESIRED_DISTANCE_M == 1.3
```

- [ ] **Step 3: Run test to verify it fails**

Run: `/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_gpsr_approach_person.py -v`
Expected: FAIL — `ImportError: cannot import name 'PERSON_APPROACH_DESIRED_DISTANCE_M'`
(the constant exists in neither the source nor the not-yet-rebuilt install copy).

- [ ] **Step 4: Add the `BtNode_Approach` import**

In `small_trees.py`, change the `Navigation` import block:

```python
from behavior_tree.TemplateNodes.Navigation import (
    BtNode_GotoAction,
    BtNode_ConvertGraspPose,
    BtNode_CaptureCurrentPose,
    BtNode_Approach,
)
```

- [ ] **Step 5: Add the standoff constants**

Directly after `WAVING_THRESHOLD_METERS = 6.0` (currently line 138):

```python
# GPSR person-approach standoff (BtNode_Approach -> go_to_approach goal).
# desired = approach_planner's tuned desired_distance_default (1.0 m,
# config/approach_planner.yaml) + 0.3 m extra personal space for GPSR
# interactions (describe / ask / handover). min/max MUST bracket desired:
# the server-side attempt-2 solve rejects desired outside [min, max] with
# STATUS_INVALID_REQUEST (approach_planner/algorithm.py:184), which would
# silently disable the costmap recompute safety net. min=1.0 keeps
# ~0.75 m bumper-to-person even at the inward ring floor (footprint front
# extent 0.25 m); max=1.6 lets a wall-blocked 1.3 m ring degrade outward
# instead of failing NO_CANDIDATE. Per-goal overrides only — the shared
# yaml defaults used by other callers (Restaurant etc.) are untouched.
PERSON_APPROACH_DESIRED_DISTANCE_M = 1.3
PERSON_APPROACH_MIN_DISTANCE_M = 1.0
PERSON_APPROACH_MAX_DISTANCE_M = 1.6
# Per-goal wall-clock cap (goal.timeout_sec; 0 would fall back to the
# server's nav_total_timeout_sec). Retry(num_failures=3) x 45 s ~= 135 s
# worst case for one approach_person plan step.
PERSON_APPROACH_TIMEOUT_SEC = 45.0
```

- [ ] **Step 6: Remove the now-dead `PERSON_NAV_POSE` key**

In the `bb_keys` class, delete this line (currently line 85):

```python
    PERSON_NAV_POSE = "gpsr/person_nav_pose"            # PoseStamped (approach goal)
```

- [ ] **Step 7: Delete the now-dead `BtNode_PointToPoseStamped` class**

Delete the whole class: from `class BtNode_PointToPoseStamped(Behaviour):`
(currently line 328) through its final `return Status.SUCCESS` (currently
line 369) — i.e. everything up to, but not including,
`class BtNode_CheckBBContains(Behaviour):` (currently line 372). Its only
call site was removed when `create_approach_person` was mocked on
2026-07-02, and the real wiring in Step 9 doesn't need a point→pose
conversion — `BtNode_Approach` reads a `PointStamped` directly off the
blackboard. (Verified workspace-wide during review: no other importer or
user of the class or the key, including codegen.py / plan_viz.py /
orchestrator.py / render_planned_trees.py / all tests.)

- [ ] **Step 8: Register `BtNode_Approach` in the mock configs**

`BtNode_Approach` is not listed in any mock config today — it only mocks
correctly via the "unlisted node under global mock" catch-all
(`config.py:335-345`). A config that sets `navigation.enabled=false` to
exercise real nav would then run `BtNode_GotoAction` real but keep
`BtNode_Approach` mocked — inconsistent, and a violation of the package's
CLAUDE.md rule ("Add node class name to mock_config.json under appropriate
subsystem").

Find every config enumerating navigation nodes:

```bash
grep -rln '"BtNode_GotoAction"' /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/
```

In EACH file found, add alongside `BtNode_GotoAction` in the `navigation`
subsystem's node map:

```json
"BtNode_Approach": "IMMEDIATE",
```

(match the neighboring entries' value style in each file — e.g. if siblings
use `{"interaction_mode": "IMMEDIATE"}` objects, mirror that shape).

- [ ] **Step 9: Rewrite `create_approach_person`**

Replace the mocked body (currently lines 776-788, ending at the
`return py_trees.behaviours.Success(...)` call) with:

```python
def create_approach_person():
    """Navigate to the person pose stored by the most recent ``find_person``.

    Atomic action — the LLM plans ``find_person`` then ``approach_person``
    when an interaction (describe / follow / guide / handover) needs the
    robot standing next to the person rather than across the room. Drives
    the ``go_to_approach`` action (``approach_planner`` package) against
    ``bb_keys.TARGET_PERSON_POSE`` (PointStamped from vision, map frame):
    attempt 1 projects a standoff pose ``desired_distance`` back along the
    robot→person axis and hands it to Nav2; on nav abort/stall, attempt 2
    ring-searches the live global costmap between min/max_distance with
    reachability gating (see ``BtNode_Approach``).

    Requires the ``approach_planner`` node to be running (``ros2 launch
    approach_planner approach_planner.launch.py``) — wired as pane 2 of
    master_gpsr.sh's navigation window.

    CAUTION: if ``two_stage_approach`` is ever flipped ON in
    approach_planner, Stage B stops at its own ``final_standoff`` param
    (0.7 m) and IGNORES this goal's desired_distance — the GPSR standoff
    would silently regress. Default-off today, pinned by
    ``test_config_invariants.test_two_stage_approach_default_off``.
    """
    seq = py_trees.composites.Sequence("small/approach_person", memory=True)
    seq.add_child(_tuck_arm_for_nav("tuck arm before approach"))
    seq.add_child(py_trees.decorators.Retry(
        "retry approach",
        BtNode_Approach(
            "approach person",
            bb_target_key=bb_keys.TARGET_PERSON_POSE,
            desired_distance=PERSON_APPROACH_DESIRED_DISTANCE_M,
            min_distance=PERSON_APPROACH_MIN_DISTANCE_M,
            max_distance=PERSON_APPROACH_MAX_DISTANCE_M,
            timeout_sec=PERSON_APPROACH_TIMEOUT_SEC,
        ),
        num_failures=3,
    ))
    return seq
```

- [ ] **Step 10: Build (must precede the green test — install is a copy)**

Run: `tkbuild tk25_decision --packages-select behavior_tree`
Expected: build succeeds; tkbuild reports shebang patch + verify OK.

- [ ] **Step 11: Run the new test to verify it passes**

Run: `/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_gpsr_approach_person.py -v`
Expected: PASS (2 passed).

- [ ] **Step 12: Regression check — no NEW failures vs the Step 1 baseline**

Run: `/home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/python -m pytest /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/ -q 2>&1 | tail -20`
Expected: identical failure/error set to Step 1's baseline plus the new
test's 2 passes. The `test_gpsr_start_gate.py` collection error
(`KeyError: 'pose_command'`) is pre-existing (stale installed
constants.json / package_data bug) and stays — it is NOT caused by this
change and must not be "fixed" here by editing setup.py (separate known
issue, separate change).

- [ ] **Step 13: Commit**

```bash
git -C /home/tinker/tk25_ws/src/tk25_decision add \
  src/behavior_tree/behavior_tree/GPSR/small_trees.py \
  src/behavior_tree/behavior_tree/mock_config.json \
  src/behavior_tree/test/test_gpsr_approach_person.py
# plus any other mock-config files Step 8 touched (git status to confirm)
git -C /home/tinker/tk25_ws/src/tk25_decision commit -m "feat(GPSR): wire approach_person to real go_to_approach navigation, 1.3m standoff"
```

---

### Task 2: Fix pre-existing duplicate timeout keys in `approach_planner.yaml`

The config defines `nav_first_attempt_timeout_sec`, `nav_total_timeout_sec`,
`nav_no_progress_timeout_sec`, and `debug_default` TWICE in the same mapping
(intended values 25.0/75.0/8.0/true at lines 34-37; stale leftovers
60.0/300.0/10.0/false at lines 76-79). Verified during review: both PyYAML
and the real ROS param parser (rcl_yaml_param_parser) resolve duplicates
last-wins, so the node currently runs with a **300 s** total timeout per
goal, and `test_config_invariants.py::test_timeout_stack_is_bounded` is
**already failing** (`assert 60.0 <= 30.0`). Without this fix, enabling
attempt-2 (Task 1) makes worst-case failures take the full 300 s per
attempt. This must land with (or before) Task 1.

**Files:**
- Modify: `src/tk26_navigation/src/approach_planner/config/approach_planner.yaml:76-79`

**Interfaces:** None new — restores the values the CI invariant already pins.

- [ ] **Step 1: Confirm the invariant is red before the fix**

Find the interpreter this package's tests run under: check for a
tk26_navigation sub-workspace venv (`ls -d /home/tinker/tk25_ws/src/tk26_navigation/.venv* 2>/dev/null`);
use its python if present, else `conda run -n base python` (never bare
system python for ad-hoc work, per workspace policy).

Run: `<python> -m pytest /home/tinker/tk25_ws/src/tk26_navigation/src/approach_planner/test/test_config_invariants.py::test_timeout_stack_is_bounded -v`
Expected: FAIL (`assert 60.0 <= 30.0`) — proving the duplicate-key override
is live.

- [ ] **Step 2: Delete the duplicate block**

In `config/approach_planner.yaml`, delete these four lines (currently
76-79, at the very end of the file, AFTER the Stage-B comment block —
do not touch the identical-looking keys at lines 34-37):

```yaml
    nav_first_attempt_timeout_sec: 60.0   # cap on nav_attempt_1 alone
    nav_total_timeout_sec: 300.0           # default outer total when goal.timeout_sec=0
    nav_no_progress_timeout_sec: 10.0      # stall watchdog on distance_remaining
    debug_default: false                  # currently informational; goal.debug controls feedback
```

Effective values become the intended first occurrences: 25.0 / 75.0 / 8.0 /
true. (`debug_default` flipping false→true is informational only —
`goal.debug` from the BT controls feedback, and the BT sends
`debug=False`.)

- [ ] **Step 3: Verify the invariant goes green**

Run: `<python> -m pytest /home/tinker/tk25_ws/src/tk26_navigation/src/approach_planner/test/test_config_invariants.py -v`
Expected: ALL tests in the file pass (not just the timeout one — the other
invariants pin two_stage/occlusion defaults and must stay green).

- [ ] **Step 4: Rebuild so the installed copy of the yaml refreshes**

Run: `tkbuild tk26_navigation --packages-select approach_planner`
Expected: success. Deploy = relaunch the approach_planner node (it reads
params at startup); note this in the operator handoff.

- [ ] **Step 5: Commit (concurrent-committer caution)**

tk26_navigation main has a concurrent committer. Before committing:
`git -C /home/tinker/tk25_ws/src/tk26_navigation status && git -C /home/tinker/tk25_ws/src/tk26_navigation log --oneline -3`
— confirm HEAD is where you expect and only your yaml change is staged.
Commit new only; never amend/rebase.

```bash
git -C /home/tinker/tk25_ws/src/tk26_navigation add src/approach_planner/config/approach_planner.yaml
git -C /home/tinker/tk25_ws/src/tk26_navigation commit -m "fix(approach_planner): remove duplicate timeout keys (last-wins made runtime total 300s, invariant red)"
```

---

### Task 3: Launch `approach_planner` in the GPSR tmux session

**Files:**
- Modify: `src/tk25_basic/src/scripts/master_gpsr.sh` (navigation window, currently lines 16-28)

**Interfaces:**
- Consumes: `ros2 launch approach_planner approach_planner.launch.py` — the
  existing, unmodified launch entry point, already used identically by
  `tmux_slam_navigation.sh` / `tmux_slam_navigation2.sh` for Restaurant.
- Produces: a third pane in the `robot:navigation` tmux window so
  `go_to_approach` / `find_approach_pose` are advertised before GPSR
  reaches its first `approach_person` step. (Startup order vs
  `bringup_dwb_launch.py` is safe: `planner_node.__init__` creates the
  ActionServer and a transient_local costmap subscription without blocking,
  so simultaneous pane launch is fine — verified during review.)

- [ ] **Step 1: Add the approach_planner pane**

In `master_gpsr.sh`, the navigation window currently ends at:

```bash
tmux send-keys -t $SESSION:$WINDOW.1 \
    "source ~/tk25_ws/install/setup.zsh && ros2 launch navigation_bringup bringup_dwb_launch.py; exec zsh" C-m
```

Add immediately after it:

```bash
tmux split-window -h -t $SESSION:$WINDOW
tmux select-layout -t $SESSION:$WINDOW tiled

tmux send-keys -t $SESSION:$WINDOW.2 \
    "source ~/tk25_ws/install/setup.zsh && ros2 launch approach_planner approach_planner.launch.py; exec zsh" C-m
```

(Mirrors the pane-3 pattern in `tmux_slam_navigation.sh:20-22`. Pane
indexing verified against this machine's tmux config: no
`base-index`/`pane-base-index` overrides, so `.2` is correct.)

- [ ] **Step 2: Manual verification (operator, live machine)**

Preconditions — in the shell that runs the script:
`export ROBOT_NAME=tinker1|tinker2` (or `source src/tk25_basic/tools/robot-env.sh`).
`approach_planner.launch.py` resolves `RobotField('base.footprint')` at
launch and DIES with `ResolverError: ROBOT_NAME not set` otherwise (pane 1's
`bringup_dwb_launch.py` shares this requirement, so it's not a new
precondition — but the plan's smoke test fails without it). Beware the
stale-session trap: if tmux session `robot` already exists from a shell
that lacked `ROBOT_NAME`, new windows inherit the OLD session env — kill
the session first if in doubt.

Run: `bash /home/tinker/tk25_ws/src/tk25_basic/src/scripts/master_gpsr.sh`
Then: confirm the `navigation` window has 3 panes, the third running
`planner_node` with no `ResolverError` / parameter-parse errors.
Then: `ros2 action list | grep go_to_approach` → expect `/go_to_approach`.

- [ ] **Step 3: Commit**

```bash
git -C /home/tinker/tk25_ws/src/tk25_basic add src/scripts/master_gpsr.sh
git -C /home/tinker/tk25_ws/src/tk25_basic commit -m "feat(scripts): launch approach_planner in master_gpsr.sh so approach_person can navigate"
```

---

### Task 4: Docs — README/CHANGELOG for all three touched packages

**Files:**
- Modify: `src/tk25_decision/src/behavior_tree/CHANGELOG.md`
- Modify: `src/tk25_decision/src/behavior_tree/README.md` (only if it documents GPSR action behavior/dependencies — check first; if not, note the dependency in the CHANGELOG entry instead)
- Modify: `src/tk26_navigation/src/approach_planner/` README/CHANGELOG (check which exist; document the duplicate-key fix and effective-timeout change 300→75 s)
- Modify: `src/tk25_basic/src/scripts/CHANGELOG.md`
- Modify: `src/tk25_basic/src/scripts/README.md` (note the new `approach_planner` dependency in the GPSR bring-up notes)

**Interfaces:** None (docs only).

- [ ] **Step 1: `behavior_tree/CHANGELOG.md` entry**

Add a new top entry above `## [2.2.14] - 2026-07-02` (bump to `[2.2.15]`,
or the next free number/date if other work landed meanwhile — check the
file's head first; a concurrent session also commits to this repo).
Describe: mock-then-real-wire sequence; the goal now pins
`desired=1.3 / min=1.0 / max=1.6 / timeout=45 s` and WHY the bounds trio is
mandatory (`STATUS_INVALID_REQUEST` guard); removed
`BtNode_PointToPoseStamped` / `PERSON_NAV_POSE`; `BtNode_Approach`
registered in mock configs; new hard dependency on `approach_planner`
being launched (link `master_gpsr.sh`); the two-stage Stage-B caution.
Follow the existing emoji-header + prose style.

- [ ] **Step 2: approach_planner docs entry**

In `src/tk26_navigation/src/approach_planner/` — find its README/CHANGELOG
(`ls` the package root; if only a README with a Changelog section exists,
append there). Document: duplicate timeout keys removed; runtime-effective
values changed 60/300/10/false → 25/75/8/true (last-wins duplicate
resolution verified on the real ROS param parser);
`test_timeout_stack_is_bounded` green again; deploy requires node relaunch.

- [ ] **Step 3: `tk25_basic/src/scripts/CHANGELOG.md` entry**

Add an `### Added` bullet under `## [Unreleased]`:

```markdown
- `master_gpsr.sh` now launches `approach_planner approach_planner.launch.py`
  as a third pane in the `navigation` window. GPSR's `approach_person`
  action (`behavior_tree.GPSR.small_trees.create_approach_person`) calls
  the `go_to_approach` action hosted by that node; without it the action
  server never advertises and the BT step fails after its retries. Mirrors
  the existing `tmux_slam_navigation.sh` pane used by the Restaurant task.
  Requires `ROBOT_NAME` in the invoking shell (same as the existing
  navigation panes).
```

- [ ] **Step 4: `tk25_basic/src/scripts/README.md` note**

Add a short bullet under `## Notes`: `master_gpsr.sh`'s navigation window
now has 3 panes; the third (`approach_planner`) is required for GPSR
`approach_person` to actually move the robot.

- [ ] **Step 5: Commit (one per repo)**

```bash
git -C /home/tinker/tk25_ws/src/tk25_decision add src/behavior_tree/CHANGELOG.md src/behavior_tree/README.md
git -C /home/tinker/tk25_ws/src/tk25_decision commit -m "docs(behavior_tree): document approach_person real-navigation wiring"

git -C /home/tinker/tk25_ws/src/tk26_navigation add src/approach_planner/<README/CHANGELOG as found>
git -C /home/tinker/tk25_ws/src/tk26_navigation commit -m "docs(approach_planner): changelog for duplicate-timeout-key fix"

git -C /home/tinker/tk25_ws/src/tk25_basic add src/scripts/CHANGELOG.md src/scripts/README.md
git -C /home/tinker/tk25_ws/src/tk25_basic commit -m "docs(scripts): document approach_planner pane in master_gpsr.sh"
```

---

## Post-plan manual verification checklist (operator, not agent)

- [ ] `export ROBOT_NAME=...`, bring up `master_gpsr.sh`, then
  `ros2 run behavior_tree gpsr-test-approach-person` against a live/sim
  robot: confirm the robot drives toward the seeded/detected person and
  stops **1.3–1.8 m** away (1.3 m nominal; the attempt-1 projection may
  legally back off up to +0.5 m radial extra when the exact standoff cell
  is blocked; attempt-2 may land anywhere in 1.0–1.6 m). NOT on top of
  them, and never closer than ~1.0 m center (~0.75 m bumper).
- [ ] Offline mock smoke — use the full-mock config (plain
  `BT_MOCK_MODE=true` is NOT enough: the tree tucks the arm first via
  `BtNode_MoveArmSingle`, and the default `mock_config.json` has the
  manipulation subsystem DISABLED, so setup would try to reach the real
  arm action server):
  `BT_MOCK_CONFIG=$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/full_mock.json ros2 run behavior_tree gpsr-test-approach-person`
  → completes without contacting any ROS action server.
- [ ] Relaunch `approach_planner` after Task 2 lands (params read at
  startup) — verify with
  `ros2 param get /approach_planner nav_total_timeout_sec` → **75.0**.
- [ ] Full GPSR run-through: `find_person` → `approach_person` →
  `describe_person`/`ask_person` sequencing with real nav in the loop.
  Timing budget: ≤45 s per approach attempt, ≤~135 s per step worst case
  (Retry ×3).

---

## Adversarial-review findings incorporated (2026-07-02, 3 independent reviewers)

| # | Severity | Finding | Fix in this revision |
|---|----------|---------|----------------------|
| 1 | BLOCKER | `desired=1.3` with default bounds (min 0.5/max 1.0) → `STATUS_INVALID_REQUEST` at `algorithm.py:184`; attempt-1 unaffected, but the entire attempt-2 recompute becomes dead code | Goal passes `min=1.0/max=1.6` explicitly; pinned by new bounds test |
| 2 | BLOCKER | Duplicate YAML keys in `approach_planner.yaml` resolve last-wins → runtime total timeout 300 s; `test_timeout_stack_is_bounded` already red on main | New Task 2 deletes lines 76-79; BT also caps `timeout_sec=45` per goal |
| 3 | BLOCKER | Plan's pytest commands used bare `python3` (no `openai` → collection error) | All pytest via `.venv_decision/bin/python` |
| 4 | BLOCKER | Green-test-before-build ordering: install is a COPY; pytest imports the install tree | `tkbuild` moved before the green run (Task 1 Steps 10→11) |
| 5 | MAJOR | Default `min=0.5` would let the action return SUCCESS ~0.25 m bumper-to-person (BT accepts any status==0 without distance check) | `min=1.0` goal-side |
| 6 | MAJOR | "Full suite passes" unachievable: pre-existing `test_gpsr_start_gate.py` collection error (stale installed constants.json) | Baseline-diff regression gate (Steps 1/12) |
| 7 | MAJOR | Mock-smoke claim false under default config: `_tuck_arm_for_nav` adds `BtNode_MoveArmSingle` (manipulation subsystem disabled → runs REAL under BT_MOCK_MODE) | Checklist uses `full_mock.json` invocation (the runner's own documented pattern) |
| 8 | MAJOR (latent) | Stage B (`two_stage_approach=true`) ignores goal `desired_distance`, stops at `final_standoff=0.7` | CAUTION in factory docstring + Global Constraints; default-off pinned by CI |
| 9 | MINOR | `BtNode_Approach` unregistered in all mock configs (worked via catch-all only) | New Task 1 Step 8 registers it |
| 10 | MINOR | Operator smoke lacked `ROBOT_NAME` precondition (`RobotField` → `ResolverError`, node dies pre-advertise); tmux stale-session-env trap | Task 3 Step 2 preconditions |
| 11 | MINOR | "+0.3 m" rationale conflated baselines (old GPSR drove onto the raw point — real delta is the whole 1.3 m) | Goal section reworded |
| 12 | MINOR | Step 6 deletion boundary said `return pose` (doesn't exist; class ends `return Status.SUCCESS`); stale line ranges 776-786 vs 776-788 | Corrected |

Cleared by review (no action): dead-code deletion is workspace-safe; `root.iterate()` reaches nodes under `Retry` (same idiom as `test_restaurant_timeouts.py`); `BtNode_Approach` constructs without ROS; pane index `.2` correct on this machine; approach_planner's deps are system-importable (Restaurant precedent); concurrent pane startup safe; doc version claims accurate; frames safe (vision writes map-frame points; non-map + TF-fail → graceful abort); Nav2 goal slop only ±5 cm (`general_goal_checker`).
