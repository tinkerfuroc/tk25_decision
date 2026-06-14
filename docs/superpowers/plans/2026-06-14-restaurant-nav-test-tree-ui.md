# Restaurant pure-nav test tree + web UI — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A one-shot pure-nav test — pan-tilt sweep → scan for waving → approach the closest waver — plus a track_web-style browser dashboard that brings up prerequisites, launches the test, and shows live status + camera + approach progress.

**Architecture:** Two components in two repos. (1) A BT test tree in `behavior_tree` (`tk25_decision`) composed from the existing `test_scan` sweep + `BtNode_Approach`, with a small status-publishing leaf, wrapped in `OneShot`. (2) A new `restaurant_nav_test_web` package in `tk26_vision` mirroring `vision_track/track_web` (FastAPI + thread-safe ROS bridge + `webui/` + an allowlisted ProcessManager that spawns the BT and the prerequisite stack).

**Tech Stack:** ROS 2 Humble, `py_trees`/`py_trees_ros`, `rclpy`, FastAPI + uvicorn (py3.10 venv), OpenCV/`cv_bridge` (MJPEG), `tf2_ros`.

**Spec:** `tk25_decision/docs/superpowers/specs/2026-06-14-restaurant-nav-test-tree-ui-design.md`

---

## Process invariants (every task)

- **Two repos, both on branch `dev`:** Task A1 commits to `tk25_decision` (`/home/tinker/tk25_ws/src/tk25_decision`); Tasks B1–B5 commit to `tk26_vision` (`/home/tinker/tk25_ws/src/tk26_vision`). Both are currently on `dev` — **do not switch branches**; verify with `git rev-parse --abbrev-ref HEAD` before committing.
- **Concurrent committers may be active.** Never `--amend`/rebase/reset. Commit NEW only. `git add` EXACT named paths — never `git add -A`/`.`. Before each commit run `git rev-parse HEAD` (capture), after run `git log -1 --format='parent=%P'` and confirm it equals the captured HEAD.
- Commit message last line MUST be: `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`
- **Builds:** `behavior_tree` → `cd /home/tinker/tk25_ws && colcon build --packages-select behavior_tree` then `source install/setup.bash`. The web package → `cd /home/tinker/tk25_ws && ./src/tk26_vision/scripts/build.sh --packages-select restaurant_nav_test_web` (this wrapper fixes the entry-point shebangs to the vision venv that holds fastapi/uvicorn/cv2; plain colcon would produce `#!/usr/bin/python3` shebangs that can't see the venv).
- **Pure tests** (no ROS): run with the host python: `python -m pytest <path> -q`.
- No edits to reused BT nodes, the waving service, the nav stack, or any deployed config.

**Known host caveat:** `behavior_tree/messages.py` can fail to import on some hosts (`TTSCnRequest` missing from `tinker_audio_msgs`). That is a documented pre-existing issue, not caused by this work. Task A1's pure test is deliberately decoupled from it (imports only `nav_test_select`). If a `ros2 run`/import step trips that error, report it as the pre-existing issue and proceed.

---

## Task A1: BT test tree (`tk25_decision` / `behavior_tree`)

**Files:**
- Create: `src/behavior_tree/behavior_tree/Restaurant/nav_test_select.py` (pure, no ROS)
- Create: `src/behavior_tree/behavior_tree/Restaurant/restaurant_nav_test.py`
- Modify: `src/behavior_tree/setup.py` (add a console script after the `test-go-to-approach` line)
- Create: `src/behavior_tree/test/test_restaurant_nav_test.py` (pure)

(Paths relative to the `tk25_decision` repo root `/home/tinker/tk25_ws/src/tk25_decision`.)

- [ ] **Step 1: Write the pure failing test**

Create `src/behavior_tree/test/test_restaurant_nav_test.py`:

```python
"""Pure unit tests for the restaurant nav-test selection helper (no ROS)."""
from behavior_tree.Restaurant.nav_test_select import nearest_index, SWEEP_PANS, TILT_DEG


def test_sweep_positions_are_centre_left_right():
    assert SWEEP_PANS == [0.0, -60.0, 60.0]
    assert TILT_DEG == 10.0


def test_nearest_index_picks_closest_to_robot():
    pts = [(5.0, 0.0), (1.0, 0.0), (3.0, 0.0)]
    assert nearest_index(pts, (0.0, 0.0)) == 1


def test_nearest_index_robot_none_returns_zero():
    assert nearest_index([(5.0, 0.0), (1.0, 0.0)], None) == 0


def test_nearest_index_empty_returns_minus_one():
    assert nearest_index([], (0.0, 0.0)) == -1
```

- [ ] **Step 2: Run it — verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision && python -m pytest src/behavior_tree/test/test_restaurant_nav_test.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'behavior_tree.Restaurant.nav_test_select'` (or import error). If `behavior_tree` isn't importable at all from the host, run from `src/behavior_tree`: `cd src/behavior_tree && python -m pytest test/test_restaurant_nav_test.py -q`.

- [ ] **Step 3: Create the pure helper module**

Create `src/behavior_tree/behavior_tree/Restaurant/nav_test_select.py`:

```python
"""Pure (no-ROS) helpers + constants for the restaurant nav test, kept separate
from the BT module so they unit-test without importing behavior_tree.messages."""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

# Sweep positions (for now): centre, then left, then right. Tunable.
SWEEP_PANS: List[float] = [0.0, -60.0, 60.0]
TILT_DEG: float = 10.0


def nearest_index(points_xy: List[Tuple[float, float]],
                  robot_xy: Optional[Tuple[float, float]]) -> int:
    """Index of the point nearest robot_xy (Euclidean). Empty -> -1; robot_xy
    None -> 0 (caller falls back to first when TF is unavailable)."""
    if not points_xy:
        return -1
    if robot_xy is None:
        return 0
    rx, ry = robot_xy
    best_i, best_d = 0, float("inf")
    for i, (x, y) in enumerate(points_xy):
        d = math.hypot(x - rx, y - ry)
        if d < best_d:
            best_i, best_d = i, d
    return best_i
```

- [ ] **Step 4: Run the pure test — verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk25_decision && python -m pytest src/behavior_tree/test/test_restaurant_nav_test.py -q`
Expected: PASS (4 passed).

- [ ] **Step 5: Create the BT tree module**

Create `src/behavior_tree/behavior_tree/Restaurant/restaurant_nav_test.py`:

```python
"""Restaurant pure-nav test: pan-tilt sweep -> scan for waving -> approach the
closest waver. Composes the test_scan sweep + BtNode_Approach, publishes JSON
status on /restaurant_nav_test/status, and is wrapped in OneShot (runs once,
then latches/idles until the process is stopped).

Run:                ros2 run behavior_tree restaurant-nav-test
Mock smoke:         BT_MOCK_MODE=true ros2 run behavior_tree restaurant-nav-test
Stop with Ctrl+C (or the dashboard's Stop, which SIGTERMs -> run_tree unwinds).
"""
from __future__ import annotations

import json

import py_trees
import rclpy
import tf2_ros
from geometry_msgs.msg import PointStamped  # noqa: F401  (type used via blackboard)
from std_msgs.msg import String

from behavior_tree.runtime import run_tree
from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.Navigation import BtNode_Approach
from .nav_test_select import SWEEP_PANS, TILT_DEG, nearest_index
from .test_scan import scan_once, KEY_CUSTOMER_CENTROIDS

STATUS_TOPIC = "/restaurant_nav_test/status"
KEY_APPROACH_TARGET = "restaurant_nav_test_target"
TARGET_FRAME = "map"
ROBOT_FRAME = "base_link"


class BtNode_SelectClosestWaver(py_trees.behaviour.Behaviour):
    """Pick the centroid nearest the robot (TF map->base_link; falls back to
    index 0 if TF unavailable) from KEY_CUSTOMER_CENTROIDS, write it as the
    approach target. FAILURE when no centroids were accumulated."""

    def __init__(self, name, bb_centroids_key=KEY_CUSTOMER_CENTROIDS,
                 bb_target_key=KEY_APPROACH_TARGET):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="centroids", access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_centroids_key))
        self.blackboard.register_key(
            key="target", access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_key))
        self._tf_buffer = None
        self._tf_listener = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(f"'{self.name}': 'node' missing in setup kwargs") from e
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self.node)

    def _robot_xy(self):
        try:
            t = self._tf_buffer.lookup_transform(
                TARGET_FRAME, ROBOT_FRAME, rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:  # noqa: BLE001 — TF not ready -> caller falls back
            return None

    def update(self):
        if not self.blackboard.exists("centroids") or not self.blackboard.centroids:
            self.feedback_message = "no waving centroids accumulated"
            return py_trees.common.Status.FAILURE
        centroids = list(self.blackboard.centroids)
        xy = [(c.point.x, c.point.y) for c in centroids]
        idx = nearest_index(xy, self._robot_xy())
        self.blackboard.target = centroids[idx]
        self.feedback_message = f"selected waver {idx} of {len(centroids)}"
        return py_trees.common.Status.SUCCESS


class BtNode_PublishNavTestStatus(py_trees.behaviour.Behaviour):
    """Publish a JSON status snapshot on STATUS_TOPIC for the web dashboard."""

    def __init__(self, name, phase, result=None,
                 bb_centroids_key=KEY_CUSTOMER_CENTROIDS,
                 bb_target_key=KEY_APPROACH_TARGET):
        super().__init__(name)
        self.phase = phase
        self.result = result
        self._pub = None
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            key="centroids", access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_centroids_key))
        self.blackboard.register_key(
            key="target", access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", bb_target_key))

    def setup(self, **kwargs):
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError(f"'{self.name}': 'node' missing in setup kwargs") from e
        self._pub = self.node.create_publisher(String, STATUS_TOPIC, 10)

    def update(self):
        centroids = (list(self.blackboard.centroids)
                     if self.blackboard.exists("centroids") else [])
        target = self.blackboard.target if self.blackboard.exists("target") else None
        doc = {
            "phase": self.phase,
            "result": self.result,
            "waver_count": len(centroids),
            "wavers": [{"x": c.point.x, "y": c.point.y} for c in centroids],
            "target": ({"x": target.point.x, "y": target.point.y}
                       if target is not None else None),
        }
        if self._pub is not None:
            msg = String()
            msg.data = json.dumps(doc)
            self._pub.publish(msg)
        self.feedback_message = f"status: {self.phase} result={self.result}"
        return py_trees.common.Status.SUCCESS


def _status(phase, result=None):
    return BtNode_PublishNavTestStatus(
        name=f"status: {phase}", phase=phase, result=result)


def build_tree() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="Restaurant Nav Test", memory=True)
    root.add_child(BtNode_Announce(
        name="announce start", bb_source=None,
        message="Starting restaurant nav test."))
    root.add_child(_status("scanning"))
    sweep = py_trees.composites.Sequence(name="scan sweep", memory=True)
    for pan in SWEEP_PANS:
        sweep.add_child(scan_once(pan, tilt=TILT_DEG, target_frame=TARGET_FRAME))
    root.add_child(sweep)
    root.add_child(_status("scanned"))
    outcome = py_trees.composites.Selector(name="approach outcome", memory=False)
    success = py_trees.composites.Sequence(name="select+approach", memory=True)
    success.add_child(BtNode_SelectClosestWaver(name="select closest waver"))
    success.add_child(_status("approaching"))
    success.add_child(BtNode_Approach(
        name="approach closest waver", bb_target_key=KEY_APPROACH_TARGET))
    success.add_child(_status("done", result="success"))
    outcome.add_child(success)
    outcome.add_child(_status("done", result="failed"))
    root.add_child(outcome)
    # OneShot: run the scan->approach once, then latch + idle (no auto re-run
    # under tick_tock). The dashboard re-runs by respawning the process.
    return py_trees.decorators.OneShot(
        name="nav test (one-shot)", child=root,
        policy=py_trees.common.OneShotPolicy.ON_COMPLETION)


def main():
    run_tree(build_tree, period_ms=500.0, title="Restaurant Nav Test",
             node_name="restaurant_nav_test")


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Add the console-script entry point**

In `src/behavior_tree/setup.py`, in the `console_scripts` list, immediately after the line
`"test-go-to-approach = behavior_tree.Restaurant.test_go_to_approach:main",`
add:

```python
            "restaurant-nav-test = behavior_tree.Restaurant.restaurant_nav_test:main",
```

- [ ] **Step 7: Build + pure test + import smoke**

Run:
```bash
cd /home/tinker/tk25_ws && colcon build --packages-select behavior_tree && source install/setup.bash
cd /home/tinker/tk25_ws/src/tk25_decision && python -m pytest src/behavior_tree/test/test_restaurant_nav_test.py -q
python -c "import py_trees; from behavior_tree.Restaurant.restaurant_nav_test import build_tree; t=build_tree(); print(type(t).__name__)"
```
Expected: pure test PASS (4 passed); the import line prints `OneShot` (confirms the tree constructs). If the `import` line raises `ImportError: cannot import name 'TTSCnRequest'`, that is the documented pre-existing `messages.py` host issue — note it and continue (the pure test still passes; the tree wiring is verified by the spec reviewer reading the code).

- [ ] **Step 8: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk25_decision
git rev-parse --abbrev-ref HEAD   # expect: dev
git rev-parse HEAD                 # capture
git add src/behavior_tree/behavior_tree/Restaurant/nav_test_select.py \
        src/behavior_tree/behavior_tree/Restaurant/restaurant_nav_test.py \
        src/behavior_tree/setup.py \
        src/behavior_tree/test/test_restaurant_nav_test.py
git diff --cached --name-only   # confirm exactly those 4
git commit -m "feat(behavior_tree): restaurant pure-nav test tree (scan-waving -> approach closest)

One-shot tree composing the test_scan sweep ([0,-60,+60] deg) + BtNode_Approach,
selecting the waver nearest the robot (TF, fallback index 0), publishing JSON
status on /restaurant_nav_test/status for the dashboard. New console script
restaurant-nav-test.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log -1 --format='parent=%P'   # must equal the captured HEAD
```

---

## Task B1: web package scaffold + ProcessManager (`tk26_vision`)

**Files (relative to `tk26_vision` repo root `/home/tinker/tk25_ws/src/tk26_vision`):**
- Create: `src/restaurant_nav_test_web/package.xml`
- Create: `src/restaurant_nav_test_web/setup.py`
- Create: `src/restaurant_nav_test_web/resource/restaurant_nav_test_web`
- Create: `src/restaurant_nav_test_web/restaurant_nav_test_web/__init__.py`
- Create: `src/restaurant_nav_test_web/restaurant_nav_test_web/process_manager.py`
- Create: `src/restaurant_nav_test_web/config/processes.yaml`
- Create: `src/restaurant_nav_test_web/requirements.txt`
- Create: `src/restaurant_nav_test_web/test/test_process_manager.py`

- [ ] **Step 1: Scaffold the package**

`src/restaurant_nav_test_web/package.xml`:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>restaurant_nav_test_web</name>
  <version>0.0.0</version>
  <description>Browser dashboard to run + observe the restaurant pure-nav test (scan-waving -> approach), with operator-driven prerequisite bring-up.</description>
  <maintainer email="cindy.w0135@gmail.com">cindy</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
(`fastapi`/`uvicorn` are NOT package.xml deps — they live in `requirements.txt`, installed into the vision venv, exactly like `vision_track`.)

`src/restaurant_nav_test_web/resource/restaurant_nav_test_web` — empty file (ament marker): create it empty.

`src/restaurant_nav_test_web/restaurant_nav_test_web/__init__.py` — empty file.

`src/restaurant_nav_test_web/requirements.txt`:
```
fastapi
uvicorn
```

`src/restaurant_nav_test_web/setup.py`:
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'restaurant_nav_test_web'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'webui'), glob('webui/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cindy',
    maintainer_email='cindy.w0135@gmail.com',
    description='Restaurant pure-nav test dashboard',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'restaurant_nav_test_web = restaurant_nav_test_web.restaurant_nav_test_web:main',
        ],
    },
)
```
(`webui/` and `launch/` globs are empty until Tasks B4/B5 — harmless.)

- [ ] **Step 2: Write the failing ProcessManager test**

`src/restaurant_nav_test_web/test/test_process_manager.py`:
```python
"""Unit tests for the allowlisted ProcessManager (ROS-free)."""
import time

from restaurant_nav_test_web.process_manager import ProcessManager


def _pm():
    # Trivial allowlist using `sleep` so tests need no ROS / ros2 CLI.
    return ProcessManager(
        registry={"a": ["sleep", "30"], "b": ["sleep", "30"]},
        groups={"all": ["a", "b"]},
        stagger_sec=0.0,
    )


def test_unknown_name_is_rejected_not_run():
    pm = _pm()
    out = pm.start("evil")
    assert "error" in out and "unknown" in out["error"]
    assert pm.status("evil").get("error")


def test_start_status_stop_cycle():
    pm = _pm()
    st = pm.start("a")
    assert st["running"] is True and st["pid"]
    assert pm.start("a")["running"] is True  # idempotent, no duplicate
    stopped = pm.stop("a")
    assert stopped["running"] is False
    pm.shutdown_all()


def test_group_starts_all_members():
    pm = _pm()
    out = pm.start_group("all")
    assert isinstance(out, list) and len(out) == 2
    assert all(m["running"] for m in out)
    pm.shutdown_all()
    time.sleep(0.1)
    assert pm.status("a")["running"] is False
```

- [ ] **Step 3: Run it — verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/restaurant_nav_test_web && python -m pytest test/test_process_manager.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'restaurant_nav_test_web.process_manager'`.

- [ ] **Step 4: Create the ProcessManager**

Create `src/restaurant_nav_test_web/restaurant_nav_test_web/process_manager.py` by **copying `vision_track`'s** verbatim and changing only the default REGISTRY/GROUPS to be **config-loaded**. Copy the full class body from
`/home/tinker/tk25_ws/src/tk26_vision/src/vision_track/vision_track/process_manager.py`
(the `ProcessManager` class, its `__init__`, `start/stop/status/status_all/shutdown_all/start_group/stop_group`, and the `_unknown/_status_locked/_terminate_locked/_signal_group` internals — unchanged, they are exactly what we need), and replace the module-level `REGISTRY`/`GROUPS` constants with empty defaults plus a YAML loader:

```python
# (top of file — same license header + imports as vision_track, PLUS:)
import yaml

# No hardcoded default allowlist here — the dashboard node loads the registry
# from config/processes.yaml via load_registry() and passes it to ProcessManager.
REGISTRY: dict = {}
GROUPS: dict = {}


def load_registry(path):
    """Load {registry, groups, stagger_sec} from a processes.yaml. Returns
    (registry: dict[str,list[str]], groups: dict[str,list[str]], stagger_sec)."""
    with open(path, "r") as f:
        doc = yaml.safe_load(f) or {}
    registry = {str(k): list(v) for k, v in (doc.get("registry") or {}).items()}
    groups = {str(k): list(v) for k, v in (doc.get("groups") or {}).items()}
    stagger = float(doc.get("stagger_sec", 1.5))
    return registry, groups, stagger
```
Keep the `ProcessManager.__init__` signature `def __init__(self, registry=REGISTRY, groups=GROUPS, stagger_sec=1.5)` and the rest of the class **identical** to vision_track's. (Do not import anything ROS — this module stays ROS-free and unit-testable.)

- [ ] **Step 5: Run the test — verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/restaurant_nav_test_web && python -m pytest test/test_process_manager.py -q`
Expected: PASS (3 passed). (Requires `pyyaml`, already in the vision venv; if running with host python and yaml is missing, `pip install pyyaml` or run inside the venv.)

- [ ] **Step 6: Create the prerequisite registry config**

Create `src/restaurant_nav_test_web/config/processes.yaml`. Commands are the real bring-up commands from the launch files (see spec); `<WS>` is substituted by the node at load (Task B3) from the `workspace_root` param. `ROBOT_NAME` and `FASTRTPS_DEFAULT_PROFILES_FILE` are inherited from the launching shell (operator runs `source src/tk25_basic/tools/robot-env.sh` and exports the FASTRTPS profile first).

```yaml
# Allowlist: the dashboard can ONLY spawn these named commands. argv lists are
# never built from request input. <WS> -> workspace root, substituted at load.
stagger_sec: 2.0
registry:
  camera_femto:
    - ros2
    - launch
    - orbbec_camera
    - femto_bolt.launch.py
    - depth_registration:=true
    - enable_ir:=false
    - enable_frame_sync:=false
  camera_realsense:
    - ros2
    - launch
    - realsense2_camera
    - rs_launch.py
    - camera_name:=xarm_camera
    - align_depth.enable:=true
    - config_file:=<WS>/src/tk26_vision/config/realsense_qos.yaml
  pan_tilt:
    - ros2
    - launch
    - pan_tilt
    - pan_tilt.launch.py
    - device:=/dev/ttyUSB0
  waving:
    - ros2
    - launch
    - tk_vision_specialized
    - detect_waving.launch.py
  nav_driver:
    - ros2
    - launch
    - navigation_bringup
    - driver.launch.py
  nav2:
    - ros2
    - launch
    - navigation_bringup
    - bringup_launch.py
    - slam:=false
    - map:=<WS>/src/tk26_navigation/src/navigation_bringup/maps/xlab_2d_0430.map.yaml
    - use_sim_time:=false
    - autostart:=true
  approach:
    - ros2
    - launch
    - approach_planner
    - approach_planner.launch.py
    - use_sim_time:=false
  test_bt:
    - ros2
    - run
    - behavior_tree
    - restaurant-nav-test
groups:
  prereqs:
    - camera_femto
    - pan_tilt
    - waving
    - nav_driver
    - nav2
    - approach
```

- [ ] **Step 7: Build the package**

Run: `cd /home/tinker/tk25_ws && ./src/tk26_vision/scripts/build.sh --packages-select restaurant_nav_test_web && source install/setup.bash`
Expected: build succeeds.

- [ ] **Step 8: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
git rev-parse --abbrev-ref HEAD   # expect: dev
git rev-parse HEAD                 # capture
git add src/restaurant_nav_test_web/package.xml \
        src/restaurant_nav_test_web/setup.py \
        src/restaurant_nav_test_web/resource/restaurant_nav_test_web \
        src/restaurant_nav_test_web/restaurant_nav_test_web/__init__.py \
        src/restaurant_nav_test_web/restaurant_nav_test_web/process_manager.py \
        src/restaurant_nav_test_web/config/processes.yaml \
        src/restaurant_nav_test_web/requirements.txt \
        src/restaurant_nav_test_web/test/test_process_manager.py
git diff --cached --name-only
git commit -m "feat(restaurant_nav_test_web): package scaffold + allowlisted ProcessManager

New tk26_vision package: config-loaded ProcessManager (allowlist of real
prerequisite bring-up commands + test_bt) mirroring vision_track's supervisor.
ROS-free + unit-tested (unknown rejected, start/stop/status, group start).

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log -1 --format='parent=%P'   # must equal the captured HEAD
```

---

## Task B2: FastAPI app factory + test (`tk26_vision`)

**Files:**
- Create: `src/restaurant_nav_test_web/restaurant_nav_test_web/restaurant_nav_test_web_app.py`
- Create: `src/restaurant_nav_test_web/test/test_web_app.py`

- [ ] **Step 1: Write the failing app test**

`src/restaurant_nav_test_web/test/test_web_app.py`:
```python
"""ROS-free tests for the FastAPI factory using a fake bridge."""
from pathlib import Path

import pytest
from fastapi.testclient import TestClient

from restaurant_nav_test_web.restaurant_nav_test_web_app import create_app


class FakeBridge:
    def __init__(self):
        self.started = False
        self.stopped = False
    def snapshot(self):
        return {"state": {"phase": "idle"}, "readiness": {"goto": False},
                "proc": {"camera_femto": {"running": False}}}
    def latest_state(self):
        return 1, {"phase": "scanning", "waver_count": 2}
    def latest_jpeg(self):
        return 0, None
    def start_test(self, mock=False):
        self.started = True
        return {"name": "test_bt", "running": True}
    def stop_test(self):
        self.stopped = True
        return {"name": "test_bt", "running": False}
    def proc_status(self):
        return {"camera_femto": {"running": False}}
    def proc_start(self, name):
        return {"name": name, "running": True}
    def proc_stop(self, name):
        return {"name": name, "running": False}
    def proc_group_start(self, group):
        return [{"name": "camera_femto", "running": True}]
    def proc_group_stop(self, group):
        return [{"name": "camera_femto", "running": False}]


@pytest.fixture
def client(tmp_path):
    webui = tmp_path / "webui"
    webui.mkdir()
    (webui / "index.html").write_text("<html>nav test</html>")
    (webui / "app.js").write_text("// js")
    (webui / "style.css").write_text("/* css */")
    return TestClient(create_app(FakeBridge(), webui_dir=webui))


def test_index_served(client):
    r = client.get("/")
    assert r.status_code == 200 and "nav test" in r.text

def test_status_endpoint(client):
    r = client.get("/api/status")
    assert r.status_code == 200 and r.json()["readiness"] == {"goto": False}

def test_start_and_stop_test(client):
    assert client.post("/api/test/start").json()["running"] is True
    assert client.post("/api/test/stop").json()["running"] is False

def test_proc_group_route_before_name(client):
    # group route must resolve (declared before /{name}) not be shadowed
    r = client.post("/api/proc/group/prereqs/start")
    assert r.status_code == 200 and isinstance(r.json(), list)

def test_proc_named_start(client):
    r = client.post("/api/proc/camera_femto/start")
    assert r.status_code == 200 and r.json()["name"] == "camera_femto"
```

- [ ] **Step 2: Run it — verify it fails**

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/restaurant_nav_test_web && python -m pytest test/test_web_app.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'restaurant_nav_test_web.restaurant_nav_test_web_app'`. (Needs `fastapi` + `httpx` in the env; both in the vision venv. Run inside the venv if the host lacks them: `source /home/tinker/tk25_ws/src/tk26_vision/.venv-vision-main/bin/activate`.)

- [ ] **Step 3: Create the app factory**

Create `src/restaurant_nav_test_web/restaurant_nav_test_web/restaurant_nav_test_web_app.py` by **modelling on** `vision_track/track_web_app.py` (copy its imports, `_NO_CACHE`, the index/style/app.js `FileResponse` routes, the `_mjpeg_gen` heartbeat generator + `/stream.mjpg`, and the exception handler verbatim). Use this exact body for the routes that differ:

```python
"""ROS-free FastAPI factory for the restaurant nav-test dashboard.

Bridge contract (the node implements these):
  snapshot() -> dict                  # {state, readiness, proc}
  latest_state() -> (seq:int, dict|None)
  latest_jpeg() -> (seq:int, bytes|None)
  start_test(mock: bool=False) -> dict
  stop_test() -> dict
  proc_status() -> dict
  proc_start(name) -> dict ; proc_stop(name) -> dict
  proc_group_start(group) -> list ; proc_group_stop(group) -> list
"""
from __future__ import annotations

import asyncio
import json
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse

_NO_CACHE = {"Cache-Control": "no-cache"}
_STATE_POLL_S = 0.05
_MJPEG_POLL_S = 1 / 15


def create_app(bridge, webui_dir: Optional[Path] = None) -> FastAPI:
    app = FastAPI(title="restaurant_nav_test_web")
    webui = Path(webui_dir) if webui_dir else Path(__file__).resolve().parents[1] / "webui"

    @app.exception_handler(Exception)
    async def _unhandled(request, exc):  # noqa: ANN001
        return JSONResponse({"error": f"{type(exc).__name__}: {exc}"}, status_code=500)

    @app.get("/")
    def index():
        return FileResponse(webui / "index.html", media_type="text/html", headers=_NO_CACHE)

    @app.get("/style.css")
    def style():
        return FileResponse(webui / "style.css", media_type="text/css", headers=_NO_CACHE)

    @app.get("/app.js")
    def appjs():
        return FileResponse(webui / "app.js", media_type="application/javascript", headers=_NO_CACHE)

    @app.get("/api/status")
    def status():
        return bridge.snapshot()

    @app.post("/api/test/start")
    def test_start(mock: bool = False):
        return bridge.start_test(mock=mock)

    @app.post("/api/test/stop")
    def test_stop():
        return bridge.stop_test()

    @app.get("/api/proc/status")
    def proc_status():
        return bridge.proc_status()

    # NOTE: group routes BEFORE /{name} routes — FastAPI matches in declaration order.
    @app.post("/api/proc/group/{group}/start")
    def proc_group_start(group: str):
        return bridge.proc_group_start(group)

    @app.post("/api/proc/group/{group}/stop")
    def proc_group_stop(group: str):
        return bridge.proc_group_stop(group)

    @app.post("/api/proc/{name}/start")
    def proc_start(name: str):
        return bridge.proc_start(name)

    @app.post("/api/proc/{name}/stop")
    def proc_stop(name: str):
        return bridge.proc_stop(name)

    @app.websocket("/ws/state")
    async def ws_state(ws: WebSocket):
        await ws.accept()
        last_state_seq = -1
        last_proc = None
        try:
            while True:
                seq, state = bridge.latest_state()
                if state is not None and seq != last_state_seq:
                    last_state_seq = seq
                    await ws.send_text(json.dumps({"type": "state", "data": state}))
                snap = bridge.snapshot()
                proc = {"proc": snap.get("proc"), "readiness": snap.get("readiness")}
                if proc != last_proc:
                    last_proc = proc
                    await ws.send_text(json.dumps({"type": "proc", "data": proc}))
                await asyncio.sleep(_STATE_POLL_S)
        except WebSocketDisconnect:
            return

    async def _mjpeg_gen(request: Request):
        heartbeat_polls = max(1, int(0.5 / _MJPEG_POLL_S))
        last_seq = -1
        idle = 0
        while True:
            if await request.is_disconnected():
                return
            seq, jpeg = bridge.latest_jpeg()
            idle += 1
            fresh = jpeg is not None and seq != last_seq
            if jpeg is not None and (fresh or idle >= heartbeat_polls):
                last_seq = seq
                idle = 0
                yield (b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                       + str(len(jpeg)).encode() + b"\r\n\r\n" + jpeg + b"\r\n")
            await asyncio.sleep(_MJPEG_POLL_S)

    @app.get("/stream.mjpg")
    def stream(request: Request):
        return StreamingResponse(
            _mjpeg_gen(request),
            media_type="multipart/x-mixed-replace; boundary=frame")

    return app
```

- [ ] **Step 4: Run the test — verify it passes**

Run: `cd /home/tinker/tk25_ws/src/tk26_vision/src/restaurant_nav_test_web && python -m pytest test/test_web_app.py -q` (inside the vision venv).
Expected: PASS (5 passed).

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
git rev-parse HEAD   # capture
git add src/restaurant_nav_test_web/restaurant_nav_test_web/restaurant_nav_test_web_app.py \
        src/restaurant_nav_test_web/test/test_web_app.py
git commit -m "feat(restaurant_nav_test_web): ROS-free FastAPI factory + tests

create_app(bridge, webui_dir): index/app.js/css, /api/status, /api/test/start|stop,
/api/proc[/group]/... (group routes before {name}), WS /ws/state, /stream.mjpg
heartbeat MJPEG. Unit-tested with a fake bridge via TestClient.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log -1 --format='parent=%P'   # must equal captured HEAD
```

---

## Task B3: ROS node + bridge + main (`tk26_vision`)

**Files:**
- Create: `src/restaurant_nav_test_web/restaurant_nav_test_web/restaurant_nav_test_web.py`

This node is **modelled on** `vision_track/track_web.py`. Copy its `main()` verbatim (uvicorn in a daemon thread + `MultiThreadedExecutor(num_threads=4)` + the `webui_dir` resolution via `ament_index_python.packages.get_package_share_directory` with a `Path(__file__).resolve().parents[1] / "webui"` fallback + the `server.should_exit` teardown), changing the node class to `RestaurantNavTestWebNode` and the app import to `restaurant_nav_test_web_app.create_app`. Copy its `_on_image` JPEG encoder verbatim. Implement the node body below.

- [ ] **Step 1: Create the node**

Create `src/restaurant_nav_test_web/restaurant_nav_test_web/restaurant_nav_test_web.py`:

```python
"""ROS node + thread-safe bridge for the restaurant nav-test dashboard.
Mirrors vision_track/track_web.py. Subscribes the BT status topic + a color
camera topic (MJPEG), computes live robot->target distance from TF, derives
graph-based readiness, and drives the allowlisted ProcessManager.
"""
from __future__ import annotations

import asyncio
import json
import math
import os
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    import cv2
except Exception:  # noqa: BLE001
    cv2 = None

from restaurant_nav_test_web.process_manager import ProcessManager, load_registry
from restaurant_nav_test_web.restaurant_nav_test_web_app import create_app


def _share_config_path():
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("restaurant_nav_test_web")) / "config" / "processes.yaml"


class RestaurantNavTestWebNode(Node):
    def __init__(self):
        super().__init__("restaurant_nav_test_web")
        self.declare_parameter("bind", "0.0.0.0")
        self.declare_parameter("port", 8768)
        self.declare_parameter("camera_topic", "/camera/color/image_raw")
        self.declare_parameter("status_topic", "/restaurant_nav_test/status")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("workspace_root", "/home/tinker/tk25_ws")
        self.bind_host = str(self.get_parameter("bind").value)
        self.bind_port = int(self.get_parameter("port").value)
        ws = str(self.get_parameter("workspace_root").value)

        self._lock = threading.Lock()
        self._state = None           # latest BT status dict
        self._state_seq = 0
        self._state_ts = 0.0
        self._jpeg = None
        self._jpeg_seq = 0

        # ProcessManager from the allowlist config, with <WS> substituted.
        try:
            registry, groups, stagger = load_registry(str(_share_config_path()))
        except Exception as exc:  # noqa: BLE001 — config missing -> empty allowlist
            self.get_logger().warn(f"processes.yaml load failed: {exc}; empty allowlist")
            registry, groups, stagger = {}, {}, 1.5
        registry = {k: [a.replace("<WS>", ws) for a in argv] for k, argv in registry.items()}
        self.proc = ProcessManager(registry=registry, groups=groups, stagger_sec=stagger)

        cb = ReentrantCallbackGroup()
        self.create_subscription(
            String, str(self.get_parameter("status_topic").value),
            self._on_status, 10, callback_group=cb)
        self.create_subscription(
            Image, str(self.get_parameter("camera_topic").value),
            self._on_image, 1, callback_group=cb)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    # -- subscriptions ----------------------------------------------------
    def _on_status(self, msg: String):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._state = state
            self._state_seq += 1
            self._state_ts = time.time()

    def _on_image(self, msg: Image):
        if cv2 is None:
            return
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        except Exception:  # noqa: BLE001 — non-bgr8 / malformed frame
            return
        if not ok:
            return
        with self._lock:
            self._jpeg = buf.tobytes()
            self._jpeg_seq += 1

    # -- readiness (graph-based, no type deps) ----------------------------
    def _readiness(self) -> dict:
        topics = dict(self.get_topic_names_and_types())
        services = dict(self.get_service_names_and_types())
        cam = str(self.get_parameter("camera_topic").value)
        with self._lock:
            cam_fresh = self._jpeg is not None
        return {
            "camera": cam in topics and cam_fresh,
            "pan_tilt": "/pan_tilt_controller/state" in topics,
            "waving": "/detect_waving_persons" in services,
            "goto": "/go_to_approach/_action/send_goal" in services,
        }

    def _distance_to_target(self):
        with self._lock:
            state = self._state
        if not state or not state.get("target"):
            return None
        tx, ty = state["target"]["x"], state["target"]["y"]
        try:
            t = self._tf_buffer.lookup_transform(
                str(self.get_parameter("target_frame").value),
                str(self.get_parameter("robot_frame").value),
                rclpy.time.Time())
            rx, ry = t.transform.translation.x, t.transform.translation.y
            return round(math.hypot(tx - rx, ty - ry), 2)
        except Exception:  # noqa: BLE001
            return None

    # -- bridge contract --------------------------------------------------
    def snapshot(self):
        with self._lock:
            state, ts = self._state, self._state_ts
        return {
            "state": state,
            "state_age_s": (round(time.time() - ts, 1) if ts else None),
            "distance_m": self._distance_to_target(),
            "readiness": self._readiness(),
            "proc": self.proc.status_all(),
        }

    def latest_state(self):
        with self._lock:
            # fold live distance into the state pushed to the UI
            if self._state is None:
                return self._state_seq, None
            merged = dict(self._state)
            merged["distance_m"] = self._distance_to_target()
            return self._state_seq, merged

    def latest_jpeg(self):
        with self._lock:
            return self._jpeg_seq, self._jpeg

    def start_test(self, mock: bool = False):
        if mock:
            os.environ["BT_MOCK_MODE"] = "true"
        else:
            os.environ.pop("BT_MOCK_MODE", None)
        return self.proc.start("test_bt")

    def stop_test(self):
        return self.proc.stop("test_bt")

    def proc_status(self):
        return self.proc.status_all()

    def proc_start(self, name):
        return self.proc.start(name)

    def proc_stop(self, name):
        return self.proc.stop(name)

    def proc_group_start(self, group):
        return self.proc.start_group(group)

    def proc_group_stop(self, group):
        return self.proc.stop_group(group)


def main():
    os.environ.setdefault("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4")
    rclpy.init()
    node = RestaurantNavTestWebNode()
    try:
        from ament_index_python.packages import get_package_share_directory
        webui_dir = Path(get_package_share_directory("restaurant_nav_test_web")) / "webui"
    except Exception:  # noqa: BLE001
        webui_dir = Path(__file__).resolve().parents[1] / "webui"
    app = create_app(node, webui_dir=webui_dir)

    import uvicorn
    config = uvicorn.Config(app, host=node.bind_host, port=node.bind_port,
                            log_level="info", access_log=False, loop="asyncio")
    server = uvicorn.Server(config)

    def _serve():
        asyncio.run(server.serve())

    thread = threading.Thread(target=_serve, daemon=True, name="uvicorn")
    thread.start()
    node.get_logger().info(
        f"restaurant_nav_test_web on http://{node.bind_host}:{node.bind_port}")

    executor = MultiThreadedExecutor(num_threads=4)
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        server.should_exit = True
        try:
            node.proc.shutdown_all()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        rclpy.try_shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Build (compile + entry point)**

Run: `cd /home/tinker/tk25_ws && ./src/tk26_vision/scripts/build.sh --packages-select restaurant_nav_test_web && source install/setup.bash`
Expected: build succeeds; `ros2 pkg executables restaurant_nav_test_web` lists `restaurant_nav_test_web`.

- [ ] **Step 3: Import smoke**

Run (in the vision venv): `python -c "import restaurant_nav_test_web.restaurant_nav_test_web as m; print('ok', hasattr(m, 'main'))"`
Expected: `ok True`. (The node itself isn't unit-tested — its logic is the bridge contract exercised in B2 + the PM in B1 + the manual run in B5.)

- [ ] **Step 4: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
git rev-parse HEAD   # capture
git add src/restaurant_nav_test_web/restaurant_nav_test_web/restaurant_nav_test_web.py
git commit -m "feat(restaurant_nav_test_web): ROS node + bridge + uvicorn main

Subscribes BT status + color camera (MJPEG), TF robot->target distance,
graph-based readiness (camera/pan_tilt/waving/goto), drives the ProcessManager
(test_bt + prereqs, <WS> substituted, BT_MOCK_MODE toggle). Mirrors track_web.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log -1 --format='parent=%P'   # must equal captured HEAD
```

---

## Task B4: webui (`tk26_vision`)

**Files:**
- Create: `src/restaurant_nav_test_web/webui/index.html`
- Create: `src/restaurant_nav_test_web/webui/app.js`
- Create: `src/restaurant_nav_test_web/webui/style.css`

- [ ] **Step 1: Create index.html**

```html
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Restaurant Nav Test</title>
<link rel="stylesheet" href="/style.css">
</head>
<body>
<header>
  <h1>Restaurant Nav Test</h1>
  <span id="conn" class="badge off">connecting…</span>
</header>
<main>
  <section id="video-pane">
    <div id="video-wrap">
      <img id="video" src="/stream.mjpg" alt="camera">
      <canvas id="overlay"></canvas>
    </div>
    <div id="controls">
      <button id="btn-start">▶ Start test</button>
      <button id="btn-stop">■ Stop test</button>
      <label><input type="checkbox" id="mock"> mock</label>
    </div>
  </section>
  <aside id="rail">
    <div class="panel">
      <h2>Prerequisites <button id="btn-prereqs">▶ Start all</button></h2>
      <ul id="proc-list"></ul>
    </div>
    <div class="panel">
      <h2>Readiness</h2>
      <div id="readiness"></div>
    </div>
    <div class="panel">
      <h2>Status</h2>
      <table id="state-table">
        <tr><td>phase</td><td id="s-phase">—</td></tr>
        <tr><td>wavers</td><td id="s-wavers">—</td></tr>
        <tr><td>target</td><td id="s-target">—</td></tr>
        <tr><td>result</td><td id="s-result">—</td></tr>
        <tr><td>distance</td><td id="s-distance">—</td></tr>
      </table>
    </div>
    <div class="panel"><h2>Log</h2><ul id="log"></ul></div>
  </aside>
</main>
<script src="/app.js"></script>
</body>
</html>
```

- [ ] **Step 2: Create app.js**

```javascript
"use strict";
const $ = (id) => document.getElementById(id);
const PROCS = ["camera_femto", "pan_tilt", "waving", "nav_driver", "nav2", "approach"];
const READY = ["camera", "pan_tilt", "waving", "goto"];

function log(msg) {
  const li = document.createElement("li");
  li.innerHTML = `<b>${new Date().toLocaleTimeString()}</b> ${msg}`;
  $("log").prepend(li);
  while ($("log").children.length > 50) $("log").lastChild.remove();
}

async function post(url) {
  try { return await (await fetch(url, { method: "POST" })).json(); }
  catch (e) { return { message: `request failed: ${e}` }; }
}

function renderState(s) {
  $("s-phase").textContent = s.phase ?? "—";
  $("s-wavers").textContent = s.waver_count ?? "—";
  $("s-target").textContent = s.target ? `(${s.target.x.toFixed(2)}, ${s.target.y.toFixed(2)})` : "—";
  $("s-result").textContent = s.result ?? "—";
  $("s-distance").textContent = (s.distance_m != null) ? `${s.distance_m} m` : "—";
  drawOverlay(s);
}

function renderProc(data) {
  const proc = data.proc || {};
  $("proc-list").innerHTML = "";
  for (const name of PROCS) {
    const st = proc[name] || { running: false };
    const li = document.createElement("li");
    const dot = st.running ? "●" : "○";
    li.innerHTML = `<span class="${st.running ? 'on' : 'off'}">${dot}</span> ${name}
      <button data-proc="${name}" data-act="${st.running ? 'stop' : 'start'}">${st.running ? 'stop' : 'start'}</button>`;
    $("proc-list").appendChild(li);
  }
  $("proc-list").querySelectorAll("button").forEach((b) => {
    b.onclick = async () =>
      log(`${b.dataset.proc} ${b.dataset.act} → ` +
          JSON.stringify(await post(`/api/proc/${b.dataset.proc}/${b.dataset.act}`)));
  });
  const r = data.readiness || {};
  $("readiness").innerHTML = READY.map(
    (k) => `<span class="${r[k] ? 'on' : 'off'}">${r[k] ? '●' : '○'} ${k}</span>`).join("  ");
}

function drawOverlay(s) {
  const img = $("video"), cv = $("overlay");
  if (!img.naturalWidth) return;
  cv.width = img.clientWidth; cv.height = img.clientHeight;
  const ctx = cv.getContext("2d");
  ctx.clearRect(0, 0, cv.width, cv.height);
  // Map-frame points can't be drawn on the image without intrinsics; show count.
  ctx.fillStyle = "#0f0"; ctx.font = "16px monospace";
  ctx.fillText(`wavers: ${s.waver_count ?? 0}`, 8, 20);
}

function connectWS() {
  const ws = new WebSocket(`ws://${location.host}/ws/state`);
  ws.onopen = () => { $("conn").textContent = "live"; $("conn").className = "badge on"; };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === "state") renderState(msg.data);
    if (msg.type === "proc") renderProc(msg.data);
  };
  ws.onclose = () => {
    $("conn").textContent = "reconnecting…"; $("conn").className = "badge off";
    setTimeout(connectWS, 1500);
  };
}

$("btn-start").onclick = async () =>
  log("start test → " + JSON.stringify(await post(`/api/test/start?mock=${$("mock").checked}`)));
$("btn-stop").onclick = async () => log("stop test → " + JSON.stringify(await post("/api/test/stop")));
$("btn-prereqs").onclick = async () =>
  log("start all → " + JSON.stringify(await post("/api/proc/group/prereqs/start")));

connectWS();
```

- [ ] **Step 3: Create style.css**

```css
* { box-sizing: border-box; }
body { margin: 0; font-family: system-ui, sans-serif; background: #111; color: #ddd; }
header { display: flex; align-items: center; gap: 1rem; padding: .5rem 1rem; background: #1b1b1b; }
h1 { font-size: 1.1rem; margin: 0; }
.badge { padding: .1rem .5rem; border-radius: 4px; font-size: .8rem; }
.badge.on { background: #163; color: #afa; } .badge.off { background: #533; color: #fbb; }
main { display: flex; gap: 1rem; padding: 1rem; }
#video-pane { flex: 2; }
#video-wrap { position: relative; }
#video { width: 100%; background: #000; border-radius: 6px; display: block; }
#overlay { position: absolute; left: 0; top: 0; pointer-events: none; }
#controls { margin-top: .5rem; display: flex; gap: .5rem; align-items: center; }
#rail { flex: 1; display: flex; flex-direction: column; gap: 1rem; }
.panel { background: #1b1b1b; border-radius: 6px; padding: .5rem .8rem; }
.panel h2 { font-size: .9rem; margin: .2rem 0 .5rem; display: flex; justify-content: space-between; align-items: center; }
#proc-list, #log { list-style: none; margin: 0; padding: 0; font-size: .85rem; }
#proc-list li { display: flex; align-items: center; gap: .4rem; padding: .15rem 0; }
#proc-list button, header button, #controls button { background: #2a2a2a; color: #ddd; border: 1px solid #444; border-radius: 4px; padding: .15rem .5rem; cursor: pointer; }
.on { color: #6f6; } .off { color: #d66; }
table { width: 100%; font-size: .85rem; } td:first-child { color: #999; }
#log li { border-top: 1px solid #2a2a2a; padding: .2rem 0; }
#readiness span { margin-right: .6rem; }
```

- [ ] **Step 4: Rebuild (installs webui to share/) + serve smoke**

Run:
```bash
cd /home/tinker/tk25_ws && ./src/tk26_vision/scripts/build.sh --packages-select restaurant_nav_test_web && source install/setup.bash
ls install/restaurant_nav_test_web/share/restaurant_nav_test_web/webui/
```
Expected: `app.js  index.html  style.css` listed (data_files installed the webui). Also re-run B2's app test to confirm index still serves: `python -m pytest src/restaurant_nav_test_web/test/test_web_app.py -q` → PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
git rev-parse HEAD   # capture
git add src/restaurant_nav_test_web/webui/index.html \
        src/restaurant_nav_test_web/webui/app.js \
        src/restaurant_nav_test_web/webui/style.css
git commit -m "feat(restaurant_nav_test_web): webui (video + prereqs + readiness + status + log)

track_web-style dashboard: MJPEG pane, per-process prereq toggles + Start-all,
graph-readiness dots, status panel (phase/wavers/target/result/distance), log;
WS state + reconnect + fetch-POST controls.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log -1 --format='parent=%P'   # must equal captured HEAD
```

---

## Task B5: launch file + README + final build (`tk26_vision`)

**Files:**
- Create: `src/restaurant_nav_test_web/launch/restaurant_nav_test_web.launch.py`
- Create: `src/restaurant_nav_test_web/README.md`

- [ ] **Step 1: Create the launch file**

`src/restaurant_nav_test_web/launch/restaurant_nav_test_web.launch.py`:
```python
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Camera frames arrive at 30 Hz only if the SHM FastDDS profile is set on the
# SUBSCRIBER too (this node subscribes the color topic for MJPEG). Set it here.
_FASTRTPS = '/home/tinker/tk25_ws/src/tk26_vision/config/fastdds_shm.xml'


def generate_launch_description():
    bind = LaunchConfiguration('bind')
    port = LaunchConfiguration('port')
    camera_topic = LaunchConfiguration('camera_topic')
    return LaunchDescription([
        DeclareLaunchArgument('bind', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8768'),
        DeclareLaunchArgument('camera_topic', default_value='/camera/color/image_raw'),
        SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                               os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', _FASTRTPS)),
        Node(
            package='restaurant_nav_test_web',
            executable='restaurant_nav_test_web',
            output='screen',
            parameters=[{
                'bind': ParameterValue(bind, value_type=str),
                'port': ParameterValue(port, value_type=int),
                'camera_topic': ParameterValue(camera_topic, value_type=str),
            }],
        ),
    ])
```

- [ ] **Step 2: Create the README**

`src/restaurant_nav_test_web/README.md` — a user guide with: what it is (restaurant pure-nav test dashboard), prerequisites (`source src/tk25_basic/tools/robot-env.sh`, export `FASTRTPS_DEFAULT_PROFILES_FILE`), how to launch (`ros2 launch restaurant_nav_test_web restaurant_nav_test_web.launch.py` → `http://<host>:8768`), the workflow (Start-all prerequisites → watch readiness → Start test → scan→approach→result), the camera_topic note (femto `/camera/color/image_raw` vs realsense `/camera/xarm_camera/color/image_raw`), the mock checkbox, and an **append-only Changelog** with a `2026-06-14` first entry describing this feature. (Free-form prose; mirror the structure of `vision_track`'s README if present.)

- [ ] **Step 3: Final whole-package build + checks**

Run:
```bash
cd /home/tinker/tk25_ws && ./src/tk26_vision/scripts/build.sh --packages-select restaurant_nav_test_web && source install/setup.bash
ls install/restaurant_nav_test_web/share/restaurant_nav_test_web/launch/
python -m pytest src/restaurant_nav_test_web/test/ -q   # in the vision venv
```
Expected: launch file installed; all package tests pass (process_manager + web_app).

- [ ] **Step 4: Commit**

```bash
cd /home/tinker/tk25_ws/src/tk26_vision
git rev-parse HEAD   # capture
git add src/restaurant_nav_test_web/launch/restaurant_nav_test_web.launch.py \
        src/restaurant_nav_test_web/README.md
git commit -m "feat(restaurant_nav_test_web): launch file + README

Launch sets the SHM FastDDS profile so the MJPEG camera subscription hits 30 Hz;
README documents bring-up (robot-env + FASTRTPS), the dashboard workflow, and a
changelog. Completes the restaurant nav-test dashboard.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
git log -1 --format='parent=%P'   # must equal captured HEAD
```

- [ ] **Step 5: Manual end-to-end (operator, not automated)**

Documented in the README; not a blocking step for the plan:
```bash
source /home/tinker/tk25_ws/src/tk25_basic/tools/robot-env.sh
export FASTRTPS_DEFAULT_PROFILES_FILE=$(pwd)/src/tk26_vision/config/fastdds_shm.xml
ros2 launch restaurant_nav_test_web restaurant_nav_test_web.launch.py
# open http://<host>:8768 → Start all prerequisites → readiness green → Start test
```

---

## Self-Review

**Spec coverage:** BT tree composed from test_scan sweep + approach + status leaf, OneShot, `restaurant-nav-test` entry (A1) ✓; sweep `[0,-60,+60]` (A1 constant + test) ✓; nearest-to-robot select with TF fallback (A1) ✓; new web package in tk26_vision mirroring track_web (B1–B5) ✓; allowlisted ProcessManager seeded from processes.yaml with the real bring-up commands + `prereqs` group + `test_bt` (B1) ✓; FastAPI bridge with status/test/proc/group routes + MJPEG + WS (B2) ✓; ROS node: status + camera subs, JPEG, TF distance, graph readiness, `<WS>` substitution, BT_MOCK_MODE toggle (B3) ✓; webui with video + prereq toggles + Start-all + readiness + status + log (B4) ✓; launch sets FASTRTPS, README + changelog (B5) ✓; error handling (no_wavers via FAILURE→status; goto status via result branch; proc returncode in status; camera/TF absent → heartbeat/`—`) ✓; tests (PM, app factory, BT pure helper) ✓; non-goals respected (one-shot via OneShot, no gating, no per-frame re-detect, no node edits) ✓.

**Placeholder scan:** all code steps contain full code; the README (B5 Step 2) is prose-by-spec with an explicit required content list (not a code artifact) — acceptable. The node + app + PM are copy-from-track_web with the exact differing code shown in full. No TBD/TODO.

**Type/name consistency:** `restaurant_nav_test_web` (pkg + node module + entry), `RestaurantNavTestWebNode`, `create_app(bridge, webui_dir)`, bridge methods (`snapshot/latest_state/latest_jpeg/start_test/stop_test/proc_status/proc_start/proc_stop/proc_group_start/proc_group_stop`) match between B2 (fake bridge + routes) and B3 (node). `KEY_CUSTOMER_CENTROIDS`/`KEY_APPROACH_TARGET`/`STATUS_TOPIC` consistent in A1. `nearest_index`/`SWEEP_PANS`/`TILT_DEG` consistent between `nav_test_select.py`, the tree, and the test. Status JSON keys (`phase/result/waver_count/wavers/target`) match between the BT publisher (A1) and the UI renderer (B4). `processes.yaml` group `prereqs` matches the webui Start-all + the fake-bridge test. `test_bt` registry key matches `start_test`/`stop_test`.
