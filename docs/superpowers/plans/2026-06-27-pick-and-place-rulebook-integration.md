# Pick-and-Place Rulebook Integration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire `arm_api.scan_and_place_server` into the `tk25_decision` PickAndPlace behavior tree, restructure the tree to the RoboCup@Home 2026 Pick-and-Place Challenge (three main goals), and extend the server for rulebook use — all smoke-testable in mock with no robot.

**Architecture:** The place server gains 3 placement modes (FREE_SPACE / NEAR_SIMILAR / FIXED_POINT) + scan-pose control + a `dry_run`, with its pure math/prompt logic extracted to `placement_logic.py`. The BT gets a new `BtNode_ScanAndPlace` action node, inventory/queue/guard/deadline plain-`Behaviour` nodes, and a data-driven rulebook tree (inventory→queue→per-item loop, then breakfast, then extra-surface) under a `--place-policy {hardcoded,vlm}` switch with first-class whole-tree mock.

**Tech Stack:** ROS2 Humble, `py_trees` / `py_trees_ros`, `rclpy`, `numpy`, `pytest`; `tinker_arm_msgs` (ROS action), `arm_api` (ament_python), `behavior_tree` (ament_python).

**Spec:** `docs/superpowers/specs/2026-06-27-pick-and-place-rulebook-integration-design.md`

## Global Constraints

- **NEVER** raw `colcon build` — use `tkbuild <ws>`. Pure-python tests run with `python3 -m pytest` (no `pytest` console script is installed) — no build.
- `tinker_arm_msgs` + `arm_api` are in the COLCON_IGNORE'd `tk25_manipulation` tree → build via the sentinel dance (Task A1). No hardware needed for builds.
- **No real robot:** every test runs in mock (`BT_MOCK_MODE=true`, interaction `IMMEDIATE`) or as pure `pytest`. On-robot validation is later.
- **Branches:** `tk25_decision` = `dev`; `tk25_manipulation` = its working branch. Concurrent committers → selective `git add <path>`, commit new only, **NEVER** `-A` / `--amend` / rebase.
- **Every commit** ends with the two trailers:
  ```
  Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
  Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
  ```
- **`place_policy` default = `vlm`.** Bonus goals (dishwasher door/rack, tablet-in-slot, pour, milk-open) are **OUT OF SCOPE** — no nodes.
- **Build/dependency order:** `tinker_arm_msgs` → `arm_api` → `behavior_tree`.
- **Reuse** existing `config.py` keys (`KEY_POINT_BREAKFAST_*`, `KEY_ACTIVE_SOURCE_POSE`, `KEY_BREAKFAST_QUEUE`, `KEY_VISION_RESULT`, `KEY_OBJECT_LABEL`, `KEY_SCORE_TRACE`, `KEY_POINT_WASH_STAGING`, `KEY_POINT_CABINET_DEFAULT`, …) — never redefine.

## Frozen Interface Contract

Every task uses these exact names/signatures/paths. Per-task **Interfaces** blocks restate the relevant slice; this is the canonical index for out-of-order reading.

**Message `tinker_arm_msgs/action/ScanAndPlace`** — Goal: `item_description, margin_m, orientation, max_candidates, placement_mode(uint8: 0 FREE_SPACE/1 NEAR_SIMILAR/2 FIXED_POINT), reference_label, fixed_target(PointStamped), scan_pose_deg(float32[]), skip_scan_move(bool), dry_run(bool)`; Result: `status, placed_at, error_msg, placement_mode_used(uint8)`; Feedback: `stage`.

**`arm_api/arm_api/placement_logic.py`** (pure, numpy): `PLACEMENT_MODE_FREE_SPACE=0/_NEAR_SIMILAR=1/_FIXED_POINT=2`; `resolve_margin_m(margin_m, default_margin_m)`; `decode_bbox(box_norm, img_w, img_h)`; `depth_image_to_meters(data, encoding, height, width)`; `bbox_centroid_3d(depth_m, K, bbox)`; `short_side_m(bbox, depth_m, K)`; `select_candidate(boxes, depth_m, K, margin_m)`; `build_vlm_messages(mode, item_description, reference_label, max_candidates)`.

**`behavior_tree` messages:** add `ScanAndPlace` to `messages.py` real branch + `mock_messages.py` stub; nodes import `from behavior_tree.messages import ScanAndPlace`.

**`PickAndPlace/categorization.py`** (pure): `Destination = namedtuple('Destination',['klass','reference_label'])` (`klass ∈ {wash_staging,trash,cabinet}`); `classify_destination(label, *, cutlery, tableware, trash, category_map)`.

**`TemplateNodes/Manipulation.py`:** `BtNode_ScanAndPlace(ActionHandler)` overriding `send_goal()`/`process_result()`, `action_name=SCAN_AND_PLACE_ACTION_NAME`.

**`PickAndPlace/custom_nodes.py`:** `BtNode_BuildInventory(in_key, out_inventory, out_queue, source_pose_key, mock_seed)`; `BtNode_PopWorkItem(queue, place_policy)`; `BtNode_DeadlineGuard(budget_sec, clock)`; `BtNode_GuardActiveClass(expected, key)`; `BtNode_MarkPhase(phase, key)`; `record_event(blackboard, phase, item, action, outcome, points_est=0)`. Score-trace: `{'visited_phases':[], 'events':[...], 'place_policy':str}`.

**`PickAndPlace/config.py` (new):** `SCAN_AND_PLACE_ACTION_NAME`, `PLACEMENT_MODE_*` (+`_NONE=255`), `DESTINATION_ROUTING` (4-tuple `(nav_pose, arm_scan_pose, vlm_mode, hardcoded_point)`), `KEY_POSE_EXTRA_SURFACE`/`KEY_POINT_EXTRA_SURFACE`, `POSE_/POINT_EXTRA_SURFACE`, `CATEGORY_MAP`, `TABLE_/BREAKFAST_/EXTRA_BUDGET_SEC`.

**`PickAndPlace/pick_and_place_rulebook.py` (new):** `pickAndPlaceRulebook(place_policy='vlm')`; phase/helper factories per the spec §8 composites. Root: `Sequence(memory)[ createConstantWriter(place_policy), phaseEnterArena(), Parallel(SuccessOnOne)[ DeadlineGuard(MAX_RUNTIME_SEC), FailureIsSuccess(missionPhases(place_policy)) ], phaseSummary() ]`. Cleanup loop: `Repeat(num_success=-1)[ Sequence(memory)[ PopWorkItem (UNWRAPPED), FailureIsSuccess(handleOneItem) ] ]`.

**`cli.py`** repoint + `--place-policy` (default `vlm`); **`cli_demo.py`** runs old `pickAndPlaceShortened`; **`samplings.py`**: `main_scan_place/main_categorize/main_cleanup_loop/main_breakfast`. **`setup.py`** entries: `pick-and-place`(→rulebook), `pick-and-place-demo`, `pp-test-scan-place/-categorize/-cleanup-loop/-breakfast`. **`mock_config.json`**: register `BtNode_ScanAndPlace` only.

---

## Phase A — Message + place server (`tinker_arm_msgs`, `arm_api`)

### Task A1: Extend `ScanAndPlace.action` with the new Goal fields + `placement_mode_used`, rebuild `tinker_arm_msgs`, verify the generated message

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_manipulation/src/tinker_arm_msgs/action/ScanAndPlace.action` (append Goal block lines 12→ after; append one Result field after line 26)
- (No CMake change — `action/ScanAndPlace.action` is already registered at `tinker_arm_msgs/CMakeLists.txt:36`.)

**Interfaces (FROZEN — exact):**
- Produces message `tinker_arm_msgs/action/ScanAndPlace`:
  - Goal: `string item_description; float32 margin_m; geometry_msgs/Quaternion orientation; uint8 max_candidates; uint8 placement_mode; string reference_label; geometry_msgs/PointStamped fixed_target; float32[] scan_pose_deg; bool skip_scan_move; bool dry_run`
  - Result: `int32 status; geometry_msgs/PointStamped placed_at; string error_msg; uint8 placement_mode_used`
  - Feedback: `string stage` (unchanged)
- `placement_mode`: 0 FREE_SPACE, 1 NEAR_SIMILAR, 2 FIXED_POINT. **Append-only; zero callers → safe.**

Steps:

1. [ ] **Write the failing verify first (it will fail against the currently-installed message).** Run, expecting `AttributeError`/FAIL because the installed `.action` has no `placement_mode`:
   ```bash
   source /home/tinker/tk25_ws/install/setup.zsh
   python3 -c "from tinker_arm_msgs.action import ScanAndPlace; g=ScanAndPlace.Goal(); assert hasattr(g,'placement_mode'), 'no placement_mode yet'"
   ```
   Expected: `AssertionError: no placement_mode yet` (or the old generated class simply lacks the attr).

2. [ ] **Edit the `.action`** — append the six Goal fields after the existing `uint8 max_candidates` (current line 12), and append `placement_mode_used` after the existing Result `string error_msg` (current line 26). Final file content:
   ```
   # ============ Goal ============
   # Free-form description of the item currently in the gripper, e.g. "a coke can ~6 cm wide".
   # Server forwards to the VLM with an appended clearance hint.
   string item_description
   # Desired clearance margin around the object on the table, in metres.
   # Server clamps to [0.05, 0.10] (the "moderate" band); 0 => use server default (0.07 m).
   float32 margin_m
   # Optional placement orientation; if the quaternion's norm is < 0.5 the server
   # uses a top-down default (TCP +z pointing DOWN in base_link).
   geometry_msgs/Quaternion orientation
   # Max VLM candidates to request. 0 => server default (5); else clamped to [1, 10].
   uint8 max_candidates
   # Placement strategy: 0 FREE_SPACE (legacy/default) | 1 NEAR_SIMILAR | 2 FIXED_POINT.
   uint8 placement_mode
   # NEAR_SIMILAR (1): category/label to place the item beside. Empty => degrade to FREE_SPACE.
   string reference_label
   # FIXED_POINT (2): explicit place point (TF'd to base_link); VLM is skipped.
   geometry_msgs/PointStamped fixed_target
   # Optional 7-joint scan-pose override (degrees). Empty => server param default.
   float32[] scan_pose_deg
   # true => BT has already positioned the arm; capture without moving.
   bool skip_scan_move
   # true => skip camera/VLM/place; return a canned placed_at (server-side mock).
   bool dry_run

   ---

   # ============ Result ============
   #   0 = placed (object released on the chosen spot, arm retreated)
   #   1 = no viable candidate (VLM returned none, or all failed the margin filter)
   #   2 = placement plan failed (Place action did not succeed)
   #   3 = scan/move failed (could not reach the scan pose / no camera frame)
   #  -1 = hard error (e.g. missing OPENROUTER_API_KEY, VLM client exception)
   int32 status
   # Where the object was actually placed, in base_link. Zeroed on failure.
   geometry_msgs/PointStamped placed_at
   # Human-readable error / explanation. Empty on success.
   string error_msg
   # Echoes the mode actually executed; NEAR_SIMILAR (1) may degrade to FREE_SPACE (0).
   uint8 placement_mode_used

   ---

   # ============ Feedback ============
   # Coarse progress tag: "scanning" | "querying_vlm" | "filtering" | "placing".
   string stage
   ```

3. [ ] **Build `tinker_arm_msgs` via the COLCON_IGNORE sentinel dance.** Builds are user-approved — if the implementer is unsure of the exact `tkbuild` form, ask the user to run this; Claude may run `tkbuild` + read-only checks. (Note: as of this writing the *root* sentinel `src/tk25_manipulation/COLCON_IGNORE` is **absent** — only `.venv_*/COLCON_IGNORE` exist; the `mv` of the root sentinel is a no-op if the file is missing, which is fine. Run the dance verbatim so it works whether or not the sentinel is present.)
   ```bash
   [ -f /home/tinker/tk25_ws/src/tk25_manipulation/COLCON_IGNORE ] && mv /home/tinker/tk25_ws/src/tk25_manipulation/COLCON_IGNORE /tmp/_mani_ci
   cd /home/tinker/tk25_ws && tkbuild tk25_manipulation --packages-select tinker_arm_msgs
   [ -f /tmp/_mani_ci ] && mv /tmp/_mani_ci /home/tinker/tk25_ws/src/tk25_manipulation/COLCON_IGNORE
   source /home/tinker/tk25_ws/install/setup.zsh
   ```
   NEVER raw `colcon build`.

4. [ ] **Run the verify (now expecting PASS):**
   ```bash
   source /home/tinker/tk25_ws/install/setup.zsh
   python3 -c "from tinker_arm_msgs.action import ScanAndPlace; \
   g=ScanAndPlace.Goal(); \
   assert all(hasattr(g,a) for a in ('item_description','margin_m','orientation','max_candidates','placement_mode','reference_label','fixed_target','scan_pose_deg','skip_scan_move','dry_run')), 'Goal missing field'; \
   r=ScanAndPlace.Result(); \
   assert all(hasattr(r,a) for a in ('status','placed_at','error_msg','placement_mode_used')), 'Result missing field'; \
   f=ScanAndPlace.Feedback(); assert hasattr(f,'stage'); \
   print('A1 OK')"
   ```
   Expected output: `A1 OK`.

5. [ ] **Commit (manip tree, branch `dev`, selective add only):**
   ```bash
   git -C /home/tinker/tk25_ws/src/tk25_manipulation add src/tinker_arm_msgs/action/ScanAndPlace.action
   git -C /home/tinker/tk25_ws/src/tk25_manipulation commit -m "$(cat <<'EOF'
   feat(tinker_arm_msgs): append rulebook fields to ScanAndPlace.action

   Append-only Goal fields (placement_mode, reference_label, fixed_target,
   scan_pose_deg, skip_scan_move, dry_run) + Result.placement_mode_used.
   Zero callers; placement_mode=0 + zeros reproduces legacy behavior.

   Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
   Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
   EOF
   )"
   ```

**Deliverable:** rebuilt `tinker_arm_msgs/action/ScanAndPlace` whose Goal/Result expose every new field; `A1 OK` verify passes. (Layer C in A3 depends on this rebuild.)

---

### Task A2: Create `arm_api/arm_api/placement_logic.py` (pure, numpy-only) + `arm_api/test/test_placement_logic.py`

**Files:**
- Create `/home/tinker/tk25_ws/src/tk25_manipulation/src/arm_api/arm_api/placement_logic.py`
- Create `/home/tinker/tk25_ws/src/tk25_manipulation/src/arm_api/test/test_placement_logic.py`

**Interfaces (FROZEN — exact):**
```
PLACEMENT_MODE_FREE_SPACE=0; PLACEMENT_MODE_NEAR_SIMILAR=1; PLACEMENT_MODE_FIXED_POINT=2
resolve_margin_m(margin_m: float, default_margin_m: float) -> float        # 0->default, clamp [0.05,0.10]
decode_bbox(box_norm, img_w, img_h) -> (x0,y0,x1,y1) | None                # box=[ymin,xmin,ymax,xmax] norm 0..1000
depth_image_to_meters(data, encoding: str, height: int, width: int) -> np.ndarray  # 16UC1/mono16 *0.001; 32FC1 passthrough; np.frombuffer (no CvBridge)
bbox_centroid_3d(depth_m: np.ndarray, K: tuple, bbox: tuple) -> (x,y,z) | None   # camera coords or None
short_side_m(bbox: tuple, depth_m: np.ndarray, K: tuple) -> float
select_candidate(boxes: list, depth_m: np.ndarray, K: tuple, margin_m: float) -> (point|None, reason:str)
build_vlm_messages(mode: int, item_description: str, reference_label: str, max_candidates: int) -> list[dict]  # NEAR_SIMILAR injects reference_label
```
- `decode_bbox`/`short_side_m` extracted verbatim from `scan_and_place_server.py:118` / `:431`. `resolve_margin_m`/`depth_image_to_meters`/`bbox_centroid_3d` refactored to pure signatures (no `self`, no CvBridge, K as tuple).
- **Pure pytest, NO build.** Module needs only `numpy`. Consumed by A3's server refactor.

Steps:

1. [ ] **Write the failing test first** — `arm_api/test/test_placement_logic.py` (matches the existing `test/test_fold_geometry.py` style: numpy-only, top-level `from arm_api.<mod> import`):
   ```python
   """Unit tests for the pure placement-logic module (no ROS, numpy only)."""
   import numpy as np
   import pytest

   from arm_api.placement_logic import (
       PLACEMENT_MODE_FREE_SPACE,
       PLACEMENT_MODE_NEAR_SIMILAR,
       PLACEMENT_MODE_FIXED_POINT,
       resolve_margin_m,
       decode_bbox,
       depth_image_to_meters,
       bbox_centroid_3d,
       short_side_m,
       select_candidate,
       build_vlm_messages,
   )

   # K: fx=fy=100, cx=cy=5 (the 9-element row-major camera matrix as a tuple).
   K = (100.0, 0.0, 5.0, 0.0, 100.0, 5.0, 0.0, 0.0, 1.0)


   def test_mode_constants():
       assert (PLACEMENT_MODE_FREE_SPACE, PLACEMENT_MODE_NEAR_SIMILAR,
               PLACEMENT_MODE_FIXED_POINT) == (0, 1, 2)


   def test_resolve_margin_clamp():
       assert resolve_margin_m(0.0, 0.07) == pytest.approx(0.07)   # 0 -> default
       assert resolve_margin_m(0.20, 0.07) == pytest.approx(0.10)  # clamp high
       assert resolve_margin_m(0.01, 0.07) == pytest.approx(0.05)  # clamp low
       assert resolve_margin_m(0.08, 0.07) == pytest.approx(0.08)  # passthrough
       assert resolve_margin_m(0.0, 0.20) == pytest.approx(0.10)   # default also clamped


   def test_decode_bbox_basic_and_swap_and_reject():
       # box = [ymin, xmin, ymax, xmax] normalized 0..1000
       assert decode_bbox([0, 0, 1000, 1000], 100, 200) == (0, 0, 99, 199)
       # reversed coords get swapped to a valid box
       assert decode_bbox([1000, 1000, 0, 0], 100, 200) == (0, 0, 99, 199)
       # degenerate (zero area) -> None
       assert decode_bbox([500, 500, 500, 500], 100, 200) is None
       # malformed input -> None
       assert decode_bbox("nope", 100, 200) is None
       assert decode_bbox([1, 2, 3], 100, 200) is None


   def test_depth_uint16_scaled_vs_float_passthrough():
       raw16 = np.array([[1000, 2000], [3000, 0]], dtype=np.uint16)
       out = depth_image_to_meters(raw16.tobytes(), "16UC1", 2, 2)
       assert out.dtype == np.float32
       np.testing.assert_allclose(out, [[1.0, 2.0], [3.0, 0.0]])
       # mono16 alias
       out2 = depth_image_to_meters(raw16.tobytes(), "mono16", 2, 2)
       np.testing.assert_allclose(out2, [[1.0, 2.0], [3.0, 0.0]])
       # 32FC1 passthrough (NOT scaled)
       rawf = np.array([[0.5, 1.5], [2.5, 0.0]], dtype=np.float32)
       outf = depth_image_to_meters(rawf.tobytes(), "32FC1", 2, 2)
       np.testing.assert_allclose(outf, [[0.5, 1.5], [2.5, 0.0]])


   def test_bbox_centroid_backproject():
       depth = np.full((10, 10), 2.0, dtype=np.float32)
       # centered bbox -> u=v=5=cx=cy -> x=y=0, z=median=2.0
       assert bbox_centroid_3d(depth, K, (0, 0, 10, 10)) == pytest.approx((0.0, 0.0, 2.0))
       # off-center bbox (2,2,4,4): u=v=3 -> x=y=(3-5)*2/100=-0.04
       p = bbox_centroid_3d(depth, K, (2, 2, 4, 4))
       assert p == pytest.approx((-0.04, -0.04, 2.0))
       # all-invalid depth -> None
       bad = np.zeros((10, 10), dtype=np.float32)
       assert bbox_centroid_3d(bad, K, (0, 0, 10, 10)) is None


   def test_short_side_and_reject_in_select():
       depth = np.full((40, 40), 2.0, dtype=np.float32)
       big = (0, 0, 10, 10)      # short_px=10 -> 10*2/100 = 0.20 m
       small = (20, 20, 22, 22)  # short_px=2  -> 2*2/100  = 0.04 m
       assert short_side_m(big, depth, K) == pytest.approx(0.20)
       assert short_side_m(small, depth, K) == pytest.approx(0.04)
       # margin 0.05 -> 2*margin=0.10: small (0.04) rejected, big (0.20) accepted
       pt, reason = select_candidate([small, big], depth, K, 0.05)
       assert reason == "ok"
       assert pt == pytest.approx(bbox_centroid_3d(big, K, big))


   def test_select_candidate_order_and_empty():
       depth = np.full((40, 40), 2.0, dtype=np.float32)
       a = (0, 0, 10, 10)
       b = (20, 20, 32, 32)
       # both valid -> first in list wins
       pt, reason = select_candidate([a, b], depth, K, 0.05)
       assert reason == "ok"
       assert pt == pytest.approx(bbox_centroid_3d(depth, K, a))
       # empty list
       assert select_candidate([], depth, K, 0.05) == (None, "no candidates")
       # all rejected by margin
       tiny = (0, 0, 2, 2)
       pt2, reason2 = select_candidate([tiny], depth, K, 0.05)
       assert pt2 is None and "margin" in reason2


   def test_build_vlm_messages_reference_label_iff_near_similar():
       def flat(msgs):
           parts = []
           for m in msgs:
               c = m["content"]
               if isinstance(c, str):
                   parts.append(c)
               else:
                   parts.extend(seg.get("text", "") for seg in c)
           return " ".join(parts)

       free = build_vlm_messages(PLACEMENT_MODE_FREE_SPACE, "a coke can", "cups", 5)
       near = build_vlm_messages(PLACEMENT_MODE_NEAR_SIMILAR, "a coke can", "cups", 5)
       fixed = build_vlm_messages(PLACEMENT_MODE_FIXED_POINT, "a coke can", "cups", 5)
       assert "cups" not in flat(free)
       assert "cups" in flat(near)
       assert "cups" not in flat(fixed)
       # well-formed: a system msg + a user msg with a text segment
       assert any(m["role"] == "system" for m in near)
       user = [m for m in near if m["role"] == "user"][0]
       assert any(seg.get("type") == "text" for seg in user["content"])
   ```

2. [ ] **Run the test, expecting FAIL** (module does not exist yet):
   ```bash
   cd /home/tinker/tk25_ws/src/tk25_manipulation/src/arm_api && python3 -m pytest test/test_placement_logic.py -q
   ```
   Expected: collection/import error `ModuleNotFoundError: No module named 'arm_api.placement_logic'`.

3. [ ] **Write `arm_api/arm_api/placement_logic.py`:**
   ```python
   """Pure, ROS-free placement geometry + VLM-prompt helpers for ScanAndPlace.

   Extracted from scan_and_place_server.py so the placement math and prompt
   building are unit-testable with no ROS node, no CvBridge, no camera, no
   VLM/network. numpy is the only dependency. The server keeps all I/O
   (subs/TF/action-clients/OpenAI) and imports these functions.
   """
   from typing import List, Optional, Tuple

   import numpy as np

   PLACEMENT_MODE_FREE_SPACE = 0
   PLACEMENT_MODE_NEAR_SIMILAR = 1
   PLACEMENT_MODE_FIXED_POINT = 2


   def resolve_margin_m(margin_m: float, default_margin_m: float) -> float:
       """0 => default; result clamped to the moderate band [0.05, 0.10] m."""
       m = float(margin_m)
       if m == 0.0:
           m = float(default_margin_m)
       return max(0.05, min(0.10, m))


   def decode_bbox(box_norm, img_w: int, img_h: int) -> Optional[Tuple[int, int, int, int]]:
       """Decode a VLM box [ymin, xmin, ymax, xmax] (0..1000 normalized) to
       pixel (x0, y0, x1, y1); None if malformed or zero-area. Verbatim from
       scan_and_place_server._decode_bbox."""
       if not isinstance(box_norm, (list, tuple)) or len(box_norm) < 4:
           return None
       try:
           y0, x0, y1, x1 = (float(box_norm[i]) for i in range(4))
       except (TypeError, ValueError):
           return None
       if y1 < y0:
           y0, y1 = y1, y0
       if x1 < x0:
           x0, x1 = x1, x0
       px1 = max(0, min(int(round(x0 * img_w / 1000.0)), img_w - 1))
       py1 = max(0, min(int(round(y0 * img_h / 1000.0)), img_h - 1))
       px2 = max(0, min(int(round(x1 * img_w / 1000.0)), img_w - 1))
       py2 = max(0, min(int(round(y1 * img_h / 1000.0)), img_h - 1))
       if px2 <= px1 or py2 <= py1:
           return None
       return (px1, py1, px2, py2)


   def depth_image_to_meters(data, encoding: str, height: int, width: int) -> np.ndarray:
       """Decode a depth image buffer to float32 metres with np.frombuffer
       (no CvBridge). 16UC1/mono16 => uint16 * 0.001; 32FC1 => passthrough."""
       enc = (encoding or "").lower()
       if enc in ("16uc1", "mono16"):
           arr = np.frombuffer(data, dtype=np.uint16).reshape(height, width)
           return arr.astype(np.float32) * 0.001
       if enc in ("32fc1", "32f"):
           arr = np.frombuffer(data, dtype=np.float32).reshape(height, width)
           return arr.astype(np.float32)
       raise ValueError(f"unsupported depth encoding: {encoding!r}")


   def bbox_centroid_3d(depth_m: np.ndarray, K: tuple, bbox: Tuple[int, int, int, int]
                        ) -> Optional[Tuple[float, float, float]]:
       """Back-project a bbox's median-depth centre to camera-frame (x, y, z);
       None if no finite positive depth in the ROI. K is the 9-element row-major
       camera matrix (fx=K[0], fy=K[4], cx=K[2], cy=K[5])."""
       x1, y1, x2, y2 = bbox
       roi = depth_m[y1:y2, x1:x2]
       valid = np.isfinite(roi) & (roi > 0.0)
       if not np.any(valid):
           return None
       z = float(np.median(roi[valid]))
       if not (z > 0.0):
           return None
       fx = float(K[0])
       fy = float(K[4])
       cx = float(K[2])
       cy = float(K[5])
       u = (x1 + x2) * 0.5
       v = (y1 + y2) * 0.5
       x_cam = (u - cx) * z / fx
       y_cam = (v - cy) * z / fy
       return (float(x_cam), float(y_cam), float(z))


   def short_side_m(bbox: Tuple[int, int, int, int], depth_m: np.ndarray, K: tuple) -> float:
       """Metric length of the bbox short side at its median depth; 0.0 if no
       valid depth. (Server's _short_side_m, refactored to take depth_m + K and
       derive z internally rather than receiving fx, z.)"""
       x1, y1, x2, y2 = bbox
       roi = depth_m[y1:y2, x1:x2]
       valid = np.isfinite(roi) & (roi > 0.0)
       if not np.any(valid):
           return 0.0
       z = float(np.median(roi[valid]))
       fx = float(K[0])
       short_px = float(min(x2 - x1, y2 - y1))
       return short_px * z / fx


   def select_candidate(boxes: list, depth_m: np.ndarray, K: tuple, margin_m: float
                        ) -> Tuple[Optional[Tuple[float, float, float]], str]:
       """Pick the first geometrically-valid candidate: back-projects, requires
       short_side >= 2*margin. Returns (camera-frame point | None, reason)."""
       if not boxes:
           return None, "no candidates"
       for bbox in boxes:
           point = bbox_centroid_3d(depth_m, K, bbox)
           if point is None:
               continue
           if short_side_m(bbox, depth_m, K) < 2.0 * float(margin_m):
               continue
           return point, "ok"
       return None, f"no candidate satisfied {margin_m * 100:.0f} cm margin"


   def _system_prompt(mode: int, reference_label: str, max_candidates: int) -> str:
       base = (
           "You are helping a service robot place an item on a desktop or "
           "tabletop visible in the image. Identify clear, flat, unoccupied "
           "regions on the desktop large enough to fit the item described by "
           "the user. Rank them best (#1) to worst. Best = (a) clearly empty, "
           "(b) flat, (c) safely away from edges and existing objects, "
           "(d) large enough for the item with margin. "
           f"Return up to {max_candidates} regions as bounding boxes "
           "[ymin, xmin, ymax, xmax] normalized to 0-1000, where (0,0) is the "
           "top-left of the image. Each box must enclose only empty surface, "
           "not surrounding objects. The label field must be 'rank1', 'rank2', "
           "... matching the rank position. If no suitable region exists, "
           "return an empty detections list. Never return masks, depth, or 3D "
           "coordinates."
       )
       if mode == PLACEMENT_MODE_NEAR_SIMILAR and reference_label:
           base += (
               f" Prefer empty regions immediately adjacent to existing "
               f"'{reference_label}' items so the new item is grouped beside them."
           )
       return base


   def build_vlm_messages(mode: int, item_description: str, reference_label: str,
                          max_candidates: int) -> List[dict]:
       """OpenAI-style chat messages (system + user text). The server inserts the
       image_url segment into the user content before sending. NEAR_SIMILAR is the
       only mode that injects reference_label into the prompt text."""
       max_candidates = max(1, min(int(max_candidates), 10))
       system = _system_prompt(mode, reference_label, max_candidates)
       if mode == PLACEMENT_MODE_NEAR_SIMILAR and reference_label:
           user_text = (
               f"Item to place: {item_description}. Place it in empty surface "
               f"immediately adjacent to existing '{reference_label}' items, "
               "leaving clearance on all sides."
           )
       else:
           user_text = (
               f"Item to place: {item_description}; leave clearance on all sides."
           )
       return [
           {"role": "system", "content": system},
           {"role": "user", "content": [{"type": "text", "text": user_text}]},
       ]
   ```

4. [ ] **Run the test, expecting PASS:**
   ```bash
   cd /home/tinker/tk25_ws/src/tk25_manipulation/src/arm_api && python3 -m pytest test/test_placement_logic.py -q
   ```
   Expected: all tests pass (`8 passed`). NO colcon build — pure pytest, `arm_api` importable because cwd is the package root containing the `arm_api/` dir (same as the existing fold tests).

5. [ ] **Commit (manip tree, branch `dev`, selective add):**
   ```bash
   git -C /home/tinker/tk25_ws/src/tk25_manipulation add \
     src/arm_api/arm_api/placement_logic.py \
     src/arm_api/test/test_placement_logic.py
   git -C /home/tinker/tk25_ws/src/tk25_manipulation commit -m "$(cat <<'EOF'
   feat(arm_api): pure placement_logic module + unit tests

   Extract decode_bbox/short_side_m verbatim and refactor
   resolve_margin_m/depth_image_to_meters/bbox_centroid_3d to pure,
   ROS-free signatures (no self, no CvBridge, K as tuple). Add
   select_candidate + build_vlm_messages (NEAR_SIMILAR injects
   reference_label). numpy-only; pytest hermetic, no build/ROS/VLM.

   Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
   Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
   EOF
   )"
   ```

**Deliverable:** `arm_api.placement_logic` importable; `test_placement_logic.py` green under plain pytest with no build, no ROS, no network.

---

### Task A3: Refactor `scan_and_place_server.py` to import `placement_logic`, add the 3 placement modes + scan-pose control + `dry_run` + top-down orientation default; smoke-test `dry_run` → status 0

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_manipulation/src/arm_api/arm_api/scan_and_place_server.py`
  - import: line 55 area
  - default_orientation_xyzw param: line 165
  - `_resolve_margin_m` (lines 273-276), `_depth_image_to_meters` (397-401), `_bbox_centroid_3d` (403-429), `_short_side_m` (431-436): replace bodies to delegate to `placement_logic`
  - `_call_vlm` (300-395): take `mode`/`reference_label`, build messages via `placement_logic.build_vlm_messages`, decode via `placement_logic.decode_bbox`
  - `_execute_cb` (536-703): branch on `placement_mode`; honor `skip_scan_move`/`scan_pose_deg`/`dry_run`; FIXED_POINT skips VLM; NEAR_SIMILAR degrades; set `result.placement_mode_used`
  - add `_dry_run_result` + `_fixed_point_place` helpers
- Create `/home/tinker/tk25_ws/scratchpad/scan_place_dryrun_client.py` (smoke client; ephemeral)

**Interfaces (FROZEN — exact):**
- Consumes `arm_api.placement_logic.{resolve_margin_m, depth_image_to_meters, bbox_centroid_3d, short_side_m, decode_bbox, build_vlm_messages, PLACEMENT_MODE_*}` and `tinker_arm_msgs/action/ScanAndPlace` (A1).
- Node param `default_orientation_xyzw` default = a real top-down (TCP +z down) quaternion with a `TODO(hardware)` note.
- Entry/action name unchanged: action server on `'scan_and_place_action'`, entry `scan_and_place_action = arm_api.scan_and_place_server:main`.
- **Depends on A1 rebuild** (new Goal/Result fields) for the live smoke; the static import-edit does not.

Steps:

1. [ ] **Replace the import line** to add the pure module. At `scan_and_place_server.py:55`:
   ```python
   from tinker_arm_msgs.action import JointMove, Place, ScanAndPlace

   from arm_api import placement_logic
   ```

2. [ ] **Fix the orientation default** (line 165) to a real TCP-+z-down quaternion (180° about base_link X maps +z→−z): replace
   ```python
           # quaternion (x, y, z, w) used when the goal orientation has norm^2 < 0.5
           self.declare_parameter("default_orientation_xyzw", [0.0, 0.0, 0.0, 1.0])
   ```
   with
   ```python
           # Quaternion (x, y, z, w) used when the goal orientation has norm^2 < 0.5.
           # Top-down: 180 deg about base_link +X maps TCP +z to point DOWN.
           # TODO(hardware): refine against the real TCP/flange frame on the robot.
           self.declare_parameter("default_orientation_xyzw", [1.0, 0.0, 0.0, 0.0])
   ```

3. [ ] **Delegate the four helpers to `placement_logic`.** Replace `_resolve_margin_m` (273-276):
   ```python
       def _resolve_margin_m(self, m: float) -> float:
           return placement_logic.resolve_margin_m(
               m, float(self.get_parameter("default_margin_m").value)
           )
   ```
   Replace `_depth_image_to_meters` (397-401):
   ```python
       def _depth_image_to_meters(self, depth_msg: Image) -> np.ndarray:
           return placement_logic.depth_image_to_meters(
               depth_msg.data, depth_msg.encoding, depth_msg.height, depth_msg.width
           )
   ```
   Replace `_bbox_centroid_3d` (403-429) — return a `Point` (server still uses geometry_msgs downstream), wrapping the pure tuple:
   ```python
       def _bbox_centroid_3d(
           self,
           depth_m: np.ndarray,
           info: CameraInfo,
           bbox: Tuple[int, int, int, int],
       ) -> Tuple[Optional[Point], float]:
           xyz = placement_logic.bbox_centroid_3d(depth_m, tuple(info.k), bbox)
           if xyz is None:
               return None, 0.0
           p = Point()
           p.x, p.y, p.z = float(xyz[0]), float(xyz[1]), float(xyz[2])
           return p, float(xyz[2])
   ```
   Replace `_short_side_m` (431-436) to delegate (signature changes to depth_m + info):
   ```python
       def _short_side_m(
           self,
           bbox: Tuple[int, int, int, int],
           depth_m: np.ndarray,
           info: CameraInfo,
       ) -> float:
           return placement_logic.short_side_m(bbox, depth_m, tuple(info.k))
   ```
   And update the one caller in the Stage-3 loop (currently `scan_and_place_server.py:650`) from `self._short_side_m(bbox, fx, z)` to:
   ```python
               if self._short_side_m(bbox, depth_m, info_msg) < 2.0 * margin_m:
   ```
   (The local `fx` at line 640 is now unused for short-side; keep `fx` only if still referenced elsewhere — it is not after this change, so delete the `fx = float(info_msg.k[0])` line at 640.)

4. [ ] **Refactor `_call_vlm`** (300-395) to take `mode`/`reference_label` and build messages from the pure module + decode via `placement_logic.decode_bbox`. Change the signature and the messages/decoder; keep the OpenAI call + retry/schema logic intact:
   - Signature:
     ```python
       def _call_vlm(
           self,
           rgb_bgr: np.ndarray,
           item_description: str,
           margin_m: float,
           max_candidates: int,
           mode: int,
           reference_label: str,
       ) -> List[Tuple[int, int, int, int]]:
     ```
   - Replace the inline `messages = [...]` block (316-325) with:
     ```python
           messages = placement_logic.build_vlm_messages(
               mode, item_description, reference_label, max_candidates
           )
           for m in messages:
               if m["role"] == "user":
                   m["content"].insert(
                       0, {"type": "image_url", "image_url": {"url": data_url}}
                   )
                   break
     ```
     (Drop the now-unused `margin_cm`/`item_text` locals at 311-315.)
   - Replace the decode call (356) `decoded = _decode_bbox(det.get("box_2d"), w, h)` with `decoded = placement_logic.decode_bbox(det.get("box_2d"), w, h)`.
   - The module-level `_decode_bbox` (118-135) and `_system_prompt` (65-80) are now superseded by `placement_logic`; leave `_decode_bbox`/`_system_prompt` in place (harmless) **or** delete them — deletion is cleaner; if deleted, confirm no other reference (grep shows only the two now-updated call sites).

5. [ ] **Add the `dry_run` + FIXED_POINT helpers** (insert after `_zeroed_result`, ~line 259):
   ```python
       def _dry_run_result(self, goal_handle, request, mode: int) -> "ScanAndPlace.Result":
           """Server-side mock: emit the feedback stages, return a canned
           placed_at + status=0. Touches no camera/VLM/TF/action-client."""
           for stage in ("scanning", "querying_vlm", "filtering", "placing"):
               self._publish_feedback(goal_handle, stage)
           result = self._zeroed_result()
           if (mode == placement_logic.PLACEMENT_MODE_FIXED_POINT
                   and request.fixed_target.header.frame_id):
               result.placed_at = request.fixed_target
           else:
               ps = PointStamped()
               ps.header.frame_id = self.get_parameter("target_frame").value
               ps.point.x, ps.point.y, ps.point.z = 0.5, 0.0, 0.75
               result.placed_at = ps
           result.status = 0
           result.error_msg = ""
           result.placement_mode_used = mode
           goal_handle.succeed()
           return result
   ```

6. [ ] **Rewrite `_execute_cb`** (536-703) to branch on the mode and honor the new fields. Full replacement:
   ```python
       async def _execute_cb(self, goal_handle):
           request = goal_handle.request
           result = self._zeroed_result()
           mode = int(request.placement_mode)
           result.placement_mode_used = mode

           margin_m = self._resolve_margin_m(float(request.margin_m))
           max_cand = int(request.max_candidates) or 5
           max_cand = max(1, min(max_cand, 10))
           orient_out = self._resolve_orientation(request.orientation)

           # ---- dry_run: server-side mock, no I/O ----
           if request.dry_run:
               return self._dry_run_result(goal_handle, request, mode)

           # Stage 1 — optional move to scan pose.
           self._publish_feedback(goal_handle, "scanning")
           if not request.skip_scan_move:
               scan_pose = list(request.scan_pose_deg) or list(
                   self.get_parameter("scan_pose_deg").value
               )
               if len(scan_pose) != 7:
                   self.get_logger().error(
                       f"scan_pose_deg must have 7 entries, got {len(scan_pose)}; "
                       "falling back to default"
                   )
                   scan_pose = list(DEFAULT_SCAN_POSE_DEG)
               if not await self._send_joint_move(scan_pose, add_octomap=True):
                   goal_handle.abort()
                   result.status = 3
                   result.error_msg = "joint_move_action to scan pose failed"
                   return result

           # ---- FIXED_POINT (2): skip VLM/camera; place at fixed_target ----
           if mode == placement_logic.PLACEMENT_MODE_FIXED_POINT:
               return await self._execute_fixed_point(goal_handle, request, orient_out)

           settle_s = float(self.get_parameter("vision_settle_s").value)
           if settle_s > 0:
               time.sleep(settle_s)

           wait_s = float(self.get_parameter("frame_wait_s").value)
           deadline = time.time() + wait_s
           while time.time() < deadline:
               with self._frame_lock:
                   have = (
                       self._latest_rgb is not None
                       and self._latest_depth is not None
                       and self._latest_info is not None
                   )
               if have:
                   break
               time.sleep(0.05)

           with self._frame_lock:
               rgb_msg = self._latest_rgb
               depth_msg = self._latest_depth
               info_msg = self._latest_info
               env_pc = self._latest_env_pc

           if rgb_msg is None or depth_msg is None or info_msg is None:
               goal_handle.abort()
               result.status = 3
               result.error_msg = (
                   "Missing camera data: "
                   f"rgb={'ok' if rgb_msg else 'MISSING'} "
                   f"depth={'ok' if depth_msg else 'MISSING'} "
                   f"info={'ok' if info_msg else 'MISSING'}"
               )
               return result
           if env_pc is None:
               self.get_logger().warning(
                   "env_points topic empty; Place will run without octomap update"
               )

           try:
               rgb_bgr = self._bridge.imgmsg_to_cv2(
                   rgb_msg, desired_encoding="bgr8"
               )
           except Exception as exc:  # noqa: BLE001
               goal_handle.abort()
               result.status = -1
               result.error_msg = f"cv_bridge RGB decode failed: {exc}"
               return result

           # ---- NEAR_SIMILAR (1) degrade: empty reference => FREE_SPACE ----
           eff_mode = mode
           degrade_note = ""
           ref_label = request.reference_label or ""
           if mode == placement_logic.PLACEMENT_MODE_NEAR_SIMILAR and not ref_label:
               eff_mode = placement_logic.PLACEMENT_MODE_FREE_SPACE
               degrade_note = "NEAR_SIMILAR with empty reference_label; FREE_SPACE"

           # Stage 2 — call the VLM.
           self._publish_feedback(goal_handle, "querying_vlm")
           try:
               boxes = self._call_vlm(
                   rgb_bgr, request.item_description, margin_m, max_cand,
                   eff_mode, ref_label,
               )
           except RuntimeError as exc:
               goal_handle.abort()
               result.status = -1
               result.error_msg = str(exc)
               return result
           except Exception as exc:  # noqa: BLE001
               goal_handle.abort()
               result.status = -1
               result.error_msg = f"VLM call exception: {exc}"
               return result

           # NEAR_SIMILAR returned nothing => retry once as FREE_SPACE (degrade).
           if (not boxes
                   and mode == placement_logic.PLACEMENT_MODE_NEAR_SIMILAR
                   and eff_mode == placement_logic.PLACEMENT_MODE_NEAR_SIMILAR):
               eff_mode = placement_logic.PLACEMENT_MODE_FREE_SPACE
               degrade_note = "NEAR_SIMILAR found no adjacent region; FREE_SPACE"
               try:
                   boxes = self._call_vlm(
                       rgb_bgr, request.item_description, margin_m, max_cand,
                       eff_mode, ref_label,
                   )
               except Exception as exc:  # noqa: BLE001
                   goal_handle.abort()
                   result.status = -1
                   result.error_msg = f"VLM degrade call exception: {exc}"
                   return result

           if eff_mode != mode:
               result.placement_mode_used = eff_mode

           if not boxes:
               goal_handle.abort()
               result.status = 1
               result.error_msg = "VLM returned no usable candidates"
               return result

           # Stage 3 — back-project, margin-filter, pick the best.
           self._publish_feedback(goal_handle, "filtering")
           try:
               depth_m = self._depth_image_to_meters(depth_msg)
           except Exception as exc:  # noqa: BLE001
               goal_handle.abort()
               result.status = -1
               result.error_msg = f"depth decode failed: {exc}"
               return result

           cam_frame = (
               self.get_parameter("camera_optical_frame").value
               or depth_msg.header.frame_id
           )
           chosen_ps: Optional[PointStamped] = None
           for bbox in boxes:
               point_cam, z = self._bbox_centroid_3d(depth_m, info_msg, bbox)
               if point_cam is None:
                   continue
               if self._short_side_m(bbox, depth_m, info_msg) < 2.0 * margin_m:
                   self.get_logger().info(
                       f"bbox {bbox} short-side < 2*margin ({margin_m:.3f} m); skipping"
                   )
                   continue
               ps = self._transform_to_target(
                   point_cam, cam_frame, depth_msg.header.stamp
               )
               if ps is None:
                   continue
               chosen_ps = ps
               self.get_logger().info(
                   f"chose bbox {bbox} -> base_link "
                   f"({ps.point.x:.3f}, {ps.point.y:.3f}, {ps.point.z:.3f})"
               )
               break

           if chosen_ps is None:
               goal_handle.abort()
               result.status = 1
               result.error_msg = (
                   f"No VLM candidate satisfied {margin_m * 100:.0f} cm margin"
               )
               return result

           chosen_ps.point.z += float(self.get_parameter("place_z_lift").value)
           self._publish_place_pose(chosen_ps, orient_out)

           # Stage 4 — place.
           self._publish_feedback(goal_handle, "placing")
           if env_pc is None:
               env_pc = PointCloud2()
               env_pc.header.frame_id = chosen_ps.header.frame_id
               env_pc.header.stamp = self.get_clock().now().to_msg()
           ok, err = await self._send_place(chosen_ps, orient_out, env_pc)
           if not ok:
               goal_handle.abort()
               result.status = 2
               result.error_msg = f"place failed: {err}"
               return result

           goal_handle.succeed()
           result.status = 0
           result.placed_at = chosen_ps
           result.error_msg = degrade_note
           return result
   ```
   Add the FIXED_POINT executor + the small rviz-publish helper (factored out of the old inline block at 679-684), inserted before `_execute_cb`:
   ```python
       def _publish_place_pose(self, ps: PointStamped, orient: Quaternion) -> None:
           rviz_pose = PoseStamped()
           rviz_pose.header = ps.header
           rviz_pose.header.stamp = self.get_clock().now().to_msg()
           rviz_pose.pose.position = ps.point
           rviz_pose.pose.orientation = orient
           self._place_pose_pub.publish(rviz_pose)

       async def _execute_fixed_point(self, goal_handle, request, orient_out):
           """FIXED_POINT: TF fixed_target -> base_link, lift, place. No VLM."""
           result = self._zeroed_result()
           result.placement_mode_used = placement_logic.PLACEMENT_MODE_FIXED_POINT
           ft = request.fixed_target
           if not ft.header.frame_id:
               goal_handle.abort()
               result.status = 3
               result.error_msg = "FIXED_POINT requires fixed_target with a frame_id"
               return result
           target = self.get_parameter("target_frame").value
           if ft.header.frame_id == target:
               chosen_ps = ft
           else:
               chosen_ps = self._transform_to_target(
                   ft.point, ft.header.frame_id, ft.header.stamp
               )
               if chosen_ps is None:
                   goal_handle.abort()
                   result.status = 3
                   result.error_msg = (
                       f"TF {ft.header.frame_id}->{target} failed for fixed_target"
                   )
                   return result
           chosen_ps.point.z += float(self.get_parameter("place_z_lift").value)
           self._publish_place_pose(chosen_ps, orient_out)
           with self._frame_lock:
               env_pc = self._latest_env_pc
           if env_pc is None:
               env_pc = PointCloud2()
               env_pc.header.frame_id = chosen_ps.header.frame_id
               env_pc.header.stamp = self.get_clock().now().to_msg()
           self._publish_feedback(goal_handle, "placing")
           ok, err = await self._send_place(chosen_ps, orient_out, env_pc)
           if not ok:
               goal_handle.abort()
               result.status = 2
               result.error_msg = f"place failed: {err}"
               return result
           goal_handle.succeed()
           result.status = 0
           result.placed_at = chosen_ps
           result.error_msg = ""
           return result
   ```

7. [ ] **Write the dry_run smoke client** at `/home/tinker/tk25_ws/scratchpad/scan_place_dryrun_client.py`:
   ```python
   """One-shot dry_run smoke client for scan_and_place_action."""
   import sys
   import rclpy
   from rclpy.action import ActionClient
   from rclpy.node import Node
   from tinker_arm_msgs.action import ScanAndPlace


   class C(Node):
       def __init__(self):
           super().__init__("scan_place_dryrun_client")
           self.cli = ActionClient(self, ScanAndPlace, "scan_and_place_action")

       def run(self):
           if not self.cli.wait_for_server(timeout_sec=10.0):
               print("FAIL: server not available")
               return 2
           goal = ScanAndPlace.Goal()
           goal.item_description = "smoke test can"
           goal.dry_run = True
           goal.placement_mode = 0
           sf = self.cli.send_goal_async(goal)
           rclpy.spin_until_future_complete(self, sf)
           gh = sf.result()
           if gh is None or not gh.accepted:
               print("FAIL: goal rejected")
               return 2
           rf = gh.get_result_async()
           rclpy.spin_until_future_complete(self, rf)
           res = rf.result().result
           print(f"status={res.status} mode_used={res.placement_mode_used} "
                 f"placed_at=({res.placed_at.point.x:.2f},"
                 f"{res.placed_at.point.y:.2f},{res.placed_at.point.z:.2f})")
           return 0 if res.status == 0 else 1


   def main():
       rclpy.init()
       n = C()
       rc = n.run()
       n.destroy_node()
       rclpy.shutdown()
       print("DRY_RUN SMOKE OK" if rc == 0 else "DRY_RUN SMOKE FAIL")
       sys.exit(rc)


   if __name__ == "__main__":
       main()
   ```

8. [ ] **Build `arm_api` (after the A1 msg rebuild) and run the smoke.** Builds are user-approved — ask the user to run the build, or Claude runs the `tkbuild` dance + read-only smoke:

   _Live action-server smoke (no hardware / VLM / API key; needs the A1 message + A3 arm_api rebuilds + a sourced env). Not a pytest — skip without failing the task if a live node can't be launched here._

   ```bash
   [ -f /home/tinker/tk25_ws/src/tk25_manipulation/COLCON_IGNORE ] && mv /home/tinker/tk25_ws/src/tk25_manipulation/COLCON_IGNORE /tmp/_mani_ci
   cd /home/tinker/tk25_ws && tkbuild tk25_manipulation --packages-select arm_api
   [ -f /tmp/_mani_ci ] && mv /tmp/_mani_ci /home/tinker/tk25_ws/src/tk25_manipulation/COLCON_IGNORE
   source /home/tinker/tk25_ws/install/setup.zsh
   ```
   Then, in terminal 1 start the server (no hardware/key needed — `dry_run` touches no camera/VLM/place):
   ```bash
   source /home/tinker/tk25_ws/install/setup.zsh && ros2 run arm_api scan_and_place_action
   ```
   In terminal 2 run the client (expected FAIL before this task: the server has no `dry_run` branch / message lacks the field; expected PASS after):
   ```bash
   source /home/tinker/tk25_ws/install/setup.zsh && python3 /home/tinker/tk25_ws/scratchpad/scan_place_dryrun_client.py
   ```
   Expected output: `status=0 mode_used=0 placed_at=(0.50,0.00,0.75)` then `DRY_RUN SMOKE OK` (exit 0).
   (If running the live node is not possible in this environment, the equivalent callback-level check is: import the server module, instantiate is ROS-bound, so prefer the `ros2 run` smoke; note this step **depends on the A1 rebuild**.)

9. [ ] **Commit (manip tree, branch `dev`, selective add — do NOT add the scratchpad client):**
   ```bash
   git -C /home/tinker/tk25_ws/src/tk25_manipulation add src/arm_api/arm_api/scan_and_place_server.py
   git -C /home/tinker/tk25_ws/src/tk25_manipulation commit -m "$(cat <<'EOF'
   feat(arm_api): ScanAndPlace modes, scan-pose control, dry_run, top-down default

   Import placement_logic (pure helpers). _execute_cb branches on
   placement_mode: FIXED_POINT skips VLM and places goal.fixed_target;
   NEAR_SIMILAR degrades to FREE_SPACE (echoed in placement_mode_used);
   honor skip_scan_move / scan_pose_deg / dry_run. default_orientation_xyzw
   now a real TCP-+z-down quaternion (TODO hardware).

   Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
   Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
   EOF
   )"
   ```

**Deliverable:** `scan_and_place_action` server with the three placement modes, scan-pose control, and a working `dry_run` returning `status=0`/`placement_mode_used` with no hardware, VLM, or OpenRouter key — verified by the `dry_run` smoke (depends on the A1 message rebuild).

> Note: `placement_logic.select_candidate` is unit-tested in A2, but the server keeps its inline TF-interleaved selection loop; `select_candidate` is therefore pure-test coverage / an optional future refactor — not dead code by mistake.

---

## Phase B — BT plumbing, categorization, config, nodes (`behavior_tree`)

### Task B1: Add `ScanAndPlace` to `messages.py` (real + mock branch) and a `ScanAndPlace` stub to `mock_messages.py`

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/messages.py` (line 24 real arm branch; line 31 mock arm branch)
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/mock_messages.py` (append a `ScanAndPlace(MockAction)` class near the other arm-action mocks, after `CartesianMove` ~line 296)
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_scan_and_place_msg_import.py`

**Interfaces:**
- Produces: `behavior_tree.messages.ScanAndPlace` (resolves to `tinker_arm_msgs.action.ScanAndPlace` when arm msgs present, else `mock_messages.ScanAndPlace`). Mock stub mirrors the FROZEN CONTRACT: Goal `{item_description, margin_m, orientation, max_candidates, placement_mode, reference_label, fixed_target, scan_pose_deg, skip_scan_move, dry_run}`, Result `{status, placed_at, error_msg, placement_mode_used}`, Feedback `{stage}`.
- Note: installed `tinker_arm_msgs` already ships the legacy `ScanAndPlace.action` (4 Goal fields) — verified — so the real-branch import resolves immediately, before Phase A extends the `.action`. Phase B nodes import `from behavior_tree.messages import ScanAndPlace`.

Steps:

1. [ ] Write the failing test `test/test_scan_and_place_msg_import.py`:
```python
import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 — force mock before config loads

import importlib  # noqa: E402


def test_messages_exposes_scan_and_place():
    messages = importlib.import_module("behavior_tree.messages")
    assert hasattr(messages, "ScanAndPlace")
    goal = messages.ScanAndPlace.Goal()
    for field in (
        "item_description", "margin_m", "orientation", "max_candidates",
        "placement_mode", "reference_label", "fixed_target",
        "scan_pose_deg", "skip_scan_move", "dry_run",
    ):
        assert hasattr(goal, field), field
    result = messages.ScanAndPlace.Result()
    for field in ("status", "placed_at", "error_msg", "placement_mode_used"):
        assert hasattr(result, field), field
    fb = messages.ScanAndPlace.Feedback()
    assert hasattr(fb, "stage")


def test_mock_messages_stub_present():
    mock = importlib.import_module("behavior_tree.mock_messages")
    assert hasattr(mock, "ScanAndPlace")
    g = mock.ScanAndPlace.Goal()
    assert g.placement_mode == 0
    assert g.scan_pose_deg == []
    assert g.skip_scan_move is False and g.dry_run is False
```

2. [ ] Run it (expect FAIL — `ScanAndPlace` not yet importable from `behavior_tree.mock_messages`, and `messages` mock branch doesn't import it):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_scan_and_place_msg_import.py -q
```
Expected: `AttributeError: module 'behavior_tree.mock_messages' has no attribute 'ScanAndPlace'` (test_mock fails; test_messages fails on the mock-branch import).

3. [ ] Edit `messages.py` real arm branch (line 24) — add `ScanAndPlace`:
```python
    from tinker_arm_msgs.action import Place, Grasp, JointMove, CartesianMove, ScanAndPlace
```

4. [ ] Edit `messages.py` mock arm branch (line 31) — add `ScanAndPlace`:
```python
    from behavior_tree.mock_messages import Place, Grasp, JointMove, CartesianMove, ScanAndPlace
```

5. [ ] **(hardening, per spec §17)** Make the `tinker_audio_msgs.srv` import in `messages.py` (line 35) defensive so a present-but-incomplete `tinker_audio_msgs` (the known `TTSCnRequest` export hazard) falls back to the mock srv classes instead of failing the entire `behavior_tree.messages` import. Wrap the single srv-import line in a `try/except ImportError`:
```python
if _config.has_dependency('tinker_audio_msgs'):
    try:
        from tinker_audio_msgs.srv import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
    except ImportError:
        # tinker_audio_msgs present but missing a symbol (TTSCnRequest hazard) ->
        # fall back to the mock srvs. Does NOT reproduce on this host; harmless.
        from behavior_tree.mock_messages import TTSCnRequest, TextToSpeech, WaitForStart, PhraseExtraction, GetConfirmation, Listen, CompareInterest, QuestionAnswer, GraspRequest
    from tinker_audio_msgs.action import GetConfirmation as GetConfirmationAction, Listen as ListenAction
    from tinker_audio_msgs.action import PhraseExtraction as PhraseExtractionAction
    from tinker_audio_msgs.action import Doorbell
```
Leave the `action` imports and the `else:` mock branch unchanged. Minimal hardening only.

6. [ ] Append the stub to `mock_messages.py` (after the `CartesianMove` class, ~line 296):
```python
class ScanAndPlace(MockAction):
    """Mock ScanAndPlace action (arm_api scan_and_place_server).

    Mirrors tinker_arm_msgs/action/ScanAndPlace after the rulebook append-only
    extension: placement_mode 0 FREE_SPACE / 1 NEAR_SIMILAR / 2 FIXED_POINT,
    plus reference_label / fixed_target / scan_pose_deg / skip_scan_move /
    dry_run, and Result.placement_mode_used. Lets the BT assemble + tick the
    goal on a host where tinker_arm_msgs is absent.
    """
    class Goal(MockAction.Goal):
        def __init__(self):
            super().__init__()
            self.item_description = ""
            self.margin_m = 0.0
            self.orientation = None
            self.max_candidates = 0
            self.placement_mode = 0
            self.reference_label = ""
            self.fixed_target = None
            self.scan_pose_deg = []
            self.skip_scan_move = False
            self.dry_run = False

    class Result(MockAction.Result):
        def __init__(self):
            super().__init__()
            self.status = 0
            self.placed_at = None
            self.error_msg = ""
            self.placement_mode_used = 0

    class Feedback(MockAction.Feedback):
        def __init__(self):
            super().__init__()
            self.stage = ""
```

7. [ ] Run the test (expect PASS) — and additionally confirm the real branch resolves:
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_scan_and_place_msg_import.py -q && \
python -c "import behavior_tree.messages as m; print('real-branch ScanAndPlace:', m.ScanAndPlace)"
```
Expected: 2 passed; the real-branch print shows `<class 'tinker_arm_msgs.action._scan_and_place.ScanAndPlace'>` (arm msgs are present on this host).

8. [ ] Commit (selective add; branch `dev`; no `-A`, no `--amend`):
```
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/messages.py \
        src/behavior_tree/behavior_tree/mock_messages.py \
        src/behavior_tree/test/test_scan_and_place_msg_import.py && \
git commit -m "$(cat <<'EOF'
PickAndPlace(B1): wire ScanAndPlace through messages.py + mock stub

Add ScanAndPlace to the real and mock arm-msgs branches of messages.py and a
ScanAndPlace MockAction stub (Goal/Result/Feedback mirroring the rulebook
append-only contract) so BT nodes import it via behavior_tree.messages on a
host with tinker_arm_msgs absent.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"
```

**Deliverable:** `import behavior_tree.messages; messages.ScanAndPlace` resolves (real on this host, mock when arm msgs absent); `mock_messages.ScanAndPlace` carries every contract field.

---

### Task B2: Pure `categorization.py` + `test_categorization.py`

**Files:**
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/categorization.py`
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_categorization.py`

**Interfaces:**
- Produces (FROZEN CONTRACT): `Destination = namedtuple('Destination', ['klass','reference_label'])`; `classify_destination(label, *, cutlery, tableware, trash, category_map) -> Destination`, `klass ∈ {'wash_staging','trash','cabinet'}`.
- Routing rules: label ∈ cutlery ∪ tableware → `wash_staging` (ref `""`); label ∈ trash → `trash` (ref `""`); else → `cabinet` (ref `category_map.get(label, label)`). No ROS, no py_trees — pure pytest.

Steps:

1. [ ] Write the failing test `test/test_categorization.py`:
```python
from behavior_tree.PickAndPlace.categorization import classify_destination, Destination

CUTLERY = ["fork", "knife", "spoon"]
TABLEWARE = ["plate", "mug", "cup", "bowl"]
TRASH = ["paper cup"]


def _classify(label, category_map=None):
    return classify_destination(
        label, cutlery=CUTLERY, tableware=TABLEWARE, trash=TRASH,
        category_map=category_map or {},
    )


def test_cutlery_routes_to_wash_staging():
    d = _classify("fork")
    assert d.klass == "wash_staging" and d.reference_label == ""


def test_tableware_routes_to_wash_staging():
    assert _classify("plate").klass == "wash_staging"
    assert _classify("mug").klass == "wash_staging"


def test_designated_trash_routes_to_trash():
    d = _classify("paper cup")
    assert d.klass == "trash" and d.reference_label == ""


def test_unknown_routes_to_cabinet_with_label_reference():
    d = _classify("pringles")
    assert d.klass == "cabinet" and d.reference_label == "pringles"


def test_category_map_groups_label():
    d = _classify("pringles", category_map={"pringles": "snacks"})
    assert d.klass == "cabinet" and d.reference_label == "snacks"


def test_category_map_default_to_label_when_empty():
    # v1 empty map => each label its own group (+20 grouping best-effort).
    assert _classify("oats").reference_label == "oats"


def test_bowl_is_tableware_during_cleanup():
    # Bowl disambiguation: during cleanup a detected bowl is tableware; the
    # breakfast phase handles the bowl out-of-band (never via this fn).
    assert _classify("bowl").klass == "wash_staging"


def test_returns_destination_namedtuple():
    d = _classify("fork")
    assert isinstance(d, Destination)
    assert d._fields == ("klass", "reference_label")


def test_case_and_whitespace_insensitive():
    assert _classify("  Fork ").klass == "wash_staging"
```

2. [ ] Run it (expect FAIL — module does not exist):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_categorization.py -q
```
Expected: `ModuleNotFoundError: No module named 'behavior_tree.PickAndPlace.categorization'`.

3. [ ] Create `categorization.py`:
```python
from __future__ import annotations

"""Pure destination categorization for the PickAndPlace rulebook tree.

No ROS, no py_trees — deterministic and unit-tested. classify_destination maps
a detected object label to one of three destination classes:
  - wash_staging : cutlery + tableware (routed to the wash-staging surface)
  - trash        : designated trash labels (controlled bin release)
  - cabinet      : everything else, grouped next to similar items

`category_map` is the optional grouping table (label -> group name). v1 is
empty, so each label defaults to itself (its own group; the +20 grouping bonus
is best-effort and improves as clusters grow). For cabinet items the returned
`reference_label` is the group an item should be placed beside.

Bowl disambiguation: "bowl" is in tableware_labels, so during *cleanup* a
detected bowl routes to wash_staging. During *serve-breakfast* the bowl is
handled by the explicit breakfast phase (phase context disambiguates) and never
flows through classify_destination.
"""

from collections import namedtuple

Destination = namedtuple("Destination", ["klass", "reference_label"])


def classify_destination(label, *, cutlery, tableware, trash, category_map):
    norm = (label or "").strip().lower()
    cutlery_set = {c.strip().lower() for c in cutlery}
    tableware_set = {t.strip().lower() for t in tableware}
    trash_set = {t.strip().lower() for t in trash}
    if norm in cutlery_set or norm in tableware_set:
        return Destination("wash_staging", "")
    if norm in trash_set:
        return Destination("trash", "")
    reference_label = category_map.get(norm, norm) if category_map else norm
    return Destination("cabinet", reference_label)
```

4. [ ] Run the test (expect PASS):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_categorization.py -q
```
Expected: 9 passed.

5. [ ] Commit:
```
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/PickAndPlace/categorization.py \
        src/behavior_tree/test/test_categorization.py && \
git commit -m "$(cat <<'EOF'
PickAndPlace(B2): pure classify_destination + tests

Add PickAndPlace/categorization.py (Destination namedtuple +
classify_destination): cutlery/tableware -> wash_staging, designated trash ->
trash, else -> cabinet with category_map default-to-label grouping. Pure,
deterministic, no ROS/py_trees.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"
```

**Deliverable:** `classify_destination` routes all four label classes correctly under pure pytest, including category-map grouping and the bowl note.

---

### Task B3: `config.py` CONTRACT additions + `constants.json` keys + config-contract test

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/config.py` (append new symbols at end of file, after line 243)
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/constants.json` (insert `pose_extra_surface`, `point_extra_surface`, `category_map` after the `point_shelf_right` block, ~line 238)
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_pp_config_contract.py`

**Interfaces (FROZEN CONTRACT — REUSE existing keys, do not redefine):**
- Produces: `SCAN_AND_PLACE_ACTION_NAME='scan_and_place_action'`; `PLACEMENT_MODE_FREE_SPACE=0`, `PLACEMENT_MODE_NEAR_SIMILAR=1`, `PLACEMENT_MODE_FIXED_POINT=2`, `PLACEMENT_MODE_NONE=255`; `KEY_POSE_EXTRA_SURFACE='pp_pose_extra_surface'`, `KEY_POINT_EXTRA_SURFACE='pp_point_extra_surface'`; `POSE_EXTRA_SURFACE`, `POINT_EXTRA_SURFACE`; `CATEGORY_MAP`; `TABLE_BUDGET_SEC=200.0`, `BREAKFAST_BUDGET_SEC=110.0`, `EXTRA_BUDGET_SEC=60.0`; `DESTINATION_ROUTING = {'wash_staging': (KEY_POSE_WASH_STAGING, KEY_ARM_WASH, PLACEMENT_MODE_FREE_SPACE, KEY_POINT_WASH_STAGING), 'cabinet': (KEY_POSE_CABINET, KEY_ARM_CABINET, PLACEMENT_MODE_NEAR_SIMILAR, KEY_POINT_CABINET_DEFAULT), 'trash': (KEY_POSE_TRASH_BIN, KEY_ARM_TRASH, PLACEMENT_MODE_NONE, None)}`.
- Consumes: existing `KEY_POSE_WASH_STAGING/CABINET/TRASH_BIN`, `KEY_ARM_WASH/CABINET/TRASH`, `KEY_POINT_WASH_STAGING/CABINET_DEFAULT`, `_pose_reader`, `_point_reader`, `constants`.

Steps:

1. [ ] Write the failing test `test/test_pp_config_contract.py`:
```python
import behavior_tree.PickAndPlace.config as c


def test_action_name():
    assert c.SCAN_AND_PLACE_ACTION_NAME == "scan_and_place_action"


def test_placement_mode_constants():
    assert (c.PLACEMENT_MODE_FREE_SPACE, c.PLACEMENT_MODE_NEAR_SIMILAR,
            c.PLACEMENT_MODE_FIXED_POINT, c.PLACEMENT_MODE_NONE) == (0, 1, 2, 255)


def test_extra_surface_symbols():
    assert c.KEY_POSE_EXTRA_SURFACE == "pp_pose_extra_surface"
    assert c.KEY_POINT_EXTRA_SURFACE == "pp_point_extra_surface"
    assert c.POSE_EXTRA_SURFACE is not None
    assert c.POINT_EXTRA_SURFACE is not None


def test_category_map_is_dict():
    assert isinstance(c.CATEGORY_MAP, dict)


def test_budgets():
    assert (c.TABLE_BUDGET_SEC, c.BREAKFAST_BUDGET_SEC, c.EXTRA_BUDGET_SEC) == (200.0, 110.0, 60.0)


def test_destination_routing_shape():
    assert set(c.DESTINATION_ROUTING) == {"wash_staging", "cabinet", "trash"}
    for klass, tup in c.DESTINATION_ROUTING.items():
        assert len(tup) == 4, klass
    nav, arm, mode, point = c.DESTINATION_ROUTING["wash_staging"]
    assert (nav, arm, mode, point) == (
        c.KEY_POSE_WASH_STAGING, c.KEY_ARM_WASH,
        c.PLACEMENT_MODE_FREE_SPACE, c.KEY_POINT_WASH_STAGING)
    nav, arm, mode, point = c.DESTINATION_ROUTING["cabinet"]
    assert (nav, arm, mode, point) == (
        c.KEY_POSE_CABINET, c.KEY_ARM_CABINET,
        c.PLACEMENT_MODE_NEAR_SIMILAR, c.KEY_POINT_CABINET_DEFAULT)
    nav, arm, mode, point = c.DESTINATION_ROUTING["trash"]
    assert mode == c.PLACEMENT_MODE_NONE and point is None
```

2. [ ] Run it (expect FAIL — symbols not defined):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_pp_config_contract.py -q
```
Expected: `AttributeError: module ... has no attribute 'SCAN_AND_PLACE_ACTION_NAME'` (or `KeyError: 'pose_extra_surface'` at import if constants edited first — do the config edit and constants edit together in steps 3-4).

3. [ ] Insert the new constants into `constants.json` after the `point_shelf_right` block. Replace:
```json
  "point_shelf_right": {
    "x": 1.60,
    "y": -1.20,
    "z": 0.0
  },
  "arm_pos_navigating": [
```
with:
```json
  "point_shelf_right": {
    "x": 1.60,
    "y": -1.20,
    "z": 0.0
  },
  "pose_extra_surface": {
    "_comment": "Auxiliary 'extra surface' cleared into the cabinet. PLACEHOLDER — capture real pose at Setup Days. TODO(setup-days).",
    "point": {
      "x": 5.5,
      "y": 1.0,
      "z": 0.0
    },
    "orientation": {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "w": 1.0
    }
  },
  "point_extra_surface": {
    "_comment": "PLACEHOLDER hardcoded place point on the extra surface. TODO(setup-days).",
    "x": 1.3,
    "y": -0.5,
    "z": 0.75
  },
  "category_map": {
    "_comment": "Optional label -> cabinet group for +20 grouping. Empty v1 => default-to-label. Populate at Setup Days. TODO(setup-days)."
  },
  "arm_pos_navigating": [
```
(Note: `_point_reader`/`_pose_reader` read only `point`/`orientation`/`x`/`y`/`z` keys and ignore the `_comment` string. `dict(constants.get('category_map', {}))` will carry the `_comment` key — harmless for grouping; `classify_destination` only does `.get(label)`. If the implementer prefers a clean map, strip `_comment` in config via a dict-comprehension that drops keys starting with `_`.)

4. [ ] Append the new symbols to the end of `config.py` (after line 243, `KEY_GRASP_VISION_RES = "grasp_vision_result"`):
```python

# ============================================================
# Rulebook integration (net-new — REUSE existing KEY_* above)
# ============================================================

# ScanAndPlace action + placement modes (mirror arm_api/placement_logic.py).
SCAN_AND_PLACE_ACTION_NAME = "scan_and_place_action"
PLACEMENT_MODE_FREE_SPACE = 0
PLACEMENT_MODE_NEAR_SIMILAR = 1
PLACEMENT_MODE_FIXED_POINT = 2
PLACEMENT_MODE_NONE = 255  # sentinel: the trash branch never calls ScanAndPlace

# Extra surface — a distinct auxiliary surface, NOT one of the table poses.
KEY_POSE_EXTRA_SURFACE = "pp_pose_extra_surface"
KEY_POINT_EXTRA_SURFACE = "pp_point_extra_surface"
POSE_EXTRA_SURFACE = _pose_reader(constants["pose_extra_surface"])
POINT_EXTRA_SURFACE = _point_reader(constants["point_extra_surface"])

# Optional grouping table (label -> cabinet group). Empty v1 => default-to-label.
CATEGORY_MAP = {
    k: v for k, v in dict(constants.get("category_map", {})).items()
    if not str(k).startswith("_")
}

# Per-phase time budgets (seconds). Sum (370) <= MAX_RUNTIME_SEC (390) w/ margin.
TABLE_BUDGET_SEC = 200.0
BREAKFAST_BUDGET_SEC = 110.0
EXTRA_BUDGET_SEC = 60.0

# Destination routing: klass -> (nav_pose_key, arm_scan_pose_key, vlm_mode, hardcoded_point_key).
# BtNode_PopWorkItem chooses the effective placement_mode from the tree's
# place_policy: vlm -> vlm_mode (+reference_label); hardcoded -> FIXED_POINT with
# fixed_target = the hardcoded_point. Trash always drops (PLACEMENT_MODE_NONE).
DESTINATION_ROUTING = {
    "wash_staging": (KEY_POSE_WASH_STAGING, KEY_ARM_WASH, PLACEMENT_MODE_FREE_SPACE, KEY_POINT_WASH_STAGING),
    "cabinet": (KEY_POSE_CABINET, KEY_ARM_CABINET, PLACEMENT_MODE_NEAR_SIMILAR, KEY_POINT_CABINET_DEFAULT),
    "trash": (KEY_POSE_TRASH_BIN, KEY_ARM_TRASH, PLACEMENT_MODE_NONE, None),
}
```

5. [ ] Run the test (expect PASS):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_pp_config_contract.py -q
```
Expected: 6 passed.

6. [ ] Commit:
```
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/PickAndPlace/config.py \
        src/behavior_tree/behavior_tree/PickAndPlace/constants.json \
        src/behavior_tree/test/test_pp_config_contract.py && \
git commit -m "$(cat <<'EOF'
PickAndPlace(B3): config + constants for ScanAndPlace routing

Add net-new config symbols (SCAN_AND_PLACE_ACTION_NAME, PLACEMENT_MODE_*,
DESTINATION_ROUTING, KEY_POSE/POINT_EXTRA_SURFACE + materialized
POSE/POINT_EXTRA_SURFACE, CATEGORY_MAP, per-phase budgets), reusing all
existing KEY_* unchanged. Add pose_extra_surface/point_extra_surface/category_map
placeholders (TODO setup-days) to constants.json.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"
```

**Deliverable:** `behavior_tree.PickAndPlace.config` imports cleanly and exposes every contract symbol with the correct `DESTINATION_ROUTING` shape, no existing key redefined.

---

### Task B4: `BtNode_ScanAndPlace` (ActionHandler) in `Manipulation.py` + register in `mock_config.json` + mock unit test

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/TemplateNodes/Manipulation.py` (extend the `from behavior_tree.messages import (...)` block at lines 54-63 to add `ScanAndPlace`; add a `from behavior_tree.PickAndPlace.config import ...` import; append `class BtNode_ScanAndPlace(ActionHandler)` at end of file)
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/mock_config.json` (add `"BtNode_ScanAndPlace": "IMMEDIATE"` under `manipulation.nodes`, after line 59 `"BtNode_PointTo"`)
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_scan_and_place_node_mock.py`

**Interfaces (FROZEN CONTRACT):**
- Produces: `class BtNode_ScanAndPlace(ActionHandler)` with `__init__(self, name, bb_item_description=KEY_OBJECT_LABEL, bb_placement_mode='pp_active_placement_mode', bb_reference_label='pp_active_reference_label', bb_margin='pp_active_margin', bb_orientation='pp_active_orientation', bb_fixed_target=KEY_ACTIVE_TARGET_POINT, bb_scan_pose='pp_active_scan_pose', bb_skip_scan_move='pp_active_skip_scan', bb_dry_run='pp_active_dry_run', bb_out_placed_at='pp_active_placed_at', bb_out_status='pp_active_place_status', action_name=SCAN_AND_PLACE_ACTION_NAME)`; overrides `send_goal()` (assemble `ScanAndPlace.Goal` from blackboard) and `process_result()` (write placed_at + place_status; `record_event` on FAILURE only — success scoring is owned by the BT route leaf `_RecordEventLeaf`; `STATUS_SUCCEEDED`->SUCCESS else FAILURE).
- Consumes: `ScanAndPlace` (via `behavior_tree.messages`), `ActionHandler` base mock path, `record_event` (Task B5, imported lazily inside `process_result` to avoid the `custom_nodes <-> Manipulation` import cycle — `custom_nodes` imports `BtNode_Grasp` from this module).
- Mock: registered under `manipulation`; under `IMMEDIATE` it auto-succeeds after 2 ticks and creates no client. `process_result` is NOT reached in mock (the base `update()` short-circuits), so the lazy `record_event` import is never executed during the mock test.

Steps:

1. [ ] Write the failing test `test/test_scan_and_place_node_mock.py`:
```python
import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 — force mock before config loads

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.TemplateNodes.Manipulation import BtNode_ScanAndPlace  # noqa: E402
from behavior_tree.PickAndPlace.config import (  # noqa: E402
    SCAN_AND_PLACE_ACTION_NAME, KEY_OBJECT_LABEL, KEY_ACTIVE_TARGET_POINT,
)


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def test_constructs_in_mock_mode():
    node = BtNode_ScanAndPlace(name="ScanPlace")
    assert node.mock_mode is True
    assert node.action_name == SCAN_AND_PLACE_ACTION_NAME


def test_default_bb_keys_match_contract():
    node = BtNode_ScanAndPlace(name="ScanPlace")
    # The contract defaults for the shared keys:
    assert node._reads["item_description"] == KEY_OBJECT_LABEL
    assert node._reads["fixed_target"] == KEY_ACTIVE_TARGET_POINT
    assert node._reads["placement_mode"] == "pp_active_placement_mode"


def test_ticks_to_success_in_mock_no_server():
    node = BtNode_ScanAndPlace(name="ScanPlace")
    node.setup(node=None)  # mock setup skips client creation
    status = py_trees.common.Status.RUNNING
    for _ in range(6):
        node.tick_once()
        status = node.status
        if status == py_trees.common.Status.SUCCESS:
            break
    assert status == py_trees.common.Status.SUCCESS
```

2. [ ] Run it (expect FAIL — `BtNode_ScanAndPlace` not importable):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_scan_and_place_node_mock.py -q
```
Expected: `ImportError: cannot import name 'BtNode_ScanAndPlace'`.

3. [ ] Edit the `messages` import block in `Manipulation.py` (lines 54-63) to add `ScanAndPlace`:
```python
from behavior_tree.messages import (
    Grasp,
    ObjectDetection,
    Drop,
    Place,
    JointMove,
    CartesianMove,
    GripperCommand,
    Fold,
    ScanAndPlace,
)
```

4. [ ] Add a config import below the existing imports in `Manipulation.py` (after line 71 `import math`):
```python
from behavior_tree.PickAndPlace.config import (
    SCAN_AND_PLACE_ACTION_NAME,
    KEY_OBJECT_LABEL,
    KEY_ACTIVE_TARGET_POINT,
    PLACEMENT_MODE_FREE_SPACE,
)
```

5. [ ] Append `BtNode_ScanAndPlace` at the end of `Manipulation.py`:
```python
class BtNode_ScanAndPlace(ActionHandler):
    """Scan a destination surface and place the held object.

    Wraps arm_api/scan_and_place_server (action `scan_and_place_action`). The
    server owns *where-on-the-surface* placement; the BT owns *which surface*
    (nav + arm scan pose + mode). Mode + target are read from the blackboard,
    set by BtNode_PopWorkItem per the tree's place_policy:
        0 FREE_SPACE | 1 NEAR_SIMILAR(reference_label) | 2 FIXED_POINT(fixed_target)
    Auto-mocked under the `manipulation` subsystem (registered in
    mock_config.json); under IMMEDIATE it auto-succeeds with no ROS client.
    """

    def __init__(
        self,
        name: str,
        bb_item_description: str = KEY_OBJECT_LABEL,
        bb_placement_mode: str = "pp_active_placement_mode",
        bb_reference_label: str = "pp_active_reference_label",
        bb_margin: str = "pp_active_margin",
        bb_orientation: str = "pp_active_orientation",
        bb_fixed_target: str = KEY_ACTIVE_TARGET_POINT,
        bb_scan_pose: str = "pp_active_scan_pose",
        bb_skip_scan_move: str = "pp_active_skip_scan",
        bb_dry_run: str = "pp_active_dry_run",
        bb_out_placed_at: str = "pp_active_placed_at",
        bb_out_status: str = "pp_active_place_status",
        action_name: str = SCAN_AND_PLACE_ACTION_NAME,
    ):
        super().__init__(
            name, ScanAndPlace, action_name, None, wait_for_server_timeout_sec=-3
        )
        self.blackboard = self.attach_blackboard_client(name=self.name)
        # local-attr -> blackboard-key for every READ input.
        self._reads = {
            "item_description": bb_item_description,
            "placement_mode": bb_placement_mode,
            "reference_label": bb_reference_label,
            "margin": bb_margin,
            "orientation": bb_orientation,
            "fixed_target": bb_fixed_target,
            "scan_pose": bb_scan_pose,
            "skip_scan_move": bb_skip_scan_move,
            "dry_run": bb_dry_run,
        }
        for local, key in self._reads.items():
            self.blackboard.register_key(
                key=local,
                access=pytree.common.Access.READ,
                remap_to=pytree.blackboard.Blackboard.absolute_name("/", key),
            )
        self.blackboard.register_key(
            key="placed_at",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_out_placed_at),
        )
        # status/reason output (spec §6): {'status': int, 'reason': str}.
        self.blackboard.register_key(
            key="place_status",
            access=pytree.common.Access.WRITE,
            remap_to=pytree.blackboard.Blackboard.absolute_name("/", bb_out_status),
        )

    def setup(self, **kwargs):
        ActionHandler.setup(self, **kwargs)
        self.logger.debug(f"Setup ScanAndPlace on {self.action_name}")

    def _read(self, local, default):
        try:
            value = getattr(self.blackboard, local)
        except Exception:
            return default
        return default if value is None else value

    def send_goal(self):
        # Mock mode: defer to the base (no goal assembly, no blackboard reads).
        if self.mock_mode:
            return super().send_goal()
        try:
            goal = ScanAndPlace.Goal()
            goal.item_description = str(self._read("item_description", ""))
            goal.placement_mode = int(self._read("placement_mode", PLACEMENT_MODE_FREE_SPACE))
            goal.reference_label = str(self._read("reference_label", ""))
            goal.margin_m = float(self._read("margin", 0.0))
            goal.max_candidates = 0
            orientation = self._read("orientation", None)
            if orientation is not None:
                goal.orientation = orientation
            fixed_target = self._read("fixed_target", None)
            if fixed_target is not None:
                goal.fixed_target = fixed_target
            scan_pose = self._read("scan_pose", None)
            if scan_pose:
                goal.scan_pose_deg = [float(x) for x in scan_pose]
            goal.skip_scan_move = bool(self._read("skip_scan_move", False))
            goal.dry_run = bool(self._read("dry_run", False))
            self.send_goal_request(goal)
            self.feedback_message = f"ScanAndPlace goal sent (mode {goal.placement_mode})"
        except Exception as e:
            self.feedback_message = f"Failed to send ScanAndPlace goal: {e}"
            self.logger.error(self.feedback_message)
            return pytree.common.Status.FAILURE

    def process_result(self):
        # Lazy import breaks the custom_nodes <-> Manipulation import cycle
        # (custom_nodes imports BtNode_Grasp from this module).
        from behavior_tree.PickAndPlace.custom_nodes import record_event

        item = self._read("item_description", "")
        if self.result_status != action_msgs.GoalStatus.STATUS_SUCCEEDED:
            err, st = "", -1
            try:
                err = self.result_message.result.error_msg
                st = self.result_message.result.status
            except Exception:
                pass
            try:
                self.blackboard.place_status = {"status": st, "reason": err}
            except Exception:
                pass
            record_event(self.blackboard, phase="", item=item,
                         action="scan_and_place", outcome="failure", points_est=0)
            self.feedback_message = f"ScanAndPlace failed: status={self.result_status} err={err}"
            return pytree.common.Status.FAILURE
        result = self.result_message.result
        try:
            self.blackboard.placed_at = result.placed_at
        except Exception:
            pass
        try:
            self.blackboard.place_status = {"status": result.status, "reason": result.error_msg}
        except Exception:
            pass
        # Scoring is owned by the BT route leaf (_RecordEventLeaf in
        # pick_and_place_rulebook). Do NOT record_event here on success, or the
        # placement double-counts on the real robot.
        self.feedback_message = "ScanAndPlace succeeded"
        return pytree.common.Status.SUCCESS

    def feedback_callback(self, msg: Any):
        return super().feedback_callback(msg)
```

6. [ ] Register the node in `mock_config.json` under `manipulation.nodes` — replace:
```json
          "BtNode_GripperAction": "KEYPRESS",
          "BtNode_PointTo": "KEYPRESS"
        }
```
with:
```json
          "BtNode_GripperAction": "KEYPRESS",
          "BtNode_PointTo": "KEYPRESS",
          "BtNode_ScanAndPlace": "IMMEDIATE"
        }
```

7. [ ] Run the test (expect PASS):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_scan_and_place_node_mock.py -q
```
Expected: 3 passed (constructs, default keys, ticks-to-SUCCESS with `node.setup(node=None)` — auto-completes after 2 RUNNING ticks).

8. [ ] Commit:
```
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/TemplateNodes/Manipulation.py \
        src/behavior_tree/behavior_tree/mock_config.json \
        src/behavior_tree/test/test_scan_and_place_node_mock.py && \
git commit -m "$(cat <<'EOF'
PickAndPlace(B4): BtNode_ScanAndPlace ActionHandler + mock registration

Add BtNode_ScanAndPlace wrapping arm_api scan_and_place_action: reads
mode/reference/margin/orientation/fixed_target/scan-pose/skip/dry-run from the
blackboard, assembles ScanAndPlace.Goal, writes placed_at + records a
score-trace event on result. Register under manipulation (IMMEDIATE) in
mock_config.json. record_event imported lazily in process_result to avoid the
custom_nodes<->Manipulation cycle.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"
```

**Deliverable:** `BtNode_ScanAndPlace` constructs with the exact contract signature/defaults and ticks to `SUCCESS` in `BT_MOCK_MODE` with no server. (The `record_event` lazy import resolves once Task B5 lands; the mock test does not exercise it.)

---

### Task B5: Inventory/queue/guard/score nodes + `record_event` in `custom_nodes.py` + `test_work_queue.py` + `test_timeout_policy.py`

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/custom_nodes.py` (add imports near top after line 17; append the new nodes + `record_event` at end of file)
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_work_queue.py`
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_timeout_policy.py`

**Interfaces (FROZEN CONTRACT — all plain `py_trees.behaviour.Behaviour`; do NOT register in mock_config.json):**
- `class BtNode_BuildInventory(...).__init__(self, name, in_key=KEY_SCAN_RESULTS_TABLE, out_inventory=KEY_INVENTORY_TABLE, out_queue=KEY_WORK_QUEUE, source_pose_key=KEY_POSE_TABLE, mock_seed=None)` — reads scan result, classifies via `classify_destination`, sorts cabinet items by category, mock-seeds a canned queue when empty (mock or `mock_seed` set), writes inventory+queue, always SUCCESS. Registers `in_key` READ, inventory+queue WRITE.
- `class BtNode_PopWorkItem(...).__init__(self, name, queue=KEY_WORK_QUEUE, place_policy='vlm')` — pops front; resolves `DESTINATION_ROUTING`; applies `place_policy` to derive effective `placement_mode` + `fixed_target`; writes `KEY_ACTIVE_OBJECT_CLASS, KEY_OBJECT_LABEL, KEY_ACTIVE_PROMPT, KEY_ACTIVE_SOURCE_POSE, KEY_ACTIVE_TARGET_POSE, KEY_ACTIVE_TARGET_POINT, pp_active_reference_label, pp_active_placement_mode, pp_active_scan_pose (empty list), pp_active_skip_scan (True)`. SUCCESS if popped, FAILURE if empty (loop terminator — un-masked). Registers `KEY_WORK_QUEUE` READ+WRITE, all `KEY_ACTIVE_*` + fixed_target WRITE.
- `class BtNode_DeadlineGuard(...).__init__(self, name, budget_sec, clock=None)` — `clock` default `time.monotonic`; latch `_deadline` in `initialise()`; `update()` RUNNING until `clock() >= deadline` then SUCCESS, never FAILURE.
- `class BtNode_GuardActiveClass(...).__init__(self, name, expected, key=KEY_ACTIVE_OBJECT_CLASS)` — SUCCESS if match else FAILURE.
- `class BtNode_MarkPhase(...).__init__(self, name, phase, key=KEY_SCORE_TRACE)` — appends `phase` to `visited_phases`.
- `def record_event(blackboard, phase, item, action, outcome, points_est=0) -> None` — appends a scored event to `KEY_SCORE_TRACE`. Score-trace shape: `{'visited_phases': [], 'events': [{phase,item,action,outcome,points_est}], 'place_policy': str}`.
- Consumes: `classify_destination` (B2), `DESTINATION_ROUTING`/`PLACEMENT_MODE_*`/label sets/`CATEGORY_MAP`/KEY_* (B3), `get_config().is_mock_mode()`.

Steps:

1. [ ] Write the failing tests. `test/test_work_queue.py`:
```python
import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.PickAndPlace.custom_nodes import (  # noqa: E402
    BtNode_BuildInventory, BtNode_PopWorkItem,
)
from behavior_tree.PickAndPlace import config as c  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _write(key, value):
    cl = py_trees.blackboard.Client(name="w")
    cl.register_key(key="k", access=py_trees.common.Access.WRITE,
                    remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key))
    cl.k = value


def _read(key):
    cl = py_trees.blackboard.Client(name="r")
    cl.register_key(key="k", access=py_trees.common.Access.READ,
                    remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key))
    return cl.k


def test_build_inventory_seeds_when_empty_and_sorts_cabinet():
    # Empty upstream + explicit mock_seed: bowl (wash), pringles/apple/cereal (cabinet).
    node = BtNode_BuildInventory(
        name="build", source_pose_key=c.KEY_POSE_TABLE,
        mock_seed=["pringles", "apple", "bowl", "cereal"],
    )
    assert node.update() == py_trees.common.Status.SUCCESS
    queue = _read(c.KEY_WORK_QUEUE)
    classes = [it["destination"] for it in queue]
    # non-cabinet (wash_staging) first, then cabinet items sorted by reference_label.
    assert classes[0] == "wash_staging"
    cabinet_refs = [it["reference_label"] for it in queue if it["destination"] == "cabinet"]
    assert cabinet_refs == sorted(cabinet_refs) == ["apple", "cereal", "pringles"]
    # source_pose_key threaded onto every item.
    assert all(it["source_pose_key"] == c.KEY_POSE_TABLE for it in queue)


def test_pop_drains_then_fails_on_empty_without_raising():
    _write(c.KEY_WORK_QUEUE, [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
        {"label": "pringles", "segment": None, "destination": "cabinet",
         "reference_label": "pringles", "source_pose_key": c.KEY_POSE_TABLE},
    ])
    pop = BtNode_PopWorkItem(name="pop", place_policy="vlm")
    assert pop.update() == py_trees.common.Status.SUCCESS
    assert len(_read(c.KEY_WORK_QUEUE)) == 1
    assert pop.update() == py_trees.common.Status.SUCCESS
    assert len(_read(c.KEY_WORK_QUEUE)) == 0
    # empty -> FAILURE (loop terminator) and must NOT raise.
    assert pop.update() == py_trees.common.Status.FAILURE


def test_pop_writes_source_distinct_from_target():
    _write(c.KEY_WORK_QUEUE, [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
    ])
    BtNode_PopWorkItem(name="pop").update()
    source = _read(c.KEY_ACTIVE_SOURCE_POSE)
    target = _read(c.KEY_ACTIVE_TARGET_POSE)
    assert source is c.POSE_TABLE
    assert target is c.POSE_WASH_STAGING
    assert source is not target


def test_pop_placement_mode_vlm_vs_hardcoded():
    seed = [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
        {"label": "pringles", "segment": None, "destination": "cabinet",
         "reference_label": "pringles", "source_pose_key": c.KEY_POSE_TABLE},
    ]
    # vlm: wash -> FREE_SPACE, cabinet -> NEAR_SIMILAR.
    _write(c.KEY_WORK_QUEUE, list(seed))
    pop = BtNode_PopWorkItem(name="pop_vlm", place_policy="vlm")
    pop.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_FREE_SPACE
    pop.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_NEAR_SIMILAR
    assert _read("pp_active_reference_label") == "pringles"
    # hardcoded: both -> FIXED_POINT with the routing's hardcoded point.
    _write(c.KEY_WORK_QUEUE, list(seed))
    pop2 = BtNode_PopWorkItem(name="pop_hc", place_policy="hardcoded")
    pop2.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_FIXED_POINT
    assert _read(c.KEY_ACTIVE_TARGET_POINT) is c.POINT_WASH_STAGING
    pop2.update()
    assert _read("pp_active_placement_mode") == c.PLACEMENT_MODE_FIXED_POINT
    assert _read(c.KEY_ACTIVE_TARGET_POINT) is c.POINT_CABINET_DEFAULT


def test_pop_queue_access_is_read_and_write():
    # Functional READ+WRITE: the node reads the queue and writes the shortened
    # list back through the same key without a fresh writer.
    _write(c.KEY_WORK_QUEUE, [
        {"label": "bowl", "segment": None, "destination": "wash_staging",
         "reference_label": "", "source_pose_key": c.KEY_POSE_TABLE},
    ])
    pop = BtNode_PopWorkItem(name="pop")
    pop.update()
    assert _read(c.KEY_WORK_QUEUE) == []
```

   `test/test_timeout_policy.py`:
```python
import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402

import py_trees  # noqa: E402

from behavior_tree.PickAndPlace.custom_nodes import BtNode_DeadlineGuard  # noqa: E402

R = py_trees.common.Status.RUNNING
S = py_trees.common.Status.SUCCESS


def test_running_until_boundary_then_success():
    t = [0.0]
    g = BtNode_DeadlineGuard("g", budget_sec=10.0, clock=lambda: t[0])
    g.initialise()
    assert g.update() == R
    t[0] = 9.999
    assert g.update() == R
    t[0] = 10.0  # boundary fires
    assert g.update() == S
    t[0] = 11.0
    assert g.update() == S  # never FAILURE


def test_deadline_latched_in_initialise():
    t = [100.0]
    g = BtNode_DeadlineGuard("g", budget_sec=5.0, clock=lambda: t[0])
    g.initialise()  # latch deadline = 105 (not at construction)
    t[0] = 104.0
    assert g.update() == R
    t[0] = 105.0
    assert g.update() == S


def test_default_clock_is_monotonic_and_does_not_fire_early():
    g = BtNode_DeadlineGuard("g", budget_sec=1000.0)
    g.initialise()
    assert g.update() == R
```

2. [ ] Run them (expect FAIL — symbols not yet defined):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_work_queue.py test/test_timeout_policy.py -q
```
Expected: `ImportError: cannot import name 'BtNode_BuildInventory'` (and `BtNode_DeadlineGuard`).

3. [ ] Add imports to `custom_nodes.py` after line 17 (`from geometry_msgs.msg import Pose`):
```python
import time

from behavior_tree.config import get_config
from behavior_tree.PickAndPlace.categorization import classify_destination
from behavior_tree.PickAndPlace.config import (
    CUTLERY_LABELS,
    TABLEWARE_LABELS,
    DESIGNATED_TRASH_LABELS,
    CATEGORY_MAP,
    DESTINATION_ROUTING,
    PLACEMENT_MODE_FIXED_POINT,
    PLACEMENT_MODE_NONE,
    KEY_SCAN_RESULTS_TABLE,
    KEY_INVENTORY_TABLE,
    KEY_WORK_QUEUE,
    KEY_POSE_TABLE,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_OBJECT_LABEL,
    KEY_ACTIVE_PROMPT,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ACTIVE_TARGET_POINT,
    KEY_SCORE_TRACE,
)
```

4. [ ] Append the new nodes + `record_event` to the end of `custom_nodes.py`:
```python
def _abs(key):
    return py_trees.blackboard.Blackboard.absolute_name("/", key)


# Lazily-built map from a config KEY_* name to its materialized constant. Built
# on first use so config import order never matters.
_CONST_BY_KEY = None


def _const_by_key():
    global _CONST_BY_KEY
    if _CONST_BY_KEY is None:
        import behavior_tree.PickAndPlace.config as cfg
        _CONST_BY_KEY = {
            cfg.KEY_POSE_TABLE: cfg.POSE_TABLE,
            cfg.KEY_POSE_WASH_STAGING: cfg.POSE_WASH_STAGING,
            cfg.KEY_POSE_CABINET: cfg.POSE_CABINET,
            cfg.KEY_POSE_TRASH_BIN: cfg.POSE_TRASH_BIN,
            cfg.KEY_POSE_KITCHEN_SHELF: cfg.POSE_KITCHEN_SHELF,
            cfg.KEY_POSE_EXTRA_SURFACE: cfg.POSE_EXTRA_SURFACE,
            cfg.KEY_ARM_TABLE: cfg.ARM_POS_TABLE,
            cfg.KEY_ARM_WASH: cfg.ARM_POS_WASH,
            cfg.KEY_ARM_CABINET: cfg.ARM_POS_CABINET,
            cfg.KEY_ARM_TRASH: cfg.ARM_POS_TRASH,
            cfg.KEY_POINT_WASH_STAGING: cfg.POINT_WASH_STAGING,
            cfg.KEY_POINT_CABINET_DEFAULT: cfg.POINT_CABINET_DEFAULT,
            cfg.KEY_POINT_EXTRA_SURFACE: cfg.POINT_EXTRA_SURFACE,
        }
    return _CONST_BY_KEY


def _new_score_trace():
    return {"visited_phases": [], "events": [], "place_policy": ""}


def record_event(blackboard, phase, item, action, outcome, points_est=0):
    """Append a scored event to the global KEY_SCORE_TRACE.

    `blackboard` is accepted for call-site symmetry (nodes pass their own
    client); the score-trace is a single global key, so a dedicated client does
    the read-modify-write to avoid per-caller key-registration coupling.
    Shape: {'visited_phases': [], 'events': [{phase,item,action,outcome,points_est}], 'place_policy': str}.
    """
    client = py_trees.blackboard.Client(name="pp_record_event")
    client.register_key(key="trace", access=py_trees.common.Access.READ, remap_to=_abs(KEY_SCORE_TRACE))
    client.register_key(key="trace", access=py_trees.common.Access.WRITE, remap_to=_abs(KEY_SCORE_TRACE))
    try:
        trace = client.trace
    except Exception:
        trace = None
    if not isinstance(trace, dict):
        trace = _new_score_trace()
    trace.setdefault("visited_phases", [])
    trace.setdefault("events", [])
    trace.setdefault("place_policy", "")
    trace["events"].append({
        "phase": phase, "item": item, "action": action,
        "outcome": outcome, "points_est": points_est,
    })
    client.trace = trace
    return None


class BtNode_BuildInventory(py_trees.behaviour.Behaviour):
    """Build the cleanup inventory + work queue from a generalist scan result.

    Reads the scan result, classifies each label via classify_destination,
    sorts cabinet-bound items by category (so same-category placements run
    consecutively for the +20 grouping), and writes inventory + queue. In mock
    mode (or when `mock_seed` is set) with an empty upstream result, seeds a
    canned queue so the per-item loop body actually runs. Always SUCCESS.
    Plain Behaviour — never mocked, always runs real logic.
    """

    def __init__(self, name, in_key=KEY_SCAN_RESULTS_TABLE, out_inventory=KEY_INVENTORY_TABLE,
                 out_queue=KEY_WORK_QUEUE, source_pose_key=KEY_POSE_TABLE, mock_seed=None):
        super().__init__(name)
        self.source_pose_key = source_pose_key
        self.mock_seed = mock_seed
        self._in = self.attach_blackboard_client(name=f"{name}_in")
        self._in.register_key(key="scan", access=py_trees.common.Access.READ, remap_to=_abs(in_key))
        self._out = self.attach_blackboard_client(name=f"{name}_out")
        self._out.register_key(key="inventory", access=py_trees.common.Access.WRITE, remap_to=_abs(out_inventory))
        self._out.register_key(key="queue", access=py_trees.common.Access.WRITE, remap_to=_abs(out_queue))

    def _labels_from_scan(self):
        try:
            scan = self._in.scan
        except Exception:
            return []
        objs = getattr(scan, "objects", None) or []
        labels = []
        for o in objs:
            lbl = getattr(o, "cls", None) or getattr(o, "class_name", None)
            if lbl:
                labels.append((str(lbl), getattr(o, "segment", None)))
        return labels

    def update(self):
        labels = self._labels_from_scan()
        if not labels and (self.mock_seed is not None or get_config().is_mock_mode()):
            seed = self.mock_seed or ["bowl", "paper cup", "pringles"]
            labels = [(str(s), None) for s in seed]

        items = []
        for label, segment in labels:
            dest = classify_destination(
                label, cutlery=CUTLERY_LABELS, tableware=TABLEWARE_LABELS,
                trash=DESIGNATED_TRASH_LABELS, category_map=CATEGORY_MAP,
            )
            items.append({
                "label": label, "segment": segment, "destination": dest.klass,
                "reference_label": dest.reference_label, "source_pose_key": self.source_pose_key,
            })

        cabinet = [it for it in items if it["destination"] == "cabinet"]
        other = [it for it in items if it["destination"] != "cabinet"]
        cabinet.sort(key=lambda it: it["reference_label"])
        ordered = other + cabinet

        self._out.inventory = items
        self._out.queue = ordered
        self.feedback_message = f"inventory={len(items)} queue={len(ordered)}"
        return py_trees.common.Status.SUCCESS


class BtNode_PopWorkItem(py_trees.behaviour.Behaviour):
    """Pop the front work item and stamp the active-item blackboard keys.

    Resolves DESTINATION_ROUTING and applies place_policy to derive the
    effective placement_mode + fixed_target. SUCCESS if an item was popped,
    FAILURE on an empty queue — this is the cleanup-loop terminator and must
    stay un-masked (only its FAILURE exits the Repeat loop). Plain Behaviour —
    never mocked.
    """

    def __init__(self, name, queue=KEY_WORK_QUEUE, place_policy="vlm"):
        super().__init__(name)
        self.place_policy = place_policy
        self._q = self.attach_blackboard_client(name=f"{name}_q")
        self._q.register_key(key="queue", access=py_trees.common.Access.READ, remap_to=_abs(queue))
        self._q.register_key(key="queue", access=py_trees.common.Access.WRITE, remap_to=_abs(queue))
        self._w = self.attach_blackboard_client(name=f"{name}_w")
        self._writes = {
            "object_class": KEY_ACTIVE_OBJECT_CLASS,
            "object_label": KEY_OBJECT_LABEL,
            "prompt": KEY_ACTIVE_PROMPT,
            "source_pose": KEY_ACTIVE_SOURCE_POSE,
            "target_pose": KEY_ACTIVE_TARGET_POSE,
            "target_point": KEY_ACTIVE_TARGET_POINT,
            "reference_label": "pp_active_reference_label",
            "placement_mode": "pp_active_placement_mode",
            "scan_pose": "pp_active_scan_pose",
            "skip_scan": "pp_active_skip_scan",
        }
        for local, key in self._writes.items():
            self._w.register_key(key=local, access=py_trees.common.Access.WRITE, remap_to=_abs(key))

    def update(self):
        try:
            queue = self._q.queue
        except Exception:
            queue = None
        if not queue:
            self.feedback_message = "work queue empty -> FAILURE (loop exit)"
            return py_trees.common.Status.FAILURE

        item = queue[0]
        self._q.queue = list(queue[1:])

        klass = item["destination"]
        nav_pose_key, arm_pose_key, vlm_mode, hardcoded_point_key = DESTINATION_ROUTING[klass]
        consts = _const_by_key()

        self._w.object_class = klass
        self._w.object_label = item["label"]
        self._w.prompt = item["label"]
        self._w.reference_label = item.get("reference_label", "")
        self._w.source_pose = consts.get(item["source_pose_key"])
        self._w.target_pose = consts.get(nav_pose_key)
        # The BT positions the arm itself (handleOneItem._arm); hand the server an
        # empty scan pose + skip_scan_move=True so it does NOT re-move the arm.
        # NB: ARM_POS_* are RADIANS but scan_pose_deg is DEGREES — never forward
        # the arm_pose_key constant (arm_pose_key) here as degrees.
        self._w.scan_pose = []
        self._w.skip_scan = True

        if klass == "trash":
            self._w.placement_mode = PLACEMENT_MODE_NONE
            self._w.target_point = None
        elif self.place_policy == "hardcoded":
            self._w.placement_mode = PLACEMENT_MODE_FIXED_POINT
            self._w.target_point = consts.get(hardcoded_point_key)
        else:  # 'vlm'
            self._w.placement_mode = vlm_mode
            self._w.target_point = consts.get(hardcoded_point_key)

        self.feedback_message = f"popped {item['label']} -> {klass} (mode {self._w.placement_mode})"
        return py_trees.common.Status.SUCCESS


class BtNode_DeadlineGuard(py_trees.behaviour.Behaviour):
    """Wall-clock budget guard. Plain Behaviour — never mocked, so it runs its
    real logic even in mock (large budgets never fire in a fast run).

    initialise() latches deadline = clock() + budget_sec (on entry, not at
    construction). update() returns RUNNING until clock() >= deadline, then
    SUCCESS. Never returns FAILURE. `clock` is injectable for deterministic
    tests; default time.monotonic needs no ROS node handle.
    """

    def __init__(self, name, budget_sec, clock=None):
        super().__init__(name)
        self.budget_sec = float(budget_sec)
        self.clock = clock or time.monotonic
        self._deadline = None

    def initialise(self):
        self._deadline = self.clock() + self.budget_sec

    def update(self):
        if self._deadline is None:
            self._deadline = self.clock() + self.budget_sec
        if self.clock() >= self._deadline:
            self.feedback_message = "deadline reached"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = "within budget"
        return py_trees.common.Status.RUNNING


class BtNode_GuardActiveClass(py_trees.behaviour.Behaviour):
    """Route guard: SUCCESS iff the active object class matches `expected`,
    else FAILURE. Plain condition Behaviour — never a Handler (a mocked guard
    would auto-succeed and route everything to the first branch)."""

    def __init__(self, name, expected, key=KEY_ACTIVE_OBJECT_CLASS):
        super().__init__(name)
        self.expected = expected
        self._bb = self.attach_blackboard_client(name=f"{name}_g")
        self._bb.register_key(key="klass", access=py_trees.common.Access.READ, remap_to=_abs(key))

    def update(self):
        try:
            klass = self._bb.klass
        except Exception:
            klass = None
        if klass == self.expected:
            self.feedback_message = f"class {klass} == {self.expected}"
            return py_trees.common.Status.SUCCESS
        self.feedback_message = f"class {klass} != {self.expected}"
        return py_trees.common.Status.FAILURE


class BtNode_MarkPhase(py_trees.behaviour.Behaviour):
    """Append `phase` to the score-trace visited_phases. Always SUCCESS."""

    def __init__(self, name, phase, key=KEY_SCORE_TRACE):
        super().__init__(name)
        self.phase = phase
        self._bb = self.attach_blackboard_client(name=f"{name}_phase")
        self._bb.register_key(key="trace", access=py_trees.common.Access.READ, remap_to=_abs(key))
        self._bb.register_key(key="trace", access=py_trees.common.Access.WRITE, remap_to=_abs(key))

    def update(self):
        try:
            trace = self._bb.trace
        except Exception:
            trace = None
        if not isinstance(trace, dict):
            trace = _new_score_trace()
        trace.setdefault("visited_phases", [])
        trace.setdefault("events", [])
        trace.setdefault("place_policy", "")
        if self.phase not in trace["visited_phases"]:
            trace["visited_phases"].append(self.phase)
        self._bb.trace = trace
        self.feedback_message = f"phase {self.phase} marked"
        return py_trees.common.Status.SUCCESS
```

5. [ ] Run the tests (expect PASS):
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_work_queue.py test/test_timeout_policy.py -q
```
Expected: all passed (build order + cabinet sort, source≠target, drain-then-FAILURE-without-raising, READ+WRITE queue, vlm vs hardcoded placement_mode; deadline boundary + latch + monotonic default).

6. [ ] Add a quick record_event self-check to the work-queue test file and re-run (proves the score-trace shape consumed by Task B4's `process_result`). Append to `test/test_work_queue.py`:
```python
def test_record_event_appends_to_score_trace():
    from behavior_tree.PickAndPlace.custom_nodes import record_event
    record_event(None, phase="table", item="bowl", action="scan_and_place",
                 outcome="success", points_est=40)
    trace = _read(c.KEY_SCORE_TRACE)
    assert isinstance(trace, dict)
    assert trace["events"][-1] == {
        "phase": "table", "item": "bowl", "action": "scan_and_place",
        "outcome": "success", "points_est": 40,
    }
    assert "visited_phases" in trace and "place_policy" in trace
```
Re-run:
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
python -m pytest test/test_work_queue.py test/test_timeout_policy.py -q
```
Expected: all passed.

7. [ ] Confirm Task B4's lazy `record_event` import now resolves (no cycle) — sanity check:
```
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && \
source /home/tinker/tk25_ws/src/tk25_decision/.venv_decision/bin/activate && \
BT_MOCK_MODE=true python -c "from behavior_tree.PickAndPlace.custom_nodes import record_event; from behavior_tree.TemplateNodes.Manipulation import BtNode_ScanAndPlace; print('no cycle OK')"
```
Expected: `no cycle OK`.

8. [ ] Commit:
```
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/PickAndPlace/custom_nodes.py \
        src/behavior_tree/test/test_work_queue.py \
        src/behavior_tree/test/test_timeout_policy.py && \
git commit -m "$(cat <<'EOF'
PickAndPlace(B5): inventory/queue/guard/score nodes + record_event

Add plain-Behaviour BtNode_BuildInventory (classify + cabinet-category sort +
mock-seed), BtNode_PopWorkItem (place_policy-aware placement_mode/fixed_target,
FAILURE on empty queue as the loop terminator), BtNode_DeadlineGuard (injectable
clock, latch in initialise, RUNNING->SUCCESS), BtNode_GuardActiveClass,
BtNode_MarkPhase, and record_event (score-trace
visited_phases/events/place_policy). None registered in mock_config (plain
Behaviours run real logic in mock). Pure-ish pytest for work queue + timeout.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"
```

**Deliverable:** `custom_nodes` provides the five plain Behaviours + `record_event`; build order/sort, source≠target, drain-then-FAILURE, READ+WRITE access, vlm-vs-hardcoded placement, and the fake-clock deadline boundary all pass under pytest with no ROS server, and Task B4's lazy `record_event` import resolves without an import cycle.

---

## Phase C — Rulebook tree, cli, samplings, integration test (`behavior_tree`)

I have everything I need. Here are the Phase C tasks.

---

### Task C1: `pick_and_place_rulebook.py` — rulebook tree factory, phase factories, helpers

**Files:**
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/pick_and_place_rulebook.py`

**Interfaces:**
- Consumes (Phase A config.py): `SCAN_AND_PLACE_ACTION_NAME`, `PLACEMENT_MODE_FIXED_POINT`, `DESTINATION_ROUTING`, `KEY_POSE_EXTRA_SURFACE`, `KEY_POINT_EXTRA_SURFACE`, `POSE_EXTRA_SURFACE`, `POINT_EXTRA_SURFACE`, `TABLE_BUDGET_SEC`, `BREAKFAST_BUDGET_SEC`, `EXTRA_BUDGET_SEC`; reused: `KEY_POSE_TABLE/KITCHEN_SHELF/CABINET`, `KEY_ARM_TABLE/WASH/CABINET/TRASH`, `KEY_ACTIVE_SOURCE_POSE/TARGET_POSE/OBJECT_CLASS/PROMPT`, `KEY_VISION_RESULT`, `KEY_OBJECT_LABEL`, `KEY_SCORE_TRACE`, `KEY_SUMMARY_MESSAGE`, `KEY_SCAN_RESULTS_TABLE`, `KEY_INVENTORY_TABLE`, `KEY_WORK_QUEUE`, `KEY_POINT_BREAKFAST_BOWL/SPOON/CEREAL/MILK`, `MAX_RUNTIME_SEC`, `GRASP_RETRY_LIMIT`, `TABLE_SCAN_PROMPT`.
- Consumes (Phase B custom_nodes.py): `BtNode_BuildInventory(name,in_key,out_inventory,out_queue,source_pose_key,mock_seed)`, `BtNode_PopWorkItem(name,queue,place_policy)`, `BtNode_DeadlineGuard(name,budget_sec,clock)`, `BtNode_GuardActiveClass(name,expected,key)`, `BtNode_MarkPhase(name,phase,key)`, `record_event(blackboard,phase,item,action,outcome,points_est=0)`.
- Consumes (Phase B Manipulation.py): `BtNode_ScanAndPlace(name, bb_item_description=KEY_OBJECT_LABEL, bb_placement_mode='pp_active_placement_mode', bb_fixed_target=KEY_ACTIVE_TARGET_POINT, …, action_name=SCAN_AND_PLACE_ACTION_NAME)`.
- Reuses (pick_and_place.py): `createConstantWriter`, `enterArena`, `_gotoRetryWith_Announcement`, `_moveArmRetry`, `_scanForGeneralistRetry`.
- Produces: `pickAndPlaceRulebook(place_policy='vlm') -> py_trees root`, plus `createConstantWriter`, `phaseEnterArena`, `phaseTableCleanup`, `phaseServeBreakfast`, `phaseExtraSurfaceCleanup`, `phaseSummary`, `missionPhases`, `budgeted`, `handleOneItem`, `trashRelease`, `maybeHelpOrSkip`, `BREAKFAST`.

Steps:

1. [ ] Write the failing structural test (proves the module imports and the frozen composites are wired). Append to a new scratch test `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_rulebook_structure.py`:

```python
import os
os.environ.setdefault("BT_MOCK_MODE", "true")
import py_trees
from behavior_tree.PickAndPlace import pick_and_place_rulebook as R


def test_root_is_memory_sequence_with_four_children():
    root = R.pickAndPlaceRulebook(place_policy="vlm")
    assert isinstance(root, py_trees.composites.Sequence)
    assert root.memory is True
    assert len(root.children) == 4  # constants, enter-arena, mission-parallel, summary


def test_mission_is_success_on_one_parallel_with_deadline_and_failureissuccess():
    root = R.pickAndPlaceRulebook()
    par = root.children[2]
    assert isinstance(par, py_trees.composites.Parallel)
    assert isinstance(par.policy, py_trees.common.ParallelPolicy.SuccessOnOne)
    guard, mission = par.children
    assert guard.__class__.__name__ == "BtNode_DeadlineGuard"
    assert isinstance(mission, py_trees.decorators.FailureIsSuccess)


def test_cleanup_loop_is_repeat_minus_one_over_unwrapped_pop():
    phase = R.phaseTableCleanup("vlm")
    repeat = phase.children[-1]
    assert isinstance(repeat, py_trees.decorators.Repeat)
    body = repeat.decorated
    # PopWorkItem must be UNWRAPPED (the only node allowed to FAIL the body).
    assert body.children[0].__class__.__name__ == "BtNode_PopWorkItem"
    assert isinstance(body.children[1], py_trees.decorators.FailureIsSuccess)


def test_breakfast_table_is_the_frozen_four():
    assert [row[0] for row in R.BREAKFAST] == ["bowl", "spoon", "cereal", "milk"]
```

2. [ ] Run it, expect FAIL (module does not exist yet):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_rulebook_structure.py -q
```
Expected: `ModuleNotFoundError: No module named 'behavior_tree.PickAndPlace.pick_and_place_rulebook'`.

3. [ ] Create the module with full code:

```python
from __future__ import annotations

"""Pick-and-Place rulebook mission tree (RoboCup@Home 2026 §5.2).

Three main scored goals: (1) dining-table cleanup, (2) serve breakfast,
(3) extra-surface cleanup. Cleanup is data-driven (inventory -> queue ->
generic per-item loop); breakfast is an explicit fixed-point 4-item table.

`place_policy` ('vlm' default | 'hardcoded') is threaded to every surface
place leaf via BtNode_PopWorkItem; it is inert under BT_MOCK_MODE (place nodes
auto-succeed) and matters only on the real robot. The old narrow demo lives on
in pick_and_place.pickAndPlaceShortened (entry `pick-and-place-demo`).
"""

import py_trees

from behavior_tree.TemplateNodes.Audio import BtNode_Announce
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard
from behavior_tree.TemplateNodes.Manipulation import (
    BtNode_Grasp,
    BtNode_GripperAction,
    BtNode_ScanAndPlace,
)

from .config import (
    GRASP_RETRY_LIMIT,
    KEY_ARM_CABINET,
    KEY_ARM_TABLE,
    KEY_ARM_TRASH,
    KEY_ARM_WASH,
    KEY_ACTIVE_OBJECT_CLASS,
    KEY_ACTIVE_SOURCE_POSE,
    KEY_ACTIVE_TARGET_POSE,
    KEY_ANNOUNCEMENT_MSG,
    KEY_INVENTORY_TABLE,
    KEY_OBJECT_LABEL,
    KEY_POINT_BREAKFAST_BOWL,
    KEY_POINT_BREAKFAST_CEREAL,
    KEY_POINT_BREAKFAST_MILK,
    KEY_POINT_BREAKFAST_SPOON,
    KEY_POINT_EXTRA_SURFACE,
    KEY_POSE_CABINET,
    KEY_POSE_EXTRA_SURFACE,
    KEY_POSE_KITCHEN_SHELF,
    KEY_POSE_TABLE,
    KEY_SCAN_RESULTS_TABLE,
    KEY_SCORE_TRACE,
    KEY_SUMMARY_MESSAGE,
    KEY_VISION_RESULT,
    KEY_WORK_QUEUE,
    MAX_RUNTIME_SEC,
    PLACEMENT_MODE_FIXED_POINT,
    POINT_EXTRA_SURFACE,
    POSE_EXTRA_SURFACE,
    TABLE_BUDGET_SEC,
    BREAKFAST_BUDGET_SEC,
    EXTRA_BUDGET_SEC,
    TABLE_SCAN_PROMPT,
)
from .custom_nodes import (
    BtNode_BuildInventory,
    BtNode_DeadlineGuard,
    BtNode_GuardActiveClass,
    BtNode_MarkPhase,
    BtNode_PopWorkItem,
    BtNode_WriteFoundItems,
    record_event,
)
from .pick_and_place import (
    createConstantWriter as _writeBaseConstants,
    enterArena,
    _gotoRetryWith_Announcement,
    _moveArmRetry,
    _scanForGeneralistRetry,
)

# 'pp_active_placement_mode' is BtNode_ScanAndPlace's default bb_placement_mode
# key. PopWorkItem writes it per-policy for cleanup; breakfast writes FIXED_POINT.
_KEY_ACTIVE_PLACEMENT_MODE = "pp_active_placement_mode"

# Frozen breakfast table: (item, source_pose_key, arm_pose_key, point_key).
BREAKFAST = [
    ("bowl", KEY_POSE_KITCHEN_SHELF, KEY_ARM_TABLE, KEY_POINT_BREAKFAST_BOWL),
    ("spoon", KEY_POSE_KITCHEN_SHELF, KEY_ARM_TABLE, KEY_POINT_BREAKFAST_SPOON),
    ("cereal", KEY_POSE_CABINET, KEY_ARM_CABINET, KEY_POINT_BREAKFAST_CEREAL),
    ("milk", KEY_POSE_CABINET, KEY_ARM_CABINET, KEY_POINT_BREAKFAST_MILK),
]


# --------------------------------------------------------------------------- #
# Plain-Behaviour leaves (run REAL logic in mock; NEVER listed in mock_config) #
# --------------------------------------------------------------------------- #


class _RecordEventLeaf(py_trees.behaviour.Behaviour):
    """Append one score-trace event for the active item via record_event().

    Plain Behaviour: under BT_MOCK_MODE it runs its real logic (it is not a
    Handler and must NOT be registered in mock_config). Always SUCCESS.
    """

    def __init__(self, name, *, action, outcome, phase, points_est=0):
        super().__init__(name=name)
        self._action = action
        self._outcome = outcome
        self._phase = phase
        self._points_est = points_est
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(
            key="label",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_OBJECT_LABEL),
        )
        # record_event reads+appends the score-trace through this client.
        self.bb.register_key(
            key="score",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SCORE_TRACE),
        )
        self.bb.register_key(
            key="score_r",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SCORE_TRACE),
        )

    def update(self):
        try:
            item = str(self.bb.label)
        except Exception:  # pragma: no cover - label unset in standalone runs
            item = ""
        record_event(self.bb, self._phase, item, self._action, self._outcome, self._points_est)
        return py_trees.common.Status.SUCCESS


class _SummarizeScoreLeaf(py_trees.behaviour.Behaviour):
    """Build a spoken rollup from the score-trace; write it to KEY_SUMMARY_MESSAGE."""

    def __init__(self, name):
        super().__init__(name=name)
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(
            key="score",
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SCORE_TRACE),
        )
        self.bb.register_key(
            key="summary",
            access=py_trees.common.Access.WRITE,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", KEY_SUMMARY_MESSAGE),
        )

    def update(self):
        try:
            trace = self.bb.score or {}
        except Exception:
            trace = {}
        events = trace.get("events", [])
        placed = sum(1 for e in events if e.get("action") == "place")
        points = sum(int(e.get("points_est", 0) or 0) for e in events)
        phases = ", ".join(trace.get("visited_phases", [])) or "none"
        self.bb.summary = (
            f"Mission summary. Visited phases: {phases}. "
            f"Placed {placed} item{'s' if placed != 1 else ''}. "
            f"Estimated {points} points."
        )
        return py_trees.common.Status.SUCCESS


# --------------------------------------------------------------------------- #
# Small wiring helpers                                                         #
# --------------------------------------------------------------------------- #


def _goto(label, pose_key):
    return _gotoRetryWith_Announcement(label, pose_key)


def _arm(label, arm_key):
    return _moveArmRetry(label, arm_key, add_octomap=True)


def _reDetectActive():
    # NOTE(hardware): narrow the prompt to KEY_ACTIVE_PROMPT once the generalist
    # node exposes a bb-sourced prompt; the literal is mock-inert.
    # Generic re-detect is acceptable because Grasp targets the popped item via
    # object_label=KEY_OBJECT_LABEL; narrowing the scan prompt to the per-item
    # label is an on-robot refinement.
    return _scanForGeneralistRetry(
        name="re-detect active item",
        bb_source=None,
        bb_key=KEY_VISION_RESULT,
        object=TABLE_SCAN_PROMPT,
        use_orbbec=False,
    )


# --------------------------------------------------------------------------- #
# Constant writer + summary                                                    #
# --------------------------------------------------------------------------- #


def createConstantWriter(place_policy="vlm"):
    """Reuse the base PP constant writer, add extra-surface + score-trace init."""
    seq = py_trees.composites.Sequence(
        "write constants + init score-trace", memory=True
    )
    seq.add_child(_writeBaseConstants())
    seq.add_child(
        BtNode_WriteToBlackboard(
            name="write extra-surface pose",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_POSE_EXTRA_SURFACE,
            object=POSE_EXTRA_SURFACE,
        )
    )
    seq.add_child(
        BtNode_WriteToBlackboard(
            name="write extra-surface point",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_POINT_EXTRA_SURFACE,
            object=POINT_EXTRA_SURFACE,
        )
    )
    seq.add_child(
        BtNode_WriteToBlackboard(
            name="init score-trace",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_SCORE_TRACE,
            object={"visited_phases": [], "events": [], "place_policy": place_policy},
        )
    )
    return seq


def phaseSummary():
    seq = py_trees.composites.Sequence("phase: summary", memory=True)
    seq.add_child(_SummarizeScoreLeaf(name="summarize score-trace"))
    seq.add_child(
        BtNode_Announce(name="announce summary", bb_source=KEY_SUMMARY_MESSAGE)
    )
    return py_trees.decorators.FailureIsSuccess(
        name="summary (always success)", child=seq
    )


# --------------------------------------------------------------------------- #
# Enter arena                                                                  #
# --------------------------------------------------------------------------- #


def phaseEnterArena():
    # Reuse: Retry(cap) > BtNode_DoorDetection, wrapped FailureIsSuccess.
    return enterArena()


# --------------------------------------------------------------------------- #
# Per-item handling (cleanup + extra-surface)                                  #
# --------------------------------------------------------------------------- #


def trashRelease(phase="cleanup"):
    """Kinematic controlled release over the bin (low) + best-effort gripper open.

    NOTE: if a dedicated `start_drop` server lands, swap the gripper open for
    BtNode_Drop. Records a 'place' event so trashed items count in the trace.
    """
    seq = py_trees.composites.Sequence("trash release", memory=True)
    seq.add_child(
        BtNode_Announce(
            name="announce trash release",
            bb_source=None,
            message="Releasing item into the trash bin.",
        )
    )
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="open gripper over bin (best effort)",
            child=BtNode_GripperAction(name="open gripper over bin", open_gripper=True),
        )
    )
    seq.add_child(
        _RecordEventLeaf(
            name="record trash place", action="place", outcome="released",
            phase=phase, points_est=40
        )
    )
    return seq


def maybeHelpOrSkip(allow_human_assistance=False, phase="cleanup"):
    """Terminal always-SUCCESS fallback: skip the item (partial credit) by default,
    or request human assistance if `allow_human_assistance`. Records a 'skip' event."""
    if allow_human_assistance:
        msg = "Please hand me or reposition the item."
        outcome = "assist_requested"
    else:
        msg = "Skipping this item."
        outcome = "skipped"
    seq = py_trees.composites.Sequence("maybe help or skip", memory=True)
    seq.add_child(
        _RecordEventLeaf(
            name="record skip/help", action="skip", outcome=outcome,
            phase=phase, points_est=0
        )
    )
    seq.add_child(
        BtNode_Announce(name="announce skip/help", bb_source=None, message=msg)
    )
    return py_trees.decorators.FailureIsSuccess(
        name="skip/help is always success", child=seq
    )


def _routeByDestination(place_policy, phase="cleanup"):
    """Selector of guard-routed destination branches. PopWorkItem has already set
    KEY_ACTIVE_OBJECT_CLASS + the per-policy placement_mode/fixed_target, so the
    place leaves are policy-agnostic (vlm -> FREE_SPACE/NEAR_SIMILAR, hardcoded ->
    FIXED_POINT)."""
    sel = py_trees.composites.Selector("route by destination", memory=True)

    wash = py_trees.composites.Sequence("route wash-staging", memory=True)
    wash.add_child(
        BtNode_GuardActiveClass(name="is wash_staging?", expected="wash_staging")
    )
    wash.add_child(_goto("wash staging", KEY_ACTIVE_TARGET_POSE))
    wash.add_child(_arm("arm to wash place", KEY_ARM_WASH))
    wash.add_child(BtNode_ScanAndPlace(name="place at wash-staging"))
    wash.add_child(
        _RecordEventLeaf(
            name="record wash place", action="place", outcome="placed",
            phase=phase, points_est=40
        )
    )
    sel.add_child(wash)

    cab = py_trees.composites.Sequence("route cabinet", memory=True)
    cab.add_child(BtNode_GuardActiveClass(name="is cabinet?", expected="cabinet"))
    cab.add_child(_goto("cabinet", KEY_ACTIVE_TARGET_POSE))
    cab.add_child(_arm("arm to cabinet place", KEY_ARM_CABINET))
    cab.add_child(BtNode_ScanAndPlace(name="place at cabinet (grouped)"))
    cab.add_child(
        _RecordEventLeaf(
            name="record cabinet place", action="place", outcome="placed",
            phase=phase, points_est=40
        )
    )
    sel.add_child(cab)

    tr = py_trees.composites.Sequence("route trash", memory=True)
    tr.add_child(BtNode_GuardActiveClass(name="is trash?", expected="trash"))
    tr.add_child(_goto("trash bin", KEY_ACTIVE_TARGET_POSE))
    tr.add_child(_arm("arm to trash", KEY_ARM_TRASH))
    tr.add_child(trashRelease(phase))
    sel.add_child(tr)

    return sel


def handleOneItem(place_policy="vlm", phase="cleanup"):
    """Source-nav + arm + re-detect + announce + grasp + route, with a terminal
    skip fallback.

    The grasp + route sit under an outer Selector whose terminal child is
    maybeHelpOrSkip(): a grasp FAILURE OR an unroutable class falls through to
    the skip leaf (records 'skip', returns SUCCESS). So the only way handleOneItem
    FAILS is a goto/arm/re-detect failure above the Selector, which the cleanup
    loop's FailureIsSuccess masks. `place_policy` is consumed by PopWorkItem, not
    here; it is inert in mock.
    """
    body = py_trees.composites.Sequence("handle one item", memory=True)
    body.add_child(_goto("active item source", KEY_ACTIVE_SOURCE_POSE))
    body.add_child(_arm("arm to scan (active item)", KEY_ARM_TABLE))
    body.add_child(_reDetectActive())
    # Per-object perception+destination announce (speaks the popped label).
    body.add_child(
        BtNode_Announce(name="announce active item", bb_source=KEY_OBJECT_LABEL)
    )

    happy = py_trees.composites.Sequence("grasp then route", memory=True)
    happy.add_child(
        py_trees.decorators.Retry(
            name="Retry grasp active item",
            child=BtNode_Grasp(
                name="grasp active item",
                bb_key_vision_res=KEY_VISION_RESULT,
                bb_key_object_label=KEY_OBJECT_LABEL,
            ),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    happy.add_child(_routeByDestination(place_policy, phase))

    grasp_or_skip = py_trees.composites.Selector("grasp+route or skip", memory=True)
    grasp_or_skip.add_child(happy)
    grasp_or_skip.add_child(maybeHelpOrSkip(phase=phase))  # terminal, always SUCCESS
    body.add_child(grasp_or_skip)
    return body


def _cleanupLoop(place_policy="vlm", phase="cleanup"):
    body = py_trees.composites.Sequence("cleanup loop body", memory=True)
    # PopWorkItem is UNWRAPPED — the one node allowed to FAIL the body. It FAILS
    # exactly on an empty queue, which is the loop's only exit.
    body.add_child(BtNode_PopWorkItem(name="pop work item", place_policy=place_policy))
    body.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="item failure -> continue loop", child=handleOneItem(place_policy, phase)
        )
    )
    # LOAD-BEARING: loop exit == child FAILURE (empty queue). num_success=-1 never
    # reaches the success count, so Repeat re-ticks until the body FAILS. The
    # FailureIsSuccess(handleOneItem) above is REQUIRED — without it, a per-item
    # failure would FAIL the body and break the loop early (or, worse, if the body
    # could not fail, the loop would be infinite). Do not remove it.
    return py_trees.decorators.Repeat(
        name="drain work queue (num_success=-1)", child=body, num_success=-1
    )


# --------------------------------------------------------------------------- #
# Phases                                                                       #
# --------------------------------------------------------------------------- #


def phaseTableCleanup(place_policy="vlm"):
    seq = py_trees.composites.Sequence("phase: table cleanup", memory=True)
    seq.add_child(BtNode_MarkPhase(name="mark table", phase="table"))
    seq.add_child(_goto("dining table", KEY_POSE_TABLE))
    seq.add_child(_arm("arm to table scan", KEY_ARM_TABLE))
    seq.add_child(
        _scanForGeneralistRetry(
            name="scan table for cleanup",
            bb_source=None,
            bb_key=KEY_SCAN_RESULTS_TABLE,
            object=TABLE_SCAN_PROMPT,
            use_orbbec=True,
        )
    )
    # Perception summary (spec §8.3): announce what the scan found before building
    # the inventory. WriteFoundItems is a plain Behaviour (runs real logic in mock;
    # empty objects -> "could not find any objects", still SUCCESS).
    seq.add_child(
        BtNode_WriteFoundItems(
            name="write found table items",
            bb_key_vision_res=KEY_SCAN_RESULTS_TABLE,
            bb_key_announcement=KEY_ANNOUNCEMENT_MSG,
        )
    )
    seq.add_child(
        BtNode_Announce(name="announce found table items", bb_source=KEY_ANNOUNCEMENT_MSG)
    )
    seq.add_child(
        BtNode_BuildInventory(
            name="build table inventory",
            in_key=KEY_SCAN_RESULTS_TABLE,
            out_inventory=KEY_INVENTORY_TABLE,
            out_queue=KEY_WORK_QUEUE,
            source_pose_key=KEY_POSE_TABLE,
            mock_seed=None,  # BuildInventory seeds a canned queue under mock if empty
        )
    )
    seq.add_child(_cleanupLoop(place_policy, "table"))
    return seq


def _breakfastItem(item, src_key, arm_key, point_key):
    s = py_trees.composites.Sequence(f"retrieve+place {item}", memory=True)
    s.add_child(_goto(f"{item} source", src_key))
    s.add_child(_arm(f"arm to {item} retrieve", arm_key))
    s.add_child(
        _scanForGeneralistRetry(
            name=f"re-detect {item}",
            bb_source=None,
            bb_key=KEY_VISION_RESULT,
            object=item,
            use_orbbec=False,
        )
    )
    s.add_child(
        BtNode_Announce(
            name=f"announce retrieving {item}",
            bb_source=None,
            message=f"Retrieving {item}.",
        )
    )
    s.add_child(
        py_trees.decorators.Retry(
            name=f"Retry grasp {item}",
            child=BtNode_Grasp(
                name=f"grasp {item}", bb_key_vision_res=KEY_VISION_RESULT
            ),
            num_failures=GRASP_RETRY_LIMIT,
        )
    )
    s.add_child(_goto("clean table (breakfast)", KEY_POSE_TABLE))
    s.add_child(_arm("arm to table place (breakfast)", KEY_ARM_TABLE))
    # Breakfast is ALWAYS FIXED_POINT at the item's own point (policy-independent).
    s.add_child(
        BtNode_WriteToBlackboard(
            name=f"set FIXED_POINT mode ({item})",
            bb_namespace="",
            bb_source=None,
            bb_key=_KEY_ACTIVE_PLACEMENT_MODE,
            object=PLACEMENT_MODE_FIXED_POINT,
        )
    )
    s.add_child(
        BtNode_ScanAndPlace(name=f"place {item} (FIXED_POINT)", bb_fixed_target=point_key)
    )
    s.add_child(
        _RecordEventLeaf(
            name=f"record place {item}", action="place", outcome="placed",
            phase="breakfast", points_est=40
        )
    )
    return s


def phaseServeBreakfast(place_policy="vlm"):
    seq = py_trees.composites.Sequence("phase: serve breakfast", memory=True)
    seq.add_child(BtNode_MarkPhase(name="mark breakfast", phase="breakfast"))
    for item, src_key, arm_key, point_key in BREAKFAST:
        # Per-item FailureIsSuccess: one failed retrieval doesn't abort breakfast.
        seq.add_child(
            py_trees.decorators.FailureIsSuccess(
                name=f"breakfast {item} (best effort)",
                child=_breakfastItem(item, src_key, arm_key, point_key),
            )
        )
    seq.add_child(
        BtNode_Announce(
            name="announce breakfast served",
            bb_source=None,
            message="Breakfast is served.",
        )
    )
    return seq


def phaseExtraSurfaceCleanup(place_policy="vlm"):
    seq = py_trees.composites.Sequence("phase: extra-surface cleanup", memory=True)
    seq.add_child(BtNode_MarkPhase(name="mark extra", phase="extra"))
    seq.add_child(_goto("extra surface", KEY_POSE_EXTRA_SURFACE))
    seq.add_child(_arm("arm to extra-surface scan", KEY_ARM_TABLE))
    seq.add_child(
        _scanForGeneralistRetry(
            name="scan extra surface",
            bb_source=None,
            bb_key=KEY_SCAN_RESULTS_TABLE,
            object=TABLE_SCAN_PROMPT,
            use_orbbec=True,
        )
    )
    # Perception summary (spec §8.3): announce found items before BuildInventory.
    seq.add_child(
        BtNode_WriteFoundItems(
            name="write found extra items",
            bb_key_vision_res=KEY_SCAN_RESULTS_TABLE,
            bb_key_announcement=KEY_ANNOUNCEMENT_MSG,
        )
    )
    seq.add_child(
        BtNode_Announce(name="announce found extra items", bb_source=KEY_ANNOUNCEMENT_MSG)
    )
    seq.add_child(
        BtNode_BuildInventory(
            name="build extra inventory",
            in_key=KEY_SCAN_RESULTS_TABLE,
            out_inventory=KEY_INVENTORY_TABLE,
            out_queue=KEY_WORK_QUEUE,
            source_pose_key=KEY_POSE_EXTRA_SURFACE,
            mock_seed=None,
        )
    )
    seq.add_child(_cleanupLoop(place_policy, "extra"))
    return seq


# --------------------------------------------------------------------------- #
# Mission assembly                                                             #
# --------------------------------------------------------------------------- #


def budgeted(body, budget_sec, label="phase"):
    par = py_trees.composites.Parallel(
        name=f"budgeted {label}",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    par.add_child(BtNode_DeadlineGuard(name=f"{label} deadline", budget_sec=budget_sec))
    par.add_child(
        py_trees.decorators.FailureIsSuccess(
            name=f"{label} body (never fails the budget)", child=body
        )
    )
    return par


def missionPhases(place_policy="vlm"):
    seq = py_trees.composites.Sequence("mission phases", memory=True)
    seq.add_child(budgeted(phaseTableCleanup(place_policy), TABLE_BUDGET_SEC, "table cleanup"))
    seq.add_child(
        budgeted(phaseServeBreakfast(place_policy), BREAKFAST_BUDGET_SEC, "serve breakfast")
    )
    seq.add_child(
        budgeted(phaseExtraSurfaceCleanup(place_policy), EXTRA_BUDGET_SEC, "extra cleanup")
    )
    # EXTENSION HOOK: a future optional-goals phase (dishwasher / pour / tablet)
    # would attach here, after extra-surface cleanup. No code (out of scope).
    return seq


def pickAndPlaceRulebook(place_policy="vlm"):
    root = py_trees.composites.Sequence("Pick and Place (rulebook)", memory=True)
    root.add_child(createConstantWriter(place_policy))
    root.add_child(phaseEnterArena())

    mission_par = py_trees.composites.Parallel(
        name="mission under global deadline",
        policy=py_trees.common.ParallelPolicy.SuccessOnOne(),
    )
    # DeadlineGuard returns RUNNING->SUCCESS (never FAILURE); the mission is
    # FailureIsSuccess-wrapped, so the Parallel NEVER sees FAILURE from either
    # child. Normal finish: mission SUCCEEDS -> SuccessOnOne -> SUCCESS. Timeout:
    # guard SUCCEEDS -> SUCCESS (running place leaf cancelled by terminate). Either
    # way phaseSummary (OUTSIDE the guard) still runs.
    mission_par.add_child(
        BtNode_DeadlineGuard(name="global deadline", budget_sec=MAX_RUNTIME_SEC)
    )
    mission_par.add_child(
        py_trees.decorators.FailureIsSuccess(
            name="mission never fails the parallel", child=missionPhases(place_policy)
        )
    )
    root.add_child(mission_par)
    root.add_child(phaseSummary())
    return root
```

4. [ ] Run the structural test, expect PASS:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_rulebook_structure.py -q
```
Expected: `4 passed`. (If `ImportError` on a Phase A/B symbol, Phases A+B are incomplete — fix there, not here.)

5. [ ] Commit (branch `dev`, selective add):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/PickAndPlace/pick_and_place_rulebook.py \
        src/behavior_tree/test/test_rulebook_structure.py && \
git commit -m "PickAndPlace: rulebook tree factory + phases + helpers (C1)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN"
```

**Deliverable:** `pickAndPlaceRulebook('vlm')` constructs with the frozen root/mission/loop composites and BREAKFAST table; structural test green.

---

### Task C2: `cli.py` repoint + `--place-policy`, and `cli_demo.py` for the old tree

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/cli.py`
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/cli_demo.py`

**Interfaces:**
- Consumes: `pickAndPlaceRulebook(place_policy=...)` (C1), `pickAndPlaceShortened()` (existing), `run_tree(root_factory, *, period_ms, title, node_name='root_node')`.
- Produces: `cli.main()` parsing `--place-policy {hardcoded,vlm}` (default `vlm`); `cli_demo.main()`.

Steps:

1. [ ] Write the failing arg-parse test `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_pp_cli_args.py`:

```python
import os
os.environ.setdefault("BT_MOCK_MODE", "true")
from behavior_tree.PickAndPlace.cli import _build_parser


def test_place_policy_default_is_vlm():
    args = _build_parser().parse_args([])
    assert args.place_policy == "vlm"


def test_place_policy_accepts_hardcoded():
    args = _build_parser().parse_args(["--place-policy", "hardcoded"])
    assert args.place_policy == "hardcoded"


def test_place_policy_rejects_unknown():
    import pytest
    with pytest.raises(SystemExit):
        _build_parser().parse_args(["--place-policy", "vision"])
```

2. [ ] Run it, expect FAIL:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_pp_cli_args.py -q
```
Expected: `ImportError: cannot import name '_build_parser'`.

3. [ ] Replace `cli.py` with:

```python
import argparse

from behavior_tree.runtime import run_tree


def _build_parser():
    parser = argparse.ArgumentParser(prog="pick-and-place")
    parser.add_argument(
        "--place-policy",
        choices=["hardcoded", "vlm"],
        default="vlm",
        help="Surface-place strategy: 'vlm' (competition, FREE_SPACE/NEAR_SIMILAR) "
        "or 'hardcoded' (on-robot bring-up, FIXED_POINT, no VLM/network).",
    )
    return parser


def main():
    # parse_known_args so ros2 run's --ros-args ... pass through untouched.
    args, _ = _build_parser().parse_known_args()
    from .pick_and_place_rulebook import pickAndPlaceRulebook

    run_tree(
        lambda: pickAndPlaceRulebook(place_policy=args.place_policy),
        period_ms=500.0,
        title=f"Pick And Place (rulebook, place_policy={args.place_policy})",
    )
```

4. [ ] Create `cli_demo.py`:

```python
from behavior_tree.runtime import run_tree


def main():
    """Run the legacy narrow demo tree (pickAndPlaceShortened), preserved as-is."""
    from .pick_and_place import pickAndPlaceShortened

    run_tree(pickAndPlaceShortened, period_ms=500.0, title="Pick And Place (demo)")
```

5. [ ] Run the test, expect PASS:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_pp_cli_args.py -q
```
Expected: `3 passed`.

6. [ ] Commit:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/PickAndPlace/cli.py \
        src/behavior_tree/behavior_tree/PickAndPlace/cli_demo.py \
        src/behavior_tree/test/test_pp_cli_args.py && \
git commit -m "PickAndPlace: cli repoint to rulebook + --place-policy, cli_demo for old tree (C2)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN"
```

**Deliverable:** `--place-policy` parses (default `vlm`); the legacy tree reachable via `cli_demo.main`.

---

### Task C3: `setup.py` entry points (repoint + samplings + demo)

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/setup.py` (console_scripts list, around line 72)

**Interfaces:**
- Consumes: `cli:main` (C2), `cli_demo:main` (C2), `samplings:main_scan_place/main_categorize/main_cleanup_loop/main_breakfast` (C4 — entry strings registered now, modules land in C4).
- Produces frozen entry points: `pick-and-place` (repointed, unchanged string), `pick-and-place-demo`, `pp-test-scan-place`, `pp-test-categorize`, `pp-test-cleanup-loop`, `pp-test-breakfast`.

Steps:

1. [ ] The existing line stays (`cli:main` already repoints internally via C2). Add the five new entries directly after the `pick-and-place` line (setup.py:72). Edit:

old:
```python
            "pick-and-place = behavior_tree.PickAndPlace.cli:main",
```
new:
```python
            "pick-and-place = behavior_tree.PickAndPlace.cli:main",
            "pick-and-place-demo = behavior_tree.PickAndPlace.cli_demo:main",
            "pp-test-scan-place = behavior_tree.PickAndPlace.samplings:main_scan_place",
            "pp-test-categorize = behavior_tree.PickAndPlace.samplings:main_categorize",
            "pp-test-cleanup-loop = behavior_tree.PickAndPlace.samplings:main_cleanup_loop",
            "pp-test-breakfast = behavior_tree.PickAndPlace.samplings:main_breakfast",
```

2. [ ] Verify the setup.py still parses (no build yet — samplings module lands in C4, so a full build is deferred to after C4):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && python -c "import ast; ast.parse(open('setup.py').read()); print('setup.py OK')"
```
Expected: `setup.py OK`.

3. [ ] Commit (entry strings only; the resolve-check happens in C4 after the modules exist):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/setup.py && \
git commit -m "PickAndPlace: register rulebook + demo + pp-test-* entry points (C3)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN"
```

**Deliverable:** all six console_scripts present; entry-point resolution verified at the end of C4 via `tkbuild` + `ros2 run … --help`.

---

### Task C4: `samplings.py` — `main_scan_place / main_categorize / main_cleanup_loop / main_breakfast`

**Files:**
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/samplings.py`

**Interfaces:**
- Consumes: `createConstantWriter`, `phaseTableCleanup`, `phaseServeBreakfast`, `handleOneItem` (C1); `BtNode_BuildInventory`, `BtNode_PopWorkItem`, `BtNode_ScanAndPlace` (Phase B); `run_tree`; config keys `KEY_OBJECT_LABEL`, `KEY_ACTIVE_TARGET_POINT`, `KEY_INVENTORY_TABLE`, `KEY_WORK_QUEUE`, `KEY_SCAN_RESULTS_TABLE`, `KEY_POSE_TABLE`, `POINT_WASH_STAGING`, `PLACEMENT_MODE_FREE_SPACE`.
- Produces: `main_scan_place()`, `main_categorize()`, `main_cleanup_loop()`, `main_breakfast()` — each seeds the blackboard inside a factory and hands it to `run_tree`.

Steps:

1. [ ] Write a failing import-shape test `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_pp_samplings_build.py`:

```python
import os
os.environ.setdefault("BT_MOCK_MODE", "true")
import py_trees
from behavior_tree.PickAndPlace import samplings as S


def test_all_factory_builders_return_a_tree():
    for builder in (
        S._scan_place_factory,
        S._categorize_factory,
        S._cleanup_loop_factory,
        S._breakfast_factory,
    ):
        root = builder()
        assert isinstance(root, py_trees.behaviour.Behaviour)


def test_mains_are_callable_attrs():
    for name in ("main_scan_place", "main_categorize", "main_cleanup_loop", "main_breakfast"):
        assert callable(getattr(S, name))
```

2. [ ] Run it, expect FAIL:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_pp_samplings_build.py -q
```
Expected: `ModuleNotFoundError: No module named 'behavior_tree.PickAndPlace.samplings'`.

3. [ ] Create `samplings.py`:

```python
"""PickAndPlace samplings — standalone `ros2 run` dev trees (mock-runnable).

Each main_<x> seeds the blackboard inside a factory and hands it to run_tree::

    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-scan-place
    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-categorize
    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-cleanup-loop
    BT_MOCK_MODE=true ros2 run behavior_tree pp-test-breakfast
"""

import os

import py_trees

from behavior_tree.runtime import run_tree
from behavior_tree.TemplateNodes.BaseBehaviors import BtNode_WriteToBlackboard

from .config import (
    KEY_ACTIVE_TARGET_POINT,
    KEY_INVENTORY_TABLE,
    KEY_OBJECT_LABEL,
    KEY_POSE_TABLE,
    KEY_SCAN_RESULTS_TABLE,
    KEY_WORK_QUEUE,
    PLACEMENT_MODE_FREE_SPACE,
    POINT_WASH_STAGING,
)
from .custom_nodes import BtNode_BuildInventory, BtNode_PopWorkItem
from behavior_tree.TemplateNodes.Manipulation import BtNode_ScanAndPlace
from .pick_and_place_rulebook import (
    createConstantWriter,
    handleOneItem,
    phaseServeBreakfast,
    phaseTableCleanup,
)

# 'pp_active_placement_mode' is BtNode_ScanAndPlace's default placement-mode key.
_KEY_ACTIVE_PLACEMENT_MODE = "pp_active_placement_mode"


def _write(name, key, value):
    return BtNode_WriteToBlackboard(
        name=name, bb_namespace="", bb_source=None, bb_key=key, object=value
    )


def _idle():
    return py_trees.behaviours.Running("idle")


# --------------------------------------------------------------------------- #
# pp-test-scan-place — seed a held object + FREE_SPACE mode, tick ScanAndPlace #
# --------------------------------------------------------------------------- #


def _scan_place_factory():
    seq = py_trees.composites.Sequence("pp-test-scan-place", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(_write("seed label", KEY_OBJECT_LABEL, "cup"))
    seq.add_child(_write("seed mode FREE_SPACE", _KEY_ACTIVE_PLACEMENT_MODE, PLACEMENT_MODE_FREE_SPACE))
    seq.add_child(_write("seed fixed target", KEY_ACTIVE_TARGET_POINT, POINT_WASH_STAGING))
    seq.add_child(BtNode_ScanAndPlace(name="scan and place (sampling)"))
    seq.add_child(_idle())
    return seq


def main_scan_place():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_scan_place_factory, period_ms=400.0, title="pp scan-place")


# --------------------------------------------------------------------------- #
# pp-test-categorize — BuildInventory then drain PopWorkItem over a canned set #
# --------------------------------------------------------------------------- #


def _categorize_factory():
    seq = py_trees.composites.Sequence("pp-test-categorize", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(
        BtNode_BuildInventory(
            name="build inventory (canned)",
            in_key=KEY_SCAN_RESULTS_TABLE,
            out_inventory=KEY_INVENTORY_TABLE,
            out_queue=KEY_WORK_QUEUE,
            source_pose_key=KEY_POSE_TABLE,
            # mock_seed is a list of plain label STRINGS — BtNode_BuildInventory
            # does `[(str(s), None) for s in seed]`, so dicts would break it.
            # Routing: fork->wash_staging, paper cup->trash, snack->cabinet.
            mock_seed=["fork", "paper cup", "snack"],
        )
    )
    # Drain the queue: PopWorkItem FAILS on empty -> Repeat FAILS -> FailureIsSuccess.
    drain = py_trees.decorators.Repeat(
        name="drain (num_success=-1)",
        child=BtNode_PopWorkItem(name="pop (sampling)", place_policy="vlm"),
        num_success=-1,
    )
    seq.add_child(
        py_trees.decorators.FailureIsSuccess(name="drained", child=drain)
    )
    seq.add_child(_idle())
    return seq


def main_categorize():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_categorize_factory, period_ms=400.0, title="pp categorize")


# --------------------------------------------------------------------------- #
# pp-test-cleanup-loop — full table-cleanup phase (BuildInventory mock-seeds)  #
# --------------------------------------------------------------------------- #


def _cleanup_loop_factory():
    seq = py_trees.composites.Sequence("pp-test-cleanup-loop", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(phaseTableCleanup("vlm"))
    seq.add_child(_idle())
    return seq


def main_cleanup_loop():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_cleanup_loop_factory, period_ms=300.0, title="pp cleanup-loop")


# --------------------------------------------------------------------------- #
# pp-test-breakfast — serve-breakfast phase alone                             #
# --------------------------------------------------------------------------- #


def _breakfast_factory():
    seq = py_trees.composites.Sequence("pp-test-breakfast", memory=True)
    seq.add_child(createConstantWriter("vlm"))
    seq.add_child(phaseServeBreakfast("vlm"))
    seq.add_child(_idle())
    return seq


def main_breakfast():
    os.environ.setdefault("BT_MOCK_MODE", "true")
    run_tree(_breakfast_factory, period_ms=300.0, title="pp breakfast")
```

4. [ ] Run the build-shape test, expect PASS:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_pp_samplings_build.py -q
```
Expected: `2 passed`. (`handleOneItem` is imported to assert C1 exposes it; the four factories build without a ROS graph.)

5. [ ] Build the package so the new entry points (C3) resolve, then smoke-run each sampling in mock with a timeout, confirming each ticks cleanly with no Python traceback (the trailing idle leaf holds the root RUNNING until the timeout — validate on "no traceback", not root SUCCESS). Builds are user-approved; Claude may run `tkbuild` + the read-only smoke:

```bash
cd /home/tinker/tk25_ws && tkbuild tk25_decision --packages-select behavior_tree && source install/setup.zsh
for e in pp-test-categorize pp-test-breakfast pp-test-cleanup-loop pp-test-scan-place; do
  echo "=== $e ==="; BT_MOCK_MODE=true timeout 30 ros2 run behavior_tree "$e" 2>&1 | tail -5 || true
done
```
Expected: each ticks cleanly with no Python traceback; the phase sub-tree completes and the trailing idle leaf holds the root RUNNING until the 30s timeout — validate on "no traceback", not root SUCCESS. (If `tkbuild`'s exact invocation is unclear, ask the user to build, then run the `ros2 run` smokes.)

6. [ ] Commit:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/behavior_tree/PickAndPlace/samplings.py \
        src/behavior_tree/test/test_pp_samplings_build.py && \
git commit -m "PickAndPlace: pp-test-* samplings (scan-place/categorize/cleanup-loop/breakfast) (C4)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN"
```

**Deliverable:** four samplings build headless and tick green in `BT_MOCK_MODE`; entry points resolve via `ros2 run`.

---

### Task C5: `test_rulebook_tree_mock.py` — whole-tree mock integration (happy path + forced-grasp-FAILURE)

**Files:**
- Create `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/test/test_rulebook_tree_mock.py`

**Interfaces:**
- Consumes: `pickAndPlaceRulebook()` (C1); `record_event` place/skip events recorded by the route leaves + `BtNode_MarkPhase` (Phase B); the mock framework (`behavior_tree.config`).
- Score-trace shape `{'visited_phases': [], 'events': [{phase,item,action,outcome,points_est}], 'place_policy': str}`.
- Produces: two pytest cases under `BT_MOCK_MODE=true` + IMMEDIATE.

Steps:

1. [ ] Write the test (this is the deliverable — write it to FAIL first by running it before C1/Phase-B are wired; here C1+B exist, so it should pass once the tree is correct):

```python
# Copyright 2025 Tinker Team
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

"""Layer-B whole-tree mock integration test for pickAndPlaceRulebook().

Runs the real rulebook tree end-to-end under BT_MOCK_MODE=true with every
subsystem mocked and keyboard control off (KEYPRESS -> IMMEDIATE). The plain
Behaviours (BuildInventory/PopWorkItem/DeadlineGuard/guards/markPhase) run their
real logic; BuildInventory mock-seeds a non-empty queue so the cleanup loop body
actually runs. An explicit tick cap asserts FAIL (not hang) if the tree never
terminates.
"""

import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 must precede config import

import py_trees  # noqa: E402
import py_trees_ros  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402

import behavior_tree.config as btcfg  # noqa: E402

TICK_CAP = 4000  # leaf-count x ~3 mock ticks x phases, with margin


def _force_full_mock():
    """Enable every subsystem so all Handlers mock; keyboard off => IMMEDIATE."""
    cfg = btcfg._config._mock_config
    cfg["mock_mode"]["enabled"] = True
    for sub in cfg["mock_mode"]["subsystems"].values():
        sub["enabled"] = True
    cfg["keyboard_control"]["enabled"] = False


def _get_bb(key):
    client = py_trees.blackboard.Client(name="pp_mock_reader")
    client.register_key(
        key="v",
        access=py_trees.common.Access.READ,
        remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
    )
    return client.v


def _run_to_completion(root, node_name):
    """setup() the tree against a real rclpy node, tick to terminal under a cap."""
    tree = py_trees_ros.trees.BehaviourTree(root)
    tree.setup(node_name=node_name, timeout=15)
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
    # Distinct from the in-tree wall-clock DeadlineGuard: a runaway tree FAILS the
    # test here rather than hanging the suite.
    assert ticks < TICK_CAP, f"tree did not terminate within {TICK_CAP} ticks"
    return root.status, ticks


@pytest.fixture(autouse=True)
def _ros_and_blackboard():
    _force_full_mock()
    py_trees.blackboard.Blackboard.clear()
    if not rclpy.ok():
        rclpy.init()
    yield
    py_trees.blackboard.Blackboard.clear()


def test_rulebook_ticks_to_success_visiting_all_phases_with_a_place():
    from behavior_tree.PickAndPlace.pick_and_place_rulebook import pickAndPlaceRulebook

    root = pickAndPlaceRulebook(place_policy="vlm")
    status, _ = _run_to_completion(root, "pp_mock_happy")

    assert status == py_trees.common.Status.SUCCESS

    trace = _get_bb("pp_score_trace")
    assert set(trace["visited_phases"]) >= {"table", "breakfast", "extra"}, trace[
        "visited_phases"
    ]
    place_events = [e for e in trace["events"] if e["action"] == "place"]
    assert len(place_events) >= 1, trace["events"]
    assert trace["place_policy"] == "vlm"


def test_forced_grasp_failure_skips_item_and_mission_still_completes(monkeypatch):
    import behavior_tree.PickAndPlace.pick_and_place_rulebook as R

    class _FailGrasp(py_trees.behaviour.Behaviour):
        def __init__(self, name, *args, **kwargs):
            super().__init__(name=name)

        def update(self):
            return py_trees.common.Status.FAILURE

    # Patch the symbol the rulebook factories reference at build time.
    monkeypatch.setattr(R, "BtNode_Grasp", _FailGrasp)

    root = R.pickAndPlaceRulebook(place_policy="vlm")
    status, _ = _run_to_completion(root, "pp_mock_failgrasp")

    # Mission still completes: grasp failures fall through to maybeHelpOrSkip
    # (skip), the cleanup loop's FailureIsSuccess keeps draining, breakfast's
    # per-item FailureIsSuccess keeps serving.
    assert status == py_trees.common.Status.SUCCESS

    trace = _get_bb("pp_score_trace")
    assert set(trace["visited_phases"]) >= {"table", "breakfast", "extra"}
    skip_events = [e for e in trace["events"] if e["action"] == "skip"]
    assert len(skip_events) >= 1, trace["events"]
```

2. [ ] Run it, expect PASS (with C1+Phase-A+Phase-B in place):

```bash
cd /home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree && BT_MOCK_MODE=true python -m pytest test/test_rulebook_tree_mock.py -q
```
Expected: `2 passed`.

Diagnostics if it fails:
- Hang / `tree did not terminate` → a Handler is not mocked: a subsystem stayed `enabled:false` (the `_force_full_mock` patch must precede tree construction) or `BtNode_ScanAndPlace` is missing from `mock_config.json` manipulation (Phase B).
- `visited_phases` missing one → a `budgeted` guard fired (budget too small) or `BtNode_MarkPhase` did not append.
- `>= 1 place event` fails in the happy test → the route `_RecordEventLeaf` did not run; confirm `record_event(blackboard, …)` appends to `pp_score_trace` via the passed client (Phase B `record_event` contract). The place events here come from the BT-level route leaves, so this does NOT depend on `BtNode_ScanAndPlace.process_result` firing in mock.
- `>= 1 skip event` fails in the failure test → `maybeHelpOrSkip` was not reached; verify the outer `grasp+route or skip` Selector in `handleOneItem` (C1) and that BuildInventory mock-seeded a non-empty queue (Phase B).

3. [ ] Commit:

```bash
cd /home/tinker/tk25_ws/src/tk25_decision && \
git add src/behavior_tree/test/test_rulebook_tree_mock.py && \
git commit -m "PickAndPlace: whole-tree mock integration test (happy + forced-grasp-failure) (C5)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN"
```

**Deliverable:** `pickAndPlaceRulebook()` ticks to SUCCESS under mock, `visited_phases ⊇ {table,breakfast,extra}`, ≥1 place event, tick-cap-bounded; the forced-grasp-FAILURE variant proves skip fires and the mission still completes.

---

## Phase D — Documentation & changelog

### Task D1: Update RULEBOOK_PLAN.md + package changelogs

**Files:**
- Modify `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/PickAndPlace/RULEBOOK_PLAN.md`
- Modify `/home/tinker/tk25_ws/src/tk25_manipulation/src/arm_api/README.md` (create if absent)
- Modify `/home/tinker/tk25_ws/src/tk25_decision/README.md` or `src/behavior_tree/README.md` (whichever holds the package changelog; create a `## Changelog` section if absent)

**Interfaces:** Consumes: the finished work of Phases A–C. Produces: nothing code-facing (docs only).

- [ ] **Step 1: Update `RULEBOOK_PLAN.md`.** Under "Refined Plan Against Current Code", append a status block:

```markdown
## Status (2026-06-27)

Implemented the rulebook tree per `docs/superpowers/specs/2026-06-27-pick-and-place-rulebook-integration-design.md`:
- `arm_api.scan_and_place_server` extended with FREE_SPACE / NEAR_SIMILAR / FIXED_POINT modes, scan-pose control, and `dry_run`.
- New `BtNode_ScanAndPlace`; inventory/queue/guard/deadline nodes; data-driven `pickAndPlaceRulebook()`.
- `--place-policy {hardcoded,vlm}` (default `vlm`); whole-tree mock.
- Main goals only: table cleanup (wash-staging / trash / cabinet-grouped), breakfast (fixed points), extra-surface.
- Out of scope: dishwasher door/rack, tablet-in-slot, pour, milk-open. Forgone: floor trash (no floor pick), dishwasher destination credit (wash-staging substitute).
```

- [ ] **Step 2: Add an `arm_api` changelog entry.** If `arm_api/README.md` lacks a `## Changelog`, create one; prepend:

```markdown
- 2026-06-27 — scan_and_place_server: rulebook modes (FREE_SPACE/NEAR_SIMILAR/FIXED_POINT),
  scan-pose control (skip_scan_move / scan_pose_deg), dry_run, top-down orientation default;
  pure math/prompt extracted to placement_logic.py (unit-tested). ScanAndPlace.action gained
  placement_mode / reference_label / fixed_target / scan_pose_deg / skip_scan_move / dry_run +
  Result.placement_mode_used (append-only).
```

- [ ] **Step 3: Add a `behavior_tree` changelog entry.** Prepend to its `## Changelog`:

```markdown
- 2026-06-27 — PickAndPlace rulebook tree: pickAndPlaceRulebook() (inventory→queue→per-item loop,
  breakfast, extra-surface), BtNode_ScanAndPlace + inventory/queue/guard/deadline nodes,
  --place-policy {hardcoded,vlm} (default vlm), whole-tree mock. Old demo kept as `pick-and-place-demo`.
```

- [ ] **Step 4: Commit (each repo separately, selective add, two trailers).**

```bash
git -C /home/tinker/tk25_ws/src/tk25_decision add \
  src/behavior_tree/behavior_tree/PickAndPlace/RULEBOOK_PLAN.md \
  src/behavior_tree/README.md
git -C /home/tinker/tk25_ws/src/tk25_decision commit -m "$(cat <<'EOF'
docs(pick_and_place): record rulebook tree status + changelog

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"

git -C /home/tinker/tk25_ws/src/tk25_manipulation add src/arm_api/README.md
git -C /home/tinker/tk25_ws/src/tk25_manipulation commit -m "$(cat <<'EOF'
docs(arm_api): changelog for scan_and_place rulebook modes

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>
Claude-Session: https://claude.ai/code/session_011wh1swC4R33Gtorm94s5AN
EOF
)"
```

**Deliverable:** RULEBOOK_PLAN.md status + both package changelogs updated and committed.

---

## Final verification (after all tasks)

- [ ] **Pure unit tests, no build:**
  ```bash
  source /home/tinker/tk25_ws/install/setup.zsh && cd /home/tinker/tk25_ws && python3 -m pytest \
    src/tk25_manipulation/src/arm_api/test/test_placement_logic.py \
    src/tk25_decision/src/behavior_tree/test/test_categorization.py \
    src/tk25_decision/src/behavior_tree/test/test_work_queue.py \
    src/tk25_decision/src/behavior_tree/test/test_timeout_policy.py -v
  ```
  Expected: all PASS.
- [ ] **Whole-tree mock:**
  ```bash
  source /home/tinker/tk25_ws/install/setup.zsh && cd /home/tinker/tk25_ws && BT_MOCK_MODE=true python3 -m pytest \
    src/tk25_decision/src/behavior_tree/test/test_rulebook_tree_mock.py -v
  ```
  Expected: PASS (root SUCCESS, all phases visited, ≥1 place event, skip-on-grasp-failure variant completes).
- [ ] **Samplings green in mock:**
  ```bash
  source /home/tinker/tk25_ws/install/setup.zsh
  for s in pp-test-categorize pp-test-cleanup-loop pp-test-breakfast pp-test-scan-place; do
    echo "=== $s ==="
    out=$(BT_MOCK_MODE=true timeout 30 ros2 run behavior_tree $s 2>&1)
    echo "$out" | grep -q 'Traceback' && echo "FAILED: $s" || echo "OK: $s"
  done
  ```
  (The trailing idle leaf holds the root RUNNING until the 30s timeout, so `timeout` exiting 124 is EXPECTED — validate on the absence of a Python traceback, not on exit code.)
- [ ] **Server dry_run (needs A1 rebuild):** `ros2 run arm_api scan_and_place_action` + a `dry_run=true` client → `status=0`. _Live action-server smoke (no hardware / VLM / API key; needs the A1 message + A3 arm_api rebuilds + a sourced env). Not a pytest — skip without failing the task if a live node can't be launched here._
