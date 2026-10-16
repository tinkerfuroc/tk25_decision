# hangzhougrasp multi-room Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `hangzhougrasp` cover several rooms (tables with grasp, then ground scan-only), with room order chosen by an `enterRoom` profile and all poses taken from Map Studio names `<room>_<furniture>_<n>_<m>`.

**Architecture:** A pose-name parser builds a build-time route (ordered stops: tables then ground, per room). Small subtree creators (scan+announce, visit table, deliver, drop elsewhere, rooms) are composed from that route. Misconfiguration fails at startup.

**Tech Stack:** ROS2 Humble, py_trees, pytest, `map_studio.poses.load_poses`, existing `TemplateNodes`.

**Spec:** `docs/superpowers/specs/2026-10-16-hangzhougrasp-multiroom-design.md`

All paths are relative to `src/behavior_tree/` (the package dir under `tk25_decision`) unless they start with `docs/`.

## Global Constraints

- Pose names: `<room>_<furniture>_<n>_<m>`; `furniture` is `table` or `ground`; room and furniture contain no underscores.
- Tables in ascending `n`; viewpoints in ascending `m`; ground poses in ascending `(n, m)`. Sort numerically, not as strings.
- Stop at the first successful grasp at a table; a table with no grasp is skipped (run continues).
- Ground poses are scan and announce only: no grasp, no delivery.
- Grasp in the enter room: retract, go to the profile destination, open gripper, continue. Grasp elsewhere: arm to navigation pose, go to the next scan pose, announce the real drop target is unreachable, open gripper, continue. No next scan pose: go to the destination and do the real drop there.
- A grasp counts as successful when `BtNode_Grasp` succeeds. Navigation arm pose alone prevents re-detecting a dropped object.
- Scan node: `BtNode_ScanForYolo26`, `service_name="object_detection_yolo26_sam"`, `use_orbbec=True` for the scan-and-announce subtree.
- No fallback: the old `pose_scan` / `pose_destination` config and `--scan-pose` / `--destination-pose` flags are removed. A missing `enter_profiles`, missing or unknown `enter_room`, or unresolvable pose name is a startup error before any setup or motion.
- Every subtree creator is independently runnable on the robot as a `--part` (Task 6): same creator as the full tree, constants for only the poses it uses, no dependence on another part's blackboard output.
- Do not change Map Studio or other tasks.

## Review Focus

1. Table numbers 2 and 10 (and viewpoints 2 and 10) sort numerically — pinned in Task 1.
2. A profile room with no matching poses, or an `enter_room` not in its own `rooms` list, fails at startup — Task 2.
3. A room with only tables or only ground poses builds correctly — Task 5.
4. The last stop being a table triggers the final real drop at the destination — Task 5.
5. `--no-grasp` builds no arm, grasp, gripper or delivery nodes and waits for none of their servers — Task 5.
6. A part asked for a room, table or pose that doesn't exist, or run without its required arguments, fails at startup instead of mid-motion — Task 6.

---

### Task 1: Pose-name parser

**Files:**
- Create: `behavior_tree/hangzhougrasp/pose_index.py`
- Test: `test/test_hangzhougrasp_poses.py`

**Interfaces:**
- Produces:
  - `NAME_RE: re.Pattern` matching `(?P<room>[a-z][a-z0-9]*)_(?P<furniture>table|ground)_(?P<n>\d+)_(?P<m>\d+)`
  - `@dataclass(frozen=True) class PoseRecord: room: str; furniture: str; n: int; m: int; name: str; pose: PoseStamped`
  - `class PoseIndex: __init__(self, poses: dict[str, PoseStamped])`; attribute `unmatched: list[str]` (names that don't parse); `rooms(self) -> list[str]`; `tables(self, room: str) -> list[list[PoseRecord]]` (outer ascending `n`, inner ascending `m`); `ground(self, room: str) -> list[PoseRecord]` (ascending `(n, m)`)
  - Two names that parse to the same `(room, furniture, n, m)` (e.g. `a_table_0_0` and `a_table_00_0`) raise `ValueError` in `__init__`.

- [ ] **Step 1: Write failing tests** in `test/test_hangzhougrasp_poses.py` using a helper `ps(x, y)` that builds a `PoseStamped`:
  - `test_tables_sorted_numerically`: poses `bedroom_table_10_0`, `bedroom_table_2_0`, `bedroom_table_2_10`, `bedroom_table_2_2` → `[[r.name for r in t] for t in idx.tables("bedroom")] == [["bedroom_table_2_0","bedroom_table_2_2","bedroom_table_2_10"], ["bedroom_table_10_0"]]`.
  - `test_ground_sorted_and_room_filtered`: ground poses of `kitchen` and `bedroom` return only the asked room, ascending `(n, m)`.
  - `test_unmatched_names_reported`: `grasp2`, `point_area`, `Bedroom_table_0_0`, `bedroom_desk_0_0` land in `idx.unmatched` and in no query.
  - `test_duplicate_numeric_names_rejected`: `ValueError`.
  - `test_unknown_room_returns_empty`: `tables("x") == []`, `ground("x") == []`.
- [ ] **Step 2: Run** `pytest test/test_hangzhougrasp_poses.py -q` — expect FAIL (module missing).
- [ ] **Step 3: Implement** `pose_index.py` per the Interfaces block.
- [ ] **Step 4: Run** the same command — expect PASS.
- [ ] **Step 5: Commit** `feat(hangzhougrasp): parse room_furniture_n_m pose names`.

### Task 2: Route and enterRoom profiles in config

**Files:**
- Create: `behavior_tree/hangzhougrasp/route.py`
- Modify: `behavior_tree/hangzhougrasp/config.py`, `behavior_tree/hangzhougrasp/constants.json`
- Test: `test/test_hangzhougrasp_poses.py` (route and config tests), `test/test_hangzhougrasp.py` (fixture update)

**Interfaces:**
- Consumes: `PoseIndex`, `PoseRecord` (Task 1).
- Produces:
  - `route.py`: `@dataclass(frozen=True) class Stop: room: str; kind: str  # "table" | "ground"; viewpoints: list[PoseRecord]` and `build_route(index: PoseIndex, rooms: list[str]) -> list[Stop]`. For each room in order: one `Stop(kind="table")` per table (viewpoints ascending `m`), then one `Stop(kind="ground")` per ground pose (one viewpoint each). Raises `ValueError` naming the room when a room yields no stops.
  - `config.py`: `pose_key(name: str) -> str` returns `"hg_pose_" + name`. `load_config(path=None, *, enter_room=None, prompt=None, poses=None) -> TaskConfig`, where `poses` (optional `dict[str, PoseStamped]`) replaces the `load_poses(map_name)` call (used by tests).
  - `TaskConfig` loses `pose_scan`, `pose_destination`; gains `enter_room: str`, `rooms: list[str]`, `destination_name: str`, `destination: PoseStamped`, `route: list[Stop]`.
  - `constants.json` loses `pose_scan`, `pose_destination`; gains `"enter_room": "bedroom"` and `"enter_profiles": {"<room>": {"rooms": [...], "destination": "<pose name>"}}` (initial values are placeholders for the field to fill: profile for `bedroom` with `["bedroom"]` and destination `point_area`); `detection_service` becomes `"object_detection_yolo26_sam"`.
  - Startup errors (`ValueError` with a message naming the cause): `enter_profiles` missing or empty; `enter_room` missing or not a key of `enter_profiles`; `enter_room` not in its own `rooms`; destination name not in the poses dict; a room with no stops; unknown JSON keys (existing behaviour). `--print-tree` and the CLI turn these into `parser.error`.

- [ ] **Step 1: Write failing tests**:
  - `test_build_route_order`: rooms `["kitchen","bedroom"]`, poses `kitchen_table_1_0`, `kitchen_table_0_0`, `kitchen_table_0_1`, `kitchen_ground_0_0`, `bedroom_table_0_0` → stop `(room, kind, [names])` list equals `[("kitchen","table",[k_t_0_0,k_t_0_1]), ("kitchen","table",[k_t_1_0]), ("kitchen","ground",[k_g_0_0]), ("bedroom","table",[b_t_0_0])]`.
  - `test_room_without_poses_fails`: `pytest.raises(ValueError, match="livingroom")`.
  - `test_load_config_unknown_enter_room_fails`, `test_enter_room_not_in_rooms_fails`, `test_missing_destination_pose_fails`, `test_missing_enter_profiles_fails`.
  - `test_load_config_builds_route_and_destination` with `poses=` injected and `enter_room="bedroom"`.
  - `test_old_keys_rejected`: a JSON override with `pose_scan` raises `ValueError` (unknown key).
- [ ] **Step 2: Run** the new tests — expect FAIL.
- [ ] **Step 3: Implement** `route.py` and the `config.py` changes; remove `_pose_reader`/`pose_from_xyyaw` and `KEY_POSE_SCAN`/`KEY_POSE_DESTINATION` if nothing else uses them (grep first). Update `constants.json`.
- [ ] **Step 4: Update the `config` fixture** in `test/test_hangzhougrasp.py` to call `cfg.load_config(enter_room=..., poses=<small dict>)`; tests broken only by removed fields move to the later tasks that replace them.
- [ ] **Step 5: Run** `pytest test/test_hangzhougrasp_poses.py -q` — expect PASS.
- [ ] **Step 6: Commit** `feat(hangzhougrasp): enterRoom profiles and route from Map Studio names`.

### Task 3: Scan-and-announce subtree on yolo26_sam

**Files:**
- Modify: `behavior_tree/hangzhougrasp/hangzhougrasp.py`
- Test: `test/test_hangzhougrasp.py`

**Interfaces:**
- Produces: `createScanAnnounce(config: TaskConfig, *, grasp: bool = False) -> py_trees.composites.Sequence`. With `grasp=False`: Orbbec scan through `BtNode_ScanForYolo26` (`service_name=config.detection_service`, `use_orbbec=True`, `object=config.prompt`, nearest first) then `BtNode_AnnounceDetections` (as today), both wrapped in the existing Retry helpers. With `grasp=True`: arm to table pose first, request RGB/depth/segments, same announce step. Replaces `createScanAndAnnounce` and `_scanForYoloWorldRetry`.
- `BtNode_ScanForYolo26` returns FAILURE on empty detections, so after its scan retries a ground miss would fail the sweep. In the ground phase (Task 5) wrap `createScanAnnounce(config, grasp=False)` in a `Selector` whose last child is `py_trees.behaviours.Success`, so a miss is a success.

- [ ] **Step 1: Write failing tests**: `test_scan_announce_uses_yolo26_sam_and_orbbec` finds the scan node in `createScanAnnounce(config)` (walk `root.iterate()`), asserts `isinstance(node, BtNode_ScanForYolo26)`, `node.service_name == "object_detection_yolo26_sam"`, and `use_orbbec` is true; `test_scan_announce_has_announce_node`; `test_grasp_variant_requests_payload_and_moves_arm`.
- [ ] **Step 2: Run** them — expect FAIL.
- [ ] **Step 3: Implement** `createScanAnnounce`; delete the replaced functions and their callers' references (callers are rebuilt in Tasks 4–5).
- [ ] **Step 4: Run** the new tests — expect PASS.
- [ ] **Step 5: Commit** `feat(hangzhougrasp): scan-and-announce subtree on yolo26_sam`.

### Task 4: Holding state, visit-table and drop subtrees

**Files:**
- Modify: `behavior_tree/hangzhougrasp/config.py` (add `KEY_HOLDING = "hg_holding"`), `behavior_tree/hangzhougrasp/hangzhougrasp.py`
- Create: `behavior_tree/hangzhougrasp/subtrees.py` if `hangzhougrasp.py` exceeds ~300 lines after this task (builder's call)
- Modify: `behavior_tree/mock_config.json` only if `BtNode_GripperAction` or `BtNode_SetHolding` is not already registered
- Test: `test/test_hangzhougrasp.py`

**Interfaces:**
- Consumes: `createScanAnnounce`, `createGrasp`, `_gotoRetry`, `_moveArmRetry`, `pose_key`, `PoseRecord`.
- Produces:
  - `createVisitTable(config, viewpoints: list[PoseRecord]) -> Behaviour`. Per viewpoint: `Selector[holding already set → skip, Sequence[Retry goto viewpoint, Selector[Sequence[scan+announce+grasp (existing cleanup form), set holding True], Success]]]`. Contract: navigation failure after retries fails the whole tree; scan or grasp failure after cleanup moves to the next viewpoint; the first successful grasp skips the remaining viewpoints; the node succeeds with holding true or false.
  - `createDeliver(config) -> Sequence`: arm to navigation pose, `_gotoRetry` to `config.destination`, `BtNode_GripperAction(open_gripper=True)`, set holding False.
  - `createDropElsewhere(config, next_pose: PoseRecord | None) -> Sequence`: arm to navigation pose, goto `next_pose`, `BtNode_Announce` (message: that the real drop target is unreachable; Chinese, like the other announcements), `BtNode_GripperAction(open_gripper=True)`, set holding False. With `next_pose=None` it behaves as `createDeliver` (real drop at the destination).
  - Holding is read and written through small custom nodes in `custom_nodes.py`: `BtNode_SetHolding(name, value: bool)` and a check using `py_trees.behaviours.CheckBlackboardVariableValue`.

- [ ] **Step 1: Write failing tests** (tree shape and simulated ticks with stubbed leaf nodes where the existing tests do so):
  - `test_visit_table_stops_after_first_grasp`: with stubbed scan/grasp that succeed at viewpoint 0, viewpoint 1's goto node is never ticked.
  - `test_visit_table_skips_on_miss`: stubbed grasp fails at every viewpoint → table node returns SUCCESS and holding is false.
  - `test_visit_table_navigation_failure_fails`: goto failing after its retries → FAILURE.
  - `test_deliver_opens_gripper_and_clears_holding`; `test_drop_elsewhere_announces_and_opens_gripper`; `test_drop_elsewhere_without_next_pose_goes_to_destination`.
- [ ] **Step 2: Run** — expect FAIL.
- [ ] **Step 3: Implement** the three creators and the holding nodes. Gripper node: `BtNode_GripperAction` in `TemplateNodes/Manipulation.py`; announce: `BtNode_Announce` with `message=` argument.
- [ ] **Step 4: Run** — expect PASS.
- [ ] **Step 5: Commit** `feat(hangzhougrasp): visit-table, deliver and drop subtrees`.

### Task 5: Phases, rooms, top-level builder, no-grasp mode

**Files:**
- Modify: `behavior_tree/hangzhougrasp/hangzhougrasp.py`
- Test: `test/test_hangzhougrasp.py`

**Interfaces:**
- Consumes: Tasks 1–4.
- Produces:
  - `createConstantWriter(config, *, enable_grasp=False)` writes every route pose and the destination under `pose_key(name)`, plus arm poses when grasping.
  - `createTablesPhase(config, stops: list[tuple[Stop, PoseRecord | None]]) -> Sequence`: for each table stop, `createVisitTable`, then `Selector[holding false → Success, Sequence[holding true, delivery]]`, where delivery is `createDeliver` when `stop.room == config.enter_room`, else `createDropElsewhere(config, next_pose)`. `next_pose` is the first viewpoint of the next stop in the whole route (the second tuple element; `None` for the last stop).
  - `createGroundPhase(config, stops) -> Sequence`: per ground stop, `_gotoRetry` then `createScanAnnounce(config, grasp=False)` (miss is a success).
  - `createRoom(config, room, route_slice) -> Sequence`: tables phase then ground phase.
  - `createHangzhouGrasp(*, enable_grasp=False, config=None)`: constants writer, arm to navigation pose first (grasp only), one `createRoom` per profile room in order. Raises as in Task 2 when `config` is invalid; builds no arm, grasp, gripper, delivery or holding nodes when `enable_grasp=False` (table stops then only scan and announce, like ground).

- [ ] **Step 1: Write failing tests**:
  - `test_route_order_in_tree`: for rooms `["bedroom","kitchen"]` the goto nodes appear in route order (names include pose names).
  - `test_enter_room_uses_deliver_other_room_uses_drop`: table stop in the enter room contains `createDeliver`; in another room contains the drop-elsewhere announce.
  - `test_next_pose_is_next_stop_first_viewpoint`: the drop in `kitchen_table_0_*` targets the first viewpoint of the next stop.
  - `test_last_table_stop_drops_at_destination`: last stop is a table → its delivery goes to the destination (Review Focus 4).
  - `test_room_with_only_ground_poses` and `test_room_with_only_tables` build.
  - `test_no_grasp_builds_no_manipulation_nodes`: no `BtNode_MoveArmSingle`, `BtNode_Grasp`, `BtNode_GripperAction` instances anywhere in the tree (Review Focus 5).
  - `test_ground_phase_has_no_grasp`: no `BtNode_Grasp` below any ground stop.
- [ ] **Step 2: Run** — expect FAIL.
- [ ] **Step 3: Implement** the creators and the top-level builder; remove the old single-destination structure.
- [ ] **Step 4: Run** `pytest test/test_hangzhougrasp.py test/test_hangzhougrasp_poses.py -q` — expect PASS.
- [ ] **Step 5: Commit** `feat(hangzhougrasp): multi-room phases and top-level builder`.

### Task 6: Independently runnable parts

Every creator from Tasks 3–5 must run on the robot alone, so small parts can be tested before the full mission. A part is built by the same creator the full tree uses, wrapped with only the constants it needs.

**Files:**
- Create: `behavior_tree/hangzhougrasp/parts.py`
- Test: `test/test_hangzhougrasp_parts.py`

**Interfaces:**
- Consumes: all creators from Tasks 3–5, `pose_key`, `PoseIndex`.
- Produces:
  - `PARTS: dict[str, str]` mapping part name to a one-line description (used for `--help`): `scan-announce`, `scan-grasp`, `visit-table`, `deliver`, `drop-elsewhere`, `ground`, `room`, `full`.
  - `createPart(name: str, config: TaskConfig, *, enable_grasp: bool, room: str | None = None, table: int | None = None, at: str | None = None, next_pose: str | None = None) -> py_trees.behaviour.Behaviour`. Each returned root is `Sequence[constant writer for only the poses this part uses, optional goto(at), part subtree]`:
    - `scan-announce`: `createScanAnnounce(grasp=False)` where the robot stands; with `at=<pose name>`, navigate there first. Needs no arm, gripper or grasp server.
    - `scan-grasp`: the scan, announce, select and grasp sequence (`createScanAnnounce(grasp=True)` plus `createGrasp`, including failure cleanup) at the current position, or after `at`. Needs arm and grasp servers.
    - `visit-table`: `createVisitTable` for `--room` and `--table N` (all its viewpoints), without delivery; holding is left set for inspection.
    - `deliver`: `createDeliver`. `drop-elsewhere`: `createDropElsewhere(next_pose=<pose name>)`.
    - `ground`: the ground phase of `--room`. `room`: `createRoom` for `--room`. `full`: `createHangzhouGrasp`.
  - Errors (`ValueError` naming the cause, before any setup): unknown part name; a part that needs `room`/`table`/`at`/`next_pose` and does not get it; a room, table number or pose name that doesn't exist; a grasp-needing part with `enable_grasp=False`.
  - Independence rule: no part creator may read a blackboard key that another part writes, except through its own constants writer. Test this by building each part in an empty blackboard and ticking it with stubbed leaf nodes.

- [ ] **Step 1: Write failing tests** in `test/test_hangzhougrasp_parts.py` (small injected `poses` dict as in Task 2):
  - `test_each_part_builds_alone`: parametrize over `PARTS`; `createPart` returns a behaviour for valid arguments.
  - `test_scan_announce_part_needs_no_manipulation_nodes`: tree has no `BtNode_MoveArmSingle`, `BtNode_Grasp`, `BtNode_GripperAction`.
  - `test_scan_grasp_part_contains_grasp_and_cleanup`.
  - `test_visit_table_part_only_that_table`: goto nodes are exactly that table's viewpoints.
  - `test_part_writes_only_poses_it_uses`: for `scan-announce` with `at`, the constants writer has exactly one pose write.
  - `test_missing_arguments_fail`, `test_unknown_part_fails`, `test_unknown_room_or_table_fails`, `test_grasp_part_without_enable_grasp_fails`.
- [ ] **Step 2: Run** `pytest test/test_hangzhougrasp_parts.py -q` — expect FAIL.
- [ ] **Step 3: Implement** `parts.py`. Reuse creators; do not duplicate subtree logic.
- [ ] **Step 4: Run** — expect PASS.
- [ ] **Step 5: Commit** `feat(hangzhougrasp): independently runnable parts`.

### Task 7: CLI, packaging, README

**Files:**
- Modify: `behavior_tree/hangzhougrasp/cli.py`, `behavior_tree/hangzhougrasp/README.md`, `setup.py` (only if a new data file is added)
- Test: `test/test_hangzhougrasp.py`

**Interfaces:**
- Consumes: `createPart`, `PARTS` (Task 6).
- Produces: `--enter-room NAME` (overrides the JSON key); `--part NAME` (default `full`; choices from `PARTS`), with `--room`, `--table`, `--at` and `--next-pose` passed to `createPart`; `--scan-pose` and `--destination-pose` removed; `_build_parser()` keeps `--config`, `--grasp/--no-grasp`, `--prompt`, `--print-tree`. All `load_config` and `createPart` errors (`OSError, ValueError, TypeError, KeyError`, and `FileNotFoundError` from missing pose files) go to `parser.error` before any ROS setup. `--print-tree` works for every part.

- [ ] **Step 1: Write failing tests**: `test_cli_unknown_enter_room_exits_nonzero` (`pytest.raises(SystemExit)` from `main(["--enter-room","nowhere","--print-tree"])`); `test_cli_rejects_removed_pose_flags` (they are an error, not ignored: use `parse_args` or check leftovers); `test_print_tree_with_profile` prints a tree containing the first route pose name (inject poses via monkeypatching `map_studio.poses.load_poses`); `test_cli_part_scan_announce_print_tree` prints only the scan and announce nodes; `test_cli_part_missing_room_exits_nonzero`.
- [ ] **Step 2: Run** — expect FAIL.
- [ ] **Step 3: Implement** CLI changes and rewrite README sections (decision tree, pose naming, profiles, delivery rules, new detector, removed flags, the Map Studio rebuild pitfall, and a "test small parts first" section listing the parts with example commands).
- [ ] **Step 4: Run** the full file and `pytest test/ -q -k hangzhou` — expect PASS for hangzhougrasp tests; other hangzhou tests unaffected.
- [ ] **Step 5: Commit** `feat(hangzhougrasp): --enter-room and --part CLI, docs`.

### Task 8: Field prerequisite and staged robot test (manual, not code)

- [ ] **Step 1:** In Map Studio, mark poses named `<room>_table_<n>_<m>` and `<room>_ground_<n>_<m>` on the competition map, plus the destination pose per profile, then rebuild `navigation_bringup` (`./tkbuild tk26_navigation --packages-select navigation_bringup`).
- [ ] **Step 2:** Edit `enter_profiles` in `constants.json` (or pass `--config`), then run `ros2 run behavior_tree hangzhougrasp --enter-room <room> --grasp --print-tree`. Expected: the tree prints with the route poses in order; a typo in a name or profile exits with a clear error.
- [ ] **Step 3:** Start `yolo26_sam_node`, then test from the smallest part up, stopping at the first failure: `--part scan-announce` (no motion), `--part scan-announce --at <pose>`, `--no-grasp --part ground --room <r>`, `--grasp --part scan-grasp`, `--grasp --part visit-table --room <r> --table 0`, `--part deliver`, `--part drop-elsewhere --next-pose <pose>`, `--part room --room <r>`, then the full run.

---

## Self-review notes

- Spec coverage: naming and parser (Task 1), profiles, no fallback and startup errors (Task 2), scan-and-announce on `yolo26_sam_node` (Task 3), first-grasp stop, skip on miss, delivery, drop, final real drop (Tasks 4–5), ground scan-only (Task 5), CLI flag and docs (Task 6), field setup (Task 7).
- Open point left to the builder per the spec: which camera the grasp-variant scan uses (current code uses RealSense for grasp and reacquire). Keep the current behaviour unless the field test shows a problem.
- Gripper state after the real or elsewhere drop is left open (not closed again); the next grasp node is expected to command the gripper itself. Verify in the dry pass.
