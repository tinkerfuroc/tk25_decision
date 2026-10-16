# hangzhougrasp: multi-room table/ground search with enterRoom profiles

Status: draft for approval. Date: 2026-10-16.

## Intent

Today `hangzhougrasp` visits one scan pose, optionally grasps, then goes to one
destination (`config.py`, `hangzhougrasp.py`). The real task covers several
rooms. In each room the robot approaches tables first (scan, try to grasp),
then sweeps the floor. Which rooms are covered, and in what order, depends on
where the robot enters. That is decided about 30 minutes before the run.

The subtree factories must therefore take **ordered lists of poses** chosen by
an `enterRoom` profile, instead of the single `pose_scan` / `pose_destination`.

## Pose naming and parsing

- Map Studio pose names follow `<room>_<furniture>_<n>_<m>`, e.g.
  `bedroom_table_0_0`, `kitchen_ground_1_2`.
  - `furniture` is `table` or `ground`.
  - `n` is the furniture number, `m` the viewpoint index.
  - `room` and `furniture` contain no underscores (`livingroom`, not `living_room`).
- A new module in the decision package (suggested
  `hangzhougrasp/pose_index.py`) parses the dict from
  `map_studio.poses.load_poses(map_name)` into records
  `(room, furniture, n, m, PoseStamped)`.
  - Names that don't match are kept out of the index but reported by a
    validation function, so a typo made in Map Studio fails early.
  - Queries needed: all tables of a room sorted by `n`; the viewpoints of one
    table sorted by `m`; all ground poses of a room sorted by `(n, m)`.
- Map Studio itself is not changed. The pose files are not changed.

## enterRoom profiles

- `constants.json` gains an `enter_profiles` mapping. Each entry gives, for one
  enterRoom, the ordered list of rooms to cover and the destination pose name
  for the run. Example shape (names illustrative):

  ```json
  "enter_room": "bedroom",
  "enter_profiles": {
    "bedroom":    {"rooms": ["bedroom", "livingroom", "kitchen"], "destination": "point_area"},
    "livingroom": {"rooms": ["livingroom", "kitchen", "bedroom"], "destination": "<pose name>"}
  }
  ```

- `enter_room` is selected before the run by the `--enter-room` CLI flag, which
  overrides the JSON key. An unknown value is a startup error.
- Within a room, tables are visited in ascending `n`. Viewpoints of a table are
  tried in ascending `m`. Ground poses are visited in ascending `(n, m)`.
- Validation at startup: every listed room has at least one table or ground
  pose, and every pose name resolves.

## Behaviour per room

For each room in the profile's order:

1. **Tables.** For each table in ascending `n`:
   - Go to the first viewpoint, scan, try to grasp.
   - On a failed scan or grasp, go to the next viewpoint.
   - **Stop at the first successful grasp** at that table.
   - If every viewpoint finds nothing graspable, **skip to the next table**.
     This is logged and is not a run failure.
2. **Ground.** After the tables, visit the room's `ground` poses in ascending
   `(n, m)`. Ground poses are **scan and announce only**: no grasp, no
   delivery. This reuses the existing `createScanAndAnnounce(config)` without
   the grasp option (the table phase uses it with the grasp option).

Existing failure handling stays: a failed grasp that leaves the arm unsafe
triggers the existing retract-on-failure cleanup.

## Subtree decomposition

Each phase is its own creator function in `hangzhougrasp.py` (or a small
module next to it), returning a `Sequence(memory=True)` and taking only the
poses and config it needs. Nothing below the top-level builder reads global
config.

| Creator | Contents | Inputs |
|---|---|---|
| `createScanAnnounce(config, *, grasp=False)` | Scan with the **Orbbec** camera through `yolo26_sam_node`, then announce in Chinese what was seen, if anything. | label map, prompt |
| `createVisitTable(config, table)` | For one table, try its viewpoints in order: go to the viewpoint, `createScanAnnounce(grasp=True)`, stop at the first successful grasp. A miss at every viewpoint succeeds (skip). | table's viewpoint poses |
| `createDeliver(config, ...)` | Arm to navigation pose, navigate, then open the gripper. | target pose |
| `createDropElsewhere(config, next_pose)` | Arm to navigation pose, navigate to `next_pose`, announce the real drop target is unreachable, open gripper. | next scan pose |
| `createTablesPhase(config, room)` | `createVisitTable` for each table of the room in ascending `n`, with the delivery logic after each grasp. | room's tables, enter-room flag |
| `createGroundPhase(config, room)` | Go to each ground pose in order and run `createScanAnnounce(grasp=False)`. | room's ground poses |
| `createRoom(config, room)` | `createTablesPhase` then `createGroundPhase`. | room |
| `createHangzhouGrasp(config)` | Constants writer, then `createRoom` for each room in the profile, then the final real drop if still holding an object. | profile |

Independent testing: each creator can be run alone on the robot through a
`--part` option (`scan-announce`, `scan-grasp`, `visit-table`, `deliver`,
`drop-elsewhere`, `ground`, `room`, `full`). A part uses the same creator as
the full tree, writes only the pose constants it needs, and does not depend on
another part's blackboard output. `--print-tree` works for every part. This
lets the small pieces be tested on the robot before the full mission.

Scan and announce details:
- The scan node is the existing `BtNode_ScanForYolo26` pointed at
  `service_name="object_detection_yolo26_sam"` (the `yolo26_sam_node` default),
  with `use_orbbec=True`. `yolo26_sam_node` uses bbox-only YOLO26 weights with
  MobileSAM masks, and a nonempty prompt must match checkpoint labels exactly.
- Announcing uses `BtNode_AnnounceDetections` with `label_map`, as today.
- The `grasp=True` variant requests RGB, depth and segments in the payload,
  as the current `_scanForYoloWorldRetry(grasp_payload=True)` does. Which
  camera the table scan uses with `grasp=True` is left to the builder: the
  current code switches to RealSense for grasping and the arm-mounted camera
  for the reacquire step.
- This replaces the YOLO-World detector in this task. `detection_service`
  in `constants.json` becomes `object_detection_yolo26_sam`, and `prompt` and
  `label_map` keys must be labels the checkpoint knows.

## Delivery after a grasp

- **Grasp in the enter room:** retract, navigate to the profile's destination
  pose, open the gripper to release the object, then continue with the
  remaining poses.
- **Grasp in any other room:** the robot cannot reach the real drop target. It
  puts the arm in the navigation pose, navigates to the **next scan pose**,
  announces that it cannot reach the real drop target, and opens the gripper to
  drop the object there. The navigation arm pose is what keeps the dropped
  object out of the camera's view, so the scan at that pose proceeds normally.
- **No next scan pose:** the robot returns to the destination area and does the
  real drop there.
- **Many objects per run.** After any delivery or drop, the robot keeps going
  through the remaining tables, ground poses and rooms. The run ends after the
  last pose in the last room (including a final real drop if it is still
  holding an object).
- A grasp counts as successful when `BtNode_Grasp` succeeds; no separate
  verification. After a success the remaining viewpoints of that table are
  skipped, so the "next scan pose" is the first pose of the next table, else
  the first ground pose of the room, else the first pose of the next room.

## Goals, non-goals, anti-goals

Goals:
- Pose lists are inputs to the phase factories, chosen by the enterRoom profile.
- All pose names come from Map Studio; no coordinates in the task code.
- The `--no-grasp` and `--print-tree` modes keep working, driven by the new
  profiles.
- Misconfiguration fails loudly at startup; nothing silently falls back.

Non-goals:
- Changing Map Studio, its file format, or other tasks.
- Object tracking across frames, or choosing objects by anything but the
  existing nearest-first rule.

Anti-goals:
- The robot must never re-detect an object it just dropped at a non-enter room.
- A skipped table must not stop the run.
- An unknown enterRoom or an unresolvable pose name must fail before any
  motion, not midway through the run.

## Calls I made (not yet confirmed with you)

1. "Next scan pose" means the next pose in the run order after skipping the
   rest of the current table's viewpoints; it can be in the next room.
2. The enter-room delivery now opens the gripper. The current README says the
   task never opens it, so this is a behaviour change.
3. The destination pose name lives in the profile, because different entries
   may use different destinations.
4. Destination names resolve through the raw `load_poses` dict, so they need
   not match the naming pattern.
5. If the final real drop cannot reach the destination, the normal navigation
   retries apply and then the run fails.
6. **No fallback (your instruction).** The old single `pose_scan` /
   `pose_destination` config is removed. A missing `enter_profiles`, a missing
   or unknown `enter_room`, or any unresolvable pose name is a startup error
   with a clear message, before any setup or motion. `--print-tree` also
   needs a valid profile, since the tree is built from it.
7. The navigation arm pose alone prevents re-detecting a dropped object; no
   position exclusion or viewpoint skipping.

## Left to the builder

- Tree structure for the per-room loop (generated at build time, since pose
  lists are known before the run).
- The gripper-open and announce node choices, reusing `TemplateNodes`
  primitives where they exist.
- Test layout: extend `test/test_hangzhougrasp.py` with parser, profile and
  tree-shape tests, and a mock-mode run.
