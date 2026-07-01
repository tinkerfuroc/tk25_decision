# Pick-and-Place Rulebook Integration — Design Spec

- **Date:** 2026-06-27
- **Status:** DRAFT v2 (revised after 5-critic adversarial review; awaiting user review)
- **Repos touched:** `tk25_decision` (behavior_tree), `tk25_manipulation` (arm_api, tinker_arm_msgs)
- **Author:** Claude (Opus 4.8), with cindy
- **Related:** `PickAndPlace/RULEBOOK_PLAN.md`, `arm_api/scan_and_place_server.py`, RoboCup@Home 2026 Rulebook §5.2 "Pick and Place Challenge"

> **Revision note (v2):** v1 was verified by five independent critics against the real source + rulebook. This version fixes 5 blockers (py_trees deadline/loop mechanics, re-detect→grasp wiring, source-vs-destination pose, ScanAndPlace mock stub, vacuous mock) and ~13 majors. Each fix is marked `[Rv2]` at its section.

---

## 1. Goal

Wire `arm_api.scan_and_place_server` into `tk25_decision`'s PickAndPlace behavior tree, restructure that tree to follow the RoboCup@Home 2026 **Pick and Place Challenge** for its three **main** scored goals, and modify the server so it is rulebook-compatible. The **real robot is unavailable for now** (full ROS2 *is* installed; on-robot testing comes later), so **everything must be smoke-testable with no robot hardware** — whole-tree **mock mode** (§11A) + pure-logic unit tests — and the place strategy must be switchable to a deterministic, network-free mode for first on-robot bring-up (**place policy**, §8.0).

### 1.1 In scope (three main goals)

1. **Dining-table cleanup** — detect each table object, categorize it, pick it, route it: cutlery+tableware → **wash-staging surface**; designated trash → **trash bin** (controlled release); other objects → **cabinet, grouped next to similar items**.
2. **Serve breakfast** — retrieve bowl+spoon (kitchen surface) and milk+cereal (cabinet) and place each on the cleaned table at its **own** point so the layout is typical and uncluttered (spoon beside bowl, milk beside cereal).
3. **Extra-surface cleanup** — clear the two "common objects" from the extra surface into the cabinet (grouped).

### 1.2 Out of scope (bonus goals — user directive: "you don't have to implement these nodes")

Dishwasher door open/close, dishwasher rack pull/push, tablet-into-slot, pour milk/cereal, open milk container. **No nodes, no stubs, no branches.** One documented extension hook (§17) marks where a future phase would attach.

### 1.3 Honestly forgone scored items `[Rv2]`

These are scored but deliberately not attempted, for stated hardware reasons (not silently dropped):
- **Floor trash item** — `config.py:6` records the robot **cannot pick from the floor**; the floor trash is conceded (no floor scan/approach). The +50 pick + +30 floor-bonus + +40 place for that item are forgone.
- **Dishwasher destination credit** — the robot cannot place into the dishwasher rack, so cutlery/tableware go to a wash-staging surface (`constants.json:pose_wash_staging`). This forgoes the **per-item correct-destination place credit for all cutlery/tableware (place 40 ea + the +70 dishwasher bonus)**, not merely a single +70. Sanctioned hardware substitution; documented, not hidden.
- **+20 grouping is best-effort** — the first object of each cabinet category necessarily finds no reference and places into free space (§7.1); grouping yield improves as clusters grow.

### 1.4 Success criteria

- `pickAndPlaceRulebook()` constructs, `setup()`s, and ticks to `SUCCESS` end-to-end under `BT_MOCK_MODE=true` + `IMMEDIATE`, **with the cleanup/extra loops actually traversing a (mock-seeded) non-empty queue** so the per-item body runs — visiting all three phases, no exceptions.
- `scan_and_place_server` exposes the three placement modes + a working `dry_run`; its pure placement logic passes unit tests with no ROS/hardware/VLM/network.
- Categorization, work-queue, score-trace, and timeout policy each pass deterministic pytest.
- New samplings exercise the node and subtrees in isolation, green in mock.
- Additive: other tasks (GPSR, Receptionist) untouched; old `pickAndPlaceShortened` preserved.

---

## 2. Background — current state (source-verified)

### 2.1 Live tree vs. design docs diverged

`RULEBOOK_PLAN.md` (2026-03-10) + `PickAndPlace/config.py` describe a **rulebook-first, inventory-driven** design (destination classes, work queue, `KEY_INVENTORY_TABLE`/`KEY_WORK_QUEUE`/`KEY_ACTIVE_*`, breakfast points, label sets) — **mostly unused scaffolding**. The live tree, `pickAndPlaceShortened()` (`PickAndPlace/pick_and_place.py:646`), is a narrow hard-coded demo with **no real place step** (`BtNode_Place`/`place_action` commented out at `:621-625`; every "place" is a kinematic arm move + `BtNode_GripperAction(open)`; `scanShelf()` has dead code after an early `return`).

### 2.2 `scan_and_place_server` today

Action server `tinker_arm_msgs/action/ScanAndPlace` on `scan_and_place_action` (`ros2 run arm_api scan_and_place_action`; entry `setup.py:41`; node `scan_and_place_server.py:140`). Current `.action`: Goal `{item_description, margin_m, orientation, max_candidates}`; Result `{status, placed_at, error_msg}` (status 0 placed / 1 no candidate / 2 place failed / 3 scan failed / −1 hard error); Feedback `{stage}`. Behavior: move to a hard-coded scan pose via `joint_move_action(add_octomap=True)` → wrist RGB-D → Gemini 2.5 Pro/OpenRouter finds **empty** regions → back-project + margin-filter → `place_action`. `goal_handle.succeed()` only when `status==0`, `abort()` otherwise (so action `STATUS_SUCCEEDED` ⟺ `status==0`). **Limits:** empty-space only (no grouping); single hard-coded scan pose; **no mock/dry-run**; WIP — **zero callers, no launch file**; `default_orientation_xyzw` default is placeholder identity.

### 2.3 BT base-class & mock contract (what new nodes must follow)

- `ActionHandler` (`TemplateNodes/ActionBase.py:98`): override `send_goal()` (build goal from blackboard, call `self.send_goal_request(goal)`) and optionally `process_result()`. `__init__(name, action_type, action_name, key, wait_for_server_timeout_sec=-3.0, action_timeout_ticks=0)` registers one READ key as `"goal"` when `key` is not None; subclasses needing several inputs register their own keys and pass `key=None`.
- **Mock is automatic:** in mock, `setup()` skips client creation (`:211-227`); `initialise()` pre-sets `result_status=STATUS_SUCCEEDED` (`:405-419`); `update()` returns `wait_for_keypress_in_mock()` (`:518-519`) → under `IMMEDIATE` auto-succeeds after `_mock_auto_ticks_required` (2) ticks. A correctly-built `ActionHandler` subtree is mock-testable with **no server**.
- Per-node toggle: list the class under a subsystem in `mock_config.json`; an unlisted node under global mock **defaults to mocked** (`config.py:341-345`). **Critically `[Rv2]`: only `ServiceHandler`/`ActionHandler` subclasses honor this. Plain `Behaviour`s (our guards, inventory nodes, DeadlineGuard) are NEVER mocked — they always run their real logic.** This is required for correct routing and deadline behavior in mock.

### 2.4 "Samplings"

User's term for the family of small standalone single-behaviour `ros2 run` entry points (e.g. `GPSR/dev_tests.py`+`small_trees.py` → `gpsr-test-grasp/-place`; `grasp_intel_demo`). We add PickAndPlace samplings in the same style.

---

## 3. Rulebook → capability map

| Rulebook step | Destination | Mechanism | Server (exists?) |
|---|---|---|---|
| Enter arena | — | `BtNode_DoorDetection` retry(cap) | `door_detection_srv` ✅ |
| Navigate | — | `BtNode_GotoAction` | `navigate_to_pose` ✅ |
| Detect table objects | — | `BtNode_ScanForGeneralist` (orbbec) | `object_detection_generalist` ✅ |
| Categorize → destination | — | `classify_destination()` (label sets) + optional `grocery_categorize` | static + `Categorize` ✅ |
| Re-detect active item before grasp `[Rv2]` | — | `BtNode_ScanForGeneralist` filtered to active label → `KEY_VISION_RESULT` | ✅ |
| Pick | — | `BtNode_Grasp` (reads `KEY_VISION_RESULT`, `KEY_OBJECT_LABEL`) | `start_grasp` ✅ |
| Announce per-object perception+destination `[Rv2]` | — | `BtNode_Announce(f"{label} → {dest}")` per item | TTS ✅ |
| Place cutlery/tableware | wash-staging | `BtNode_ScanAndPlace` FREE_SPACE | `scan_and_place_action` ✅ (extended) |
| Place "other" | cabinet, grouped | `BtNode_ScanAndPlace` NEAR_SIMILAR(reference_label) | ✅ (extended) |
| Place trash | trash bin | `BtNode_Drop` if `start_drop` available, else `MoveArm(arm_pos_trash, low)`+`GripperAction(open)` | drop/joint ✅ |
| Serve breakfast | clean table | per-item `BtNode_ScanAndPlace` FIXED_POINT(POINT_BREAKFAST_\<item\>) | ✅ (extended) |
| Extra-surface cleanup | cabinet | same loop, source = `pose_extra_surface` | ✅ |
| Cabinet access | — | **rulebook cabinet is DOORLESS** — no door handling needed `[Rv2]` | — |
| Safe placing | — | server places gently; trash release at low bin-relative height | — |
| Human-assistance fallback | — | **default: skip the object (partial credit); help-request only if `allow_human_assistance` param set** `[Rv2]` | TTS ✅ |
| Timeout-aware completion | — | global hard cap + per-phase budgets → always reach summary | — |
| Bonus goals | — | **OUT OF SCOPE** | none |

---

## 4. Architecture overview

```
 tk25_decision (behavior_tree)              tk25_manipulation (arm_api + tinker_arm_msgs)
 ┌───────────────────────────────┐         ┌──────────────────────────────────────────┐
 │ pickAndPlaceRulebook()        │         │ scan_and_place_server (ScanAndPlace)       │
 │  global DeadlineGuard +       │         │  modes: FREE_SPACE|NEAR_SIMILAR|FIXED_POINT│
 │  per-phase budgets; summary   │ scan_   │  scan-pose control; dry_run                │
 │  OUTSIDE the guard            │ and_    │  placement_logic.py (pure, unit-tested)    │
 │  inventory→queue→item loop    │ place ──┼─→ joint_move_action + place_action          │
 │  classify_destination() pure  │ action  │                                            │
 │  BtNode_ScanAndPlace (new) ───┼────────→│                                            │
 │  imports msg via messages.py  │         └──────────────────────────────────────────┘
 │   (real + mock_messages stub) │
 └───────────────────────────────┘
```

Principles: (1) **the server owns *where-on-the-surface* placement**, the BT owns *which surface* (nav + arm scan pose + mode); (2) **cleanup is data-driven** (inventory→queue→generic loop); **breakfast is an explicit 4-item table** (fixed points, not vision-driven) `[Rv2]`.

---

## 5. Component 1 — `scan_and_place_server` rulebook modifications

### 5.1 Interface: `ScanAndPlace.action` (append-only)

Zero callers → appending is safe. **Append** to Goal:
```
uint8 placement_mode        # 0 FREE_SPACE (default=legacy) | 1 NEAR_SIMILAR | 2 FIXED_POINT
string reference_label      # NEAR_SIMILAR: category/label to place beside
geometry_msgs/PointStamped fixed_target  # FIXED_POINT: explicit place point (TF'd to base_link)
float32[] scan_pose_deg     # optional 7-joint scan-pose override; empty => server param default
bool skip_scan_move         # true => BT already positioned the arm; capture without moving
bool dry_run                # true => skip camera/VLM/place; canned placed_at
```
**Append** to Result: `uint8 placement_mode_used` (echoes mode actually executed; NEAR_SIMILAR may degrade to FREE_SPACE). Feedback unchanged. `placement_mode=0` + zeros reproduces today's behavior exactly. Mode enum mirrored as Python constants in `arm_api/placement_logic.py` **and** `behavior_tree/PickAndPlace/config.py` (`PLACEMENT_MODE_FREE_SPACE=0`, `_NEAR_SIMILAR=1`, `_FIXED_POINT=2`, plus `PLACEMENT_MODE_NONE=255` sentinel for the trash branch `[Rv2]`).

### 5.2 Behavior by mode
- **FREE_SPACE (0):** unchanged.
- **NEAR_SIMILAR (1):** VLM prompt asks for the empty area adjacent to existing `reference_label` items; reuse the existing bbox→3D→margin→TF pipeline. If no reference visible/none returned → **degrade to FREE_SPACE**, set `placement_mode_used=0`, note in `error_msg` (non-fatal).
- **FIXED_POINT (2):** **skip VLM.** TF `fixed_target`→`base_link`; clearance check vs `env_points` if available; apply `place_z_lift`; `place_action`.

### 5.3 Scan-pose control
`skip_scan_move` → do not call `joint_move_action`. Else `scan_pose_deg` (7 entries) → move there. Else → server param default. Lets the BT aim one server at table/wash/cabinet.

### 5.4 `dry_run` (server-side mock)
Emit feedback stages, set `placed_at`=`fixed_target` (FIXED_POINT) else a canned `target_frame` point, `status=0`, `placement_mode_used=placement_mode`, `succeed()`. Touches no camera/VLM/TF/action-client. (Independent of the BT-level mock, which never contacts the server.)

### 5.5 Testability refactor — `arm_api/arm_api/placement_logic.py` (new, pure) `[Rv2 — corrected]`

**Why extract (motivation corrected — full ROS *is* installed):** importing the server module is *not* the obstacle. The reasons are (a) three of the helpers are **instance methods** that need a live ROS Node + declared params or CvBridge (e.g. `_resolve_margin_m` reads `self.get_parameter('default_margin_m')`; `_depth_image_to_meters` uses `self._bridge`), so they can't be unit-tested without constructing a node + camera/VLM context; and (b) the placement **math + prompt should be testable deterministically on a dev machine with no camera, no VLM/network, and no grasp node**. Extracting pure module-level functions (numpy-only) gives fast, hermetic `pytest` over the geometry and prompt-building while the server keeps all I/O. (The server's `cv2`/`cv_bridge`/`rclpy`/`tf2` are module-level and `openai` is lazy — moot now that ROS is present; the pure split still isolates the logic under test.)

**Purity status (corrected):** only `decode_bbox` (module-level `:118`) and `short_side_m` (`:431`) are pure today. The other three are ROS-coupled and **need new signatures** (not a lift-and-shift):
- `resolve_margin_m(m, default_margin_m) -> float` — was an instance method reading `self.get_parameter(...)`; now takes the default as an arg; the node passes `self.get_parameter('default_margin_m')` in.
- `depth_image_to_meters(raw_bytes_or_ndarray, encoding) -> np.ndarray` — replace `self._bridge.imgmsg_to_cv2` with `np.frombuffer(...).reshape(...)`; no CvBridge.
- `bbox_centroid_3d(depth_m, K_tuple, bbox) -> tuple|None` — take the K intrinsics tuple, return a plain `(x,y,z)` tuple (no `geometry_msgs.Point`).

Plus two new pure fns: `build_vlm_messages(mode, item_description, reference_label, max_candidates) -> list[dict]` (NEAR_SIMILAR injects `reference_label`); `select_candidate(boxes, depth_m, K, margin_m) -> (point|None, reason)`. The node imports from `placement_logic`; TF/subs/clients/OpenAI stay in the node. Pure module needs only `numpy`.

### 5.6 Orientation default fix
Set `default_orientation_xyzw` to a real TCP-+z-down quaternion, replacing identity. **Value `TODO(hardware)`** (best guess); mock paths (FIXED_POINT/dry_run) don't depend on it.

---

## 6. Component 2 — `BtNode_ScanAndPlace` (new) `[Rv2]`

New `ActionHandler` subclass in `TemplateNodes/Manipulation.py`. **Import the message via `behavior_tree.messages` (NOT directly from `tinker_arm_msgs`)**, and add a `ScanAndPlace` real-branch to `messages.py` + a stub to `mock_messages.py`, so the node imports on a clean host with msgs absent (matching every other arm action).

- `action_type = ScanAndPlace`, `action_name = SCAN_AND_PLACE_ACTION_NAME`.
- Registers its own keys (pass `key=None`): READ `item_description, placement_mode, reference_label, margin_m, orientation, fixed_target, scan_pose_deg, skip_scan_move, dry_run`; WRITE `placed_at` + a status/reason output. Keys are ctor args defaulting to PickAndPlace `KEY_*` so samplings can override.
- `send_goal()`: real mode assembles `ScanAndPlace.Goal()` from the blackboard; mock handled by base.
- `process_result()`: `STATUS_SUCCEEDED` → write `placed_at`, append score-trace event, `SUCCESS`; else read `result.status`/`error_msg`, record, `FAILURE`.
- Register under `manipulation` in `mock_config.json`.

---

## 7. Component 3 — categorization, inventory & queue

### 7.1 `categorization.py` (new, pure) in `PickAndPlace/`
`classify_destination(label, *, cutlery, tableware, trash, category_map) -> Destination(klass, reference_label)`, `klass ∈ {"wash_staging","trash","cabinet"}`:
- label ∈ cutlery ∪ tableware → `wash_staging`, ref="".
- label ∈ designated_trash → `trash`, ref="".
- else → `cabinet`, ref=`category_map.get(label, label)`.
Pure, deterministic, unit-tested. **`category_map` v1 = empty → default-to-label** (each label its own group; +20 grouping best-effort). Optional starter map in `constants.json` (`snacks`/`drinks`/`cleaning`/…) improves yield — `TODO(setup-days)`. **Bowl disambiguation:** `tableware_labels` includes `"bowl"`; during cleanup a detected bowl is tableware, during breakfast the bowl is handled by the breakfast phase (phase context disambiguates; documented in code).

### 7.2 Inventory / queue nodes (new) in `PickAndPlace/custom_nodes.py` — plain `Behaviour`s
- `BtNode_BuildInventory(in_key=KEY_SCAN_RESULTS_TABLE, out_inventory=KEY_INVENTORY_TABLE, out_queue=KEY_WORK_QUEUE, source_pose_key, mock_seed=None)`: reads the generalist result, builds `[{label, segment, destination, reference_label, source_pose_key}]` via `classify_destination`, **sorts cabinet-bound items by category** so same-category items are placed consecutively `[Rv2]`, writes inventory + queue. **Under `BT_MOCK_MODE` (or when `mock_seed` is set), if the upstream result is empty, seed a canned 2–3 item queue** so the full-tree mock traverses the loop body `[Rv2]`. Always `SUCCESS`. Registers `in_key` READ, inventory+queue **WRITE**.
- `BtNode_PopWorkItem(queue=KEY_WORK_QUEUE, place_policy="vlm")`: pops front; resolves destination → `(nav_pose, arm_scan_pose, vlm_mode, hardcoded_point)` from `DESTINATION_ROUTING`; **applies `place_policy`** to derive the effective `placement_mode` + `fixed_target` (`vlm` → `vlm_mode` + `reference_label`; `hardcoded` → `FIXED_POINT` + `fixed_target=hardcoded_point`); writes `KEY_ACTIVE_OBJECT_CLASS, KEY_OBJECT_LABEL, KEY_ACTIVE_PROMPT, KEY_ACTIVE_SOURCE_POSE` (the item's source surface) **and** `KEY_ACTIVE_TARGET_POSE` (destination), `reference_label`, the effective `placement_mode`, `fixed_target`, arm scan pose `[Rv2 — source≠target]`. `SUCCESS` if popped, **`FAILURE` if empty** (loop terminator — must stay un-masked). Registers `KEY_WORK_QUEUE` **READ+WRITE** (stores replacement list via `.set`), all `KEY_ACTIVE_*` + `fixed_target` **WRITE** `[Rv2]`.
- (Optional, default off) `BtNode_CategorizeUnknown` via `grocery_categorize` for labels in no set.

### 7.3 Routing table (new in `config.py`) `[place-policy aware]`
`DESTINATION_ROUTING = {`
`  "wash_staging": (KEY_POSE_WASH_STAGING, KEY_ARM_WASH, PLACEMENT_MODE_FREE_SPACE,   KEY_POINT_WASH_STAGING),`
`  "cabinet":      (KEY_POSE_CABINET,      KEY_ARM_CABINET, PLACEMENT_MODE_NEAR_SIMILAR, KEY_POINT_CABINET_DEFAULT),`
`  "trash":        (KEY_POSE_TRASH_BIN,    KEY_ARM_TRASH, PLACEMENT_MODE_NONE,         None) }`
Tuple = `(nav_pose, arm_scan_pose, vlm_mode, hardcoded_point)`. The actual `placement_mode` is chosen from the tree's **`place_policy`** arg (§8.0): `vlm` → `vlm_mode` (+`reference_label`); `hardcoded` → `FIXED_POINT` with `fixed_target = hardcoded_point` (a predetermined `constants.json` point, **no VLM**). Trash ignores both (always a drop). `PLACEMENT_MODE_NONE` (=255) is the explicit sentinel; the Selector never ticks ScanAndPlace on the trash branch (asserted in tests) `[Rv2]`. (`KEY_POINT_WASH_STAGING`/`KEY_POINT_CABINET_DEFAULT` already exist in `config.py:181-182`.)

---

## 8. Component 4 — the rulebook tree `[Rv2 — mechanics corrected]`

New module `PickAndPlace/pick_and_place_rulebook.py`, factory `pickAndPlaceRulebook(place_policy="vlm")`. Old `pick_and_place.py` untouched; `cli.py:main` repointed to the rulebook tree (parsing `--place-policy`); new `cli_demo.py` / entry `pick-and-place-demo` keeps the old tree.

### 8.0 Place policy — tree-level argument `[user-requested]`
The tree takes **`place_policy ∈ {hardcoded, vlm}`**, parsed in `cli.py` as `--place-policy` (and accepted as a `pickAndPlaceRulebook(place_policy=...)` kwarg so samplings/tests set it directly), threaded down to every place leaf. It selects how the **cleanup/extra-surface surface places** are executed:
- **`vlm` (DEFAULT — competition):** surface places use the VLM modes — wash-staging → **FREE_SPACE**, cabinet → **NEAR_SIMILAR(reference_label)** (chasing the +20 grouping). Requires `OPENROUTER_API_KEY` + network at runtime.
- **`hardcoded` (on-robot bring-up safe):** every surface place uses `BtNode_ScanAndPlace` **FIXED_POINT** with the destination's predetermined `constants.json` point (wash-staging → `point_wash_staging`, cabinet → `point_cabinet_default`). **No VLM, no OpenRouter/network call, fully deterministic** — the object lands at a fixed known spot. Recommended for first real-robot validation of the nav→grasp→place plumbing without the VLM dependency (`--place-policy hardcoded`).

Independent of policy: **breakfast** is always FIXED_POINT (its layout points are inherently fixed) and **trash** is always a kinematic drop — `place_policy` only swaps the *surface* places between fixed-point and VLM. In **mock mode the policy is inert** (the place node auto-succeeds regardless), so both policies tick green in mock; the policy matters only on the real robot. `place_policy` is stamped into the score-trace for run provenance. Implementation: `BtNode_PopWorkItem` resolves the routing tuple (§7.3) and, given `place_policy`, sets `KEY_ACTIVE_*` `placement_mode` + `fixed_target` accordingly, so the route Selector's place leaf is policy-agnostic.

### 8.1 Root composite (summary survives preemption)
```
DeadlineGuard returns only RUNNING→SUCCESS (never FAILURE).
root = Sequence(memory=True)[
   createConstantWriter(),                      # poses/points/labels/runtime + init score-trace
   phaseEnterArena(),                           # Retry(cap) > BtNode_DoorDetection
   Parallel(policy=SuccessOnOne)[
       GlobalDeadlineGuard(MAX_RUNTIME_SEC),    # 390 s hard cap
       FailureIsSuccess(missionPhases()),       # mission never presents FAILURE to the parallel
   ],
   phaseSummary(),                              # OUTSIDE the guard → always runs (normal OR timeout)
]
```
On normal completion the mission child SUCCEEDs → Parallel SUCCESS → summary. On timeout GlobalDeadlineGuard SUCCEEDs → Parallel SUCCESS (running leaf cancelled via `ActionHandler.terminate`) → summary. Parallel never returns FAILURE (guard never fails; mission is `FailureIsSuccess`). `phaseSummary` reads the score-trace from the blackboard (persists across mission teardown).

### 8.2 Mission + per-phase budgets (avoid cleanup starving breakfast)
```
missionPhases() = Sequence(memory=True)[
   budgeted(phaseTableCleanup,  TABLE_BUDGET_SEC),
   budgeted(phaseServeBreakfast, BREAKFAST_BUDGET_SEC),
   budgeted(phaseExtraSurfaceCleanup, EXTRA_BUDGET_SEC),
]
budgeted(phase, sec) = Parallel(SuccessOnOne)[ PhaseDeadlineGuard(sec), FailureIsSuccess(phase()) ]
```
A phase that overruns its budget yields (its guard SUCCEEDS) and the mission advances. Budgets sum ≤ `MAX_RUNTIME_SEC` with margin (placeholders: table 200 s, breakfast 110 s, extra 60 s; tune later). Guards are **plain Behaviours** with an injectable clock latched on first tick (§11) — never Handlers, never in `mock_config.json`, so they don't fire in fast mock tests.

### 8.3 Table-cleanup phase
```
phaseTableCleanup() = Sequence(memory=True)[
   markPhase("table"),
   goto(KEY_POSE_TABLE); arm(KEY_ARM_TABLE); scanTable(generalist→KEY_SCAN_RESULTS_TABLE);
   WriteFoundItems; Announce(perception summary),
   BtNode_BuildInventory(source_pose_key=KEY_POSE_TABLE),   # mock-seeds if empty
   Repeat(num_success=-1)[ Sequence(memory=True)[
       BtNode_PopWorkItem,                                   # UNWRAPPED — FAILURE exits the loop
       FailureIsSuccess( handleOneItem() ),                 # any item failure → continue loop
   ]],
]
handleOneItem() = Sequence(memory=True)[
   goto(KEY_ACTIVE_SOURCE_POSE); arm(KEY_ARM_TABLE);
   reDetect(KEY_ACTIVE_PROMPT → KEY_VISION_RESULT);          # filtered generalist scan
   Announce(f"{KEY_OBJECT_LABEL} → {KEY_ACTIVE_OBJECT_CLASS}"),  # per-object perception
   Retry(grasp) > BtNode_Grasp(vision_res=KEY_VISION_RESULT, object_label=KEY_OBJECT_LABEL),
   Selector[                                                 # route by destination
       Sequence[ guardClass("wash_staging"), goto(TARGET), arm(KEY_ARM_WASH),   ScanAndPlace(mode+target per place_policy) ],
       Sequence[ guardClass("cabinet"),      goto(TARGET), arm(KEY_ARM_CABINET), ScanAndPlace(mode+target per place_policy) ],
       Sequence[ guardClass("trash"),        goto(TARGET), arm(KEY_ARM_TRASH),   trashRelease() ],
       maybeHelpOrSkip(),                                    # lowest-priority, always SUCCESS
   ],
   # place_policy=hardcoded -> ScanAndPlace FIXED_POINT(predetermined point); =vlm -> FREE_SPACE / NEAR_SIMILAR.
   # The leaf reads placement_mode + fixed_target from KEY_ACTIVE_* (set by PopWorkItem per policy), so it is policy-agnostic.
]
```
- **Loop terminator discipline `[Rv2]`:** `Repeat(num_success=-1)` re-ticks the body each iteration; the body Sequence returns FAILURE **only** when `PopWorkItem` hits an empty queue (it is the one node NOT wrapped) → Repeat FAILURE → the enclosing phase's `FailureIsSuccess` converts to phase-SUCCESS. Every real item failure is masked by `FailureIsSuccess(handleOneItem())` so the loop continues (partial credit). **Code comment required** at the Repeat: "loop exit == child FAILURE (empty queue); the FailureIsSuccess is load-bearing — removing it makes the loop infinite."
- **Route guards are plain condition Behaviours** (`guardClass(expected)` reads `KEY_ACTIVE_OBJECT_CLASS`) — never Handlers (else a mocked guard auto-succeeds and routes everything to branch 1) `[Rv2]`.
- **`maybeHelpOrSkip()` is the terminal always-SUCCESS Selector child `[Rv2]`:** default = `Announce("skipping {label}")` (abandon → partial credit). If param `allow_human_assistance` is set, instead `Announce("please hand me / reposition {label}")` and record the assistance event (with its score cost). Score-aware: a −100 handover per item is usually net-negative, so **skip is the default**.
- **`trashRelease()`:** prefer `BtNode_Drop` (`start_drop`) if that server exists; else `MoveArm(arm_pos_trash)` to a **low, bin-relative pose** + `GripperAction(open)` (low release ⇒ within safe-placing tolerance). Confirm `start_drop` server status in the plan.

### 8.4 Breakfast phase (explicit per-item table) `[Rv2]`
```
BREAKFAST = [
  ("bowl",   KEY_POSE_KITCHEN_SHELF, KEY_ARM_TABLE,   KEY_POINT_BREAKFAST_BOWL),
  ("spoon",  KEY_POSE_KITCHEN_SHELF, KEY_ARM_TABLE,   KEY_POINT_BREAKFAST_SPOON),
  ("cereal", KEY_POSE_CABINET,       KEY_ARM_CABINET, KEY_POINT_BREAKFAST_CEREAL),
  ("milk",   KEY_POSE_CABINET,       KEY_ARM_CABINET, KEY_POINT_BREAKFAST_MILK),
]
phaseServeBreakfast() = Sequence(memory=True)[ markPhase("breakfast"),
   *[ FailureIsSuccess(Sequence[
        goto(src); arm(arm_pose); reDetect(item→KEY_VISION_RESULT); Announce(f"retrieving {item}");
        Retry(grasp) > BtNode_Grasp(...);
        goto(KEY_POSE_TABLE); ScanAndPlace FIXED_POINT(fixed_target=point) ])
      for (item, src, arm_pose, point) in BREAKFAST ],
   Announce("breakfast served"),
]
```
Each item → its **own** `POINT_BREAKFAST_*` (the four constants already encode spoon-beside-bowl / milk-beside-cereal at the same z=0.75 table height; verify geometry yields the required adjacency). Per-item `FailureIsSuccess` ⇒ one failed retrieval doesn't abort breakfast.

### 8.5 Extra-surface phase
Same as table cleanup but `source_pose_key=KEY_POSE_EXTRA_SURFACE` and its own `BtNode_BuildInventory` (2 items; mock-seeds in mock). `markPhase("extra")`.

---

## 9. Component 5 — samplings (new entry points)
`run_tree`-style, mock-runnable, in `setup.py` `console_scripts`:
- `pp-test-scan-place` → seed held-object + mode/reference, tick `BtNode_ScanAndPlace` once (mock green; real hits server).
- `pp-test-categorize` → `BtNode_BuildInventory` + drain `BtNode_PopWorkItem` over a canned result; prints routing (no servers).
- `pp-test-cleanup-loop` → the `handleOneItem` loop over a canned 2-item queue (mock) — authoritative loop/fallback cover.
- `pp-test-breakfast` → `phaseServeBreakfast()` alone (mock).

Server smoke: `ros2 run arm_api scan_and_place_action` + a documented `dry_run=true` one-shot client → `status=0`, no hardware (**requires the rebuilt msg**, §11/§12).

(The v1 idea to upgrade GPSR `create_place` is **dropped** — out of scope / regression surface on a working task.)

---

## 10. Score-trace schema (new) `[Rv2]`
`KEY_SCORE_TRACE` (already declared, unused) holds:
```
{ "visited_phases": [],   # phase names, appended by markPhase(name)
  "events": [],           # list of {phase, item, action, outcome, points_est}
  "place_policy": str }   # 'vlm' (default) | 'hardcoded'; mirrors §8.0 + record_event
```
Initialized in `createConstantWriter`. A small `record_event(blackboard, phase, item, action, outcome, points_est)` helper is called by `BtNode_ScanAndPlace.process_result`, the grasp-failure/skip leaf, and `markPhase`. `phaseSummary()` announces a rollup. **Layer-B test asserts `set(visited_phases) ⊇ {"table","breakfast","extra"}`** and that ≥1 place event was recorded.

---

## 11. Timeout & budget design (new) `[Rv2]`
- **`BtNode_DeadlineGuard(budget_sec, clock=time.monotonic)`** — plain `Behaviour`. `initialise()` latches `self._deadline = self.clock() + budget_sec` (latched on entry, not construction). `update()` returns `RUNNING` until `clock() >= deadline`, then `SUCCESS` (never FAILURE). Injectable `clock` lets `test_timeout_policy` drive a fake clock; default `time.monotonic` works headless (no ROS node handle needed).
- **Global guard:** `MAX_RUNTIME_SEC` (390 s) at root (§8.1). **Per-phase guards:** budgets in §8.2. In fast mock tests neither fires, so all phases run fully.

---

## 11A. Mock-mode operation (whole-tree, first-class) `[user-requested]`
The rulebook tree must run **end-to-end with no servers and no robot**, reusing the package's existing mock framework (`config.is_mock_mode`, `mock_config.json`, the `ServiceHandler`/`ActionHandler` mock paths). Made explicit as a deliverable:
- **Action/Service nodes auto-mock:** `BtNode_ScanAndPlace`, `BtNode_Grasp`, `BtNode_GotoAction`, `BtNode_MoveArmSingle`, `BtNode_ScanForGeneralist`, `BtNode_DoorDetection`, `BtNode_GripperAction`, `BtNode_Announce` — under `IMMEDIATE` they auto-succeed after `_mock_auto_ticks_required` (2) ticks and **create no ROS client**. `BtNode_ScanAndPlace` is registered in `mock_config.json` (manipulation). No real server, camera, VLM, OpenRouter key, or grasp node is contacted.
- **Plain Behaviours run their real, hardware-free logic in mock** (must NOT inherit Handler mock): `BtNode_BuildInventory` (mock-seeds a canned 2–3 item queue when the upstream vision result is empty, so the per-item loop actually runs), `BtNode_PopWorkItem`, `BtNode_DeadlineGuard` (real `time.monotonic`; large budgets ⇒ never fires in a fast run), `guardClass`, `markPhase`, `record_event`.
- **Toggle surface:** global `BT_MOCK_MODE=true` (or `mock_config.json`); per-node interaction modes — `IMMEDIATE` for unattended smoke, `KEYPRESS`/`TELEOP` for step-through; per-subsystem toggles (mock vision+manip while keeping real TTS, etc.). `place_policy` is honored but inert in mock.
- **Result:** `pickAndPlaceRulebook()` ticks to SUCCESS in mock, traversing all three phases **and a populated cleanup loop body**, on a laptop. This *is* the Layer-B smoke test (§12) and the substrate for the `pp-test-*` samplings, and the way the tree is exercised before the robot is available.

---

## 12. Testing strategy (no robot hardware; full ROS available)

**Layer 0 — preconditions:**
- `python -c "import behavior_tree.messages"` must pass in mock. **The TTSCnRequest hazard currently does NOT reproduce on this host (import OK as of 2026-06-27; the risky line is `messages.py:35`, not `:43`)** `[Rv2]`. Keep a defensive per-symbol try/except around the `tinker_audio_msgs` import as harmless hardening, but do **not** block Layer A/B on it; re-verify at plan start.
- `tinker_arm_msgs` rebuilt with the extended `.action` (COLCON_IGNORE build-wrapper dance, §13) — required before Layer C and before `BtNode_ScanAndPlace`'s real branch resolves. The **mock** path (via `mock_messages.ScanAndPlace` stub) does not need the rebuild.

**Layer A — pure unit tests (pytest, no ROS):** `test_categorization.py` (routing incl. bowl note, category-map default-to-label, cabinet sort); `test_work_queue.py` (BuildInventory order, source≠target poses, PopWorkItem drains then FAILS on empty without raising, READ+WRITE access); `test_placement_logic.py` in arm_api (`resolve_margin_m(m,default)` clamps, `decode_bbox`, `depth_image_to_meters` uint16×0.001 vs float via `np.frombuffer`, `bbox_centroid_3d(K_tuple)`, `short_side_m`, `select_candidate` order, `build_vlm_messages` includes `reference_label` iff NEAR_SIMILAR); `test_timeout_policy.py` (DeadlineGuard with fake clock).

**Layer B — BT mock integration (pytest + `ros2 run` smoke):** build `pickAndPlaceRulebook()`, `setup()` under `BT_MOCK_MODE=true`+`IMMEDIATE`, **with BuildInventory mock-seeding a 2–3 item queue** so the loop body runs; tick to completion under an **explicit tick cap** (sized ≈ leaf-count × `_mock_auto_ticks_required` × phases, a few hundred; the test **asserts FAIL — not hang — if exceeded**, a safety net distinct from the in-tree wall-clock DeadlineGuard). Assert root `SUCCESS`, `visited_phases ⊇ {table,breakfast,extra}`, ≥1 place event. **Failure-injection variant:** force one grasp `FAILURE` in a populated cleanup route → assert the skip/help leaf fired and the mission still completes (remaining items still placed). Each sampling ticks green in mock.

**Layer C — server dry_run:** `ros2 run arm_api scan_and_place_action` + `dry_run=true` client → `status=0` (no hardware; `openai` lazy so no `OPENROUTER_API_KEY`). **Depends on the Layer-0 msg rebuild** (the new Goal fields don't exist in the installed message) `[Rv2]`.

Build/verify follows workspace rules: no raw colcon; `tkbuild`/build-wrapper; prefer `ros2 run` over ad-hoc python.

---

## 13. Build & deployment
- **`tinker_arm_msgs`** (`.action`): COLCON_IGNORE'd manip tree → move root sentinel aside, build with the manip wrapper, restore (`feedback_tkbuild_colcon_ignore`). No hardware. Downstream rebuilds (hash changed).
- **`arm_api`** (server + `placement_logic.py`): rebuild after msgs.
- **`behavior_tree`**: additive; `tkbuild tk25_decision` (prefer wrapper; never raw colcon). Pure pytest runs without a full build if importable on `PYTHONPATH`.
- **Order:** tinker_arm_msgs → arm_api → behavior_tree. User drives/approves builds; Claude may run the wrapper + read-only checks.

---

## 14. Config inventory — existing vs net-new `[Rv2]`
**Already exist in `config.py` (REUSE, do not redefine):** `KEY_POINT_BREAKFAST_BOWL/SPOON/CEREAL/MILK` (`:184-187`) + materialized `POINT_BREAKFAST_*` (`:123-126`); `KEY_BREAKFAST_QUEUE` (`:209`); `KEY_ACTIVE_SOURCE_POSE` (`:214`) + `KEY_ACTIVE_TARGET_POSE/POINT`; `KEY_VISION_RESULT` (`:221`), `KEY_OBJECT_LABEL` (`:224`); `KEY_SCORE_TRACE` (`:229`); `KEY_ARM_WASH/CABINET/TRASH/TABLE`; `KEY_POSE_KITCHEN_SHELF` (`:117`); `KEY_POSE_TABLE/WASH_STAGING/CABINET/TRASH_BIN`; label sets; `MAX_RUNTIME_SEC`.
**Net-new in `config.py`:** `SCAN_AND_PLACE_ACTION_NAME`, `PLACEMENT_MODE_*` (+`_NONE`), `DESTINATION_ROUTING`, `KEY_POSE_EXTRA_SURFACE`/`KEY_POINT_EXTRA_SURFACE`, `category_map` accessor, per-phase budget constants.

---

## 15. Constants & surface identity `[Rv2]`
- **Dining table = `pose_table_1`** (`config.POSE_TABLE` reads only `pose_table_1`; `pose_table_2..6` are unused future multi-table-sweep hooks, `pose_stove` unused).
- **Extra surface:** add **new** `pose_extra_surface` + `point_extra_surface` (placeholder + `TODO(setup-days)`); it is a distinct auxiliary surface, not one of the table poses.
- **Breakfast retrieval arm poses:** reuse `arm_pos_table` (kitchen-shelf) / `arm_pos_cabinet` as first cut.
- **Designated trash** (`DESIGNATED_TRASH_LABELS`, currently `["paper cup"]`) is **announced at Setup Days** (per the `constants.json` comment); document the Setup-Days constants edit as the capture step.
- **Cabinet is doorless** (rulebook) → retrieval/placement need no door handling.

---

## 16. Risks & open questions
- Top-down quaternion value — `TODO(hardware)`; mock-independent.
- NEAR_SIMILAR reliability — graceful FREE_SPACE degrade + `placement_mode_used` echo; first-of-category seeds free space (documented).
- `grocery_categorize` / `start_drop` server availability — both behind feature checks; static label path + kinematic trash release are the self-sufficient defaults.
- `constants.json` placeholders (extra-surface pose, breakfast geometry adjacency) — verify on hardware later.
- Concurrent committers on manip/decision repos — commit new only, never `--amend`/rebase, selective `git add` by path.
- Re-detect uses the generic table prompt; the grasp targets the popped item via `object_label`. Narrowing the generalist scan prompt to the per-item label is an on-robot refinement (the generalist node would need a blackboard-sourced prompt).

---

## 17. File-by-file change list
**tinker_arm_msgs:** `action/ScanAndPlace.action` — append goal fields + `placement_mode_used`.
**arm_api:** `arm_api/placement_logic.py` (NEW pure); `arm_api/scan_and_place_server.py` (import pure module; mode branching; scan-pose control; `dry_run`; orientation fix); `test/test_placement_logic.py` (NEW).
**behavior_tree:** `TemplateNodes/Manipulation.py` (NEW `BtNode_ScanAndPlace`); `messages.py` (+`ScanAndPlace` real branch; defensive audio import); `mock_messages.py` (+`ScanAndPlace` stub); `PickAndPlace/categorization.py` (NEW pure); `PickAndPlace/custom_nodes.py` (NEW `BtNode_BuildInventory`, `BtNode_PopWorkItem`, `BtNode_DeadlineGuard`, `guardClass`, `markPhase`, `record_event`; optional `BtNode_CategorizeUnknown`); `PickAndPlace/config.py` (net-new symbols per §14); `PickAndPlace/constants.json` (extra-surface keys, optional category_map, budgets — placeholders+TODO); `PickAndPlace/pick_and_place_rulebook.py` (NEW tree + phase factories); `PickAndPlace/cli.py` (repoint to rulebook; parse `--place-policy {hardcoded,vlm}`, default `vlm`, pass to factory) + `cli_demo.py` (NEW, old tree); `mock_config.json` (register `BtNode_ScanAndPlace`; **NOT** the plain Behaviours); `setup.py` (entries: `pick-and-place`→rulebook, `pick-and-place-demo`→old, `pp-test-scan-place/-categorize/-cleanup-loop/-breakfast`); `test/` (`test_categorization.py`, `test_work_queue.py`, `test_timeout_policy.py`, `test_rulebook_tree_mock.py`).
**Docs/changelog:** update `RULEBOOK_PLAN.md` status + each touched package's README changelog.
**Extension hook:** one commented line after `phaseExtraSurfaceCleanup` marking where a future optional-goals phase would attach (no code).

## 18. Non-goals (restate)
No dishwasher door/rack, tablet-in-slot, pouring, milk-opening — no nodes/stubs/branches. No new vision capability (grouping uses the VLM the server already calls). No hardware verification. No refactor of unrelated tasks; old `pickAndPlaceShortened` preserved; GPSR untouched.
