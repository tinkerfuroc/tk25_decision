# Changelog

## [2.2.17] - 2026-07-03

### 🛋️ HRI: turn-around before follow + host/guest scripting updates

- `HRI/config.py`: new `POSE_SOFA_REVERSED` / `KEY_SOFA_POSE_REVERSED` —
  same map point as `pose_sofa`, yaw rotated exactly 180°. Computed from
  `POSE_SOFA` at load time (not stored in `constants.json`), so re-teaching
  the sofa waypoint keeps the pair consistent. Seeded onto the blackboard
  by `createConstantWriter`. A follow-up fix commit (b8af330) committed the
  `arm_pos_orbbec_look` constants key required by a concurrent-session
  `config.py` hunk that rode along in the feature commit.
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

## [2.2.16] - 2026-07-02

### 🧭 GPSR: approach_person wired to real go_to_approach navigation, 1.3 m standoff

`create_approach_person()` (`GPSR/small_trees.py`) — previously a same-day
temporary no-op mock (`py_trees.behaviours.Success`) — now drives a real
`BtNode_Approach` against the `approach_planner` package's `go_to_approach`
action, reading `bb_keys.TARGET_PERSON_POSE` directly (dropped the
`PointStamped`→`PoseStamped` conversion hop). The goal always passes
`desired_distance=1.3` / `min_distance=1.0` / `max_distance=1.6` /
`timeout_sec=45.0` together, as module constants. The bounds trio is
mandatory, not cosmetic: `approach_planner`'s attempt-2 costmap-ring
recompute rejects any goal with `desired` outside `[min, max]`
(`STATUS_INVALID_REQUEST`, `algorithm.py:184`), and a bare
`desired_distance=1.3` call (server defaults `min=0.5`/`max=1.0`) would have
silently disabled that whole fallback path. Deleted the now-dead
`BtNode_PointToPoseStamped` helper and the `bb_keys.PERSON_NAV_POSE` key
(plus 3 now-unused `geometry_msgs` imports). `BtNode_Approach` is registered
`IMMEDIATE` in all three mock configs (`mock_config.json`,
`config/f4_mock_config.json`, `config/full_mock.json`) — previously it was
reachable only via the navigation subsystem's catch-all.

New hard runtime dependency: the `approach_planner` node must be launched
for GPSR's `approach_person` step to do anything — `master_gpsr.sh`
(tk25_basic) now brings it up as a third pane in the `navigation` window,
mirroring the existing Restaurant-task pane in `tmux_slam_navigation.sh`.

CAUTION: if `approach_planner`'s `two_stage_approach` mode is ever turned
on, Stage B ignores the goal's `desired_distance` entirely and stops at its
own fixed `final_standoff` (0.7 m) — the 1.3 m standoff documented here only
holds with `two_stage_approach` at its current default-off.

New test `test/test_gpsr_approach_person.py` (2 tests) pins the
`BtNode_Approach` wiring and the bounds ordering.

HONESTY NOTE: commit `3d1c431` also carries an in-flight pan-tilt-sweep
refactor (`_pantilt_sweep`, `find_object`/`find_person`/`describe_person`
reshuffle) from a concurrent session that was interleaved uncommitted in
`small_trees.py` at commit time — unrelated to this wiring, called out here
so it isn't a surprise to future readers of that commit.

## [2.2.15] - 2026-07-02

### 🧺 DoingLaundry: "move it closer" prompt on an out-of-range fold

`foldClothingOnce()` is now a `Selector` (memory) instead of a flat `Sequence`.
The first branch resets the `dl_fold_out_of_range` blackboard flag to `False`,
runs `BtNode_FoldClothingDn(bb_key_out_of_range=KEY_FOLD_OUT_OF_RANGE)`, and
announces completion. When the `fold_dn_action` server aborts because the grasp
target is beyond its `max_range` (0.85 m default), it reports an `OUT_OF_RANGE`
token in the existing `FoldClothing.Result.message` field (no interface change);
`BtNode_FoldClothingDn.process_result()` detects the token, writes `True` to the
flag, and returns FAILURE. The Selector's second branch then runs
`BtNode_CheckIfEmpty(dl_fold_out_of_range)` — SUCCESS iff the flag is truthy —
followed by the announce "The clothing is too far, please help me put the
clothing closer.", so
the Selector succeeds and the enclosing `Repeat(num_success=999)` retries (the
operator repositions the garment and the next iteration re-prompts + folds). Any
NON-out-of-range fold failure leaves the reset flag `False` → `CheckIfEmpty`
FAILs → Selector FAILs → task ends, exactly as before.

`BtNode_FoldClothingDn` gained a `bb_key_out_of_range` ctor arg with a `setup()`
override that attaches a WRITE blackboard client and a `process_result()`
override that publishes the flag; the change is localized to the subclass — the
shared `BtNode_FoldClothingAction` base (also used by `fold_clothing_action`) is
untouched. New config constant `KEY_FOLD_OUT_OF_RANGE = "dl_fold_out_of_range"`.

## [2.2.14] - 2026-07-02

### 🔭 hri-2026: "Look at host" tilt 45° → 35°

The bag-flow follow stage commanded the pan-tilt head 45° up right before
follow start (`createBagFlowReal2026`), putting the camera on the
ceiling/crowd instead of the host — the tracker then acquired low-confidence
background "person" blobs and fed phantom points to the follower
(2026-07-02 vision_log evidence). Lowered to 35°, matching the canonical
`hri.py` "look up" value. Both `HRI/hri_2026.py` and the `HRI/HRI/`
duplicate updated in lockstep. Regression-locked by
`test_hri_2026_start_gate.py::test_look_at_host_tilt_is_35_degrees`.

## [2.2.13] - 2026-07-01

### 🚪 Inspection: announce readiness + aim pan-tilt before the door-wait

Added an opening step to `createInspection()`: right after the arm tucks to the
nav pose (so the robot genuinely *is* ready) and before the door-detection wait,
a `Parallel` (`SuccessOnAll`) runs two children together — `BtNode_Announce`
"I am ready for inspection, please open the door" and `BtNode_TurnPanTilt`
(pan=0°, tilt=45°) to aim the head at the referees. The subsequent
door-detection → "door open" announce → navigate → introduce → Enter-wait →
exit flow is unchanged. Root child count 9 → 10.

Tests: extended `test/test_inspection_tree.py` — updated child-order/count
assertions, added `test_ready_announce_and_pan_tilt_run_in_parallel` (the
parallel holds both the readiness announce and the pan=0/tilt=45 turn) and
`test_ready_announce_precedes_door_detection`.

## [2.2.12] - 2026-07-01

### 🚪 Inspection: trimmed door→announce→inspect→enter→exit flow + robust Enter-wait

Rewrote `createInspection()` to the competition flow: tuck arm to the nav pose →
wait for the door to open (`BtNode_DoorDetection` under `Retry(999)`) →
**announce "door open"** (previously missing) → navigate to InspectionPoint →
briefly introduce ("Dear referees, I am Tinker.", replacing the verbose
self-description) → wait for the operator to press Enter → announce "Heading to
the exit." → navigate to ExitPoint. Dropped the unused Q&A subtree and its dead
imports/keys. Poses still load from `Inspection/constants.json` (`map` frame);
`cli.py` / `setup.py` entry (`ros2 run behavior_tree inspection`) unchanged.

Hardened `BtNode_PressEnterToSucceed` (`Inspection/customNodes.py`): the old
`is_enter_pressed()` fired on any pending stdin and never consumed it, so a
keystroke buffered during the door-wait/navigation could skip the human
checkpoint. It now drains stale buffered input on `initialise()` (EOF-guarded so
a closed fd can't spin), returns SUCCESS only when a full Enter-terminated line
is read — consuming it — and guards EOF symmetrically in `update()` (returns
RUNNING, never a false SUCCESS on a closed/piped stdin). Also decoupled the
module from the `behavior_tree.messages` import chain (dropped a dead
`PointStamped` import).

Tests: `test/test_inspection_tree.py` (tree structure: child order, the new
"door open" announce sits right after door detection, verbose self-intro gone)
and `test/test_press_enter_node.py` (RUNNING/SUCCESS/consume, stale-input drain,
EOF guards in both `initialise` and `update`).

## [2.2.11] - 2026-06-14

### 👋 follow-person: wave-to-reseed escape from the NEEDS_HELP hold

Wires the manual recovery into the follow tree. When the tracker can no longer
auto re-acquire it escalates to NEEDS_HELP (`reacquisition_state == 2`) and
`BtNode_ReacqAnnounce` asks the operator to raise a hand — but nothing acted on
that hand: nothing in `tk25_decision`/`tk26_navigation` called the tracker's
`~/reseed_target`, so once auto passive re-ID couldn't recover, the follow sat
alive (post-2.2.10) but frozen. (The companion tracker fix — passive re-ID now
runs indefinitely under the hold instead of dead-ending at ~20 s — landed in
`vision_track` the same day.)

New `BtNode_WaveReseed` (added to the follow tree's per-tick **reactions
Sequence** beside `BtNode_ReacqAnnounce`): while latched in NEEDS_HELP it polls
`detect_waving_persons` and, on an **unambiguous single waver**, reseeds the
tracker onto that person's box via `~/reseed_target`. Zero or multiple wavers →
no reseed (precision: never re-lock onto an ambiguous candidate; keep
announcing) — the same wave-to-resume policy proven in the `track_web` dashboard.
Non-blocking (one async call in flight, IDLE→WAVE_PENDING→RESEED_PENDING across
ticks, DetectWaving throttled to 3 s) and **always returns SUCCESS**, so it
composes with the 2.2.10 never-mid-abort Parallel. Mock/unavailable → inert
no-op. `ReseedTarget` added to `messages.py` (+ mock). The follow bringup
(`navigation_bringup/follow_bringup.launch.py`) now starts `waving_person_server`
on the real robot (`sim:=false`) so the loop works end-to-end.

Tests: `test/test_wave_reseed.py` (8: gating, single/zero/multi-waver, bad
status, throttle, leave-NEEDS_HELP cancel, no-bridge inert) +
`test_follow_tree_build.py` updated for the two-reaction Sequence.

## [2.2.10] - 2026-06-14

### 🛡️ follow-person: never mid-abort — the tree stays alive through losses

Fixes "the follow BT stops when the person is ~6–7 m away." Root cause: the tree
is a `Parallel(SuccessOnAll)` over `[track, follow, reactions]`, and `SuccessOnAll`
returns FAILURE the instant any child fails — ending the whole follow. Near that
range the person gets small/ReID-weak (the lock is RGB-based) and walks past the
Femto Bolt depth sensor's ~5.5 m range, so the `TrackPerson` action ends → the
tracker publishes `reacq_state = INACTIVE` → `follow_server` →
`ABORTED_TARGET_LOST` → `BtNode_FollowAction` FAILURE → the Parallel fails → the
BT exits.

Fix: wrap **every** Parallel child in `py_trees.decorators.FailureIsRunning`, so a
child's action terminating becomes RUNNING, never FAILURE. The `SuccessOnAll`
Parallel can no longer fail, so the tree **never mid-aborts** — it stays alive,
and on the next tick the wrapped action re-initialises and re-dispatches (the
tracker re-acquires, the follow resumes) when the person is back in range. The
follow now ends only on an explicit external stop. NOTE: re-dispatching
`TrackPerson` re-seeds the lock, so a lost operator may be re-acquired as whoever
is in frame — identity persistence on re-acquire is a tracker concern, not the
BT's.

### Files modified
- `FollowPerson/follow_person.py` — wrap track / follow / reactions in
  `FailureIsRunning`; removed the non-functional `Sequence[Announce, root, Running]`
  experiment (a Sequence still propagates the child's FAILURE) and its now-unused
  `BtNode_Announce` import; module + factory docstrings updated.
- `test/test_follow_tree_build.py` — `test_every_child_is_failure_is_running_never_mid_abort`;
  existing structural tests unwrap the decorator via an `_inner()` helper.

---

## [2.2.9] - 2026-06-14

### 🗺️ follow-person: tracker emits `/target_points` in `map` when navigating

`create_follow_person_tree(target_frame=...)` now defaults to `None`, resolving
to **`"map"` when `enable_navigation=True`** and **`""` (camera frame) when
False**. With navigation on, the `TrackPerson` goal requests the `map` frame so
the tracker transforms `/target_points` into `map` at the source — every
consumer (including `follow_server`, whose `working_frame` is also `map`) then
reads a map-frame point with **no further TF transform**, instead of relying on
`follow_server` to convert from the camera frame.

This is safe because the full follow pipeline runs Nav2 + localization
(AMCL/SLAM), so the tracker's `map`→camera lookup resolves immediately. The
vision+audio-only mode (`enable_navigation=False`) has no localization, so it
**keeps the camera frame** — requesting `map` there would block the tracker
~0.2 s/frame on a failing lookup, collapse the ~30 Hz loop to ~5 Hz, and starve
reacquisition. That is exactly the **v2.2.6** regression; coupling the default to
`enable_navigation` preserves the v2.2.6 fix for the no-nav mode while giving the
real following case a map-frame product. An explicit `target_frame` still
overrides either default (e.g. `""`, or `"odom"` for out-of-arena SLAM).

### Files modified
- `FollowPerson/follow_person.py` — `target_frame: str | None = None`, mode-based
  resolution (`"map"` with nav, `""` without); rationale docstring.
- `test/test_follow_tree_build.py` — `test_track_person_outputs_map_frame_when_navigating`,
  `test_track_person_keeps_camera_frame_without_nav`,
  `test_track_person_frame_explicit_override_wins`.

---

## [2.2.8] - 2026-06-13

### 🚶 follow-person: default to open following (no breadcrumbs)

`create_follow_person_tree` now defaults `use_breadcrumbs=False`, and the
`follow-person` CLI gains a `--breadcrumbs` opt-in flag (alongside the existing
`--no-nav`). Open following — the common case — drives single-goal standoff
pursuit (`follow_server` NavigateToPose, re-planned to the person's live
position at 2 Hz), which tracks a moving person directly.

Why the flip: on a long open route the accumulated breadcrumb corridor pins the
robot to a stale trail it never advances on — instrumented sim runs showed the
breadcrumb-on robot sit at the origin while the person walked away, crumbs
piling to the cap, `follow_server` dispatching NavigateThroughPoses but Nav2
making no progress. Single-goal pursuit does not have this failure mode. Trail
routing is still the right tool for cluttered/doorway following (threading the
person's exact path through a gap), so it remains available via `--breadcrumbs`.

- `FollowPerson/follow_person.py`: new `use_breadcrumbs: bool = False` param on
  `create_follow_person_tree`, passed through to `BtNode_FollowAction`.
- `FollowPerson/cli.py`: `--breadcrumbs` flag (default off).
- `test/test_follow_tree_build.py`: `test_default_open_following_disables_breadcrumbs`
  + `test_breadcrumbs_opt_in`.
- `BtNode_FollowAction`'s own constructor default is unchanged (the open-following
  policy lives in the tree builder / CLI, not the generic node).

## [2.2.7] - 2026-06-10

### 🐛 run_tree: cancel the action goal when stopped with SIGTERM

Fixes "Stop All (or stopping the BT) does not stop the TrackPerson goal — the
tracker keeps tracking." When the track_web ProcessManager stops the
follow-person process it sends **SIGTERM** to the process group. Python's
default SIGTERM disposition terminates the process immediately *without* raising
an exception, so `run_tree`'s `try/except (KeyboardInterrupt,
ExternalShutdownException)` never fires and the `finally` (which is what cancels
in-flight action goals) never runs. The goal leaks and the tracker keeps going.

Two parts to the fix, both in `run_tree`:
1. Install a SIGTERM handler that raises `KeyboardInterrupt` (mirroring SIGINT),
   so the process unwinds through the `finally` instead of dying abruptly — and
   crucially does **not** call `rclpy.shutdown()` (which would invalidate the
   context and make the cancel un-sendable).
2. Cancel + **flush** before the node is destroyed. `tree.shutdown()` issues the
   cancel via `terminate(INVALID)` but then destroys the node in the same call,
   so the async `cancel_goal_async()` request never reaches the server. The
   `finally` now stops the tick timer, stops the tree root (RUNNING→INVALID,
   firing each action node's `send_cancel_request()`), and spins briefly
   (≤2 s, well under ProcessManager's 5 s SIGTERM→SIGKILL grace) to flush the
   cancel while the node is alive, then does the normal shutdown.

Verified against a throwaway `/track_person` server: a `killpg(SIGTERM)` on the
follow-person session (exactly what ProcessManager does) now produces a
server-side goal cancellation; pre-fix it did not.

### Files modified
- `behavior_tree/runtime.py` — SIGTERM→KeyboardInterrupt handler + cancel-and-flush in the shutdown `finally`

---

## [2.2.6] - 2026-06-10

### 🐛 follow-person: stop requesting a `map` frame the demo doesn't have

Fixes two reported symptoms with one root cause: (1) the tracker failed to
re-acquire the operator after a loss when driven by the follow-person BT
(it worked standalone from the track_web webui), and (2) no reacquisition
voice announcement ever played.

Root cause: `create_follow_person_tree()` defaulted `target_frame="map"`, so
`BtNode_TrackPersonAction` sent `TrackPerson.Goal.target_frame="map"`. The
webui sends `""`. With a non-empty frame the tracker does a blocking TF lookup
(`timeout=0.2s`) **every tracked frame**; the follow demo has no `map` frame
(dummy_nav is a stub, no nav stack / TF), so each lookup waits out its full
timeout. Measured: the tracking loop collapses from ~30 Hz to ~4.9 Hz. ReID
reacquisition (N consecutive gallery-matched frames + ByteTrack id continuity,
all built in the tracked branch) is starved at 5 Hz and never re-locks — so
`track/reacquisition_state` never reaches PASSIVE/NEEDS_HELP, and
`BtNode_ReacqAnnounce` (which only speaks on those states) stays silent. The
audio chain itself was verified correct end-to-end; it was simply never
triggered.

Fix: default `target_frame=""` so the tracker keeps the camera frame and does
zero TF lookups → full ~30 Hz → reacquisition + escalation + announcements
behave exactly as standalone. `BtNode_PublishFollowGoal` republishes the
camera-frame `PointStamped` as-is (correct for a map-less demo). Non-breaking:
no test asserts the goal frame; the finalized tracker is untouched. Docstring
warns against re-introducing `"map"` without a live TF source.

### Files modified
- `behavior_tree/FollowPerson/follow_person.py` — `create_follow_person_tree(target_frame="")` (was `"map"`) + rationale docstring

---

## [2.2.5] - 2026-06-10

### 🐛 `ActionHandler.setup()` survives context shutdown mid-wait

Fixes a crash where action-based BT nodes (e.g. `follow-person`'s
`BtNode_TrackPersonAction`) died during `setup()` with a raw
`RCLError: rcl node's context is invalid` instead of failing cleanly.

Root cause: with the `wait_for_server_timeout_sec <= 0` retry-forever
convention used by every action node, `setup()` loops on
`ActionClient.wait_for_server()`. rclpy's implementation runs
`while node.context.ok() and not server_is_ready()` then calls
`server_is_ready()` one final time **unconditionally** — so when a shutdown
signal (Ctrl-C / SIGTERM / launcher teardown) invalidates the context during
the wait, that last call touches a dead node and raises `RCLError`. The retry
loop never checked for context shutdown, so a missing server during teardown
surfaced as that cryptic error rather than a clean exit.

Fix lives at the shared base so every action node (nav/manip/audio) benefits:
- `_context_ok()` — checks `node.context.ok()` (falls back to `rclpy.ok()`).
- `_wait_for_server_once()` — wraps `wait_for_server`, translating a
  context-shutdown `RCLError` into the documented
  `py_trees_ros.exceptions.TimedOutError` (chained `from` the original);
  a genuine `RCLError` with a live context is re-raised unchanged.
- The retry loop checks `_context_ok()` before each attempt and raises a clean
  `TimedOutError` on shutdown instead of calling `wait_for_server` again.

Success path is untouched; the `track/reacquisition_state` blackboard write and
`follow` / `help-me-carry` behaviour are unaffected.

### Files modified
- `behavior_tree/TemplateNodes/ActionBase.py` — `_context_ok()` / `_wait_for_server_once()` helpers + shutdown-aware retry loop
- `test/test_action_base_setup_shutdown.py` — new regression test (fails pre-fix: raw `RCLError`; passes post-fix: clean `TimedOutError`)

---

## [2.2.4] - 2026-05-02

### 🧭 New `BtNode_GetOrientationAngle`

Adds a thin `ServiceHandler` BT node that calls
`orientation_angle_service` (`tinker_nav_msgs/srv/OrientationAngle`,
served by `tk26_navigation/orientation_angle_service`) and writes the
returned angle (radians, pan_tilt sign convention) to a blackboard key.
Compose with existing `BtNode_TurnPanTilt` / `BtNode_TurnTo` downstream
to face the server-side hardcoded target (currently SOFA).

Signature:
```python
BtNode_GetOrientationAngle(
    name, bb_dest_key,
    max_try=3, timeout=2.0,
    service_name="orientation_angle_service",
)
```

The srv has no status field; the server returns `0.0` on pose timeout,
indistinguishable from a real 0 rad. Node returns SUCCESS whenever the
service responds.

### Files modified
- `behavior_tree/messages.py` — import `OrientationAngle` from `tinker_nav_msgs.srv` (real + mock paths)
- `behavior_tree/mock_messages.py` — add `OrientationAngle` mock stub with `max_try`/`timeout` request fields and `angle` response field
- `behavior_tree/TemplateNodes/Navigation.py` — add `BtNode_GetOrientationAngle` class + module-docstring entry; import `OrientationAngle`
- `behavior_tree/mock_config.json` — register `BtNode_GetOrientationAngle: IMMEDIATE` under `navigation` subsystem

---

## [2.2.3] - 2026-04-19

### 🧹 HRI intake fallback — drop deprecated `BtNode_PhraseExtraction`

`HRI/hri.py:_create_get_info`'s fallback branch no longer imports or calls
the service-based `BtNode_PhraseExtraction` (deprecated, emits
`DeprecationWarning`). It now uses `BtNode_ListenAction` to capture raw
speech into the same storage key, then reuses the existing
`BtNode_Confirm` + `BtNode_GetConfirmationAction` tail. HRI is now
completely off deprecated service-based audio nodes — all audio leaves in
the HRI package are either action-based or wrap `TextToSpeech` (which has
no action counterpart).

Primary path behaviour is unchanged: on cross-check success (server
status=0) the `Selector` short-circuits and the fallback never ticks,
preserving the rulebook 4×15 "no non-essential questions" bonus.

Scope limited to HRI; Receptionist, Restaurant demo, GPSR, help-me-carry,
serve-breakfast, store-groceries, grasp-intel still call the deprecated
node and will migrate when next touched.

### Files modified
- `behavior_tree/HRI/hri.py`

---

## [2.2.2] - 2026-04-19

### 🧪 Standalone intro harness + mock-config catch-up

- New `HRI/intro.py` + `hri-intro` console script. Seeds two mock guest
  profiles on the blackboard and runs just `createTwoWayIntroduction`, so
  the feature-match → pre-orient → announce path can be KEYPRESS-stepped
  end-to-end without the full HRI task. Mirrors `hri-follow` / `hri-intake`.
- `mock_config.json` — added missing class-name registrations so the new
  harness runs under `BT_MOCK_MODE=true`:
    - `announcement` subsystem: `BtNode_Introduce`, `BtNode_Confirm`.
    - `vision` subsystem: `BtNode_TurnTo` (was inheriting from
      `BtNode_TurnPanTilt` but class-name lookup doesn't walk the MRO).
- `HRI/hri.py` — `_with_gaze_supervisor` migrated from the deprecated
  `BtNode_HeadTrackingAction` shim to `BtNode_MaintainEyeContact` directly,
  per the migration path documented in `.claude/rules/behavior-tree.md`.
  Removes a DeprecationWarning on every HRI boot.

---

## [2.2.1] - 2026-04-19

### ✨ Target-guided gaze during HRI introductions

Partially fixes the rulebook 5.1 "look to the correct guest while talking
about the other guest" 2×50 pts item. Previously the parallel-sibling
`BtNode_MaintainEyeContact` locked onto whichever face was geometrically
closest to the base — a coin flip for which of two seated guests got the
right intro gaze.

- `BtNode_FeatureMatching` — added `trim_last_person: bool = True` kwarg.
  Default preserves Receptionist's "new guest just walked up, not seated
  yet" behavior; `False` sends every persons's features so centroids come
  back for all seated guests. Mock path now emits one fake centroid per
  (non-trimmed) person instead of a single element.
- `HRI/config.py` — new `KEY_PERSON_CENTROIDS` blackboard key.
- `HRI/hri.py:createTwoWayIntroduction` — runs feature matching once with
  `trim_last_person=False` to populate `KEY_PERSON_CENTROIDS`, then each
  intro now does `BtNode_TurnTo(target_id)` → `BtNode_Introduce`
  inside the existing gaze supervisor. Physically turning the head first
  makes the target guest the closest face, so `MaintainEyeContact`
  locks on the correct one.
- Best-effort wrapping (`FailureIsSuccess` around feature matching and
  turn-to) ensures the intro still fires if the scan fails — falls back
  to previous closest-face behavior without crashing the tree.

---

## [2.2.0] - 2026-04-19

### ✨ New — Action-based phrase extraction integrated into HRI intake

Migrates the HRI name/drink capture flow from a confirmation-heavy
service pipeline to a high-confidence-first action pipeline, to earn the
RoboCup@Home 2026 rulebook's **4×15 "no non-essential questions"** bonus
when the ASR cross-check agrees.

#### New template node
- `BtNode_PhraseExtractionAction` (`TemplateNodes/Audio.py`) wraps
  `tk_24_audio`'s new `phrase_extraction_action`
  (`tinker_audio_msgs/action/PhraseExtraction`). The server runs Whisper
  + Qwen ASR sequentially and calls `goal_handle.succeed()` only on
  server-status=0 (both engines agreed on the same wordlist entry);
  every other status calls `goal_handle.abort()`. Terminal
  `STATUS_SUCCEEDED` is therefore a high-confidence signal.

#### HRI integration
- `HRI/hri.py:_create_get_info` rewritten as
  `Selector(Retry(2, primary), last-resort-fallback)`:
  - **Primary** (up to 2 attempts): prompt → `BtNode_PhraseExtractionAction`.
    On success, no confirmation question is asked — banks the no-nonessential-questions bonus.
  - **Fallback** (one attempt, only when both primary attempts abort):
    prompt → legacy `BtNode_PhraseExtraction` → `BtNode_Confirm` →
    `BtNode_GetConfirmationAction`. Preserves partial-score coverage in
    noisy environments, at the cost of the bonus for that field.
- New `HRI/intake.py` + `hri-intake` console script — isolated harness
  for the name/drink intake subtree. Same pattern as `hri-follow`.

#### Messages / mock plumbing
- `messages.py` imports `PhraseExtraction as PhraseExtractionAction` with
  mock fallback.
- `mock_messages.py` adds `PhraseExtractionAction(MockAction)` stub.
- `mock_config.json` adds `"BtNode_PhraseExtractionAction": "KEYPRESS"`
  under `audio_input.nodes`.

#### Deprecations
- `BtNode_PhraseExtraction` (service-based) now emits
  `DeprecationWarning`, matching the `BtNode_Listen` /
  `BtNode_GetConfirmation` retirement policy. HRI is the first migrated
  consumer; Receptionist, Restaurant demo, GPSR, grasp-intel still call
  the legacy node and will migrate when next touched.

#### Fixes
- `HRI/config.py` — `arm_pos_point_to` re-added to `HRI/constants.json`
  so the eagerly-evaluated `.get(key, constants[other])` default in the
  arm-pose loader succeeds. (Unrelated pose-tuning edits to
  `HRI/constants.json` were made separately.)

### Files modified
- `behavior_tree/TemplateNodes/Audio.py`
- `behavior_tree/messages.py`
- `behavior_tree/mock_messages.py`
- `behavior_tree/mock_config.json`
- `behavior_tree/HRI/hri.py`
- `behavior_tree/HRI/intake.py` *(new)*
- `behavior_tree/HRI/constants.json` *(added `arm_pos_point_to` key)*
- `setup.py` *(new `hri-intake` console script)*

---

## [2.0.0] - 2026-01-18

### 🎉 Major Release - Complete Mock Mode Restructuring

This release completely overhauls the mock mode system to provide subsystem-level control, keyboard-based step-through execution, and zero-code-change compatibility.

### ✨ New Features

#### JSON-Based Configuration System
- **`mock_config.json`** - Central configuration file for subsystem-level mock control
- **5 Subsystems** - Independently control vision, manipulation, navigation, audio_input, and announcement
- **Per-Node Configuration** - Each node listed under its subsystem with granular control
- **Announce Movement** - Optional TTS announcements for each subsystem (using pyttsx3)
- **Auto-Detection** - Automatically enables mock mode when Tinker packages are unavailable

#### Keyboard Control
- **Step-Through Execution** - Press 's' to advance through each mock action
- **Non-Blocking Input** - Uses termios/tty for proper terminal handling
- **Configurable** - Can be disabled via `keyboard_control.enabled` in config

#### TTS Announcements (Optional)
- **pyttsx3 Integration** - Native Python text-to-speech for mock actions
- **Per-Subsystem Control** - Enable/disable via `announce_movement` flag
- **Error Handling** - Automatically falls back to print if TTS fails
- **Reusable Engine** - Global TTS engine instance to avoid segfaults

#### Configuration System
- **`config.py`** - Enhanced configuration loader with helper functions
- **Priority System** - ENV variable > JSON config > auto-detection
- **Helper Functions**:
  - `is_mock_mode()` - Check global mock status
  - `is_subsystem_mocked(name)` - Check specific subsystem
  - `is_node_mocked(class_name)` - Check specific node
  - `should_announce_movement(class_name)` - Check TTS setting
  - `announce_node_action(name, class_name)` - Announce via TTS
- **Status Printing** - `print_status()` shows complete configuration

### 🔧 Updated Components

#### Template Nodes (30+ nodes updated)
All template nodes now have native mock support with automatic configuration checking:

**Audio.py** (9 nodes)
- BtNode_TTSCN, BtNode_Announce
- BtNode_WaitForStart, BtNode_GraspRequest
- BtNode_PhraseExtraction, BtNode_TargetExtraction
- BtNode_GetConfirmation, BtNode_Listen, BtNode_CompareInterest

**Vision.py** (9 nodes)
- BtNode_ScanFor, BtNode_TrackPerson, BtNode_FindObj
- BtNode_FeatureExtraction, BtNode_FeatureMatching
- BtNode_SeatRecommend, BtNode_GetPointCloud
- BtNode_DoorDetection, BtNode_TurnPanTilt

**Manipulation.py** (7 nodes)
- BtNode_Grasp, BtNode_Place, BtNode_Drop
- BtNode_MoveArmJoint, BtNode_MoveArmSingle
- BtNode_GripperAction, BtNode_PointTo

**Navigation.py** (3 nodes)
- BtNode_GotoAction, BtNode_ConvertGraspPose, BtNode_GoToLuggage

**HRI/follow.py** (8 nodes, action-based follow-person subtree)
- BtNode_TrackPersonAction, BtNode_FollowAction, BtNode_IsTargetVisible
- BtNode_UpdateLossElapsed, BtNode_LossElapsedAtLeast, BtNode_WriteBBIfVisible
- BtNode_FlagIsFalse, BtNode_SetFlag
- `createFollowPerson(cfg)` factory; `hri-follow` console entry point

#### Base Classes
- **BaseBehaviors.py** - Updated ServiceHandler with:
  - Node-specific mock detection via `is_node_mocked()`
  - `wait_for_keypress_in_mock()` helper
  - TTS announcement integration
  - Mock service call handling

- **ActionBase.py** - Updated ActionHandler with:
  - Same mock detection pattern as ServiceHandler
  - Mock action client and futures
  - Mock result messages
  - Keyboard control integration

### 📝 Scripts Updated

#### receptionist_2ndcall.py
- Removed `MOCK_MODE` constant
- Navigation nodes use unified configuration
- No more conditional node creation
- All mock behavior controlled via `mock_config.json`

### 🗑️ Deprecated/Removed

#### Removed Files
- `MOCK_CONFIG_GUIDE.md` - Merged into README.md
- `MOCK_MODE_README.md` - Merged into README.md
- `RESTRUCTURING_SUMMARY.md` - Merged into README.md
- `NODES_FIXED_SUMMARY.md` - Merged into README.md
- `QUICKSTART.md` - Merged into README.md
- `CHANGES_SUMMARY.md` - Replaced by this CHANGELOG.md

#### Deprecated Patterns
- ❌ Manual `MOCK_MODE` checks in behavior tree scripts
- ❌ Conditional node creation (real vs mock)
- ❌ `BtNode_WaitKeyboardPress` as mock replacement
- ✅ Now: Use template nodes directly, they handle mocking automatically

### 🔄 Migration Path

No breaking changes! All existing scripts work without modification. However, you can clean up old patterns:

**Before (v1.x):**
```python
MOCK_MODE = is_mock_mode()
if not MOCK_MODE:
    node = BtNode_GotoAction("go somewhere", "pose_key")
else:
    node = BtNode_WaitKeyboardPress("MOCK: go somewhere", 's')
root.add_child(node)
```

**After (v2.0):**
```python
# Just use the node - mocking automatic
node = BtNode_GotoAction("go somewhere", "pose_key")
root.add_child(node)
```

### 📚 Documentation

- **README.md** - Completely rewritten with:
  - What's New section
  - Migration guide
  - Comprehensive configuration documentation
  - Updated examples and troubleshooting
- **CHANGELOG.md** - This file!

### 🐛 Bug Fixes

- Fixed navigation nodes not following unified mock pattern
- Fixed pyttsx3 segmentation faults with reusable engine instance
- Fixed AttributeError when action_client is None in mock mode
- Fixed TTS announcement calls with proper error handling

### 🎯 Configuration Priority

The system now uses a clear priority order:
1. `BT_MOCK_MODE` environment variable (highest priority)
2. `mock_config.json` settings
3. Auto-detection based on available packages (lowest priority)

### 📊 Statistics

- **Files Modified**: 15+ core files
- **Nodes Updated**: 30+ template nodes
- **Lines Added**: ~2000+ lines of new functionality
- **Documentation**: 1 comprehensive README, 1 CHANGELOG
- **Backwards Compatible**: 100% - no breaking changes

### 🙏 Acknowledgments

Special thanks to the Tinker team and RoboCup@Home community for testing and feedback.

---

## [1.x] - Previous Versions

Earlier versions with basic mock mode support. See git history for details.
