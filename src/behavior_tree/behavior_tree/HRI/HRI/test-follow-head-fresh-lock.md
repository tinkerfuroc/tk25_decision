# Operator runbook: follow_head fresh-lock layered tests

Three independent drills that validate the slim `FollowHeadAction` goal
modes on real hardware. Run them in order: Layer 1 catches regressions in
the legacy "closest person" contract, Layer 2 covers the new
`track_centermost` mode, Layer 3 is the closed-loop seeded-lock test that
mirrors the receptionist / HRI flow.

Pass/fail signals throughout:
- **Audio cues** (Layer 3) — the BT speaks at every phase boundary.
- **`[follow_head perf …]` stderr** on the `follow_head` terminal (every
  ~2 s when active). `cmd_issued` should be > 0 when the head is moving;
  `tracker_no_lock` indicates the seed mode rejected all candidates.
- **`vision_log/<YYYYmmdd_HHMMSS>/` JSON overlay** — each tick writes
  `extras.fresh_lock_mode ∈ {seed, centermost, closest}` and
  `chosen_root_xyz`. Inspect after the run to confirm the right mode was
  exercised.
- **`ros2 topic echo /follow_head_action/_action/feedback`** — live
  values for `person_visible`, `error_deg`, `current_pan/tilt`.

## Pre-flight (all layers)

```bash
cd ~/tk25_ws
colcon build --packages-select tinker_vision_msgs_26 pan_tilt behavior_tree
source install/setup.bash
```

Bring up, in separate shells:

| Shell | Command | Required for |
|---|---|---|
| 1 | Camera launch (Orbbec Femto Bolt). See `src/tk26_vision/CAMERA_BRINGUP.md` for the canonical sequence — needed to sustain ~30 Hz on `/camera/color/image_raw` so first feedback fires inside the 3 s contract. | All |
| 2 | `ros2 launch pan_tilt pan_tilt.launch.py` (controller + state_publisher) | All |
| 3 | `ros2 run pan_tilt follow_head` | All |
| 4 | `ros2 run object_detection_generalist generalist_node` | Layer 3 only |
| 5 | Announce service (whatever brings up `/announce` TTS) | Layer 3 only |
| 6 | `ros2 topic echo /follow_head_action/_action/feedback` | Optional, all layers |

Sanity:
```bash
ros2 topic hz /camera/color/image_raw      # ~30 Hz
ros2 service list | grep object_detection  # Layer 3
ros2 service list | grep announce          # Layer 3
ros2 action list | grep follow_head        # /follow_head_action present
```

---

## Layer 1 — default closest (regression check)

Verifies the slim revision did not regress the upstream "closest person"
contract. No special setup; just one CLI call.

```bash
ros2 action send_goal --feedback /follow_head_action \
  tinker_vision_msgs_26/action/FollowHeadAction \
  "{start_following: true}"
```

**Expected**
- First feedback within 3 s; `person_visible: true` once a person enters
  the frame; `error_deg` decreases as the head turns.
- Head locks onto whoever is closest in pan-tilt-root XY.
- vision_log JSON entries show `fresh_lock_mode: "closest"`.
- Action result `success: true` when you Ctrl+C the client and the
  server sees the cancel.

**Pass criteria**
1. **Single-person scene** — head turns to the person and holds.
2. **Two-person scene** (2nd person enters closer mid-track) — head
   may snap to the now-closer person on the next fresh lock; this is
   legacy behavior. Sticky reassoc within `reassoc_dist_m` (0.4 m) holds
   the original lock if the bystander stays > 0.4 m from it.
3. **Empty scene** — feedback streams `person_visible: false`,
   `[follow_head perf]` shows `no_person` ticks; head holds last pose.

Stop with **Ctrl+C** on the action-client terminal.

---

## Layer 2 — track_centermost (new feature)

Verifies the centermost cost picks the image-centered person when a
closer-but-off-center bystander is also in view.

**Setup**
- Person A ~2.0 m in front of the robot, on the robot's center axis
  (the "designated" person).
- Person B ~1.0 m from the robot, off to one side (~30° pan offset),
  so they are off-center in the RGB image even though they are closer
  in 3D.
- Pre-aim the head to (0°, 30°) so person A is roughly image-centered:
  ```bash
  ros2 topic pub --once /pan_tilt_controller/cmd \
    tinker_vision_msgs_26/msg/PanTiltCommand \
    "{header: {stamp: {sec: 0, nanosec: 0}}, mode: 0, \
      pan_rad: 0.0, tilt_rad: 0.5235, speed_raw: 0, accel_raw: 0}"
  ```

**Command**
```bash
ros2 action send_goal --feedback /follow_head_action \
  tinker_vision_msgs_26/action/FollowHeadAction \
  "{start_following: true, track_centermost: true}"
```

**Expected**
- Head locks onto person A (centered) on the first detection tick, even
  though person B has a smaller XY norm.
- vision_log JSON entries show `fresh_lock_mode: "centermost"`.
- Once locked, sticky reassoc keeps the lock on A as A moves slightly.
  If A leaves the frame entirely for > `target_ttl_sec` (0.8 s), the
  next fresh lock again picks whoever is most centered.

**Pass criteria**
1. Head ends up oriented at A, not B, immediately after goal-accept.
2. Walking B toward the robot (still off to the side) does NOT steal
   the lock as long as A remains in frame.
3. Cancel → re-send the goal with B newly more-centered (e.g. A walked
   off): head locks onto B on the next fresh goal.

---

## Layer 3 — full BT seeded follow

Closed-loop test: detect a single person via
`/object_detection_generalist`, ship that centroid as `target_seed_xyz`,
then verify the gaze stays on the originally-detected person under
bystander interference.

**Setup**
Empty arena. One "guest" person, one "bystander" off to the side.

**Command**
```bash
ros2 run behavior_tree test-follow-head-fresh-lock
```

**Operator script** (driven by audio cues from the BT):

1. **Pre-detect cue** — robot says: *"Hello. Please stand alone in
   front of me, about one and a half meters away, body facing the
   robot. I will detect you when this announcement finishes."*
   → **Guest only** walks to ~1.5 m in front of the robot, head-on.
   → Bystander stays out of the camera's FoV for now.

2. **Detection happens** automatically when the announcement finishes
   (the BT calls `/object_detection_generalist` with `prompt="person"`,
   `target_frame="base_link"`, `sort_closest=true`). The detection node
   logs `🎯 Stashed person centroid xyz=(...)` to stdout.

3. **Post-detect cue (long, ~5 s of TTS)** — robot says:
   *"Person detected. Centroid stored to blackboard. Will be the target
   of maintain eye contact. Initiating maintain eye contact."*
   → **During this announcement** the bystander walks into frame, off
     to one side, and gets within ~1.0 m of the robot (closer than the
     guest's ~1.5 m). Bystander stays for ~10–15 s, then leaves.
   → Guest stays roughly put (lateral movement < 0.5 m is fine).

4. **Seeded follow** runs for `follow_timeout=30 s`. Watch the head.

5. **Final cue** — robot says: *"Follow head test complete."* and the
   BT exits.

Stop early with Ctrl+C if you need to.

**Expected**
- Detection succeeds within 3 retries. If it fails 3× the BT exits with
  the wrapping `Retry` returning FAILURE — re-run after checking the
  generalist server is up and a person is visible.
- The seeded `MaintainEyeContact` ships the goal with non-zero
  `target_seed_xyz` and `seed_radius_m=0.6`. follow_head logs
  `Fresh-lock mode: seed (xy=(<x>, <y>), radius=0.60 m).`
- vision_log entries show `fresh_lock_mode: "seed"`. `chosen_root_xyz`
  matches the centroid printed by the detection node, within depth /
  pose-keypoint slop.
- Head locks onto guest at goal start. When bystander walks in closer,
  head **does not** snap to bystander.

**Pass criteria**
1. Detection succeeds within 3 retries (one usually suffices).
2. Head turns to the guest at goal start.
3. Head ignores the bystander walking in closer but laterally-offset,
   provided the bystander stays > `seed_radius_m` (0.6 m) from where
   the guest was detected. (Spatial sticky path will reassoc onto the
   bystander if they cross the locked XYZ within `reassoc_dist_m` — a
   known limitation of pure-spatial sticky.)
4. After ~30 s the BT finishes cleanly and announces completion.
5. No abort on the 3 s first-feedback rule.

**Mock smoke (no robot)**
```bash
BT_MOCK_MODE=true ros2 run behavior_tree test-follow-head-fresh-lock
```
Press `s`+Enter when each mocked node prompts (`BtNode_Announce`,
`BtNode_DetectAndStashPersonCentroid`, `BtNode_MaintainEyeContact`,
`BtNode_TurnPanTilt`). The detect node synthesizes a centroid of
`(1.5, 0.0, 1.3)` so the seeded path is exercised under mocks. Tree
should reach the final "Follow head test complete." announce.

---

## Common troubleshooting

- **No detections at all (Layer 3)** — generalist server down or wrong
  camera topic. Check `ros2 service list | grep object_detection_generalist`
  and `ros2 topic hz /camera/color/image_raw`.
- **Head jitter** — turn down `ema_alpha` and tighten `steady_*_eps_deg`
  in `pan_tilt.yaml`. Defaults bias responsiveness over smoothness
  (per `tk26_vision/CLAUDE.md`).
- **Head doesn't move** — read the `[follow_head perf …]` stderr line.
  - `min_cmd_change`/`deadband` dominant → the target IS where the head
    already points; not a bug.
  - `tracker_no_lock` dominant → seed mode rejected all candidates
    (none within `seed_radius_m`); reposition or widen radius.
  - `no_person` dominant → YOLO sees zero people; check lighting, depth
    validity, FoV.
- **Layer 3 never speaks** — `BtNode_Announce` requires the announce
  service. Without it on real hardware the announce fails and the
  Sequence aborts before MaintainEyeContact. Check
  `ros2 service list | grep announce`. Mock smoke (`BT_MOCK_MODE=true`)
  bypasses the service.
- **First-feedback timeout** — Camera dropped below ~10 Hz; see
  `CAMERA_BRINGUP.md`. The BT will FAIL the gaze leaf with
  "feedback timeout".
- **Layer 3 detection picks the wrong person** — the generalist's
  `sort_closest=True` returns the closest YOLO person. If two people
  are similar distance, results jitter. Position the guest as the
  uniquely-closest person at detect time, or stand alone for the
  detect phase only.

## Related files

- BT script: `behavior_tree/HRI/test_follow_head_fresh_lock.py`
- Action: `tinker_vision_msgs_26/action/FollowHeadAction.action`
- Server: `pan_tilt/follow_head.py` + `pan_tilt/head_tracking_helpers.py`
- BT template: `behavior_tree/TemplateNodes/Vision.py::BtNode_MaintainEyeContact`
- Existing isolation harness for the parallel-cancel race: `test_eye_contact.py`
