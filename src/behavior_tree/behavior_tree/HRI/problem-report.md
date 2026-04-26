# HRI Pipeline — Rulebook Compliance Problem Report

- Audited: 2026-04-19
- Scope: `HRI/hri.py`, `HRI/follow.py`, `HRI/config.py` (and the template nodes they invoke)
- Rulebook: `HRI/Rulebook-HRI.pdf` (RoboCup@Home 2026, 5.1, revision 2025-12-10)
- Max score: **1450** in 6:00 minutes

This is a findings document — no code has been changed. See the "Prioritized fixes" section for the order of work once triage is agreed.

---

## Summary table

| # | Severity | Area | Rulebook ref | Score impact |
|---|---|---|---|---|
| P1 | 🚨 Critical | Follow-person — "please wait" audio | Penalties-Guiding: *Asking operator to wait* | **−50 (auto)** |
| P2 | 🚨 Critical | Follow-person — "please come back" audio | Penalties-Guiding: *Rediscovering by natural interaction* + 5.3 | **−50 (auto)** |
| P3 | 🚨 Critical | No door-opening subtree | Bonus: *Open the entrance door* | **−400 (2×200)** |
| P4 | 🚨 Critical | No doorbell-sound detection | Bonus: *Detect the doorbell sound* | **−60 (2×30)** |
| P5 | 🟠 Major | No visual description of guest 1 to guest 2 *before* living room | Bonus: *Describe the first guest visually* | **−80 (4×20)** |
| P6 | 🟠 Major | Confirmation loop fires non-essential "correct?" prompts | Bonus: *Not asking non-essential questions* | **−60 (4×15)** |
| P7 | 🟠 Major | No gaze supervisor wrapping guest intake | Main: *Look at person talking, when receiving* | **−100 (2×50)** |
| P8 | 🟠 Major | `MaintainEyeContact` is a one-shot lock inside parallel | Main: *continuously tracks the moving person* | part of P7/P9 |
| P9 | 🟠 Major | Introduction gaze targets closest face, not `target_id` guest | Main: *Look to correct guest while talking about the other* | **−100 (2×50)** |
| P10 | 🟠 Major | Navigation-direction gaze is a fixed `y=35°` tilt | Main: *Look in direction of navigation* | **up to −30 (2×15)** |
| P11 | 🟠 Major | Follow-host drop has no voice trigger from host | Main: *Drop bag on the correct area* | **−50** |
| P12 | 🟠 Major | Bag handover: no 10 cm hand-proximity, fixed 3 s timer | Handover scoring criteria | **−50** handover + drop-in-transit risk |
| P13 | 🟢 Minor | Seat indication is head-turn only, no arm pointing | Main: *clearly indicate or point* | fragile, likely partial |
| P14 | 🟢 Minor | Arrival trigger fires after arm init, not at *starting position* | Doorbell requirement: "waiting at the starting position" | prerequisite for P4 |
| P15 | 🟢 Minor | Two gaze controllers (action + pan-tilt) race on `/pan_tilt_ctrl` | N/A (reliability bug) | observed flakiness |
| P16 | 🟢 Minor | 6:00 time budget is tight under confirmation retries | Test ends at 6:00 | risk of not-attending −500 if over |

**Estimated ceiling with current code:** ≈ 700 / 1450 (assuming happy-path execution, no P11 drop failure).

---

## Critical — self-inflicted penalties

### P1. Short-loss announcement violates "Asking operator to wait"

**Where:** `HRI/config.py:146` and `HRI/constants.json:82` (default / loaded value)
```python
FOLLOW_CONFIG.setdefault("announce_short", "Please stop and wait for Tinker to catch up.")
```

**Rulebook (p.38, Penalties — Guiding):**
> Asking operator to wait –50

**Trigger path:** in `follow.py:createFollowPerson`, `short_guard` fires when `follow_lost_elapsed_sec ≥ lost_short_sec` (5.0 s by default) and the dedup flag is clear. The audio plays via `BtNode_Announce`.

**Why this is a problem:** the string is a word-for-word match for the penalty line item. On any live loss ≥ 5 s (routine in a crowded arena) we auto-forfeit 50 points.

**Recommended fix:** convert short-loss recovery to silent re-acquisition — drop the `BtNode_Announce` node from `short_guard`, or set `announce_short` to `""` and make the node skip on empty. The ReID restart path should run without verbal cues to the host.

---

### P2. Long-loss announcement violates "Rediscovering by natural interaction" + 5.3

**Where:** `HRI/config.py:149` and `HRI/constants.json:83`
```python
FOLLOW_CONFIG.setdefault("announce_long", "I've lost you. Please come back in front of Tinker.")
```

**Rulebook (p.37, Penalties — Follow the host):**
> 5.1. Natural interaction (e.g., waving and calling)
> 5.3. Returning in front of the robot to resume following.
>
> Penalties-Guiding: *Rediscovering the operator by natural interaction –50*

**Trigger path:** `long_sentinel` in `follow.py` fires at `lost_long_sec` (15.0 s default), emits the announce, then returns FAILURE to bounce out of the outer `Retry(n_restart)` and re-anchor ReID.

**Why this is a problem:** the string performs *both* penalty categories at once — verbal recall (natural interaction) and an explicit instruction to return in front of the robot (5.3).

**Recommended fix:** remove the announcement entirely. Drive the restart silently. If the ReID reanchor needs a camera view of the host, the BT should *turn the robot* (pan tilt / small rotation) rather than asking the human to reposition.

---

### P3. No door-opening subtree (400-point bonus left on the table)

**Where:** there is no tree branch that actuates door opening.

**Rulebook (p.38, Bonus Rewards):**
> Open the entrance door for a guest — 2×200

**What exists instead:** `createArrivalTrigger` in `hri.py:150` calls `BtNode_DoorDetection` (`Vision.py:786`) which *reads* open/closed state via the orbbec camera. It does not drive the arm or base to open a door.

**Recommended fix:** add an optional door-open subtree `createOpenDoor()` gated by `BtNode_DoorDetection` returning "closed". Requires new manipulation + navigation primitives (approach handle, grasp, pull/push, retract). This is the single largest score upside in the test.

---

### P4. No doorbell-sound detection (60-point bonus missing)

**Where:** `HRI/hri.py:150`, `createArrivalTrigger`.

**Rulebook (p.36, Doorbell):**
> A doorbell sound will be played when a guest is by the door ready to be met. […] In order for the doorbell sound to be considered detected, the robot must be waiting at the starting position and react instantly to the sound.

**What exists instead:** a `Selector` that tries vision-based open-door detection first, then falls through to `BtNode_MockArrivalTrigger` (no-op SUCCESS). Neither listens for audio.

**Recommended fix:** add a doorbell-audio node (likely `tk_24_audio` side). Wrap it as a primary `Selector` branch, with `BtNode_DoorDetection` remaining as a secondary vision fallback. Also addresses P14 (starting-position requirement).

---

## Major — main-goal scoring bugs

### P5. Guest-1 visual description not announced during escort of guest 2

**Where:** `HRI/hri.py:260`, `createEscortAndSeat(guest_idx=2)`.

**Rulebook (p.36, Optional goals / Bonus):**
> Visually describe the first guest to the second guest **before reaching the living room**. (4×20)

**Current flow:** features are captured via `BtNode_FeatureExtraction` at the door (`hri.py:225`). They're consumed later by `BtNode_Introduce(describe_introduced=True)` during the seated two-way intro (`hri.py:302`). There is no describe-while-walking step.

**Recommended fix:** between "Please follow me to your seat" and the sofa goto (inside `createEscortAndSeat(2)`), insert a `BtNode_Announce(bb_source=KEY_GUEST1_FEATURES)` so the features string is spoken while navigating, before arrival at the sofa.

---

### P6. Confirmation loop fires non-essential "correct?" prompts

**Where:** `HRI/hri.py:173`, `_create_get_info` → `BtNode_Confirm` (`Receptionist/customNodes.py:142`).

The confirm node synthesizes: `"Your {type} is {value}, correct?"` — then a follow-up `BtNode_GetConfirmation` waits for yes/no.

**Rulebook (p.36, Non-Essential Questions):**
> Questions such as "Did you say your name is James?" or "Is your favorite drink milk?" are considered non-essential questions. However, asking a guest to repeat an answer because the robot could not hear is not considered a non-essential question.

Our "Your name is X, correct?" is semantically identical to the example.

**Recommended fix:** replace the confirmation leaf with a confidence-gated retry: if `BtNode_PhraseExtraction` low-confidence (or fails), say *"I'm sorry, could you repeat that?"* (which is rulebook-legal) and retry. Do not ask for yes/no confirmation.

---

### P7. No gaze supervisor during guest intake

**Where:** `HRI/hri.py:204`, `createGuestIntake`. Compare with `createEscortAndSeat` and `createTwoWayIntroduction`, which *are* wrapped by `_with_gaze_supervisor`.

**Rulebook (p.36, Looking at person):**
> The robot must look at the person it is talking to. […] Points for gaze tracking are awarded only if the robot continuously tracks the moving person.

The prompts "please tell me your name" / "please tell me your favorite drink" happen inside `createGuestIntake`, unwrapped. The robot's head is not commanded toward the guest's face during the canonical "speaker gaze" moment.

**Recommended fix:** wrap the `get_info` phases of `createGuestIntake` in `_with_gaze_supervisor` (after first addressing P8 so the wrapper is actually continuous).

---

### P8. `BtNode_MaintainEyeContact` is a one-shot lock inside a parallel

**Where:** `HRI/hri.py:238`, `_with_gaze_supervisor`. Uses `BtNode_HeadTrackingAction` (deprecated alias for `BtNode_MaintainEyeContact`, `Vision.py:1133`).

**Node docstring:**
> Server returns success after a single gaze lock.

**Why this is a problem:** py_trees parallels do not re-tick children that have terminated. Once the eye-contact leaf returns SUCCESS (one lock), it freezes for the remainder of the parallel — gaze does *not* keep updating as the guest paces. Rulebook calls for *continuous* tracking.

**Recommended fix:** wrap the eye-contact leaf in `py_trees.decorators.Repeat(num_success=-1, …)` (or `FailureIsRunning + repeat`) so successive locks are re-issued for the lifetime of the main child.

---

### P9. Introduction gaze targets closest face, not the rulebook-specified guest

**Where:** `HRI/hri.py:299`, `createTwoWayIntroduction`.

**Rulebook (p.36, Introductions):**
> The robot must be looking at a guest and state the **other** arriving guest's name and favorite drink.

**Current flow:** `BtNode_Introduce(target_id=1, introduced_id=0)` announces guest 0's name + drink while (notionally) addressing guest 1. The gaze wrapper uses `BtNode_MaintainEyeContact` which locks onto the *closest face* regardless of ID. The tree has no way to map `target_id` → head-turn-direction for the correct guest.

**Recommended fix:** add a targeted-look primitive. Two options:

1. Coarse: `BtNode_TurnTo` (`Vision.py:937`) using `KEY_PERSON_CENTROIDS` from `BtNode_FeatureMatching`, parameterized by `target_id`. Same pattern receptionist_2ndcall uses.
2. Fine: extend `FollowHeadAction` server to accept a "target_track_id" parameter so the head tracks a specified person, not just the closest face.

Option 1 works with what's on hand today.

---

### P10. Navigation-direction gaze is a fixed `y=35°` tilt

**Where:** `HRI/hri.py:251`, inside `_with_gaze_supervisor`:
```python
BtNode_TurnPanTilt(name="Look to navigation direction", x=0.0, y=35.0, speed=0.0)
```

**Rulebook (p.36, Looking at direction of navigation):**
> The robot must look in the direction it is navigating. Persistently looking in an unrelated direction or at unrelated people during navigation results in a score deduction.

**Problem:** `x=0.0, y=35.0` is head-forward pitched-up. It doesn't vary with nav direction. If the robot turns 90° during navigation, the head keeps pitching forward-and-up relative to the base, which is forward along the base's x-axis — OK by coincidence, but the intent ("toward the navigation goal") is not expressed.

This is also the node that races with `BtNode_MaintainEyeContact` on the `/pan_tilt_ctrl` topic — see P15.

**Recommended fix:** compute the pan angle from the current map→goal vector each tick, or have navigation publish a "desired look direction" topic the head controller subscribes to. Minimum acceptable behavior: pan toward the goal once at the start of each navigation leg.

---

### P11. Follow-host drop has no voice trigger

**Where:** `HRI/hri.py:353`, `createBagFlow`. Sequence after `createFollowPerson`:

```
FollowPerson →
MoveArmSingle(arm_pos_drop) →
GripperAction(open) →
BtNode_MockSafetyCheck("Drop confirmation detector TODO") →
GripperAction(close)
```

**Rulebook (p.36, Follow the host):**
> Once at the correct location, the host informs the robot that it should place the bag on the floor.

Our drop fires unconditionally when `tracking_server` returns REACHED, whatever that heuristic is. The host's drop instruction is not ingested.

**Recommended fix:** between follow-completion and arm-to-drop, insert a `BtNode_ListenAction` gated on a short wordlist (e.g. "drop", "here", "place it"). Only proceed to the drop motion on a positive match, otherwise retry listening.

---

### P12. Bag handover quality risks

**Where:** `HRI/hri.py:321`, `createBagFlow`.

```
MoveArmSingle(arm_pos_handover) →  # fixed pose, no approach-to-hand
GripperAction(open) →
Announce("Please place your bag in my gripper.") →
Timer(3.0) →                       # unconditional 3 s wait
GripperAction(close)
```

**Rulebook scoring criteria (p.37, Bag handover):**
- "The manipulator must be positioned at least within 10 cm of the operator's hand." → we don't measure hand position.
- "The robot should not reach, grasp, or initiate dangerous movement toward the human." → fine, we don't reach.
- The 3 s timer is too short; if the guest hasn't placed by then, gripper closes on empty or on fingers.
- No perception verification of a successful grasp. `BtNode_MockSafetyCheck` at drop time is unrelated.

Additional failure mode: if the bag closure is insufficient, `Penalties-Guiding: Drop the bag while following the host –50` triggers silently during the follow stage.

**Recommended fix (minimum):**
1. Extend the timer or key off a hand/object perception signal.
2. Add a gripper-force or object-in-gripper check; retry the handover cycle if empty.
3. Consider wiring `BtNode_Grasp` in "handover" mode if a vision-guided approach to the guest's hand is practical — or at minimum, approach the guest's body by 30–50 cm before presenting the gripper.

---

## Minor / ambiguous

### P13. Seat indication is head-turn only; no arm pointing

**Where:** `HRI/hri.py:277`, `createEscortAndSeat`.
```
BtNode_SeatRecommend → BtNode_TurnPanTilt(x=45°, y=25°) → Announce("Please sit here.")
```

`BtNode_SeatRecommend` returns a text recommendation (e.g. "sit on the left"). The pan-tilt is a fixed 45°/25° regardless of which seat was recommended. Arm pointing (`BtNode_PointTo` in `Manipulation.py:635`) is not used.

**Rulebook (p.36, Seating People):**
> The robot must clearly indicate or point to a location or seat for each guest.

Minimum is probably satisfied by head-turn + verbal. Arm pointing would make this robust and match what receptionist_2ndcall already does during introductions.

---

### P14. Arrival trigger fires after arm initialization — not at "starting position"

**Where:** `HRI/hri.py:372`, `createHRITask`.

Sequence:
```
createConstantWriter → Announce → MoveArm(navigating) → createArrivalTrigger → ...
```

The arm-to-navigation move happens *before* the arrival trigger. Rulebook doorbell detection requires the robot to be "waiting at the starting position and react instantly to the sound."

This is a prerequisite for P4. When doorbell detection is wired, ensure nothing moves the base or arm before the trigger.

---

### P15. Two gaze controllers race on `/pan_tilt_ctrl`

**Where:** `HRI/hri.py:238`, `_with_gaze_supervisor`.

Inside the parallel: `BtNode_HeadTrackingAction` (eye-contact action) and `BtNode_TurnPanTilt(y=35°)` tick simultaneously. Both ultimately write to the pan-tilt topic — the result is last-writer-wins at the ROS topic level. Visible as flaky/jerky head behavior.

**Recommended fix:** once P10 is redesigned, the pan-tilt node should become a function of the active mode (speaker-gaze vs nav-gaze) rather than a separate always-on leaf.

---

### P16. 6:00 time budget is tight under confirmation retries

**Where:** `HRI/hri.py:173`, `_create_get_info`. `num_failures=5` × (announce prompt + extract 7 s + confirm + get_confirmation 5 s).

**Happy path:** ~27 s per field × 4 fields (2 guests × name+drink) = ~110 s intake.
**Worst case:** up to ~135 s per field = 540 s just in intake — exceeds the 6:00 budget before any navigation or handover.

**Recommended fix:** correlates with P6 — removing the confirm step shortens the loop and dodges the non-essential-question penalty at the same time. If the reduced flow still retries too long, lower `num_failures` from 5 to 3.

---

## Prioritized fixes

Order is chosen for best effort/reward ratio. Each step is a small, self-contained change.

1. **P1 + P2 (≈5 min each):** remove the two follow announcements. Stops −100 auto-bleed on any live loss event.
2. **P8 (≈10 min):** wrap `MaintainEyeContact` in `Repeat(num_success=-1, …)`. Unlocks all other gaze fixes.
3. **P7 (≈5 min):** wrap `createGuestIntake` with `_with_gaze_supervisor`. Recovers up to 100 pts.
4. **P9 (≈30 min):** add a targeted-look primitive (prefer `BtNode_TurnTo` + `BtNode_FeatureMatching`) and use it in `createTwoWayIntroduction`. Recovers up to 100 pts.
5. **P6 + P16 (≈15 min):** swap `BtNode_Confirm`/`BtNode_GetConfirmation` for a "repeat please" retry. Recovers 60 bonus and ~80 s of budget.
6. **P5 (≈5 min):** add a describe-guest-1 announce in `createEscortAndSeat(2)`. Recovers 80 bonus.
7. **P11 (≈15 min):** gate drop on `BtNode_ListenAction`. Recovers up to 50 pts and protects against wrong-location drops.
8. **P12 (≈30 min):** replace fixed timer with a perception/force check; approach the guest's body before presenting the gripper.
9. **P10 + P15 (≈45 min):** unify gaze controllers and make the nav-direction leaf actually follow the goal bearing.
10. **P13 (optional, ≈30 min):** add `BtNode_PointTo` to seat recommendation.
11. **P4 + P14 (needs audio work):** add doorbell audio detector and ensure the robot is stationary at start when it fires. 60 bonus.
12. **P3 (large, hardware-dependent):** door-opening subtree. 400 bonus — largest upside but also the biggest integration cost.

---

## Not covered by this report

- Hardware readiness of `tracking_server` + `track_person` integration under real ReID noise.
- Map-frame TF setup on the competition field (without which `target_point_topic` never publishes — follow stalls silently).
- Multi-person interference on ReID (passive guests in the arena).
- OC-announced starting position / bag identity — these are test-day parameters, not code issues.
