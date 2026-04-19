# HRI Rulebook Notes and BT Plan

Updated: 2026-03-10
Primary code reference: `HRI/hri.py`
Rulebook file: `HRI/Rulebook-HRI.pdf`

## Rulebook Extraction Notes

- The file now contains the correct HRI section (RoboCup@Home 2026, 5.1).
- Text extraction via `pdftotext` was successful.

## Requirements Summary (Rulebook 5.1)

- Main goal:
  - Welcome and assist two arriving guests.
  - Offer seats.
  - Keep appropriate gaze behavior in conversation/navigation.
  - Handle second guest bag handover, follow host, and drop bag at instructed location.
- Optional goals:
  1. Detect doorbell sound.
  2. Open entrance door for each guest.
  3. Visually describe first guest to second guest.
  4. Understand name/drink without non-essential confirmation questions.
- Key constraints:
  - Robot must approach guests at the door (do not ask them to move in front).
  - Dynamic gaze tracking at speaking person.
  - Look toward navigation direction while moving.
  - Introductions must be two-way (for both guests, name + favorite drink).
  - Seat assignment must be explicit/pointed.
  - Detailed guiding penalties exist in follow-host stage.
- Time limit: 6:00.
- Score highlights:
  - Main: seat offer, gaze, introduction correctness, bag handover/follow/drop.
  - Bonus: doorbell detection, door opening, visual attributes, no non-essential questions.
  - Penalties: wrong memorized info, alternative HRI, guiding assistance, guest-identification failures.

## Broad Implementation Plan (Rulebook-First)

1. Wait state at start position for next-guest trigger (doorbell/knock/timeout fallback).
2. Guest 1 interaction:
   - approach door
   - acquire name/drink with confidence handling
   - escort and seat assignment
3. Guest 2 interaction:
   - same intake + escort + seat
4. Introduction phase:
   - two-way introductions with explicit gaze target control
   - optional visual description branch
5. Gaze supervisor (cross-cutting):
   - speaking-person tracking
   - navigation-direction tracking
6. Bag phase:
   - safe handover
   - host follow recovery policy
   - instructed drop confirmation
7. Completion reporting with scored-action trace.

## Refined Plan Against Current Code

Current strengths (`receptionist_new.py`):
- End-to-end two-guest flow exists.
- Name/drink capture pipeline exists, with confirmations and retries.
- Seat recommendation and introductions are implemented.
- Bag handover/follow/drop branch exists (`createGraspAndCarryBag`).

Primary gaps vs 2026 scoring:
- Gaze scoring risk: `DISABLE_FOLLOW_HEAD=True` and `DISABLE_FEATURE_MATCH=True`.
- Doorbell detection not implemented as a primary state.
- Door opening bonus path not explicit.
- Current loops are confirmation-heavy; this conflicts with no-non-essential-question bonus.
- Guest-at-door approach semantics should be audited against rule wording.

Implementation priorities:
1. Re-enable and stabilize gaze-critical tracking branches.
2. Add explicit doorbell/arrival trigger handling state.
3. Add optional door-opening subtree.
4. Add configurable interaction policy:
   - conservative mode (confirmations)
   - bonus mode (minimal confirmation)
5. Add follow-host recovery policy with penalty-aware thresholds.
6. Add mock acceptance tests for all main-goal checkpoints.

## Implemented Mock/Fallback Nodes (Current `HRI/hri.py`)

- `BtNode_MockArrivalTrigger`:
  - Used as fallback when `BtNode_DoorDetection` path fails.
  - TODO in code: replace with reliable doorbell/knock event trigger.
- `BtNode_MockSafetyCheck` ("Safe handover detector TODO"):
  - Placeholder after gripper open for bag handover.
  - TODO in code: replace with actual handover completion detector.
- `BtNode_MockSafetyCheck` ("Drop confirmation detector TODO"):
  - Placeholder after bag drop open-gripper action.
  - TODO in code: replace with actual bag-drop confirmation detector.
- Follow-host subtree:
  - Implemented in `HRI/follow.py:createFollowPerson()`.
  - Wraps `tinker_vision_msgs_26/TrackPerson` (vision) + `tinker_nav_msgs/Follow`
    (tk26 `tracking_server`, Nav2-driven) in a lost-sight-tolerant tree:
    two-stage recovery (short-audio "please wait" → long-audio "please come
    back" + ReID reanchor via outer `Retry`).
  - All thresholds/audio strings live under the `follow` block of
    `Receptionist/constants.json` (loaded by `HRI/config.py:FOLLOW_CONFIG`).
  - Standalone harness: `ros2 run behavior_tree hri-follow`.
- Gaze supervisor fallback behavior:
  - `BtNode_HeadTrackingAction` and navigation-direction pan/tilt are wrapped with `FailureIsSuccess` fallback.
  - This prevents hard failure but can hide degraded gaze performance.

## Missing Features vs Rulebook (Current State)

- No explicit door-opening action branch for bonus scoring.
- Arrival trigger is not yet true doorbell/knock detection (fallback exists).
- Bag handover/drop confirmation are placeholders, not verified by perception.
- Follow-host stage now uses the real action pair (`TrackPerson` + `Follow`)
  with two-stage loss recovery; previously a navigation proxy.
- Intake is now hybrid: primary path via action-based `phrase_extraction_action`
  skips confirmation when Whisper+Qwen cross-check agrees (banks the 4×15
  no-non-essential-questions bonus); only the last-resort fallback reverts
  to the legacy confirm loop. See `HRI/hri.py:_create_get_info` and the
  standalone `hri-intake` harness in `HRI/intake.py`.

## Potential Improvements (Prioritized)

1. Replace arrival fallback with an audio/event-based doorbell node and keep door-vision as secondary fallback.
2. Add explicit door-opening subtree with retry + safety checks.
3. Integrate real handover/dropped-bag verification nodes and remove `BtNode_MockSafetyCheck`.
4. *(done)* Follow-host uses a real action pair with two-stage lost-target
   recovery — see `HRI/follow.py`. Remaining: tune thresholds on-robot and
   wire up map-frame TF so the transformed `target_position` stream to Nav2
   is continuous (server currently suppresses publication when TF fails).
5. *(done — 2026-04-19)* Skip-confirmation-when-high-confidence intake.
   `HRI/hri.py:_create_get_info` now uses `BtNode_PhraseExtractionAction`
   (cross-check → `STATUS_SUCCEEDED` = status=0) inside a `Selector(Retry(2),
   fallback)` so confirmation only fires on the last-resort branch.
6. Add task-level timing checkpoints for 6:00 budget and phase preemption.
