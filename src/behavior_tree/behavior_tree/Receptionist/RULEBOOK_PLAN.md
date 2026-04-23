# Receptionist Rulebook Notes and BT Plan

Updated: 2026-03-10
Primary code: `receptionist_new.py`
Rulebook PDF: `Rulebook-Receptionist.pdf`

## Rulebook Extraction Notes

- The PDF is an image-only scan; direct text extraction is empty.
- OCR was used to recover text for this summary.
- Rule source identified as RoboCup@Home 2025, Chapter 5.3.

## Rulebook Requirements Summary

- Main goal:
  - Welcome two guests, guide each through beverage area to living room, offer seats, and perform introductions.
- Optional goals:
  - Open entrance door for each guest.
  - Describe guest 1 visually to guest 2 before living room arrival.
  - State an interest similarity among people.
- Setup essentials:
  - Host profile (name/drink/interest) is pre-announced.
  - Two guests arrive separately, each with name/drink/interest.
  - Beverage table is available.
  - Passive guests may appear in background.
- Constraints:
  - During conversation, continuously look at the speaking person.
  - During navigation, look in direction of movement/goal.
  - Report drink position (left/center/right is sufficient).
  - Point to seat/location when offering seating.
  - Continuing with wrong memorized name/drink/interest is penalized.
- Max time: 6:00.

## Broad Implementation Plan (Rulebook-First)

1. Initialize host profile and world anchors.
2. Guest 1 flow:
   - greet + acquire/confirm name
   - acquire/confirm drink
   - acquire/confirm one interest
   - guide to beverage area, report drink location
   - escort to living room and offer seat
3. Guest 2 flow (same pattern).
4. Introduction/social phase:
   - introduce both guests to each other using name + drink + interest
   - optional visual attribute description guest1 -> guest2
   - optional shared-interest statement
5. Gaze policy supervisor:
   - conversation mode: face active speaker
   - navigation mode: face movement/goal direction
6. Optional entrance-door opening subtree per guest.
7. Task timeout handling and final completion report.

## Refined Plan Based on Current Code

Current strengths in `receptionist_new.py`:
- Full two-guest sequential flow.
- Name/drink/interest confirmation loops.
- Host/guest feature registration and person-list maintenance.
- Beverage + living-room navigation.
- Seat recommendation and guest-to-guest introductions.
- Common-interest comparison branch.
- Extra bag carry/drop branch after second guest flow.

Observed gaps to close:
- Gaze-critical paths are effectively disabled:
  - `DISABLE_FEATURE_MATCH = True`
  - `DISABLE_FOLLOW_HEAD = True`
- Drink-position reporting is constant-based rather than runtime perception.
- No explicit autonomous entrance-door opening branch.
- No explicit 6:00 phase/time budget controller.
- Wrong-memory mitigation exists, but not tracked as a quality gate.

Refined implementation steps:
1. Re-enable/stabilize head-tracking + feature-matching for gaze scoring.
2. Replace hardcoded drink broadcasts with runtime beverage localization output.
3. Add door-opening subtree with fallback policy.
4. Add confidence gate and controlled re-ask policy for guest slots.
5. Add task timer + cutover behavior to secure main-goal points.
6. Keep bag-carry extension behind a runtime feature flag (non-core rulebook behavior).
