# PickAndPlace Rulebook Notes and BT Plan

Updated: 2026-03-10
Primary code references: `TemplateNodes/Manipulation.py`, `grasp_intel_demo/*`
Rulebook file: `PickAndPlace/Rulebook-PickAndPlace.pdf`

## Rulebook Extraction Notes

- The file now contains the correct Pick and Place section (RoboCup@Home 2026, 5.2).
- Text extraction via `pdftotext` was successful.

## Requirements Summary (Rulebook 5.2)

- Main goals:
  1. Tidy all dining-table objects:
     - put dirty tableware/cutlery in dishwasher
     - place trash in trash bin
     - place remaining objects in cabinet grouped by similarity/category
  2. Set breakfast on clean table area: bowl + spoon + cereal + milk.
- Optional goals:
  1. Pick trash from floor.
  2. Open/close dishwasher door.
  3. Pull/push dishwasher rack.
  4. Place dishwasher tablet in slot.
  5. Pour milk/cereal into bowl.
- Key constraints:
  - Sequence is free (no fixed execution order).
  - Safe placing required (place, do not throw/drop).
  - Dishwasher may require assistance; robot must explicitly request help on failure.
  - Breakfast placement must be typical and uncluttered.
  - Shelf grouping must follow similarity/category semantics.
  - Robot must clearly communicate perception to referee.
- Time limit: 7:00.
- Score highlights:
  - Strong manipulation weighting: 12 pick + 12 place events.
  - Large bonuses for door/rack/tablet and pouring.
  - Penalties for drops, clutter, pouring spills, and human assistance.

## Broad Implementation Plan (Rulebook-First)

1. Enter kitchen and initialize task context.
2. Perceive and classify table objects by destination class:
   - dishwasher set (cutlery/tableware/tablet)
   - trash set (table + floor trash)
   - cabinet set (other objects)
3. Execute cleanup loop with destination-specific subtrees:
   - pick -> transport -> place -> verify
4. Execute breakfast assembly:
   - place bowl/spoon
   - retrieve milk/cereal
   - arrange typical meal layout with clear surrounding area
5. Optional manipulation branch:
   - door/rack operations
   - tablet insertion
   - pouring routines
6. Human-assistance fallback policy:
   - request only when needed
   - keep assistance events explicit and countable
7. Timeout-aware completion policy to maximize score before 7:00.

## Refined Plan Against Current Code

Current capability baseline:
- Reusable manipulation actions exist (`BtNode_Grasp`, `BtNode_Place`, `BtNode_Drop`, gripper/arm motion).
- Grasp demos (`grasp_intel_demo`) show partial pick pipeline.

Current implementation gap:
- `PickAndPlace/` has no task BT yet (only rulebook + placeholder files).

Implementation priorities:
1. Add `PickAndPlace/pick_and_place.py` with staged subtrees:
   - table cleanup
   - cabinet grouping
   - breakfast setup
2. Add blackboard model for object inventory and destination class.
3. Add dishwasher operation nodes (door/rack/tablet) and fallback handling.
4. Add pouring subtrees with spill-aware checks.
5. Add verification nodes:
   - correct destination
   - cabinet similarity placement
   - breakfast spacing/arrangement sanity
6. Add mock acceptance tests for:
   - full main goals
   - optional-goal branches
   - timeout cutover behavior.

## Confidence and Provenance

- High confidence: requirements derived directly from section 5.2 in the current file.
- Medium confidence on implementation fit until dedicated task code is added.
