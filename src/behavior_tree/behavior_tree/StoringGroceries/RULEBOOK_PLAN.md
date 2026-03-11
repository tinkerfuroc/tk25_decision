# StoringGroceries Rulebook Notes and BT Plan

Updated: 2026-03-10
Primary code: `storing_groceries.py`
Rulebook PDF: `Rulebook-StoringGroceries.pdf`

## Rulebook Extraction Notes

- The PDF is an image-only scan; direct text extraction is empty.
- OCR was used to recover text for this summary.
- Rule source identified as RoboCup@Home 2025, Chapter 5.4.

## Rulebook Requirements Summary

- Main goal:
  - Move objects from table to cabinet and place them near similar-category objects.
- Optional goals:
  - Open cabinet doors.
  - Move tiny object.
  - Move heavy object.
  - Pick objects from shopping bag.
  - Pour cereal into designated open container.
- Setup essentials:
  - Start outside arena and enter after door opens.
  - Test area has table + cabinet.
  - Table: 5-10 objects; bag: 5-10 additional objects.
  - Cabinet initially closed, shelves pre-grouped by category/similarity.
- Constraints:
  - If door opening fails, robot should request referee help.
  - Wrong shelf categorization penalized per object.
  - Unknown-category objects should be grouped as a new category.
  - Robot must clearly communicate perception to referee.
- Max time: 7:00.

## Broad Implementation Plan (Rulebook-First)

1. Enter arena and localize table/cabinet.
2. Ensure cabinet access (autonomous open -> human fallback request).
3. Build shelf-category model from cabinet scan.
4. Loop object handling:
   - detect candidate object (table then bag)
   - announce selected object + intended shelf
   - grasp with retries
   - navigate/position to cabinet
   - place on matching shelf
   - verify placement quality
5. Bonus branch handling:
   - bag objects
   - tiny/heavy object selection
   - cereal pouring with spill control
6. Apply task-level timeout policy and completion report.

## Refined Plan Based on Current Code

Current strengths in `storing_groceries.py`:
- Arena entry + door detection gate.
- Blackboard constants setup.
- Table object grasping pipeline (`BtNode_FindObjTable` + `BtNode_GraspWithPose`).
- Shelf categorization for placement (`BtNode_CategorizeGrocery`).
- Place action with retries and fallback arm/gripper sequence.
- Multi-table-pose traversal loop.

Observed gaps to close:
- Missing explicit cereal pouring subtree.
- Missing explicit tiny/heavy-object strategy.
- Bag-object branch not clearly modeled as a dedicated pipeline.
- No autonomous cabinet-door opening implementation.
- No explicit post-place semantic verification loop.
- No explicit 7:00 task budget guard.

Refined implementation steps:
1. Add `createPourCereal()` subtree and rollback behavior.
2. Add `createPickFromBag()` subtree with dedicated prompts/poses.
3. Add tiny/heavy target policy and dedicated attempt branches.
4. Add door-opening subtree before human-assist fallback.
5. Add place-verification node with re-place fallback.
6. Add global timer guard + final scored-actions summary.
