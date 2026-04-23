# Restaurant Rulebook Notes and BT Plan

Updated: 2026-03-10
Primary code: `Restaurant/restaurants.py`
Rulebook file: `Restaurant/Rulebook-Restaurantpdf`

## Rulebook Extraction Notes

- The file now contains the correct Restaurant section (RoboCup@Home 2026, 5.5).
- Text extraction via `pdftotext` was successful.

## Requirements Summary (Rulebook 5.5)

- Main goal:
  - Detect calling/waving customers.
  - Autonomously navigate to customer tables.
  - Take order, communicate with barman, pick requested items, return and serve.
- Optional goal:
  - Use unattached tray for transport.
- Setup highlights:
  - Real, previously unknown restaurant-like environment.
  - Robot starts at kitchen bar facing dining area.
  - Multiple professional and regular customers, including simultaneous calls.
- Key constraints:
  - Safety-critical: public environment, no contact.
  - No team assistance/instruction during task.
  - Robot may choose one-by-one or interleaved order strategy.
  - If customer detected but not reached, robot must clearly identify detected person for partial score.
  - Mapping in advance is disqualifying.
- Time limit: 15:00.
- Score highlights:
  - High-value actions: order understanding/confirmation, pickup, serving.
  - Bonus: tray transport.
  - Penalties: guided navigation, poor eye contact at ordering, directional confirmations, handover assistance.

## Broad Implementation Plan (Rulebook-First)

1. Initialize at kitchen bar and start service loop.
2. Customer arbitration:
   - detect caller/waver events
   - prioritize/select target customer when multiple calls occur
3. Table approach and engagement:
   - navigate to selected table autonomously
   - perform order capture + confirmation dialogue
4. Kitchen-bar fulfillment:
   - return to bar
   - communicate order to barman
   - pick requested items (tray or direct mode)
5. Delivery:
   - return to correct customer table
   - serve all requested objects and verify completion
6. Multi-order orchestration:
   - sequential or interleaved policy with explicit state tracking
7. Timeout-aware policy for maximizing points before 15:00.

## Refined Plan Against Current Code

Current strengths (`restaurants.py`):
- Good subtree decomposition:
  - `createDetectAndArbitrateCustomers`
  - `createApproachCustomer`
  - `createTakeAndConfirmOrder`
  - `createPlaceOrderWithBarman`
  - `createPickupVerification`
  - `createOptionalTrayTransport`
  - `createDeliverOrder`
- Contains first-order + optional-second-order structure.
- Includes custom nodes for customer detection, order intake, and service.
- Adds blackboard state for:
  - `customer_queue`
  - `active_customer_id`
  - `order_checklist`
  - `pickup_verified`

Main gaps vs rulebook:
- No explicit concurrent-caller arbitration policy despite rule allowing simultaneous calls.
- Pickup stage is weakly explicit in current tree (rule now scores pickup from kitchen-bar directly).
- Eye-contact scoring and directional-confirmation penalties are not explicitly monitored.
- Absolute constants path (`/home/tinker/.../constants.json`) remains fragile.

Implementation priorities:
1. Add customer-queue arbitration and selection criteria.
2. Add explicit pickup-verification subtree at kitchen-bar.
3. Add dialogue/gaze policy hooks for order-taking quality metrics.
4. Add per-order completion checklist on blackboard.
5. Move constants loading to package-relative lookup.
6. Add integration tests:
   - two simultaneous callers
   - tray and non-tray delivery
   - partial-score path when customer detected but unreachable.

## Implemented Mock/Fallback Nodes and Gaps (Current `Restaurant/restaurants.py`)

- No custom mock node classes were added in Restaurant, but there are logical placeholders:
  - Pickup verification currently uses `BtNode_MarkPickupVerified` (state flip), not real pickup sensing/manipulation verification.
  - Eye-contact/direction quality is represented by dialogue wording only, not measurable gaze-tracking node outputs.
- Partial-score fallback exists:
  - On unreachable table, robot announces failure and marks checklist `partial_score=True`.
  - Current announcement does not include explicit customer identity/details from perception.
- Simultaneous-caller handling:
  - Queue-based arbitration exists and is deterministic (oldest first).
  - Candidate collection is still simplistic (one optional second quick-detect pass).

## Missing Features vs Rulebook (Current State)

- No true object-level pickup verification from kitchen bar (currently checklist-based).
- No explicit gaze-quality monitoring node tied to score penalties.
- No explicit per-customer identity announcement when detected but unreachable.
- Multi-caller policy supports basic queueing but not robust ongoing arbitration under heavy traffic.
- No explicit 15:00 time-budget optimizer (cutoff policy / priority preemption).

## Potential Improvements (Prioritized)

1. Replace checklist-only pickup verification with real pickup detection/manipulation result checks.
2. Add gaze/contact quality nodes during order-taking and store measurable quality flags on blackboard.
3. Expand unreachable-customer partial-score behavior to announce identifiable cues (position/table descriptor).
4. Upgrade caller arbitration to continuous queue maintenance (new calls while serving, stale-call expiry, confidence decay).
5. Add order policy mode switch:
   - strict sequential
   - interleaved multi-order.
6. Add time-aware orchestration for 15:00 limit (skip/shorten low-value steps near timeout).

## Confidence and Provenance

- High confidence: requirements and scoring from section 5.5 in current rulebook file.
- Medium confidence on code alignment until pickup/arbitration/gaze metrics are implemented.
