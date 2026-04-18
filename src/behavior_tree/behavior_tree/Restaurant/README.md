# Restaurant Task Development Document

## Description

The robot retrieves and serves orders to several customers in a real restaurant previously unknown to the robot.

- **Main goal:** Detect calling or waving customer, reach a customer’s table without prior guidance/training. Take and serve all orders.
- **Optional goal:** Use an unmatched tray to transport the order.

## Focus

This task focuses on:
- Task planning
- Online mapping
- Navigation in unknown environments
- Gesture detection
- Verbal interaction
- Object manipulation

## Setup

### Locations:
- This task takes place in a real restaurant fully equipped and in business. When this is not possible, the test can be conducted in any place with the appropriate locations other than the Arena.
- The Restaurant location will remain secret until the start of the test.
- The robot starts next to the Kitchen-bar. It is a table located near the restaurant’s kitchen.

### People:
- A Professional Barman (member of the TC) awaits at the other side of the Kitchen-bar for orders to be placed. The Professional Barman assists the robot on request.
- There may be real customers and waiters around.
- There are at least three tables occupied with professional customers (member of the OC/TC).
- There are at least two tables occupied with regular customers.
- Customers may call the robot any time, even simultaneously.

### Furniture:
- The furniture is not standardized and will be kept the same as the restaurant or place selected for the task.

### Objects:
- Objects to fulfill orders are located on the Kitchen-bar.
- Orders have two or three objects randomly chosen.
- All edible/drinkable objects from the list of standard objects (see Section 3.3.5) are eligible to be part of the orders.

## Procedure

1. The referee requests the team to move the robot to the start location.
2. The referee gives the start signal and starts the timer.
3. The team leaves the area after the start signal.
4. A TC member follows the robot ready to press the emergency stop button.
5. The robot detects calling or waving customer and reaches a customer’s table.
6. The robot takes the customer’s order, places the order, and delivers it.
7. Optionally, the robot can use an unmatched tray to transport the order.

## Additional Rules and Remarks

### Remarks:
- This test takes place in a public area. The robot is expected to not even slightly touch anyone or anything and is immediately stopped in case of danger.
- Since this task is performed outside the arena, the time limit may be longer than the others tasks.
- The availability of wireless, external computing devices, or electrical outlets can’t be guaranteed. Assume unavailability.
- The robot interacts with the operators, not the team. The team is not allowed to instruct anyone. All instructions should be provided by the robot itself.
- The robot may use up to one minute to instruct the Professional Barman.
- The robot can request to be guided to a customer’s table.
- The robot can choose to take several orders and place them later on, place an order and pick the next one while the former is being served, or dispatch one order at a time.
- The robot should politely confirm the order to the client when receiving it, keeping the guest pleased.
- The robot can either transport each object individually, or using a tray. All delivered objects must be placed on the customer’s table.
- For transport with an unmatched tray, the robot must pick up the objects and place them on a tray, pick up the tray, and then place the objects from the tray on the table (first placing the tray on the table is allowed).
- If requested, the barman will place the order in a basket or tray for the robot to deliver it.
- Upon arrival to the restaurant, only two team members are allowed next to the robot for watching and charging.
- If a person from the audience (severely) interferes with the robot in a way that makes it impossible to solve the task, the teams may repeat the test immediately.
- Each Deux Ex Machina penalty for skipping manipulation will only be applied twice per order so receiving an order with three objects is not more punishing.
- If the robot detects a customer but does not reach their table, the robot must clearly show who was detected to receive points, i.e. displaying a picture of the person.
- When a team is at the front of the queue, they are allowed to begin their startup procedure (the robot must remain in place). When it’s their turn, they must bring the robot directly and in a straight line from the front of the queue to the start location. Once at the start location only slight movements are allowed (no moving back and forth, no full rotations etc).

### Disqualification:
- Touching the robot after the start signal.
- Mapping the area in advance.

## Instructions:

### To Referee
The referee needs to:
- Prepare orders for each client.

### To OC
The OC needs to:
- During Setup days: Check with local (security) management if the possible location, including a sufficient queuing area, can be used for the restaurant test.
- 1 hour before the test: Gather all teams and robots to move to some nearby queuing area and instruct the teams how/when to move to the actual test location.

## Score Sheet

The maximum time for this test is 15:00 minutes.

### Action | Score
Regular Rewards:
- Detect calling or waving customer: 2×100
- Reach a customer’s table without prior guidance/training: 2×100
- Understand and confirm the order received to the customer: 2×200
- Communicate the order to the barman: 2×100
- Return to the customer table with the order: 2×100
- Serve the order to the customer: 2×200

Bonus Rewards:
- Use an unmatched tray to transport: 2×200

Regular Penalties:
- Being guided to a table: 2×-200
- Not making eye-contact when taking an order: 2×-80
- Not reaching the bar (barman has to move from behind the bar to interact with the robot): 2×-80

Deux ex Machina Penalties:
- Asking the Barman to handover object to the robot: 4×-50
- Guest needing to take the object from a tray or the robot’s hand: 4×-50
- Being told/pointed where is a table/Kitchen-bar: 2×-100

Special Penalties & Bonuses:
- Not attending (see sec. 3.9.1): -500
- Using alternative start signal (see sec. 3.7.8): -100
- Outstanding performance (see sec. 3.9.3): 200

**Total Score (excluding special penalties & standard bonuses): 2000**

---

# Implementation (Tinker 2026)

## Entry points

- `ros2 run behavior_tree restaurant-demo` — mock/live; subsystems follow `mock_config.json`.
- `BT_MOCK_MODE=true ros2 run behavior_tree restaurant-demo` — force all subsystems to mock.

The live tree is `restaurants_fake.py::createRestaurantTask`. The older single-order-cycle version (`restaurants.py`) is retained but no longer the entry point.

## Target score

Two orders, individual carry, no tray: `2×(100+100+200+100+100+200) − 2×2×50 (capped handover penalty) = 1400 / 2000`.

Deferred for higher score: tray bonus (+400), outstanding-performance (+200).

## Design choices, bound to rulebook clauses

| Rulebook clause | Implementation |
|---|---|
| "Detect calling or waving customer" (+100) | `BtNode_ScanForWavingPerson` calls the tk26 `detect_waving_persons` service. Output is a closest-first list of `PointStamped`, plus per-person RGB crops written to `/tmp/restaurant_customer_<id>_<ts>.png`. |
| "Reach a customer's table without guidance" (+100; −200 if guided) | `BtNode_GotoAction` on the `PointStamped` promoted to a `PoseStamped` (identity orientation — nav2 handles approach yaw). No pointing/guidance node is ever ticked. |
| "Understand and confirm the order" (+200) | `BtNode_ListenAction` → `BtNode_PhraseExtraction` (wordlist from `constants.json["standard_objects"]`) → spoken echo via `BtNode_Announce` → `BtNode_GetConfirmationAction`. On FAILURE the outer `Retry(n=2)` re-ticks the whole collect-one-order sequence. |
| "Not making eye-contact when taking an order" (−80) | `BtNode_MaintainEyeContact` wraps HRI's `follow_head_action`. Ticked once before the listen/confirm pair, and once before each delivery place. |
| "Communicate the order to the barman" (+100) + "Not reaching the bar" (−80) | Single `BtNode_GotoAction(KITCHEN_BAR_POSE)` → `BtNode_FormatOrdersForBarman` builds `"Order 1 is water and juice. Order 2 is cola and chips."` → `BtNode_Announce(bb_source=...)` → `BtNode_GetConfirmationAction(timeout=120s)` (rulebook allows up to 2 min per order to instruct the barman). |
| "Asking Barman to handover" (−50, capped 2× per order) | Explicit handover path: `BtNode_MoveArmSingle(serving) → Announce("place X in my gripper") → GripperAction(open) → GetConfirmationAction → GripperAction(close)`. Cheap, legal, capped. |
| "Return with order" (+100) + "Serve" (+200) | Per-item delivery loop (see below). |
| "If detected but not reached, clearly show who was detected" | `BtNode_ShowImage` on the per-customer crop captured at detection time. Currently a **stub**: logs path + returns SUCCESS. Swap to the on-robot display topic when identified. |

## Tree topology

```
Restaurant Task (Sequence, memory=True)
├── Write constants to blackboard (Parallel, SuccessOnAll)
│   ├── kitchen_bar_pose, arm_navigating, arm_serving, empty order list
├── Kickoff announcement
│
├── Collect orders (2x)                             ── Phase 1
│   └── Repeat(n=2)
│       └── Retry(n=2)  "one retry per order"
│           └── Sequence "Collect one order"
│               ├── ScanForWavingPerson             → all_poses, all_pictures, closest
│               ├── SelectNextCustomer              → cur_id, cur_pose, cur_picture
│               ├── GotoAction(cur_pose)
│               ├── MaintainEyeContact
│               ├── Announce("Hello, may I take your order?")
│               ├── ListenAction                    → cur_order_items (raw string)
│               ├── PhraseExtraction                → cur_order_items (matched token(s))
│               ├── Announce("You ordered:" + bb cur_order_items)
│               ├── Announce("Is that correct?")
│               ├── GetConfirmationAction
│               └── RecordOrder                     → appends to order_list
│
├── Communicate orders to barman                    ── Phase 2
│   ├── Retry(3) { GotoAction(KITCHEN_BAR_POSE) }
│   ├── FormatOrdersForBarman                       → barman_text
│   ├── Announce(bb_source=barman_text)
│   └── GetConfirmationAction "Barman ready?" (120s)
│
├── Iterate + deliver                               ── Phase 3 (loop)
│   └── FailureIsSuccess
│       └── Retry(n=32) "deliver-all-items"
│           └── Sequence
│               ├── IterateOrderItems               → cur_item, cur_order_{id,pose,picture,summary}
│               └── Sequence "Deliver one item"
│                   ├── Retry(3) { GotoAction(KITCHEN_BAR_POSE) }
│                   ├── MoveArmSingle(serving)
│                   ├── Announce("please place: " + cur_order_summary)
│                   ├── GripperAction(open)
│                   ├── GetConfirmationAction "Item placed?" (60s)
│                   ├── GripperAction(close)
│                   ├── MoveArmSingle(navigating)
│                   ├── Selector "deliver-or-fallback"
│                   │   ├── Sequence "Normal delivery"
│                   │   │   ├── GotoAction(cur_order_pose)
│                   │   │   ├── MaintainEyeContact
│                   │   │   ├── MoveArmSingle(serving)
│                   │   │   ├── Announce("here is your: " + cur_order_summary)
│                   │   │   ├── Announce("Three. Two. One.")
│                   │   │   ├── GripperAction(open)       # release
│                   │   │   └── MoveArmSingle(navigating)
│                   │   └── Sequence "Show-picture fallback"
│                   │       ├── Announce("I could not reach…")
│                   │       ├── ShowImage(cur_order_picture)
│                   │       └── Success
│                   └── MarkItemDelivered
│
├── Completion announcement
└── Running("idle-at-end")
```

Loop mechanics: `BtNode_IterateOrderItems` advances one undelivered item per tick. When all items are marked delivered it returns `FAILURE`, which `Retry(32)` propagates up and `FailureIsSuccess` converts to the parent's `SUCCESS`. The `Retry` ceiling is a safety stop — in practice, 2 orders × ≤3 items = ≤6 iterations.

## Blackboard schema

All keys are root-scoped (`absolute_name("/", key)`). Module-local constants in `restaurants_fake.py`.

| Key | Writer → readers | Payload |
|---|---|---|
| `kitchen_bar_pose` | constant writer → Goto | `PoseStamped` from `constants.json` |
| `arm_navigating`, `arm_serving` | constant writer → MoveArmSingle | `list[float]` radians |
| `restaurant_all_person_poses` | ScanForWavingPerson → SelectNextCustomer | `list[PointStamped]` closest-first |
| `restaurant_all_person_pictures` | ScanForWavingPerson → SelectNextCustomer | `list[str]` filesystem paths |
| `restaurant_closest_person` | ScanForWavingPerson | `PointStamped` (unused downstream, kept for parity) |
| `restaurant_cur_id` / `_pose` / `_picture` | SelectNextCustomer → Goto, RecordOrder | one customer's fields |
| `restaurant_cur_order_items` | ListenAction / PhraseExtraction → Announce, RecordOrder | `str` or `list[str]` |
| `restaurant_order_list` | RecordOrder, MarkItemDelivered → FormatOrdersForBarman, IterateOrderItems | `list[dict]` (see schema below) |
| `restaurant_barman_text` | FormatOrdersForBarman → Announce | `str` |
| `restaurant_cur_item` / `_order_id` / `_order_pose` / `_order_picture` / `_order_summary` | IterateOrderItems → Deliver, MarkItemDelivered, ShowImage, Announce | current-delivery fields |

`restaurant_order_list` element schema:

```python
{
    "id": int,                    # 0-based customer index from ScanForWavingPerson
    "pose": PoseStamped,          # table location (identity orientation)
    "picture_path": str,          # filesystem path of cropped RGB; "" if unavailable
    "items": list[str],           # one or more item names
    "delivered_items": list[str], # populated by MarkItemDelivered
}
```

## Constants and hardcoded values

- `constants.json` is read at **tree construction time** (known bug: absolute `/home/tinker/...` path; see rule `.claude/rules/behavior-tree.md#hardcoded-paths-in-task-scripts`).
  - `pose_kitchen_bar` — `PoseStamped` in map frame, fed to every `Goto(KITCHEN_BAR_POSE)`.
  - `arm_pos_navigating`, `arm_pos_serving` — degrees in JSON, converted to radians at load.
  - `standard_objects` — used as `wordlist` for `BtNode_PhraseExtraction`.
- `DETECT_WAVING_THRESHOLD_M = 8.0` — waving-detection radius. Raise if the venue is large.
- `MaintainEyeContact._feedback_timeout_secs = 30.0` — gaze lock tolerance. The server usually returns success within a few seconds.
- `GetConfirmationAction` timeouts: 10 s for order confirmation, 60 s for barman item placement, 120 s for barman order readout (rulebook cap).

## Subsystem prerequisites

Mock (`BT_MOCK_MODE=true` or `mock_config.json`): no external servers needed.

Live hardware:
- **Vision:** `detect_waving_persons` (tk26 `tk_vision_specialized/waving_person_server`) running on Orbbec, `follow_head_action` (HRI module).
- **Audio:** `listen_action`, `get_confirmation_action` (both `tk_24_audio`), `phrase_extraction_service`, `announce` service (TTS).
- **Navigation:** nav2 bringup with a live map. Task disqualifies if pre-mapped — rely on online SLAM.
- **Manipulation:** `arm_joint_service` (xArm7), `/xarm_gripper/gripper_action`.

See `.claude/rules/behavior-tree.md` for the full mock/subsystem matrix.

## Known gaps / follow-ups

1. **`BtNode_ShowImage` is a stub** — it only logs the path and returns SUCCESS. The partial-points clause ("clearly show who was detected") will not actually render a picture on the robot display until this is wired to the real display topic.
2. **Eye-contact streaming vs. one-shot.** `follow_head_action` is modeled here as a one-shot goal (`start_following=True` → success on lock). If the server behaves as a continuous tracker, each `MaintainEyeContact` will block until the goal result is published — watch for long ticks during live runs.
3. **No TSP in delivery order.** `BtNode_IterateOrderItems` iterates `order_list` in the order collected. With N≤2 orders this is fine; revisit if the order count grows.
4. **Tray bonus (+400) not implemented.** Would need pick-onto-tray → pick-tray → place-from-tray choreography; no `place_on_tray` primitive currently exists.
5. **Waving detection has no stable id.** Customer ids are 0-based indices within a single service call. If the robot re-scans between orders, the same person may be assigned a different id. `SelectNextCustomer` only deduplicates within one scan response; this is fine because Phase 1 scans once per order, but be aware if the retry re-ticks `ScanForWavingPerson`.
6. **`waving_person_server.py` latent bugs** — line 99 typo and unbounded `/tmp/person_roi.png` writes (from prior audits). Not in scope for this pass.

## Files

```
Restaurant/
├── restaurants_fake.py    # active tree (entry point: restaurant-demo)
├── restaurants.py         # legacy single-order variant, unused
├── custumNodes.py         # task-local behaviours (sic: "custum")
├── constants.json         # poses, arm joint targets, wordlist
├── Rules_restaurant.pdf   # official rulebook §5.5
├── score_sheet.png        # scoring reference
└── README.md              # this file
```

## Verification

```bash
cd /home/tinker/tk25_ws
colcon build --packages-select behavior_tree
source install/setup.zsh

# Mock run
BT_MOCK_MODE=true ros2 run behavior_tree restaurant-demo

# Graph
ros2 run behavior_tree draw > /tmp/restaurant.dot
dot -Tpng /tmp/restaurant.dot -o /tmp/restaurant.png

# Live run (with all subsystems up)
ros2 run behavior_tree restaurant-demo
```