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