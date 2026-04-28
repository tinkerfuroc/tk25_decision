5.4. Doing Laundry Challenge
The robot assists with laundry by retrieving clothes from a washing machine and folding them
neatly on a table. This test evaluates deformable object manipulation, appliance interaction,
and task sequencing.
Main goal: Transport clothes to a table, and fold them.
Optional goals:
1. Opening the washing machine door
2. Retrieving clothes from inside the washing machine
3. Using a laundry basket for transportation
4. Folding multiple pieces of clothing and stacking them
Focus
Deformable object manipulation, bimanual grasping.
Setup
• Locations:
– Start location: Before the test, the robot waits outside the Arena and navigates to
the laundry area when the door is open.
– Test locations: The testing area contains a washing machine, a folding surface
nearby, and a laundry basket.
• People:
– No people are involved in the test, unless the robot requires human assistance.
• Furniture:
– Laundry basket: A lightweight basket requiring two arms to carry, placed near the
washing machine. The basket already has 6–8 pieces of clothing placed inside.
– Washing machine: Positioned with its door closed at the start. Clothes are placed
inside. There are 2–4 pieces of clothing in the washing machine.
– Table: Positioned near the washing machine. Used as the folding surface. There is
a single piece of clothing loosely placed (or bunched up) on the table.
• Objects:
– Clothes: A set of 8–12 pieces of laundry made from cloth or other deformable fabric.
Procedure
1. Location announcement: At least two hours before the test, referees announce which
washing machine and table will be used, along with the approximate location of each.
2. Test start: The robot enters the testing area once the arena door is opened.
3. Retrieving laundry: The robot retrieves the clothes from the laundry basket or directly
from the washing machine and places them on the table.
4. Folding: The robot attempts to fold at least one piece of clothing neatly.
RoboCup@Home Rulebook / Final version for RoboCup 2026 (Revision 2026-04-10 1)
Chapter 5. Tests 49
Additional rules and remarks
1. Laundry Types: The laundry will consist exclusively of T-shirts.
2. Cloth placement: Clothes must be placed on the table before folding. Folding on the
floor is not allowed.
3. Folding quality: Folding is evaluated by neatness and whether the cloth is flattened and
stacked.
4. Multiple items: Additional points are awarded for folding multiple pieces of clothing
and stacking them.
5. Basket transportation: Using the basket to transport clothing from the washing ma-
chine to the table. The basket must be placed in reach of the folding surface.
6. Human Assistance: Scores are reduced if human assistance is received, in particular for:
• opening the washing machine door
• handing clothes to the robot
• Folding: Penalties are proportional to the amount of help provided:
– Minimal help (e.g., smoothing wrinkles, stabilizing cloth): small penalty ( 100
points).
– Partial folding (e.g., folding one half or aligning edges): moderate penalty ( 200
points).
– Major help (e.g., completing entire fold/stack): maximum penalty.
OC Instructions
At least two hours before the test:
• Announce which washing machine and table will be used.
Referee Instructions
The referee needs to:
• Ensure the washing machine and basket contains the laundry before the test begins.
• Close the washing machine door at the start.
• Verify that the folding table is clear before the test.
Score sheet
The maximum time for this test is 7:00 minutes.
RoboCup@Home Rulebook / Final version for RoboCup 2026 (Revision 2026-04-10 1)
50 5.4 Doing Laundry Challenge
Action Score
Main Goal
Navigating to the laundry area 15
Picking up a piece of clothing from the basket 100
Picking up multiple at once –100
Folding a piece of clothing 800
Human assistance: Flattening/Arranging clothing before folding –200
Human assistance during folding will apply a
percentage penalty according to the amount of help –800
Extra Rewards
Opening the washing machine door 300
Removing one or more pieces of clothing from the washing machine 300
The clothing touches the floor –200
Using the basket for transportation 300
Laundry is dropped or otherwise lost during transportation. –200
Folding additional clothes (per item) 5×400
Human assistance: Flattening/Arranging clothing before folding 5×–200
Human assistance during folding will apply a
percentage penalty according to the amount of help 5×–400
Stacking folded clothes neatly (per item) 6×100
Penalties
Human assistance: environment changes (per item) –40
Special Penalties & Bonuses
Not attending (see sec. 3.8.1) –500
Using alternative start signal (see sec. 3.6.8) –100
Outstanding performance (see sec. 3.8.3) 441
Total Score (excluding special penalties & standard bonuses) 4415
