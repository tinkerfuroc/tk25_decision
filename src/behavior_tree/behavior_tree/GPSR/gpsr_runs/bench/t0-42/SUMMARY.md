# GPSR bench summary

Corpus: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl`

| template | class | T0 |
|---|---|---|
| countObjOnPlcmt | A | 3/3 |
| countPrsInRoom | A | 1/3 |
| edge | A | 6/7 |
| findPrsInRoom | A | 0/3 |
| goToLoc | A | 1/3 |
| meetNameAtLocThenFindInRm | A | 3/3 |
| meetPrsAtBeac | A | 2/3 |
| talkInfoToGestPrsInRoom | A | 0/3 |
| tellCatPropOnPlcmt | A | 3/3 |
| tellObjPropOnPlcmt | A | 3/3 |
| bringMeObjFromPlcmt | B | 3/3 |
| findObjInRoom | B | 2/3 |
| countClothPrsInRoom | C | 3/3 |
| followNameFromBeacToRoom | C | 1/3 |
| followPrsAtLoc | C | 0/3 |
| greetClothDscInRm | C | 0/3 |
| greetNameInRm | C | 2/3 |
| guideClothPrsFromBeacToBeac | C | 0/3 |
| guideNameFromBeacToBeac | C | 1/3 |
| guidePrsFromBeacToBeac | C | 0/3 |
| takeObjFromPlcmt | C | 3/3 |
| tellPrsInfoAtLocToPrsAtLoc | C | 2/3 |
| tellPrsInfoInLoc | C | 3/3 |

## Totals

- T0: PASS 42, FAIL 10, TIMEOUT 21, ERROR 0

## Failures

- T0 `c010-countPrsInRoom` **FAIL** — target 1: step 1: announce() with no text reports the last gathered result, but no count/describe_person/ask_person/vlm_fallback ran before it. Either give announce a literal text=..., or do the gathering action first (then goto start_position, then the text-less announce).
- T0 `c011-countPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c014-findObjInRoom-takeObj-deliverObjToPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c015-findPrsInRoom-followPrsToRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c016-findPrsInRoom-followPrsToRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c017-findPrsInRoom-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c018-followNameFromBeacToRoom` **FAIL** — target 0: command mentions 'follow ... to the kitchen' but no goto(location='kitchen') step was emitted. Append a goto after the follow so the robot can reach the destination.
- T0 `c019-followNameFromBeacToRoom` **FAIL** — target 0: command mentions 'follow ... to the kitchen' but no goto(location='kitchen') step was emitted. Append a goto after the follow so the robot can reach the destination.
- T0 `c021-followPrsAtLoc` **TIMEOUT** — planner not ready after 180s
- T0 `c022-followPrsAtLoc` **TIMEOUT** — planner not ready after 180s
- T0 `c023-followPrsAtLoc` **TIMEOUT** — planner not ready after 180s
- T0 `c024-goToLoc-findPrs-guidePrsToBeacon` **TIMEOUT** — planner not ready after 180s
- T0 `c025-goToLoc-meetName-guidePrsToBeacon` **FAIL** — target 2: step 1: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c027-greetClothDscInRm-guidePrsToBeacon` **TIMEOUT** — planner not ready after 180s
- T0 `c028-greetClothDscInRm-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c029-greetClothDscInRm-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c031-greetNameInRm-guidePrsToBeacon` **FAIL** — target 1: step 1: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c033-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c034-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c035-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c036-guideNameFromBeacToBeac` **FAIL** — target 1: step 0: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c038-guideNameFromBeacToBeac` **FAIL** — target 1: step 2: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c039-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c040-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c041-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c046-meetPrsAtBeac-followPrsToRoom` **FAIL** — target 0: command mentions 'follow ... to the bedroom' but no goto(location='bedroom') step was emitted. Append a goto after the follow so the robot can reach the destination.
- T0 `c051-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c052-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c053-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c061-tellPrsInfoAtLocToPrsAtLoc` **FAIL** — target 3: step 1: announce() with no text reports the last gathered result, but no count/describe_person/ask_person/vlm_fallback ran before it. Either give announce a literal text=..., or do the gathering action first (then goto start_position, then the text-less announce).
- T0 `e04` **FAIL** — target 2: step 0: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
