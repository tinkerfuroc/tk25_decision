# GPSR bench summary

Corpus: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl`

| template | class | T0 |
|---|---|---|
| countObjOnPlcmt | A | 0/3 |
| countPrsInRoom | A | 0/3 |
| edge | A | 4/7 |
| findPrsInRoom | A | 0/3 |
| goToLoc | A | 1/3 |
| meetNameAtLocThenFindInRm | A | 3/3 |
| meetPrsAtBeac | A | 2/3 |
| talkInfoToGestPrsInRoom | A | 0/3 |
| tellCatPropOnPlcmt | A | 0/3 |
| tellObjPropOnPlcmt | A | 0/3 |
| bringMeObjFromPlcmt | B | 0/3 |
| findObjInRoom | B | 1/3 |
| countClothPrsInRoom | C | 0/3 |
| followNameFromBeacToRoom | C | 2/3 |
| followPrsAtLoc | C | 0/3 |
| greetClothDscInRm | C | 0/3 |
| greetNameInRm | C | 3/3 |
| guideClothPrsFromBeacToBeac | C | 0/3 |
| guideNameFromBeacToBeac | C | 1/3 |
| guidePrsFromBeacToBeac | C | 0/3 |
| takeObjFromPlcmt | C | 2/3 |
| tellPrsInfoAtLocToPrsAtLoc | C | 2/3 |
| tellPrsInfoInLoc | C | 0/3 |

## Totals

- T0: PASS 21, FAIL 30, TIMEOUT 22, ERROR 0

## Failures

- T0 `c000-bringMeObjFromPlcmt` **FAIL** — target 0: step 2: recipient_location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c001-bringMeObjFromPlcmt` **FAIL** — target 0: step 2: recipient_location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c002-bringMeObjFromPlcmt` **FAIL** — target 0: step 2: recipient_location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c003-countClothPrsInRoom` **FAIL** — target 0: step 3: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c004-countClothPrsInRoom` **FAIL** — target 0: step 3: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c005-countClothPrsInRoom` **FAIL** — target 0: step 3: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c006-countObjOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c007-countObjOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c008-countObjOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c009-countPrsInRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c010-countPrsInRoom` **FAIL** — target 0: step 3: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c011-countPrsInRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c013-findObjInRoom-takeObj-deliverObjToMe` **FAIL** — target 2: step 0: recipient_location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c014-findObjInRoom-takeObj-deliverObjToPrsInRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c015-findPrsInRoom-followPrsToRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c016-findPrsInRoom-followPrsToRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c017-findPrsInRoom-talkInfo` **TIMEOUT** — planner not ready after 90s
- T0 `c019-followNameFromBeacToRoom` **FAIL** — target 0: command mentions 'follow ... to the kitchen' but no goto(location='kitchen') step was emitted. Append a goto after the follow so the robot can reach the destination.
- T0 `c021-followPrsAtLoc` **TIMEOUT** — planner not ready after 90s
- T0 `c022-followPrsAtLoc` **TIMEOUT** — planner not ready after 90s
- T0 `c023-followPrsAtLoc` **TIMEOUT** — planner not ready after 90s
- T0 `c024-goToLoc-findPrs-guidePrsToBeacon` **TIMEOUT** — planner not ready after 90s
- T0 `c025-goToLoc-meetName-guidePrsToBeacon` **FAIL** — target 2: step 1: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c027-greetClothDscInRm-guidePrsToBeacon` **TIMEOUT** — planner not ready after 90s
- T0 `c028-greetClothDscInRm-talkInfo` **TIMEOUT** — planner not ready after 90s
- T0 `c029-greetClothDscInRm-talkInfo` **TIMEOUT** — planner not ready after 90s
- T0 `c033-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 90s
- T0 `c034-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 90s
- T0 `c035-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 90s
- T0 `c036-guideNameFromBeacToBeac` **FAIL** — target 1: step 1: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c038-guideNameFromBeacToBeac` **FAIL** — target 1: step 2: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
- T0 `c039-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 90s
- T0 `c040-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 90s
- T0 `c041-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 90s
- T0 `c046-meetPrsAtBeac-followPrsToRoom` **FAIL** — target 0: command mentions 'follow ... to the bedroom' but no goto(location='bedroom') step was emitted. Append a goto after the follow so the robot can reach the destination.
- T0 `c048-takeObjFromPlcmt-deliverObjToMe` **FAIL** — target 1: step 0: recipient_location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c051-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c052-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c053-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 90s
- T0 `c054-tellCatPropOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c055-tellCatPropOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c056-tellCatPropOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c057-tellObjPropOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c058-tellObjPropOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c059-tellObjPropOnPlcmt` **FAIL** — target 0: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c062-tellPrsInfoAtLocToPrsAtLoc` **FAIL** — target 3: step 1: announce() with no text reports the last gathered result, but no count/describe_person/ask_person/vlm_fallback ran before it. Either give announce a literal text=..., or do the gathering action first (then goto start_position, then the text-less announce).
- T0 `c063-tellPrsInfoInLoc` **FAIL** — target 0: step 4: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c064-tellPrsInfoInLoc` **FAIL** — target 1: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `c065-tellPrsInfoInLoc` **FAIL** — target 1: step 2: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `e01` **FAIL** — target 1: step 1: location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `e02` **FAIL** — target 0: step 2: recipient_location='start_position' is not a known location and was not fixed by an earlier record_position(label=...). Use a known location, or record_position it first.
- T0 `e04` **FAIL** — target 1: step 2: guide() used without a prior find_person() — guide is for leading a person, not for moving the robot. Use goto() to move the robot.
