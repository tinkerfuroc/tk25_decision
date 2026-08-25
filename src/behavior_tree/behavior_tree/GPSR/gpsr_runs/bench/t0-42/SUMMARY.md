# GPSR bench summary

Corpus: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl`

Run: tier=0, timeout_s=180.0, seed=42, timestamp=2026-08-23T15:55:50.270704+00:00, commit=f1fac8a15992521a75a6812b9a3c82a4aeec7446, corpus=src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl

| template | class | T0 |
|---|---|---|
| countObjOnPlcmt | A | 3/3 |
| countPrsInRoom | A | 3/3 |
| edge | A | 5/5 |
| findPrsInRoom | A | 0/1 |
| goToLoc | A | 0/1 |
| meetNameAtLocThenFindInRm | A | 3/3 |
| meetPrsAtBeac | A | 2/2 |
| talkInfoToGestPrsInRoom | A | 0/3 |
| tellCatPropOnPlcmt | A | 3/3 |
| tellObjPropOnPlcmt | A | 3/3 |
| bringMeObjFromPlcmt | B | 3/3 |
| edge | B | 1/1 |
| findObjInRoom | B | 1/2 |
| takeObjFromPlcmt | B | 1/1 |
| countClothPrsInRoom | C | 1/3 |
| edge | C | 1/1 |
| findObjInRoom | C | 1/1 |
| findPrsInRoom | C | 0/2 |
| followNameFromBeacToRoom | C | 3/3 |
| followPrsAtLoc | C | 0/3 |
| goToLoc | C | 1/2 |
| greetClothDscInRm | C | 0/3 |
| greetNameInRm | C | 3/3 |
| guideClothPrsFromBeacToBeac | C | 0/3 |
| guideNameFromBeacToBeac | C | 3/3 |
| guidePrsFromBeacToBeac | C | 0/3 |
| meetPrsAtBeac | C | 1/1 |
| takeObjFromPlcmt | C | 2/2 |
| tellPrsInfoAtLocToPrsAtLoc | C | 3/3 |
| tellPrsInfoInLoc | C | 3/3 |

## Totals

- T0: PASS 50, FAIL 0, TIMEOUT 23, ERROR 0

## Totals by class

T0:
- class A: 22/27
- class B: 6/7
- class C: 22/39
- Class A+B pass rate: 28/34 (82 %)

## Failures

- T0 `c004-countClothPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c005-countClothPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c014-findObjInRoom-takeObj-deliverObjToPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c015-findPrsInRoom-followPrsToRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c016-findPrsInRoom-followPrsToRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c017-findPrsInRoom-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c021-followPrsAtLoc` **TIMEOUT** — planner not ready after 180s
- T0 `c022-followPrsAtLoc` **TIMEOUT** — planner not ready after 180s
- T0 `c023-followPrsAtLoc` **TIMEOUT** — planner not ready after 180s
- T0 `c024-goToLoc-findPrs-guidePrsToBeacon` **TIMEOUT** — planner not ready after 180s
- T0 `c026-goToLoc-findPrs-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c027-greetClothDscInRm-guidePrsToBeacon` **TIMEOUT** — planner not ready after 180s
- T0 `c028-greetClothDscInRm-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c029-greetClothDscInRm-talkInfo` **TIMEOUT** — planner not ready after 180s
- T0 `c033-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c034-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c035-guideClothPrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c039-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c040-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c041-guidePrsFromBeacToBeac` **TIMEOUT** — planner not ready after 180s
- T0 `c051-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c052-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 180s
- T0 `c053-talkInfoToGestPrsInRoom` **TIMEOUT** — planner not ready after 180s
