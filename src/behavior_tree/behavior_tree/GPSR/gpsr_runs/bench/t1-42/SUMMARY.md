# GPSR bench summary

Corpus: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl`

Run: tier=1, group_size=3, timeout_s=300.0, live_llm=True, seed=42, timestamp=2026-08-23T16:19:20.193623+00:00, commit=f1fac8a15992521a75a6812b9a3c82a4aeec7446, corpus=src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl

| template | class | T1 |
|---|---|---|
| countObjOnPlcmt | A | 1/3 |
| countPrsInRoom | A | 0/3 |
| edge | A | 5/5 |
| findPrsInRoom | A | 0/1 |
| goToLoc | A | 1/1 |
| meetNameAtLocThenFindInRm | A | 3/3 |
| meetPrsAtBeac | A | 2/2 |
| talkInfoToGestPrsInRoom | A | 3/3 |
| tellCatPropOnPlcmt | A | 3/3 |
| tellObjPropOnPlcmt | A | 3/3 |
| bringMeObjFromPlcmt | B | 2/3 |
| edge | B | 1/1 |
| findObjInRoom | B | 0/2 |
| takeObjFromPlcmt | B | 1/1 |
| countClothPrsInRoom | C | 0/3 |
| edge | C | 1/1 |
| findObjInRoom | C | 0/1 |
| findPrsInRoom | C | 0/2 |
| followNameFromBeacToRoom | C | 3/3 |
| followPrsAtLoc | C | 3/3 |
| goToLoc | C | 2/2 |
| greetClothDscInRm | C | 3/3 |
| greetNameInRm | C | 3/3 |
| guideClothPrsFromBeacToBeac | C | 3/3 |
| guideNameFromBeacToBeac | C | 3/3 |
| guidePrsFromBeacToBeac | C | 3/3 |
| meetPrsAtBeac | C | 1/1 |
| takeObjFromPlcmt | C | 2/2 |
| tellPrsInfoAtLocToPrsAtLoc | C | 3/3 |
| tellPrsInfoInLoc | C | 3/3 |

## Totals

- T1: PASS 58, FAIL 6, TIMEOUT 5, ERROR 4

## Totals by class

T1:
- class A: 21/27
- class B: 4/7
- class C: 33/39
- Class A+B pass rate: 25/34 (74 %)

## Failures

- T1 `c002-bringMeObjFromPlcmt` **FAIL** — postcondition unmet: person_found(user) (UNKNOWN)
- T1 `c003-countClothPrsInRoom` **FAIL** — precondition unmet: at_robot(laundry_room) (INVALID)
- T1 `c004-countClothPrsInRoom` **TIMEOUT** — slot 1 timed out after 300s
- T1 `c005-countClothPrsInRoom` **ERROR** — not reached: slot 1 timed out
- T1 `c007-countObjOnPlcmt` **FAIL** — postcondition unmet: answered(how many drinks there are on the kitchen_table) (UNKNOWN)
- T1 `c008-countObjOnPlcmt` **TIMEOUT** — slot 2 timed out after 300s
- T1 `c009-countPrsInRoom` **FAIL** — precondition unmet: at_robot(bedroom) (INVALID)
- T1 `c010-countPrsInRoom` **TIMEOUT** — slot 1 timed out after 300s
- T1 `c011-countPrsInRoom` **ERROR** — not reached: slot 1 timed out
- T1 `c012-findObjInRoom-takeObj-putObjInTrash` **FAIL** — postcondition unmet: object_seen(kitchen item) (UNKNOWN)
- T1 `c013-findObjInRoom-takeObj-deliverObjToMe` **TIMEOUT** — slot 1 timed out after 300s
- T1 `c014-findObjInRoom-takeObj-deliverObjToPrsInRoom` **ERROR** — not reached: slot 1 timed out
- T1 `c015-findPrsInRoom-followPrsToRoom` **FAIL** — postcondition unmet: person_found(sitting person) (UNKNOWN)
- T1 `c016-findPrsInRoom-followPrsToRoom` **TIMEOUT** — slot 1 timed out after 300s
- T1 `c017-findPrsInRoom-talkInfo` **ERROR** — not reached: slot 1 timed out
