# GPSR bench summary

Corpus: `src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/corpus-42.jsonl`

| template | class | T1 |
|---|---|---|
| countObjOnPlcmt | A | 0/3 |
| countPrsInRoom | A | 0/3 |
| edge | A | 1/7 |
| findPrsInRoom | A | 0/3 |
| goToLoc | A | 0/3 |
| meetNameAtLocThenFindInRm | A | 0/3 |
| meetPrsAtBeac | A | 0/3 |
| talkInfoToGestPrsInRoom | A | 0/3 |
| tellCatPropOnPlcmt | A | 3/3 |
| tellObjPropOnPlcmt | A | 2/3 |
| bringMeObjFromPlcmt | B | 2/3 |
| findObjInRoom | B | 0/3 |
| countClothPrsInRoom | C | 0/3 |
| followNameFromBeacToRoom | C | 2/3 |
| followPrsAtLoc | C | 3/3 |
| greetClothDscInRm | C | 0/3 |
| greetNameInRm | C | 0/3 |
| guideClothPrsFromBeacToBeac | C | 0/3 |
| guideNameFromBeacToBeac | C | 0/3 |
| guidePrsFromBeacToBeac | C | 0/3 |
| takeObjFromPlcmt | C | 1/3 |
| tellPrsInfoAtLocToPrsAtLoc | C | 0/3 |
| tellPrsInfoInLoc | C | 0/3 |

## Totals

- T1: PASS 14, FAIL 23, TIMEOUT 36, ERROR 0

## Failures

- T1 `c002-bringMeObjFromPlcmt` **FAIL** — executor node FAILURE
- T1 `c003-countClothPrsInRoom` **FAIL** — executor node FAILURE
- T1 `c004-countClothPrsInRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c005-countClothPrsInRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c006-countObjOnPlcmt` **FAIL** — executor node FAILURE
- T1 `c007-countObjOnPlcmt` **TIMEOUT** — group timed out at slot 1
- T1 `c008-countObjOnPlcmt` **TIMEOUT** — group timed out at slot 1
- T1 `c009-countPrsInRoom` **FAIL** — executor node FAILURE
- T1 `c010-countPrsInRoom` **FAIL** — executor node FAILURE
- T1 `c011-countPrsInRoom` **TIMEOUT** — group timed out at slot 2
- T1 `c012-findObjInRoom-takeObj-putObjInTrash` **FAIL** — executor node FAILURE
- T1 `c013-findObjInRoom-takeObj-deliverObjToMe` **TIMEOUT** — group timed out at slot 1
- T1 `c014-findObjInRoom-takeObj-deliverObjToPrsInRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c015-findPrsInRoom-followPrsToRoom` **FAIL** — executor node FAILURE
- T1 `c016-findPrsInRoom-followPrsToRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c017-findPrsInRoom-talkInfo` **TIMEOUT** — group timed out at slot 1
- T1 `c020-followNameFromBeacToRoom` **FAIL** — executor node FAILURE
- T1 `c024-goToLoc-findPrs-guidePrsToBeacon` **FAIL** — executor node FAILURE
- T1 `c025-goToLoc-meetName-guidePrsToBeacon` **TIMEOUT** — group timed out at slot 1
- T1 `c026-goToLoc-findPrs-talkInfo` **TIMEOUT** — group timed out at slot 1
- T1 `c027-greetClothDscInRm-guidePrsToBeacon` **FAIL** — executor node FAILURE
- T1 `c028-greetClothDscInRm-talkInfo` **TIMEOUT** — group timed out at slot 1
- T1 `c029-greetClothDscInRm-talkInfo` **TIMEOUT** — group timed out at slot 1
- T1 `c030-greetNameInRm-followPrs` **FAIL** — executor node FAILURE
- T1 `c031-greetNameInRm-guidePrsToBeacon` **TIMEOUT** — group timed out at slot 1
- T1 `c032-greetNameInRm-followPrs` **TIMEOUT** — group timed out at slot 1
- T1 `c033-guideClothPrsFromBeacToBeac` **FAIL** — executor node FAILURE
- T1 `c034-guideClothPrsFromBeacToBeac` **TIMEOUT** — group timed out at slot 1
- T1 `c035-guideClothPrsFromBeacToBeac` **TIMEOUT** — group timed out at slot 1
- T1 `c036-guideNameFromBeacToBeac` **FAIL** — executor node FAILURE
- T1 `c037-guideNameFromBeacToBeac` **TIMEOUT** — group timed out at slot 1
- T1 `c038-guideNameFromBeacToBeac` **TIMEOUT** — group timed out at slot 1
- T1 `c039-guidePrsFromBeacToBeac` **FAIL** — executor node FAILURE
- T1 `c040-guidePrsFromBeacToBeac` **TIMEOUT** — group timed out at slot 1
- T1 `c041-guidePrsFromBeacToBeac` **TIMEOUT** — group timed out at slot 1
- T1 `c042-meetNameAtLocThenFindInRm` **FAIL** — executor node FAILURE
- T1 `c043-meetNameAtLocThenFindInRm` **TIMEOUT** — group timed out at slot 1
- T1 `c044-meetNameAtLocThenFindInRm` **TIMEOUT** — group timed out at slot 1
- T1 `c045-meetPrsAtBeac-talkInfo` **FAIL** — executor node FAILURE
- T1 `c046-meetPrsAtBeac-followPrsToRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c047-meetPrsAtBeac-talkInfo` **TIMEOUT** — group timed out at slot 1
- T1 `c049-takeObjFromPlcmt-putObjInTrash` **FAIL** — executor node FAILURE
- T1 `c050-takeObjFromPlcmt-placeObjOnPlcmt` **TIMEOUT** — group timed out at slot 2
- T1 `c051-talkInfoToGestPrsInRoom` **FAIL** — executor node FAILURE
- T1 `c052-talkInfoToGestPrsInRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c053-talkInfoToGestPrsInRoom` **TIMEOUT** — group timed out at slot 1
- T1 `c059-tellObjPropOnPlcmt` **FAIL** — executor node FAILURE
- T1 `c060-tellPrsInfoAtLocToPrsAtLoc` **FAIL** — executor node FAILURE
- T1 `c061-tellPrsInfoAtLocToPrsAtLoc` **TIMEOUT** — group timed out at slot 1
- T1 `c062-tellPrsInfoAtLocToPrsAtLoc` **TIMEOUT** — group timed out at slot 1
- T1 `c063-tellPrsInfoInLoc` **FAIL** — executor node FAILURE
- T1 `c064-tellPrsInfoInLoc` **TIMEOUT** — group timed out at slot 1
- T1 `c065-tellPrsInfoInLoc` **TIMEOUT** — group timed out at slot 1
- T1 `e00` **FAIL** — executor node FAILURE
- T1 `e01` **TIMEOUT** — group timed out at slot 1
- T1 `e02` **TIMEOUT** — group timed out at slot 1
- T1 `e03` **FAIL** — executor node FAILURE
- T1 `e04` **TIMEOUT** — group timed out at slot 1
- T1 `e05` **TIMEOUT** — group timed out at slot 1
