# GPSR bench summary

Corpus: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-2026/corpus.jsonl`

Run: tier=T2, timeout_s=600.0, settle_s=10.0, live_llm=True, seed=2026, timestamp=2026-08-26T18:51:53.769355+00:00, commit=03fe889c8426e002e12cae526ce8988e0b0b048d, corpus=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-2026/corpus.jsonl

| template | class | T2 |
|---|---|---|
| countObjOnPlcmt | A | 4/4 |
| countPrsInRoom | A | 4/4 |
| greetNameInRm | A | 2/2 |
| meetNameAtLocThenFindInRm | A | 2/4 |
| meetPrsAtBeac | A | 1/1 |
| talkInfoToGestPrsInRoom | A | 2/3 |
| tellCatPropOnPlcmt | A | 3/3 |
| tellObjPropOnPlcmt | A | 3/3 |
| bringMeObjFromPlcmt | B | 4/4 |
| findObjInRoom | B | 1/2 |
| takeObjFromPlcmt | B | 2/2 |
| findObjInRoom | C | 1/1 |
| goToLoc | C | 1/1 |
| tellPrsInfoAtLocToPrsAtLoc | C | 3/3 |
| tellPrsInfoInLoc | C | 3/3 |

## Totals

- T2: PASS 36, FAIL 0, TIMEOUT 2, ERROR 2

## Totals by class

T2:
- class A: 21/24
- class B: 7/8
- class C: 8/8
- Class A+B pass rate: 28/32 (88 %)

## Failures

- T2 `s2026-028-meetNameAtLocThenFindInRm` **ERROR** — reset failed: timed out after 60s
- T2 `s2026-030-talkInfoToGestPrsInRoom` **ERROR** — reset failed: timed out after 60s
- T2 `s2026-038-findObjInRoom` **TIMEOUT** — timed out after 600s | sheet=runs/s2026-038-findObjInRoom/sheet.jpg
- T2 `s2026-039-meetNameAtLocThenFindInRm` **TIMEOUT** — timed out after 600s | sheet=runs/s2026-039-meetNameAtLocThenFindInRm/sheet.jpg

## Runs

- s2026-000-bringMeObjFromPlcmt **PASS**
- s2026-001-countObjOnPlcmt **PASS**
- s2026-002-countPrsInRoom **PASS**
- s2026-003-findObjInRoom **PASS**
- s2026-004-goToLoc **PASS**
- s2026-005-greetNameInRm **PASS**
- s2026-006-meetNameAtLocThenFindInRm **PASS**
- s2026-007-meetPrsAtBeac **PASS**
- s2026-008-talkInfoToGestPrsInRoom **PASS**
- s2026-009-tellCatPropOnPlcmt **PASS**
- s2026-010-tellObjPropOnPlcmt **PASS**
- s2026-011-tellPrsInfoAtLocToPrsAtLoc **PASS**
- s2026-012-tellPrsInfoInLoc **PASS**
- s2026-013-bringMeObjFromPlcmt **PASS**
- s2026-014-countObjOnPlcmt **PASS**
- s2026-015-countPrsInRoom **PASS**
- s2026-016-findObjInRoom **PASS**
- s2026-017-greetNameInRm **PASS**
- s2026-018-meetNameAtLocThenFindInRm **PASS**
- s2026-019-takeObjFromPlcmt **PASS**
- s2026-020-talkInfoToGestPrsInRoom **PASS**
- s2026-021-tellCatPropOnPlcmt **PASS**
- s2026-022-tellObjPropOnPlcmt **PASS**
- s2026-023-tellPrsInfoAtLocToPrsAtLoc **PASS**
- s2026-024-tellPrsInfoInLoc **PASS**
- s2026-025-bringMeObjFromPlcmt **PASS**
- s2026-026-countObjOnPlcmt **PASS**
- s2026-027-countPrsInRoom **PASS**
- s2026-028-meetNameAtLocThenFindInRm **ERROR**
- s2026-029-takeObjFromPlcmt **PASS**
- s2026-030-talkInfoToGestPrsInRoom **ERROR**
- s2026-031-tellCatPropOnPlcmt **PASS**
- s2026-032-tellObjPropOnPlcmt **PASS**
- s2026-033-tellPrsInfoAtLocToPrsAtLoc **PASS**
- s2026-034-tellPrsInfoInLoc **PASS**
- s2026-035-bringMeObjFromPlcmt **PASS**
- s2026-036-countObjOnPlcmt **PASS**
- s2026-037-countPrsInRoom **PASS**
- s2026-038-findObjInRoom **TIMEOUT** — [sheet](runs/s2026-038-findObjInRoom/sheet.jpg)
- s2026-039-meetNameAtLocThenFindInRm **TIMEOUT** — [sheet](runs/s2026-039-meetNameAtLocThenFindInRm/sheet.jpg)
