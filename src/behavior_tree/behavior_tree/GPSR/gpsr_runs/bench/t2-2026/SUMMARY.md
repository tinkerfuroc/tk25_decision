# GPSR bench summary

Run: tier=T2, timeout_s=600.0, settle_s=10.0, live_llm=True, seed=2026, timestamp=2026-08-26T18:51:53.769355+00:00, commit=03fe889c8426e002e12cae526ce8988e0b0b048d, corpus=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-2026/corpus.jsonl, invalidated=2026-08-27: OPENROUTER key over monthly quota during the battery; all plans were LLM-planner fallbacks. Verdicts re-scored; rerun required.

| template | class | T2 |
|---|---|---|
| countObjOnPlcmt | A | 0/4 |
| countPrsInRoom | A | 0/4 |
| greetNameInRm | A | 0/2 |
| meetNameAtLocThenFindInRm | A | 0/4 |
| meetPrsAtBeac | A | 0/1 |
| talkInfoToGestPrsInRoom | A | 0/3 |
| tellCatPropOnPlcmt | A | 0/3 |
| tellObjPropOnPlcmt | A | 0/3 |
| bringMeObjFromPlcmt | B | 0/4 |
| findObjInRoom | B | 0/2 |
| takeObjFromPlcmt | B | 0/2 |
| findObjInRoom | C | 0/1 |
| goToLoc | C | 0/1 |
| tellPrsInfoAtLocToPrsAtLoc | C | 0/3 |
| tellPrsInfoInLoc | C | 0/3 |

## Totals

- T2: PASS 0, FAIL 36, TIMEOUT 2, ERROR 2

## Totals by class

T2:
- class A: 0/24
- class B: 0/8
- class C: 0/8
- Class A+B pass rate: 0/32 (0 %)

## Failures

- T2 `s2026-000-bringMeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-000-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2026-001-countObjOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-001-countObjOnPlcmt/sheet.jpg
- T2 `s2026-002-countPrsInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-002-countPrsInRoom/sheet.jpg
- T2 `s2026-003-findObjInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-003-findObjInRoom/sheet.jpg
- T2 `s2026-004-goToLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-004-goToLoc/sheet.jpg
- T2 `s2026-005-greetNameInRm` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-005-greetNameInRm/sheet.jpg
- T2 `s2026-006-meetNameAtLocThenFindInRm` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-006-meetNameAtLocThenFindInRm/sheet.jpg
- T2 `s2026-007-meetPrsAtBeac` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-007-meetPrsAtBeac/sheet.jpg
- T2 `s2026-008-talkInfoToGestPrsInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-008-talkInfoToGestPrsInRoom/sheet.jpg
- T2 `s2026-009-tellCatPropOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-009-tellCatPropOnPlcmt/sheet.jpg
- T2 `s2026-010-tellObjPropOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-010-tellObjPropOnPlcmt/sheet.jpg
- T2 `s2026-011-tellPrsInfoAtLocToPrsAtLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-011-tellPrsInfoAtLocToPrsAtLoc/sheet.jpg
- T2 `s2026-012-tellPrsInfoInLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-012-tellPrsInfoInLoc/sheet.jpg
- T2 `s2026-013-bringMeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-013-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2026-014-countObjOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-014-countObjOnPlcmt/sheet.jpg
- T2 `s2026-015-countPrsInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-015-countPrsInRoom/sheet.jpg
- T2 `s2026-016-findObjInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-016-findObjInRoom/sheet.jpg
- T2 `s2026-017-greetNameInRm` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-017-greetNameInRm/sheet.jpg
- T2 `s2026-018-meetNameAtLocThenFindInRm` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-018-meetNameAtLocThenFindInRm/sheet.jpg
- T2 `s2026-019-takeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-019-takeObjFromPlcmt/sheet.jpg
- T2 `s2026-020-talkInfoToGestPrsInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-020-talkInfoToGestPrsInRoom/sheet.jpg
- T2 `s2026-021-tellCatPropOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-021-tellCatPropOnPlcmt/sheet.jpg
- T2 `s2026-022-tellObjPropOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-022-tellObjPropOnPlcmt/sheet.jpg
- T2 `s2026-023-tellPrsInfoAtLocToPrsAtLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-023-tellPrsInfoAtLocToPrsAtLoc/sheet.jpg
- T2 `s2026-024-tellPrsInfoInLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-024-tellPrsInfoInLoc/sheet.jpg
- T2 `s2026-025-bringMeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-025-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2026-026-countObjOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-026-countObjOnPlcmt/sheet.jpg
- T2 `s2026-027-countPrsInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-027-countPrsInRoom/sheet.jpg
- T2 `s2026-028-meetNameAtLocThenFindInRm` **ERROR** — reset failed: timed out after 60s
- T2 `s2026-029-takeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-029-takeObjFromPlcmt/sheet.jpg
- T2 `s2026-030-talkInfoToGestPrsInRoom` **ERROR** — reset failed: timed out after 60s
- T2 `s2026-031-tellCatPropOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-031-tellCatPropOnPlcmt/sheet.jpg
- T2 `s2026-032-tellObjPropOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-032-tellObjPropOnPlcmt/sheet.jpg
- T2 `s2026-033-tellPrsInfoAtLocToPrsAtLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-033-tellPrsInfoAtLocToPrsAtLoc/sheet.jpg
- T2 `s2026-034-tellPrsInfoInLoc` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-034-tellPrsInfoInLoc/sheet.jpg
- T2 `s2026-035-bringMeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-035-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2026-036-countObjOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-036-countObjOnPlcmt/sheet.jpg
- T2 `s2026-037-countPrsInRoom` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-037-countPrsInRoom/sheet.jpg
- T2 `s2026-038-findObjInRoom` **TIMEOUT** — timed out after 600s | sheet=runs/s2026-038-findObjInRoom/sheet.jpg
- T2 `s2026-039-meetNameAtLocThenFindInRm` **TIMEOUT** — timed out after 600s | sheet=runs/s2026-039-meetNameAtLocThenFindInRm/sheet.jpg

## Runs

- s2026-000-bringMeObjFromPlcmt **FAIL** — [sheet](runs/s2026-000-bringMeObjFromPlcmt/sheet.jpg)
- s2026-001-countObjOnPlcmt **FAIL** — [sheet](runs/s2026-001-countObjOnPlcmt/sheet.jpg)
- s2026-002-countPrsInRoom **FAIL** — [sheet](runs/s2026-002-countPrsInRoom/sheet.jpg)
- s2026-003-findObjInRoom **FAIL** — [sheet](runs/s2026-003-findObjInRoom/sheet.jpg)
- s2026-004-goToLoc **FAIL** — [sheet](runs/s2026-004-goToLoc/sheet.jpg)
- s2026-005-greetNameInRm **FAIL** — [sheet](runs/s2026-005-greetNameInRm/sheet.jpg)
- s2026-006-meetNameAtLocThenFindInRm **FAIL** — [sheet](runs/s2026-006-meetNameAtLocThenFindInRm/sheet.jpg)
- s2026-007-meetPrsAtBeac **FAIL** — [sheet](runs/s2026-007-meetPrsAtBeac/sheet.jpg)
- s2026-008-talkInfoToGestPrsInRoom **FAIL** — [sheet](runs/s2026-008-talkInfoToGestPrsInRoom/sheet.jpg)
- s2026-009-tellCatPropOnPlcmt **FAIL** — [sheet](runs/s2026-009-tellCatPropOnPlcmt/sheet.jpg)
- s2026-010-tellObjPropOnPlcmt **FAIL** — [sheet](runs/s2026-010-tellObjPropOnPlcmt/sheet.jpg)
- s2026-011-tellPrsInfoAtLocToPrsAtLoc **FAIL** — [sheet](runs/s2026-011-tellPrsInfoAtLocToPrsAtLoc/sheet.jpg)
- s2026-012-tellPrsInfoInLoc **FAIL** — [sheet](runs/s2026-012-tellPrsInfoInLoc/sheet.jpg)
- s2026-013-bringMeObjFromPlcmt **FAIL** — [sheet](runs/s2026-013-bringMeObjFromPlcmt/sheet.jpg)
- s2026-014-countObjOnPlcmt **FAIL** — [sheet](runs/s2026-014-countObjOnPlcmt/sheet.jpg)
- s2026-015-countPrsInRoom **FAIL** — [sheet](runs/s2026-015-countPrsInRoom/sheet.jpg)
- s2026-016-findObjInRoom **FAIL** — [sheet](runs/s2026-016-findObjInRoom/sheet.jpg)
- s2026-017-greetNameInRm **FAIL** — [sheet](runs/s2026-017-greetNameInRm/sheet.jpg)
- s2026-018-meetNameAtLocThenFindInRm **FAIL** — [sheet](runs/s2026-018-meetNameAtLocThenFindInRm/sheet.jpg)
- s2026-019-takeObjFromPlcmt **FAIL** — [sheet](runs/s2026-019-takeObjFromPlcmt/sheet.jpg)
- s2026-020-talkInfoToGestPrsInRoom **FAIL** — [sheet](runs/s2026-020-talkInfoToGestPrsInRoom/sheet.jpg)
- s2026-021-tellCatPropOnPlcmt **FAIL** — [sheet](runs/s2026-021-tellCatPropOnPlcmt/sheet.jpg)
- s2026-022-tellObjPropOnPlcmt **FAIL** — [sheet](runs/s2026-022-tellObjPropOnPlcmt/sheet.jpg)
- s2026-023-tellPrsInfoAtLocToPrsAtLoc **FAIL** — [sheet](runs/s2026-023-tellPrsInfoAtLocToPrsAtLoc/sheet.jpg)
- s2026-024-tellPrsInfoInLoc **FAIL** — [sheet](runs/s2026-024-tellPrsInfoInLoc/sheet.jpg)
- s2026-025-bringMeObjFromPlcmt **FAIL** — [sheet](runs/s2026-025-bringMeObjFromPlcmt/sheet.jpg)
- s2026-026-countObjOnPlcmt **FAIL** — [sheet](runs/s2026-026-countObjOnPlcmt/sheet.jpg)
- s2026-027-countPrsInRoom **FAIL** — [sheet](runs/s2026-027-countPrsInRoom/sheet.jpg)
- s2026-028-meetNameAtLocThenFindInRm **ERROR**
- s2026-029-takeObjFromPlcmt **FAIL** — [sheet](runs/s2026-029-takeObjFromPlcmt/sheet.jpg)
- s2026-030-talkInfoToGestPrsInRoom **ERROR**
- s2026-031-tellCatPropOnPlcmt **FAIL** — [sheet](runs/s2026-031-tellCatPropOnPlcmt/sheet.jpg)
- s2026-032-tellObjPropOnPlcmt **FAIL** — [sheet](runs/s2026-032-tellObjPropOnPlcmt/sheet.jpg)
- s2026-033-tellPrsInfoAtLocToPrsAtLoc **FAIL** — [sheet](runs/s2026-033-tellPrsInfoAtLocToPrsAtLoc/sheet.jpg)
- s2026-034-tellPrsInfoInLoc **FAIL** — [sheet](runs/s2026-034-tellPrsInfoInLoc/sheet.jpg)
- s2026-035-bringMeObjFromPlcmt **FAIL** — [sheet](runs/s2026-035-bringMeObjFromPlcmt/sheet.jpg)
- s2026-036-countObjOnPlcmt **FAIL** — [sheet](runs/s2026-036-countObjOnPlcmt/sheet.jpg)
- s2026-037-countPrsInRoom **FAIL** — [sheet](runs/s2026-037-countPrsInRoom/sheet.jpg)
- s2026-038-findObjInRoom **TIMEOUT** — [sheet](runs/s2026-038-findObjInRoom/sheet.jpg)
- s2026-039-meetNameAtLocThenFindInRm **TIMEOUT** — [sheet](runs/s2026-039-meetNameAtLocThenFindInRm/sheet.jpg)
