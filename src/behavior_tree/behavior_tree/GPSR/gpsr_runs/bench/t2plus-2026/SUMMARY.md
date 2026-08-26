# GPSR bench summary

Corpus: `/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2plus-2026/corpus.jsonl`

Run: tier=T2+, timeout_s=900.0, settle_s=10.0, live_llm=True, seed=2027, timestamp=2026-08-26T21:21:37.143239+00:00, commit=03fe889c8426e002e12cae526ce8988e0b0b048d, corpus=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2plus-2026/corpus.jsonl

| template | class | T2 |
|---|---|---|
| bringMeObjFromPlcmt | B | 0/4 |
| findObjInRoom | B | 0/2 |
| takeObjFromPlcmt | B | 0/3 |
| findObjInRoom | C | 0/1 |

## Totals

- T2: PASS 0, FAIL 0, TIMEOUT 9, ERROR 1

## Totals by class

T2:
- class A: 0/0
- class B: 0/9
- class C: 0/1
- Class A+B pass rate: 0/9 (0 %)

## Failures

- T2 `s2027-000-bringMeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-000-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2027-001-findObjInRoom` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-001-findObjInRoom/sheet.jpg
- T2 `s2027-002-takeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-002-takeObjFromPlcmt/sheet.jpg
- T2 `s2027-003-bringMeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-003-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2027-004-findObjInRoom` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-004-findObjInRoom/sheet.jpg
- T2 `s2027-005-takeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-005-takeObjFromPlcmt/sheet.jpg
- T2 `s2027-006-bringMeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-006-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2027-007-takeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-007-takeObjFromPlcmt/sheet.jpg
- T2 `s2027-008-bringMeObjFromPlcmt` **TIMEOUT** — timed out after 900s | sheet=runs/s2027-008-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2027-009-findObjInRoom` **ERROR** — reset failed: timed out after 60s

## Runs

- s2027-000-bringMeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-000-bringMeObjFromPlcmt/sheet.jpg)
- s2027-001-findObjInRoom **TIMEOUT** — [sheet](runs/s2027-001-findObjInRoom/sheet.jpg)
- s2027-002-takeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-002-takeObjFromPlcmt/sheet.jpg)
- s2027-003-bringMeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-003-bringMeObjFromPlcmt/sheet.jpg)
- s2027-004-findObjInRoom **TIMEOUT** — [sheet](runs/s2027-004-findObjInRoom/sheet.jpg)
- s2027-005-takeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-005-takeObjFromPlcmt/sheet.jpg)
- s2027-006-bringMeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-006-bringMeObjFromPlcmt/sheet.jpg)
- s2027-007-takeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-007-takeObjFromPlcmt/sheet.jpg)
- s2027-008-bringMeObjFromPlcmt **TIMEOUT** — [sheet](runs/s2027-008-bringMeObjFromPlcmt/sheet.jpg)
- s2027-009-findObjInRoom **ERROR**
