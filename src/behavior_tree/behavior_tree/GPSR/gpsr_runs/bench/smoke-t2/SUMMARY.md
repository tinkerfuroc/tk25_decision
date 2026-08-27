# GPSR bench summary

Run: tier=T2, timeout_s=600.0, settle_s=10.0, live_llm=True, seed=2026, timestamp=2026-08-26T15:16:35.861764+00:00, commit=03fe889c8426e002e12cae526ce8988e0b0b048d, corpus=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/smoke-t2.jsonl, invalidated=2026-08-27: OPENROUTER key over monthly quota during the battery; all plans were LLM-planner fallbacks. Verdicts re-scored; rerun required.

| template | class | T2 |
|---|---|---|
| countObjOnPlcmt | A | 0/1 |
| bringMeObjFromPlcmt | B | 0/1 |

## Totals

- T2: PASS 0, FAIL 2, TIMEOUT 0, ERROR 0

## Totals by class

T2:
- class A: 0/1
- class B: 0/1
- class C: 0/0
- Class A+B pass rate: 0/2 (0 %)

## Failures

- T2 `s2026-000-bringMeObjFromPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-000-bringMeObjFromPlcmt/sheet.jpg
- T2 `s2026-001-countObjOnPlcmt` **FAIL** — planner exhausted attempts (fallback plan executed); re-scored 2026-08-27 after LLM-key-quota postmortem | sheet=runs/s2026-001-countObjOnPlcmt/sheet.jpg

## Runs

- s2026-000-bringMeObjFromPlcmt **FAIL** — [sheet](runs/s2026-000-bringMeObjFromPlcmt/sheet.jpg)
- s2026-001-countObjOnPlcmt **FAIL** — [sheet](runs/s2026-001-countObjOnPlcmt/sheet.jpg)
