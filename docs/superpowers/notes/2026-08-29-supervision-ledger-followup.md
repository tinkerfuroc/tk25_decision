# Known follow-up: recovery ledger lags one cycle

Status: not yet implemented. F1 (wall-clock recovery budget + ledger "still-executing
counts as failed" dedupe) was reverted in the Task F fix round
(commits reverting `1857350` and `ce4c169`) because both parts of the fix introduced
worse regressions than the bug they targeted (see round-2 review, H3/H4/M1). This note
records the underlying bug and the reviewer's suggested design so a future attempt does
not have to re-derive it.

## The bug

`RecoveryLedger.exhausted()` (`supervision/recovery.py`) counts `mark_result`-recorded
outcomes for an `issue_id`. `SupervisedSubtaskSlot` only calls `mark_result` for attempt
*N* once attempt *N*'s resolution is known — but the exhaustion check that decides
whether to escalate to a global replan runs at the *start* of the next checkpoint (for
attempt N+1), before attempt N's own failure has been marked. So the exhaustion check
always sees the ledger as it stood after recovery *N-1*, one cycle behind reality:
a still-in-flight (or just-failed-but-not-yet-marked) attempt is invisible to the count
that is supposed to be capping it. In sim run 005 this let `find_person` local recovery
cycle far past the nominal `max_distinct_failures=3` cap before the ledger's view caught
up enough to trip escalation.

## Why the F1 attempt at a fix regressed

- The ledger-lag fix (`ce4c169`, "count a still-executing local_recovery attempt as
  failed before the next exhaustion check") deduped by `issue_id`
  (`_already_escalated` / `_exhausted_issues`), which is process-global and keyed on
  `subtask_goal`/effect/category/target/location — not plan revision. Once an issue was
  escalated once, every later checkpoint that happened to produce the *same* issue id
  (a replacement plan re-emitting the same `find_person(...)`, the same step recurring
  in a later target) silently short-circuited to a no-op: the record was left in stage
  `"verifying"` with no resolution, and `SupervisedEffect` waited on it forever (H4).
- The wall-clock budget (`1857350`) added a hard stop, but nothing in the orchestrator
  consumes the resulting `gpsr/task_outcome` while the tree is still ticking, so the
  slot just spun `RUNNING` forever once the deadline passed (H3) — and because the
  deadline armed unconditionally on first activation (including in SHADOW mode, and for
  actions that had never failed at all), it also hard-stopped perfectly healthy
  long-running actions like `goto`/`follow_person`/`guide` (M1).

## The reviewer's design for a future fix

1. **Dedupe per record, not per issue.** `_start_local` should always resolve the fresh
   checkpoint record (propose a local strategy or call `_start_global`); only
   `recovery_finished` should consult `_exhausted_issues` (set by `_start_local`) to
   skip its own duplicate escalation of the *previous* checkpoint's attempt. This keeps
   each new checkpoint answerable while still preventing a double escalation for the
   attempt that triggered exhaustion.
2. **Clear `_exhausted_issues`** (and consider clearing/rotating the ledger) whenever a
   global replan is applied or a new task starts, so a legitimately-retried action in a
   fresh plan is not permanently poisoned by a stale issue id from a previous plan
   revision.
3. **Use an inline (synchronous) executor for tests.** `MissionSupervisor` should accept
   an `executor=` override; a test executor whose `submit()` runs the callable
   synchronously and returns a completed `Future` makes recovery round-trips complete
   within a single `poll()` call, removing the timing-bounded polling loops (with sleeps
   and tick caps) that made the F1 tests flaky under load.
4. If a wall-clock ceiling is still wanted, it should escalate through the supervisor
   (release the slot via a real intervention / FAILURE with a feedback message) rather
   than hard-stopping into an unconsumed state, should arm only on the first *failure*
   checkpoint (not first activation), and should be skipped outside
   `SupervisionMode.ACTIVE`.

See `/home/tinker/.claude/jobs/5b873e5d/tmp/round2/task-F-review.md` (H3, H4, M1, M2) for
the full review this note summarises.
