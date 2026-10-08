"""Tier 2: run one GPSR command per orchestrator process against a live sim.

Per entry: reset sim -> start frame recorder -> fresh orchestrator with a single
BT_GPSR_CMD -> verdict from telemetry (task id 1) -> stop recorder -> build a contact
sheet. Reuses tier1's launcher/env/telemetry machinery (``bench_env``, ``_stop``,
``_events_file``) rather than duplicating it.
"""
from __future__ import annotations

import json
import os
import re
import signal
import subprocess
import time
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Sequence

from behavior_tree.GPSR.bench.corpus import CorpusEntry
from behavior_tree.GPSR.bench.events import parse_events, slot_of
from behavior_tree.GPSR.bench.report import BenchResult, pass_streak
from behavior_tree.GPSR.bench.tier1 import DEFAULT_LAUNCHER, _events_file, _stop, bench_env
from behavior_tree.GPSR.bench import truth as _truth

DEFAULT_RESET_CMD = ["ros2", "service", "call", "/reset_simulation",
                     "simulation_interfaces/srv/ResetSimulation", "{}"]

TIER = 2
RECORDER_STOP_TIMEOUT_S = 180.0


def _stop_recorder(proc: subprocess.Popen, *, timeout_s: float | None = None) -> None:
    """Give nested frame and rosbag recorders time to flush after SIGINT."""
    if proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return

    grace = RECORDER_STOP_TIMEOUT_S if timeout_s is None else timeout_s
    try:
        proc.wait(timeout=grace)
        return
    except subprocess.TimeoutExpired:
        pass

    try:
        os.killpg(pgid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        proc.wait()

# Post-run verdict guard: evidence that the planner exhausted its retries and the
# orchestrator executed (or announced) the acknowledgement-only fallback plan instead of a
# real one. A `[plan:...] all N attempts failed` line, or the fallback plan's own announce
# text, means the run's task.finished "succeeded" was really just the fallback plan
# succeeding trivially -- a hollow PASS. A `[split] all N attempts failed` line alone does
# NOT mean this: the deterministic split fallback can still be planned normally, so it is
# recorded as a detail annotation only.
_PLAN_ATTEMPTS_EXHAUSTED_RE = re.compile(r"all \d+ attempts failed")
_PLAN_TARGET_RE = re.compile(r"\[plan:(\d+):(\d+)\]")
_FALLBACK_PLAN_MARKER = "could not work out a complete plan"
PLANNER_EXHAUSTED_DETAIL = "planner exhausted attempts (fallback plan executed)"

# LOW-2 (round-2 review, task H, 2026-08-29): tier2 runs exactly one task (task id 1) per
# orchestrator process/events file -- this is the slot ``slot_of()`` derives from that
# task's ``task_id`` (e.g. "traj-1/task-1"). Not the same axis as a multi-target command's
# per-TARGET ``(slot, index)`` scoping above (MEDIUM-1): this guards against a run dir that
# ever accumulated events from a SECOND TASK mixing into a single-task tier2 check.
_TIER2_TASK_SLOT = 1


def _events_show_recovery(
    events_path: Path, exhausted_targets: "set[tuple[int, int]] | None" = None,
) -> bool:
    """H2 (round-2 review, sim run 003, 2026-08-29): True when the LAST target-outcome
    evidence recorded in ``events_path`` (a ``step.finished`` or ``target.failed`` telemetry
    event -- ``bench/events.py``'s ``parse_events`` folds these into ``TaskResult``, but this
    needs each one IN FILE ORDER, not the folded per-task summary) is a genuine step
    succeeding -- the guaranteed fallback plan's only action is ``announce``
    (``_fallback_plan``, orchestrator.py), so a succeeded ``announce`` is treated as hollow
    UNLESS it is a real, LLM-planned announce.

    Follow-up (round-2 review, task H, 2026-08-29): ``_fallback_plan`` tags its own
    acknowledgement with ``params["acknowledgement"] = True`` specifically so downstream
    consumers can tell it apart from a real announce (see that function's docstring in
    orchestrator.py, and ``validators._action_verdict``'s answered-fallback handling, which
    reads the same flag). ``BtNode_LogStepResult``/the supervisor-failure path both put the
    step's raw ``CURRENT_PARAMS`` dict straight into the ``step.finished`` event's
    ``payload["params"]`` (orchestrator.py's ``update()``/``on_target_failed`` methods), and
    ``Telemetry.emit`` copies ``payload`` verbatim (``_json_safe(dict(payload or {}))``,
    telemetry.py) -- no field is dropped or renamed in between. So a succeeded ``announce``
    event's ``params.acknowledgement`` reliably distinguishes the fallback's hollow
    acknowledgement (flag truthy -- NOT recovery) from a real, LLM-planned ``announce`` step
    such as a question-answering target's final response (flag absent/falsy -- IS recovery,
    same as any other genuine action).

    The events JSONL is append-only and single-writer, so file order is already
    chronological: no cross-file timestamp correlation with the orchestrator.log scan below
    is needed, only "what was the LAST thing that happened" -- PER TARGET, see below.

    MEDIUM-1 (round-2 review, task H, 2026-08-29): a tier2 run is one orchestrator process,
    but a single GPSR command can carry several TARGETS, each independently materialised and
    executed in sequence by the same executor slot. Looking only at the run's globally last
    outcome let a command where target 0 exhausted (hollow ``announce`` fallback) and target 1
    was real and genuinely succeeded LAST score as full recovery, even though target 0's own
    fallback was never actually completed -- a false PASS the exhaustion override should have
    caught. When ``exhausted_targets`` (the ``(slot, index)`` pairs the log's
    ``[plan:slot:index] all N attempts failed`` lines named, slot already converted to
    ``slot_of``'s 1-based ``/task-N`` numbering -- see ``_scan_planner_exhaustion``) is
    non-empty AND at least one ``plan.materialized`` event is actually present in the file,
    this instead tracks the CURRENT target as it walks events in order (each
    ``plan.materialized`` event names the target it just swapped in, via its own ``task_id``
    -- ``slot_of`` -- and ``payload["target_index"]``) and remembers the last genuine-success
    verdict PER TARGET; recovery then holds only when EVERY exhausted target's own last
    evidence is a genuine success. ``exhausted_targets=None``/empty (no ``[plan:slot:index]``
    line could be attributed -- e.g. exhaustion was only signalled by the free-text fallback
    marker) or an events file with no ``plan.materialized`` events at all (nothing to scope
    by -- a live run always emits these from ``_swap_in``, orchestrator.py, but a synthetic
    events file need not) falls back to the original whole-file "what was the last thing that
    happened" check, preserving prior behaviour exactly.

    A trailing failure (``target.failed``, or a ``step.finished`` with
    ``outcome != "succeeded"``), a trailing hollow ``announce`` (the fallback's own
    acknowledgement), or no such events at all all score ``False`` (no evidence of recovery
    -- the caller's override stays in force). Missing/unreadable files score ``False`` too.
    """
    path = Path(events_path)
    if not path.is_file():
        return False
    last_ok = False
    per_target: dict[tuple[int, int], bool] = {}
    current_target: tuple[int, int] | None = None
    saw_materialized = False
    try:
        with path.open("r", encoding="utf-8", errors="replace") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                task_slot = slot_of(event.get("task_id"))
                if task_slot is not None and task_slot != _TIER2_TASK_SLOT:
                    continue
                kind = event.get("event_type")
                payload = event.get("payload") or {}
                if kind == "plan.materialized":
                    slot = slot_of(event.get("task_id"))
                    target_index = payload.get("target_index")
                    if slot is not None and target_index is not None:
                        current_target = (slot, int(target_index))
                        saw_materialized = True
                elif kind == "step.finished":
                    succeeded = payload.get("outcome") == "succeeded"
                    is_hollow_announce = (
                        str(payload.get("action")) == "announce"
                        and bool((payload.get("params") or {}).get("acknowledgement"))
                    )
                    last_ok = succeeded and not is_hollow_announce
                    if current_target is not None:
                        per_target[current_target] = last_ok
                elif kind == "target.failed":
                    last_ok = False
                    if current_target is not None:
                        per_target[current_target] = last_ok
    except OSError:
        return False
    if exhausted_targets and saw_materialized:
        return all(per_target.get(target, False) for target in exhausted_targets)
    return last_ok


def _scan_planner_exhaustion(
    log_path: Path, events_path: Path | None = None, max_bytes: int = 5_000_000,
) -> tuple[bool, bool]:
    """Bounded scan of an orchestrator.log for planner-fallback evidence.

    Returns ``(exhausted, split_fell_back)``. Reads at most the last ``max_bytes`` of the
    file (a stuck/looping run's log can't blow up scoring). Missing/unreadable files score
    as ``(False, False)`` -- absence of evidence is not evidence of exhaustion.

    H2 (round-2 review, sim run 003, 2026-08-29): ``exhausted`` used to flip a PASS to FAIL
    on ANY exhaustion evidence anywhere in the log, even when a LATER replan/escape went on
    to complete every real target (003: goto, search_object, grasp, place, all succeeding
    after one earlier fallback) -- a real recovery scored as a hollow FAIL. When
    ``events_path`` is given and shows the run's LAST recorded outcome was a genuine (real
    action) success (``_events_show_recovery``), the override is dropped even though the log
    still carries the earlier exhaustion evidence -- the fallback was NOT this run's final
    outcome. ``events_path=None`` (a caller with no events file to offer) keeps the pre-H2
    behaviour: any exhaustion evidence still overrides.

    MEDIUM-1 (round-2 review, task H, 2026-08-29): each ``[plan:{slot}:{index}] all N
    attempts failed`` line names the exact target that exhausted (planner.py's ``slot``/
    ``index`` are the same 0-based ``self._slot``/target index the orchestrator passes to
    ``get_target_subtree``); these are collected into ``exhausted_targets`` (slot converted
    to ``slot_of``'s 1-based ``/task-N`` numbering, so it lines up with what
    ``_events_show_recovery`` reads off each ``plan.materialized`` event's own ``task_id``)
    and handed to ``_events_show_recovery`` so recovery is scoped PER TARGET rather than to
    the run's globally last event -- see that function's docstring for why (a multi-target
    command where an earlier target exhausted and a later, unrelated target genuinely
    succeeded must not let the later target's success paper over the earlier one's hollow
    fallback). A line whose ``[plan:...]`` prefix can't be parsed contributes to ``exhausted``
    but not to ``exhausted_targets``, so an empty ``exhausted_targets`` transparently falls
    back to the pre-MEDIUM-1 whole-file check inside ``_events_show_recovery``.
    """
    if not log_path.is_file():
        return False, False
    try:
        size = log_path.stat().st_size
        with log_path.open("r", encoding="utf-8", errors="replace") as f:
            if size > max_bytes:
                f.seek(size - max_bytes)
            text = f.read()
    except OSError:
        return False, False

    exhausted = _FALLBACK_PLAN_MARKER in text
    exhausted_targets: set[tuple[int, int]] = set()
    split_fell_back = False
    for line in text.splitlines():
        if not _PLAN_ATTEMPTS_EXHAUSTED_RE.search(line):
            continue
        if "[split]" in line:
            split_fell_back = True
        elif "[plan:" in line:
            exhausted = True
            match = _PLAN_TARGET_RE.search(line)
            if match:
                exhausted_targets.add((int(match.group(1)) + 1, int(match.group(2))))
    if exhausted and events_path is not None and _events_show_recovery(events_path, exhausted_targets):
        exhausted = False
    return exhausted, split_fell_back


def llm_preflight(env: dict[str, str]) -> tuple[bool, str]:
    """One minimal chat completion through the SAME client construction the planner uses.

    Catches an exhausted/invalid OpenRouter key BEFORE a whole tier-2 battery burns through
    every entry as fallback-plan runs that would otherwise each score a hollow PASS.

    ``env`` is the environment the orchestrator subprocesses will run under (normally
    ``dict(os.environ)``); an ``OPENROUTER_API_KEY``/``OPENAI_API_KEY`` there overrides the
    planner's own resolved key, mirroring what the subprocess would actually see.

    Never raises (catches everything -- a preflight probe must not itself crash the
    battery) and never returns/logs the key itself, only the provider's error text with the
    key value scrubbed out if it somehow appears there.
    """
    try:
        import openai
        from behavior_tree.GPSR.planner import OPENAI_API_KEY as _default_key, OPENAI_MODEL
    except Exception as exc:  # pragma: no cover - import machinery
        return False, f"preflight import error: {exc!r}"

    api_key = env.get("OPENROUTER_API_KEY") or env.get("OPENAI_API_KEY") or _default_key
    try:
        client = openai.OpenAI(api_key=api_key, base_url="https://openrouter.ai/api/v1", timeout=30.0)
        client.chat.completions.create(
            model=OPENAI_MODEL,
            messages=[{"role": "user", "content": "hi"}],
            max_tokens=1,
        )
    except Exception as exc:  # noqa: BLE001 - catch everything, never crash the battery
        msg = repr(exc)
        if api_key and api_key in msg:
            msg = msg.replace(api_key, "***")
        return False, msg
    return True, ""


def _substitute(cmd: Sequence[str], mapping: dict[str, str]) -> list[str]:
    out = []
    for arg in cmd:
        for key, val in mapping.items():
            arg = arg.replace("{" + key + "}", val)
        out.append(arg)
    return out


def _run_hook(cmd: Sequence[str], mapping: dict[str, str] | None, *, timeout: float, label: str,
             tail_bytes: int | None = None) -> str | None:
    """Run one external hook command; return an error detail string, or None on success.

    Shared by ``_reset``/``_run_spawn_cmd``/``_run_clear_cmd`` (all three had near-identical
    bodies: substitute -> subprocess.run -> classify timeout/OSError/nonzero-exit into a
    ``"{label} failed: ..."`` string). ``mapping is None`` skips ``_substitute`` entirely,
    preserving ``_reset``'s original behaviour of running ``reset_cmd`` verbatim (it takes no
    per-run substitution keys). ``tail_bytes`` truncates stderr to its last N characters before
    it's embedded in the error string (spawn/clear cap it; ``_reset`` does not, matching its
    prior untruncated behaviour).
    """
    resolved = _substitute(cmd, mapping) if mapping is not None else list(cmd)
    try:
        result = subprocess.run(resolved, timeout=timeout, capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return f"{label} failed: timed out after {int(timeout)}s"
    except OSError as exc:
        return f"{label} failed: {exc}"
    if result.returncode != 0:
        stderr = (result.stderr or "").strip()
        if tail_bytes is not None:
            stderr = stderr[-tail_bytes:]
        suffix = f": {stderr}" if stderr else ""
        return f"{label} failed: exit code {result.returncode}{suffix}"
    return None


def _reset(reset_cmd: Sequence[str]) -> str | None:
    """Run the reset command; return an error detail string, or None on success."""
    return _run_hook(reset_cmd, None, timeout=180, label="reset")


def _partial_spawn_is_usable(manifest_path: Path) -> bool:
    """True when a non-zero spawn still left at least one requested object in
    the scene (spawned ok, or already present in the base scenario) and every
    failure was a clean "not attempted: no room ..." refusal, not a sim error."""
    try:
        manifest = json.loads(Path(manifest_path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return False
    entities = manifest.get("entities") or []
    if any(not e.get("ok", True) for e in entities):
        return False
    refused = manifest.get("not_attempted") or []
    if not all(str(r.get("error", "")).startswith("not attempted: no room") for r in refused):
        return False
    return bool(entities) or bool(manifest.get("skipped"))


def _run_spawn_cmd(spawn_cmd: Sequence[str], mapping: dict[str, str]) -> str | None:
    """Run the scene-spawn command for one run; return an error detail
    string, or None on success. Mirrors _reset's shape."""
    return _run_hook(spawn_cmd, mapping, timeout=120, label="spawn", tail_bytes=2000)


def _run_clear_cmd(clear_cmd: Sequence[str], mapping: dict[str, str]) -> str | None:
    """Run the scene-clear command for one run; return an error detail
    string, or None on success."""
    return _run_hook(clear_cmd, mapping, timeout=60, label="clear", tail_bytes=2000)


def _apply_grasp_truth_gate(verdict: str, detail: str, run_dir: Path) -> tuple[str, str]:
    """Bench-oracle gate: downgrade a PASS to FAIL when sim physics truth shows
    that no object was ever actually retained by the gripper -- i.e. the run
    "succeeded" on a closed-on-air grasp and the BT/referee belief was a false
    positive (soup-can run 20260908T062739: verdict PASS / referee_assisted=false
    while the can never moved and was never gripped).

    Reads ``run_dir/grasp_truth.json`` written by the grasp-truth probe
    (``tools/grasp_truth_probe.py`` in the sim worktree), which subscribes to the
    sim's ``/sim/truth/object_state`` (retained_by_gripper) for the run. This is a
    BENCH ORACLE used strictly AFTER THE FACT to score the run -- never a runtime
    input of the stack or the behaviour tree (the hardware-parity / no-sim-only-
    guards contract applies to runtime, not to post-run scoring).

    Graceful and conservative: only a PASS is ever downgraded, and only when the
    probe wrote a definite ``any_object_retained == False``. A missing/unparseable
    file, or a ``None``/absent field (no probe, or probe couldn't decide), leaves
    the verdict untouched -- so runs without the probe score exactly as before.

    Subsumed by the physical-truth verdict (``bench/truth.py``, ``_physical_verdict``
    below): ``truth.load_truth`` lifts ``grasp_truth.json`` into the ``run_truth.json``
    schema so a ``held()`` check FAILs on the same evidence. This gate is kept as the
    final safety net for runs whose events carry no ``split.accepted`` targets (nothing
    for the per-fact checks to score) and is applied AFTER the combination, so a
    closed-on-air PASS can never survive either path.
    """
    if verdict != "PASS":
        return verdict, detail
    try:
        data = json.loads((Path(run_dir) / "grasp_truth.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError, ValueError):
        return verdict, detail  # no truth captured -> leave the verdict unchanged
    if data.get("any_object_retained") is False:  # explicit False only
        note = ("grasp false-positive: no object was ever retained by the gripper "
                "per sim truth (closed on air)")
        disp = data.get("max_xy_disp_m")
        if isinstance(disp, (int, float)):
            note += f"; target moved only {disp * 1000:.0f}mm"
        return "FAIL", note
    return verdict, detail


def _physical_verdict(run_dir: Path, planner_verdict: str, detail: str, referee_assisted: bool,
                      constants: Path | dict | None) -> dict:
    """Score the run's split-target postconditions against sim physics truth and combine
    with the planner verdict (``bench/truth.py``).

    Returns ``{"verdict", "detail", "planner_verdict", "physical_verdict",
    "physical_checks", "physical_detail"}``. The combination rule
    (``truth.combine_verdicts``): a non-PASS planner verdict stands as-is; a planner PASS
    becomes the physical verdict when physics decided (PASS/FAIL/REFEREE) and stays PASS
    with ``physical_detail = "physical truth unavailable"`` otherwise. A physical FAIL
    replaces ``detail`` with the failing checks' reasons (the prior detail, e.g. a clear
    error, is kept after a ``" | "``). The legacy ``_apply_grasp_truth_gate`` runs last as
    a safety net (see its docstring). Never raises: any unexpected error while reading
    truth/events scores physical UNKNOWN with the exception in ``physical_detail``.
    """
    run_dir = Path(run_dir)
    # Preserve planner-side evidence before physical checks append their own
    # diagnostics to the legacy combined ``detail`` field.
    planner_detail = detail
    try:
        targets, gates = _truth.read_split_targets(run_dir, _TIER2_TASK_SLOT)
        # ``task.finished=succeeded`` is an executor status, not proof that the
        # materialized plan matched the command.  Audit the captured action plans
        # before combining physical truth so missing announce/approach steps are
        # recorded as planner failures as well as execution failures.
        plan_audit = _truth.plan_execution_audit(run_dir, targets,
                                                 task_slot=_TIER2_TASK_SLOT)
        if planner_verdict == "PASS" and plan_audit.get("verdict") == "FAIL":
            reasons = [str(item.get("reason")) for item in plan_audit.get("issues", [])]
            planner_verdict = "FAIL"
            audit_detail = "planner audit: " + "; ".join(reasons)
            detail = audit_detail + (" | " + detail if detail else "")
            planner_detail = detail
        checks = _truth.physical_checks(run_dir, targets, constants=constants, gate_verdicts=gates)
        # Startup/replan loops are execution-integrity failures even when every
        # individual action eventually reports success.  Add a synthetic physical
        # check so the failure is visible in the viewer and breaks a clean streak.
        integrity = _truth.execution_integrity_audit(run_dir, task_slot=_TIER2_TASK_SLOT)
        if integrity.get("verdict") == "FAIL":
            reasons = [str(item.get("reason")) for item in integrity.get("issues", [])]
            checks.append({"fact": "execution_integrity", "verdict": "FAIL",
                           "reason": "; ".join(reasons), "critical": True,
                           "target": -1})
        physical, verdict, physical_detail = _truth.combine_verdicts(
            planner_verdict, checks, referee_assisted)
    except Exception as exc:  # noqa: BLE001 - scoring must never crash the battery
        checks = []
        physical, verdict = "UNKNOWN", planner_verdict
        physical_detail = f"{_truth.PHYSICAL_TRUTH_UNAVAILABLE} (error: {exc!r})"
    if verdict == "FAIL" and planner_verdict == "PASS":
        detail = physical_detail + (" | " + detail if detail else "")
    gated_verdict, gated_detail = _apply_grasp_truth_gate(verdict, detail, run_dir)
    if gated_verdict != verdict:
        verdict, detail = gated_verdict, gated_detail
        physical, physical_detail = "FAIL", gated_detail
    return {"verdict": verdict, "detail": detail, "planner_verdict": planner_verdict,
            "physical_verdict": physical, "physical_checks": checks,
            "physical_detail": physical_detail, "planner_detail": planner_detail}


def _run_orchestrator(*, env: dict[str, str], run_dir: Path, launcher: Sequence[str],
                      timeout_s: float, poll_s: float = 1.0,
                      ) -> tuple[str, str, float, list[str], Path | None, bool]:
    """Launch the orchestrator for a single command and score task id 1.

    This is tier1's ``run_group`` per-task clock semantics specialised to n=1: slot 0's
    clock starts at launch (same as ``run_group``'s ``slot_clock_start[0] = started_mono``),
    so ``timeout_s`` is a plain wall-clock budget here.

    H2 (round-2 review): the return tuple now also carries the resolved ``events_path`` (or
    ``None`` if it was never located) -- the post-run verdict guard (``_scan_planner_
    exhaustion``, called from ``run_tier2``) needs it to check for later recovery evidence.

    K3 (task-K, live-manipulation sim findings, F2): also carries
    ``referee_assisted`` -- True iff any of task 1's ``step_methods`` (bench/
    events.py) was ``"referee_fallback"``, so a referee-assisted grasp does
    not hide behind an indistinguishable PASS.
    """
    run_dir = Path(run_dir).resolve()
    tasks: dict[int, object] = {}
    exit_code = None
    timed_out = False
    with (run_dir / "orchestrator.log").open("a", encoding="utf-8") as log:
        started = time.time()
        started_mono = time.monotonic()
        proc = subprocess.Popen(list(launcher), env=env, stdout=log, stderr=subprocess.STDOUT,
                                cwd=str(run_dir), start_new_session=True)
        try:
            deadline = started_mono + timeout_s
            while time.monotonic() < deadline:
                events = _events_file(run_dir, started - 1)
                if events:
                    tasks = parse_events(events)
                    task = tasks.get(1)
                    if task is not None and task.status:
                        break
                exit_code = proc.poll()
                if exit_code is not None:
                    break
                time.sleep(poll_s)
            else:
                timed_out = True
        finally:
            _stop(proc)
        # The orchestrator may flush its final events during the SIGINT/SIGKILL window
        # _stop() just waited through; re-parse once more (mirrors run_group).
        events = _events_file(run_dir, started - 1)
        if events:
            tasks = parse_events(events)
        events_path = events

    _write_announcements(run_dir)

    task = tasks.get(1)
    plan = [a for a, _ in task.steps] if task else []
    referee_assisted = bool(task and any(m == "referee_fallback" for _, m in task.step_methods))
    seconds = time.time() - started
    if task and task.status == "succeeded":
        verdict, detail = "PASS", ""
    elif task and task.status:
        verdict, detail = "FAIL", str(task.reason or task.status)
    elif exit_code is not None:
        verdict, detail = "ERROR", f"orchestrator exited with exit code {exit_code}"
    elif timed_out:
        verdict, detail = "TIMEOUT", f"timed out after {timeout_s:.0f}s"
    else:
        # Defensive fallback; not a normal path (mirrors run_group's own fallback branch).
        verdict, detail = "TIMEOUT", f"timed out after {timeout_s:.0f}s"
    # The verdict returned here is the PLANNER verdict (executor belief). Physics truth
    # (``_physical_verdict``, incl. the legacy grasp-truth gate) is applied by ``run_tier2``
    # after the planner-exhaustion scan, so both belief-side guards run first.
    return verdict, detail, seconds, plan, events_path, referee_assisted


_ANNOUNCE_RE = re.compile(r"Finished announcing (.+?)(?:\.\s*)?$")
_TREE_ANNOUNCE_RE = re.compile(
    r"-->\s+(?P<name>announce\b[^\[]*?)\s+\[(?P<status>[^\]]+)\]"
    r"(?:\s+--\s+Finished announcing\s+(?P<text>.*))?$",
    re.IGNORECASE,
)


def _speech_events(run_dir: Path) -> list[str]:
    """Return successful speech texts from structured telemetry, in event order."""
    events = _truth._event_records(Path(run_dir))
    out: list[tuple[int, str]] = []
    for ordinal, event in enumerate(events):
        if event.get("event_type") not in {"speech.finished", "speech.spoken", "announce.finished"}:
            continue
        payload = event.get("payload") or {}
        if not isinstance(payload, dict):
            continue
        if str(payload.get("outcome", "succeeded")).lower() not in {"succeeded", "success", "spoken"}:
            continue
        text = str(payload.get("text") or payload.get("message") or "").strip()
        if text:
            try:
                sequence = int(event.get("sequence", ordinal))
            except (TypeError, ValueError):
                sequence = ordinal
            out.append((sequence, text))
    return [text for _, text in sorted(out, key=lambda item: item[0])]


def _snapshot_announcements(log_lines: list[str]) -> list[str]:
    """Extract announcements from BT snapshots using SUCCESS transitions.

    The console renderer prints old feedback in every snapshot.  Track each
    announce node's state and only emit its text on a transition into SUCCESS.
    The occurrence index keeps same-named nodes in separate branches distinct.
    """
    blocks: list[list[str]] = []
    block: list[str] = []
    for line in log_lines:
        if "{-} GPSR orchestrator" in line and block:
            blocks.append(block)
            block = []
        block.append(line)
    if block:
        blocks.append(block)
    if len(blocks) <= 1:
        return []
    previous: dict[tuple[str, int], str] = {}
    out: list[str] = []
    for snapshot in blocks:
        occurrences: dict[str, int] = {}
        current: dict[tuple[str, int], tuple[str, str]] = {}
        for line in snapshot:
            match = _TREE_ANNOUNCE_RE.search(line.rstrip())
            if not match:
                continue
            name = re.sub(r"\s+", " ", match.group("name").strip())
            occurrence = occurrences.get(name, 0)
            occurrences[name] = occurrence + 1
            key = (name, occurrence)
            status = match.group("status").strip()
            text = (match.group("text") or "").strip()
            current[key] = (status, text)
        for key, (status, text) in current.items():
            if status == "✓" and previous.get(key) != "✓" and text:
                out.append(text.rstrip("."))
            previous[key] = status
    return out


def _write_announcements(run_dir: Path) -> None:
    """Extract everything the robot announced into ``announcements.txt``.

    The user-facing record of Tinker's spoken responses for human
    verification: one line per announcement, in log order, deduplicated
    only when the same text repeats consecutively (the BT re-logs a
    finished announce on several ticks). Sourced from orchestrator.log;
    best-effort — a missing/unreadable log writes nothing.
    """
    # Audio nodes emit one speech.finished event per activation.  Prefer it to
    # console output because it is independent of the visualization snapshots.
    structured = _speech_events(run_dir)
    if structured:
        (run_dir / "announcements.txt").write_text(
            "\n".join(structured) + "\n", encoding="utf-8")
        return
    log = run_dir / "orchestrator.log"
    if not log.is_file():
        return
    try:
        log_lines = log.read_text(encoding="utf-8", errors="ignore").splitlines()
    except OSError:
        return
    lines = _snapshot_announcements(log_lines)
    if not lines:
        # Plain speech traces from older recorders have no tree headers.  Keep
        # the historical consecutive-dedup behavior for that format only.
        last = None
        for raw in log_lines:
            m = _ANNOUNCE_RE.search(raw.rstrip())
            if not m:
                continue
            text = m.group(1).strip().rstrip(".")
            if text and text != last:
                lines.append(text)
                last = text
    if lines:
        (run_dir / "announcements.txt").write_text(
            "\n".join(lines) + "\n", encoding="utf-8")


def _write_run_json(run_dir: Path, entry: CorpusEntry, tier_label: str, result: BenchResult) -> None:
    data = {
        "id": entry.id, "text": entry.text, "template": entry.template,
        "feasibility": entry.feasibility, "tier": tier_label, "verdict": result.verdict,
        "detail": result.detail, "seconds": result.seconds,
        "plan": list(result.plan),
        "referee_assisted": result.referee_assisted,
    }
    if result.planner_verdict is not None or result.physical_verdict is not None:
        data["planner_verdict"] = result.planner_verdict
        data["planner_detail"] = result.planner_detail
        data["physical_verdict"] = result.physical_verdict
        data["physical_detail"] = result.physical_detail
        data["physical_checks"] = list(result.physical_checks)
    scene = {}
    if (run_dir / "scene-plan.json").is_file():
        scene["plan"] = "scene-plan.json"
    if (run_dir / "spawned.json").is_file():
        scene["spawned"] = "spawned.json"
    if scene:
        data["scene"] = scene
    (run_dir / "run.json").write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def _halted(results: list[BenchResult], out_dir: Path, halt_after_errors: int) -> bool:
    if halt_after_errors <= 0 or len(results) < halt_after_errors:
        return False
    tail = results[-halt_after_errors:]
    if not all(r.verdict == "ERROR" for r in tail):
        return False
    reason = f"{halt_after_errors} consecutive ERROR results (last: {tail[-1].entry_id}: {tail[-1].detail})"
    (Path(out_dir) / "HALTED").write_text(reason + "\n", encoding="utf-8")
    return True


def _write_progress(out_dir: Path, results: Sequence[BenchResult], *, expected: int,
                    status: str = "running") -> None:
    """Atomically checkpoint completed tier-2 results after every command.

    A live run can be interrupted by a simulator failure or terminal timeout before
    ``gpsr_bench._finish`` gets to write ``report.json``.  Keeping this small,
    append-safe checkpoint means the completed commands and their clean planner /
    physical streak remain auditable and a subsequent run can resume from the next
    corpus index.  The final report still comes from ``write_report``; this file is
    intentionally separate so an interrupted run cannot be mistaken for completion.
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    data = {
        "status": status,
        "updated": datetime.now(timezone.utc).isoformat(),
        "completed": len(results),
        "expected": expected,
        "streak": pass_streak(results),
        "results": [asdict(result) for result in results],
    }
    tmp = out_dir / "progress.json.tmp"
    try:
        tmp.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
        tmp.replace(out_dir / "progress.json")
    except OSError:
        # Checkpointing is diagnostic; never turn a valid benchmark result into
        # an ERROR because the output volume is temporarily unwritable.
        try:
            tmp.unlink(missing_ok=True)
        except OSError:
            pass


def run_tier2(entries: Sequence[CorpusEntry], *, mock_config: Path, constants: Path, out_dir: Path,
              timeout_s: float, tier_label: str = "T2",
              launcher: Sequence[str] = DEFAULT_LAUNCHER, reset_cmd: Sequence[str] = DEFAULT_RESET_CMD,
              recorder_cmd: list[str] | None = None,
              sheet_cmd: list[str] | None = None,
              spawn_cmd: list[str] | None = None,
              clear_cmd: list[str] | None = None,
              settle_s: float = 10.0, halt_after_errors: int = 3,
              live_llm: bool = True, llm_check: bool = True) -> list[BenchResult]:
    out_dir = Path(out_dir)
    runs_root = out_dir / "runs"
    results: list[BenchResult] = []
    _write_progress(out_dir, results, expected=len(entries))

    if llm_check and live_llm:
        ok, preflight_detail = llm_preflight(dict(os.environ))
        if not ok:
            out_dir.mkdir(parents=True, exist_ok=True)
            (out_dir / "HALTED").write_text(
                f"LLM preflight failed: {preflight_detail}\n", encoding="utf-8")
            return []

    for entry in entries:
        run_dir = (runs_root / entry.id).resolve()
        run_dir.mkdir(parents=True, exist_ok=True)

        reset_error = _reset(reset_cmd)
        if reset_error is not None:
            result = BenchResult(entry.id, entry.template, entry.feasibility, TIER, "ERROR", reset_error)
            _write_run_json(run_dir, entry, tier_label, result)
            results.append(result)
            _write_progress(out_dir, results, expected=len(entries))
            if _halted(results, out_dir, halt_after_errors):
                break
            continue

        if settle_s > 0:
            time.sleep(settle_s)

        scene_mapping = {
            "run_dir": str(run_dir), "command": entry.text, "seed": str(entry.seed),
            "plan": str(run_dir / "scene-plan.json"), "manifest": str(run_dir / "spawned.json"),
        }
        if spawn_cmd:
            spawn_error = _run_spawn_cmd(spawn_cmd, scene_mapping)
            if spawn_error is not None and _partial_spawn_is_usable(run_dir / "spawned.json"):
                # fix6 run 003: the sugar box spawned, the soup can was refused
                # ("no room on 'side_table_02'") and the whole run scored ERROR.
                # A partial scene still exercises the command (one "food" is
                # enough to find/fetch; counts are scored against what actually
                # spawned), so run it and record the refusal instead.
                print(f"[tier2] partial spawn, continuing: {spawn_error}", flush=True)
                spawn_error = None
            if spawn_error is not None:
                detail = spawn_error
                if clear_cmd:
                    # A scene may have partially spawned before the failure; run clear_cmd
                    # once so the sim isn't left dirty for the next entry's reset. Its own
                    # failure is appended to the ERROR detail, not scored separately.
                    clear_error = _run_clear_cmd(clear_cmd, scene_mapping)
                    if clear_error is not None:
                        detail = detail + " | " + clear_error
                result = BenchResult(entry.id, entry.template, entry.feasibility, TIER, "ERROR",
                                     detail)
                _write_run_json(run_dir, entry, tier_label, result)
                results.append(result)
                _write_progress(out_dir, results, expected=len(entries))
                if _halted(results, out_dir, halt_after_errors):
                    break
                continue

        recorder_proc = None
        recorder_log = None
        clear_error: str | None = None
        try:
            if recorder_cmd:
                cmd = _substitute(recorder_cmd, {"run_dir": str(run_dir)})
                recorder_log = (run_dir / "recorder.log").open("a", encoding="utf-8")
                recorder_proc = subprocess.Popen(cmd, stdout=recorder_log, stderr=subprocess.STDOUT,
                                                 start_new_session=True)

            env = bench_env(mock_config=mock_config, constants=constants, plan_dir=run_dir,
                            commands=[entry.text], live_llm=live_llm,
                            # gpsr-rcw2026 starts Tinker at the command point
                            # inside the arena; do not run the physical door
                            # entry conversation in the live simulator.
                            start_outside_arena=False,
                            # Preserve the caller's explicit identity mode.
                            # Tier 1 remains relaxed by default, while a live
                            # tier-2 run can require descriptor-specific
                            # recipient selection.
                            sim_identity_relaxed=(
                                os.environ.get("GPSR_SIM_IDENTITY_RELAXED") == "1"
                            ))
            verdict, detail, seconds, plan, events_path, referee_assisted = _run_orchestrator(
                env=env, run_dir=run_dir, launcher=launcher, timeout_s=timeout_s)
        except Exception as exc:
            # Unexpected exception in a single run: score as ERROR with detail, continue batch.
            # This exception source is typically Popen(launcher) raising OSError for unexecutable
            # binary (which occurs outside _run_orchestrator's own try/finally), but guards all
            # unexpected exceptions in this span.
            verdict, detail, seconds, plan, events_path, referee_assisted = (
                "ERROR", f"exception: {exc!r}", 0.0, [], None, False,
            )
        finally:
            if recorder_proc is not None:
                _stop_recorder(recorder_proc)
            if recorder_log is not None:
                recorder_log.close()
            if clear_cmd:
                clear_error = _run_clear_cmd(clear_cmd, scene_mapping)

        if clear_error is not None:
            detail = (detail + " | " if detail else "") + clear_error

        if verdict == "PASS":
            exhausted, split_fell_back = _scan_planner_exhaustion(
                run_dir / "orchestrator.log", events_path)
            if exhausted:
                verdict, detail = "FAIL", PLANNER_EXHAUSTED_DETAIL
            if split_fell_back:
                detail = detail + " | split fell back"

        # Physical-truth verdict: the planner verdict above is the executor's BELIEF;
        # only sim physics (run_truth.json) can confirm a PASS. ERROR/TIMEOUT/FAIL stand.
        scored = _physical_verdict(run_dir, verdict, detail, referee_assisted, constants)
        result = BenchResult(entry.id, entry.template, entry.feasibility, TIER, scored["verdict"],
                             scored["detail"], seconds, plan, referee_assisted,
                             planner_verdict=scored["planner_verdict"],
                             physical_verdict=scored["physical_verdict"],
                             physical_checks=scored["physical_checks"],
                             physical_detail=scored["physical_detail"],
                             planner_detail=scored["planner_detail"])
        _write_run_json(run_dir, entry, tier_label, result)

        if sheet_cmd:
            run_json_path = run_dir / "run.json"
            sheet_out = run_dir / "sheet.jpg"
            cmd = _substitute(sheet_cmd, {"run_dir": str(run_dir), "run_json": str(run_json_path),
                                          "out": str(sheet_out)})
            try:
                sheet_result = subprocess.run(cmd, timeout=120, capture_output=True, text=True)
                ok = sheet_result.returncode == 0
            except (subprocess.TimeoutExpired, OSError):
                ok = False
            if ok:
                rel = os.path.relpath(sheet_out, out_dir)
                result.detail = (result.detail + " | " if result.detail else "") + f"sheet={rel}"
            else:
                result.detail = result.detail + "; sheet failed"

        results.append(result)
        _write_progress(out_dir, results, expected=len(entries))
        if _halted(results, out_dir, halt_after_errors):
            break

    _write_progress(out_dir, results, expected=len(entries),
                    status="complete" if len(results) >= len(entries) else "halted")
    return results
