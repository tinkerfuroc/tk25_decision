"""Per-command execution logger for the GPSR orchestrator.

Writes one human-readable log file per command so a failed grasp / nav / plan
can be diagnosed after the fact. For each command it records, in order:

  * the command text,
  * the plan the LLM produced (or a FALLBACK marker),
  * every step's SUCCESS / FAIL as it completes,
  * each self-correction (re-plan) round, and — crucially —
  * the ``feedback_message`` of the deepest node that FAILED.

That last line is what pinpoints a grasp stall: it shows whether the *arm move*
("arm to scan: Move arm failed ..."), the *vision scan* ("verify objects found:
counted 0 objects"), the *navigation*, or the *planner* is the culprit — instead
of just the generic "I had trouble with that step" the robot speaks.

Wire it as an extra post-tick handler in the orchestrator entry points and
combine it with the visualizer via :func:`combine_post_tick_handlers`.
"""

from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Optional, Tuple

import py_trees
from py_trees.common import Access, Status

from .small_trees import bb_keys


def _slug(text: str, n: int = 40) -> str:
    return "".join(c if c.isalnum() else "_" for c in (text or "cmd")).strip("_")[:n] or "cmd"


def create_command_logger(
    log_dir: str,
) -> Tuple[Callable[[Any], None], Callable[[], None]]:
    """Return ``(post_tick_handler, shutdown)`` that logs each command to a file.

    Pass the handler to ``combine_post_tick_handlers`` alongside the visualizer,
    and call ``shutdown`` in the entry point's ``finally``. Reading the
    blackboard from a standalone client keeps it independent of the tree.
    """
    out_dir = Path(log_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    bb = py_trees.blackboard.Client(name="gpsr_command_logger")
    for key in (
        bb_keys.COMMAND, bb_keys.PLAN, bb_keys.STATE_LOG,
        bb_keys.CORRECTION_COUNT, bb_keys.PLAN_INDEX,
    ):
        bb.register_key(key, access=Access.READ)

    state = {
        "cmd": None, "fh": None, "plan_logged": False,
        "state_len": 0, "corr": 0, "seen_fail": set(),
    }

    def _get(key):
        try:
            return bb.get(key)
        except (KeyError, AttributeError):
            return None

    def _open_for(cmd: str):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = out_dir / f"cmd_{stamp}_{_slug(cmd)}.log"
        fh = open(path, "a", buffering=1)  # line-buffered
        fh.write(f"==== COMMAND @ {datetime.now().isoformat(timespec='seconds')} ====\n")
        fh.write(f"command: {cmd!r}\n")
        print(f"[gpsr-log] logging this command -> {path}")
        return fh

    def handler(tree: Any) -> None:
        cmd = _get(bb_keys.COMMAND)
        # Rotate to a new file whenever the command text changes.
        if cmd and cmd != state["cmd"]:
            if state["fh"]:
                state["fh"].write("\n")
                state["fh"].close()
            state.update(
                cmd=cmd, fh=_open_for(cmd), plan_logged=False,
                state_len=0, corr=0, seen_fail=set(),
            )
        fh = state["fh"]
        if fh is None:
            return

        # Plan (logged once, when first available after planning).
        plan = _get(bb_keys.PLAN)
        if plan is not None and not state["plan_logged"]:
            if plan:
                fh.write(f"PLAN ({len(plan)} steps):\n")
                for i, s in enumerate(plan):
                    fh.write(f"  {i+1}. {s.get('action')}({s.get('params')})\n")
            else:
                fh.write("PLAN: <empty> (planner produced no steps)\n")
            state["plan_logged"] = True

        # New completed-step entries from STATE_LOG.
        slog = _get(bb_keys.STATE_LOG) or []
        if len(slog) > state["state_len"]:
            for entry in slog[state["state_len"]:]:
                fh.write(f"  STEP: {entry}\n")
            state["state_len"] = len(slog)

        # Self-correction rounds.
        corr = _get(bb_keys.CORRECTION_COUNT) or 0
        if corr > state["corr"]:
            fh.write(f"  !! self-correction #{corr} — replanning\n")
            state["corr"] = corr

        # Deepest FAILURE feedback — the real reason a step failed.
        try:
            nodes = list(tree.root.iterate())
        except Exception:
            nodes = []
        for n in nodes:
            if getattr(n, "status", None) != Status.FAILURE:
                continue
            fb = (getattr(n, "feedback_message", "") or "").strip()
            if not fb:
                continue
            key = (n.name, fb)
            if key in state["seen_fail"]:
                continue
            state["seen_fail"].add(key)
            fh.write(f"     FAIL @ {n.name}: {fb}\n")

    def shutdown() -> None:
        if state["fh"]:
            try:
                state["fh"].write("\n==== (session end) ====\n")
                state["fh"].close()
            except Exception:
                pass
            state["fh"] = None

    return handler, shutdown


def combine_post_tick_handlers(*handlers: Optional[Callable[[Any], None]]):
    """Fan a single post_tick_handler slot out to several handlers."""
    real = [h for h in handlers if h is not None]

    def combined(tree: Any) -> None:
        for h in real:
            try:
                h(tree)
            except Exception as exc:  # noqa: BLE001 — logging must never abort ticking
                print(f"[gpsr-log] post-tick handler error (ignored): {exc!r}")

    return combined
