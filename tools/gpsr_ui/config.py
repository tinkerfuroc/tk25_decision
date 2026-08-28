# tools/gpsr_ui/config.py
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

DEFAULT_BENCH_ROOT = Path(
    "/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree"
    "/GPSR/gpsr_runs/bench"
)


@dataclass(frozen=True)
class Settings:
    bench_root: Path
    state_dir: Path
    sheet_events_path: Path | None
    # Where the live dashboard's "failed to make progress" panel looks for
    # a sim worktree's gpsr_stack_logs/ -- see live.find_progress_failures.
    # That data source lives entirely outside this corpus and outside this
    # repo, in a worktree whose lifetime we do not control, so a default
    # (empty tuple) must be a legal, harmless value: it just means the
    # panel never has anything to show. A trailing default keeps every
    # existing direct `Settings(...)` construction (tests included) valid.
    sim_stack_log_roots: tuple[Path, ...] = ()


def _path_env(name: str, default: Path | None) -> Path | None:
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    return Path(raw).expanduser()


def _sim_stack_log_roots() -> tuple[Path, ...]:
    raw = os.environ.get("GPSR_UI_SIM_STACK_LOGS_ROOTS")
    if raw is not None and raw.strip():
        return tuple(Path(p).expanduser() for p in raw.split(":") if p.strip())
    # The one real layout seen on this machine: a tinker-sim checkout,
    # itself possibly holding several git worktrees, any of which might
    # currently be running the sim stack for a live battery.
    return (Path.home() / "tinker-sim",)


def load_settings() -> Settings:
    bench_root = _path_env("GPSR_UI_BENCH_ROOT", DEFAULT_BENCH_ROOT)
    state_dir = _path_env("GPSR_UI_STATE_DIR", Path.home() / ".cache" / "gpsr-ui")
    assert bench_root is not None and state_dir is not None

    # Enforce the read-only guarantee structurally, not by convention.
    if bench_root == state_dir or bench_root in state_dir.parents:
        raise ValueError(
            f"GPSR_UI_STATE_DIR ({state_dir}) must not be inside the bench "
            f"corpus ({bench_root}); the viewer never writes there"
        )

    return Settings(
        bench_root=bench_root,
        state_dir=state_dir,
        sheet_events_path=_path_env("GPSR_UI_SHEET_EVENTS", None),
        sim_stack_log_roots=_sim_stack_log_roots(),
    )
