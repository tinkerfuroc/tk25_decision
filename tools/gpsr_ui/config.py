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

    def __post_init__(self) -> None:
        # Enforce the read-only guarantee structurally, at the type
        # boundary -- not just in load_settings(). `create_app()` accepts
        # any `Settings`, and tests/test_app.py already builds one
        # directly (not via load_settings()), so a check that only lived
        # in load_settings() was a real bypass, not a hypothetical one:
        # anything constructing `Settings` some other way could still
        # point `state_dir` inside `bench_root` with no error at all.
        # Living here means it holds no matter how a `Settings` comes to
        # exist.
        if self.bench_root == self.state_dir or (
                self.bench_root in self.state_dir.parents):
            raise ValueError(
                f"state_dir ({self.state_dir}) must not be inside the bench "
                f"corpus ({self.bench_root}); the viewer never writes there"
            )


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

    # The read-only guarantee itself (state_dir must not be inside
    # bench_root) is enforced by Settings.__post_init__ now, not here --
    # it fires below, from the constructor call, and raises the same
    # ValueError regardless of which env var supplied the offending path.
    return Settings(
        bench_root=bench_root,
        state_dir=state_dir,
        sheet_events_path=_path_env("GPSR_UI_SHEET_EVENTS", None),
        sim_stack_log_roots=_sim_stack_log_roots(),
    )
