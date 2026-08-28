# tools/tests/test_config.py
from __future__ import annotations

from pathlib import Path

import pytest

from gpsr_ui.config import Settings, load_settings


def test_defaults_point_at_bench_and_user_cache(monkeypatch):
    monkeypatch.delenv("GPSR_UI_BENCH_ROOT", raising=False)
    monkeypatch.delenv("GPSR_UI_STATE_DIR", raising=False)
    monkeypatch.delenv("GPSR_UI_SHEET_EVENTS", raising=False)
    s = load_settings()
    assert s.bench_root.name == "bench"
    assert s.state_dir == Path.home() / ".cache" / "gpsr-ui"
    assert s.sheet_events_path is None


def test_environment_overrides_are_expanded(monkeypatch, tmp_path):
    monkeypatch.setenv("GPSR_UI_BENCH_ROOT", str(tmp_path / "b"))
    monkeypatch.setenv("GPSR_UI_STATE_DIR", str(tmp_path / "s"))
    monkeypatch.setenv("GPSR_UI_SHEET_EVENTS", str(tmp_path / "se.py"))
    s = load_settings()
    assert s.bench_root == tmp_path / "b"
    assert s.state_dir == tmp_path / "s"
    assert s.sheet_events_path == tmp_path / "se.py"


def test_state_dir_is_never_inside_the_corpus(monkeypatch, tmp_path):
    """The read-only guarantee starts here: refuse to write into gpsr_runs."""
    monkeypatch.setenv("GPSR_UI_BENCH_ROOT", str(tmp_path / "bench"))
    monkeypatch.setenv("GPSR_UI_STATE_DIR", str(tmp_path / "bench" / "ui"))
    with pytest.raises(ValueError, match="must not be inside"):
        load_settings()


def test_settings_constructed_directly_still_enforces_the_guarantee(tmp_path):
    """The check must hold at the `Settings` type boundary itself, not only
    inside load_settings() -- `create_app()` accepts any `Settings`, and
    tests/test_app.py already builds one directly (not via load_settings()),
    so a validation-free bypass path here would be real, not hypothetical."""
    with pytest.raises(ValueError, match="must not be inside"):
        Settings(
            bench_root=tmp_path / "bench",
            state_dir=tmp_path / "bench" / "ui",
            sheet_events_path=None,
        )


def test_settings_constructed_directly_with_state_dir_equal_to_bench_root(
        tmp_path):
    """The equality case (not just nesting) must also be rejected."""
    with pytest.raises(ValueError, match="must not be inside"):
        Settings(
            bench_root=tmp_path / "bench",
            state_dir=tmp_path / "bench",
            sheet_events_path=None,
        )
