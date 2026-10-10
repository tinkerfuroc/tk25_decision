"""Tick-stall watchdog (GPSR sim battery 2026-09-30 fix6: runs froze after
plan.materialized with no tick or log for 1500 s). Run with
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1."""
import io
import sys

from behavior_tree.GPSR.gpsr_orchestrator import TickStallWatchdog


class _Clock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t


def test_dumps_thread_stacks_only_after_the_stall_threshold(monkeypatch):
    wd = TickStallWatchdog(stall_s=60.0)
    clock = _Clock()
    wd._time = clock
    wd.post_tick()
    err = io.StringIO()
    monkeypatch.setattr(sys, "stderr", err)
    clock.t += 30
    assert wd.check() is False
    clock.t += 31
    import faulthandler
    monkeypatch.setattr(faulthandler, "dump_traceback",
                        lambda file, all_threads: file.write("Thread 0x1 (most recent call first)\n"))
    assert wd.check() is True
    assert "TICK STALL" in err.getvalue() and "most recent call first" in err.getvalue()
    wd.post_tick()
    assert wd.check() is False


def test_env_zero_disables(monkeypatch):
    monkeypatch.setenv("GPSR_TICK_STALL_S", "0")
    wd = TickStallWatchdog()
    wd._time = _Clock()
    wd._time.t += 1e6
    assert wd.check() is False
