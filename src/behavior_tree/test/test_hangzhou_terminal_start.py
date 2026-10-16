"""Exercise terminal startup without invoking the ROS runtime or robot actions."""
import errno
import os
import pty
import select
import shlex
import signal
import sys
import time

import pytest

SUPERVISOR = "/home/tinker/tk25_ws/src/tk25_basic/src/scripts/hangzhou_pane.py"
PROGRAM = """
import sys, types
runtime = types.ModuleType("behavior_tree.runtime")
runtime.run_tree = lambda *args, **kwargs: print("RUNTIME_STARTED", flush=True)
sys.modules["behavior_tree.runtime"] = runtime
from behavior_tree.HomeServiceHangzhou.cli import main
main()
"""


class Terminal:
    def __init__(self, args=(), supervised=False):
        self.output = ""
        self.pid, self.fd = pty.fork()
        if self.pid == 0:
            cmd = [sys.executable, "-u", "-c", PROGRAM, *args]
            if supervised:
                cmd = [sys.executable, SUPERVISOR, "--no-shell", shlex.join(cmd)]
            os.execv(cmd[0], cmd)

    def read(self, duration=0.2):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            ready, _, _ = select.select([self.fd], [], [], max(0, deadline - time.monotonic()))
            if not ready:
                break
            try:
                data = os.read(self.fd, 65536)
            except OSError as exc:
                if exc.errno == errno.EIO:
                    break
                raise
            if not data:
                break
            self.output += data.decode(errors="replace")
        return self.output

    def until(self, text):
        deadline = time.monotonic() + 8
        while text not in self.output and time.monotonic() < deadline:
            self.read()
        assert text in self.output, self.output
        return self.output

    def send(self, text):
        os.write(self.fd, text)

    def close(self):
        try:
            os.kill(self.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        os.waitpid(self.pid, 0)
        os.close(self.fd)


@pytest.mark.parametrize("supervised", [False, True])
def test_enter_releases_real_cli_only_after_explicit_input(supervised, monkeypatch):
    monkeypatch.delenv("HANGZHOU_CONFIG", raising=False)
    terminal = Terminal(supervised=supervised)
    try:
        terminal.until("Ctrl+C cancels.")
        assert "RUNTIME_STARTED" not in terminal.read()
        terminal.send(b"start\n")
        terminal.until("Press Enter without a command")
        assert "RUNTIME_STARTED" not in terminal.read()
        terminal.send(b"\n")
        terminal.until("RUNTIME_STARTED")
        assert terminal.output.count("RUNTIME_STARTED") == 1
    finally:
        terminal.close()


@pytest.mark.parametrize("key", [b"\x03", b"\x04"])
def test_cancel_and_eof_never_start_runtime(key, monkeypatch):
    monkeypatch.delenv("HANGZHOU_CONFIG", raising=False)
    terminal = Terminal()
    try:
        terminal.until("Ctrl+C cancels.")
        terminal.send(key)
        terminal.until("Mission start cancelled.")
        assert "RUNTIME_STARTED" not in terminal.read()
    finally:
        terminal.close()


def test_config_check_does_not_wait_for_input(monkeypatch):
    monkeypatch.delenv("HANGZHOU_CONFIG", raising=False)
    terminal = Terminal(args=("--check",))
    try:
        terminal.until("Hangzhou configuration OK")
        assert "press Enter" not in terminal.output
        assert "RUNTIME_STARTED" not in terminal.output
    finally:
        terminal.close()
