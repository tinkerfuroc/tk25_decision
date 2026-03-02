import atexit
import select
import sys
import termios
import threading
import tty
from collections import defaultdict, deque
from typing import Dict, Optional


class MockInputController:
    """
    Shared keyboard router for mock nodes.

    Control flow:
    - Press start_input_key to enable input forwarding
    - Press stop_input_key to disable input forwarding
    - Press subsystem start key (e.g. 'm') to select active subsystem and enable forwarding
    - While enabled, keys are forwarded only to active subsystem queue
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._queues = defaultdict(deque)
        self._running = False
        self._thread = None
        self._old_settings = None

        self._start_input_key = "\\"
        self._stop_input_key = "/"
        self._subsystem_start_keys = {}
        self._inverse_subsystem_start_keys = {}
        self._success_key = "ENTER"

        self._input_enabled = False
        self._active_subsystem = None
        self._help_printed = False
        self._teleop_reserved_keys = set(
            [
                "w", "s", "a", "d", "r", "f",
                "j", "l", "i", "k", "u", "o",
                "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "[", "]",
                "g", "h", " ", "z", "x", "c", "v", "b", "n",
            ]
        )

    def configure(self, cfg: Dict):
        with self._lock:
            if not isinstance(cfg, dict):
                return
            self._start_input_key = cfg.get("start_input_key", "\\")
            self._stop_input_key = cfg.get("stop_input_key", "/")
            subsystem_keys = cfg.get("subsystem_start_keys", {})
            if isinstance(subsystem_keys, dict):
                sanitized = self._sanitize_subsystem_keys(subsystem_keys)
                self._subsystem_start_keys = sanitized
                self._inverse_subsystem_start_keys = {
                    value: key for key, value in sanitized.items() if isinstance(value, str)
                }
            self._success_key = cfg.get("success_key", "ENTER")

    def _sanitize_subsystem_keys(self, subsystem_keys: Dict) -> Dict:
        """
        Ensure subsystem activation keys do not overlap teleop controls and are unique.
        """
        fallback_keys = ["p", ";", "'", "`", "q", "y", ",", ".", "/"]
        used = set()
        sanitized = {}
        for subsystem, key in subsystem_keys.items():
            if not isinstance(key, str) or len(key) != 1:
                key = None
            if key in self._teleop_reserved_keys or key in used or key in (self._start_input_key, self._stop_input_key):
                replacement = None
                for candidate in fallback_keys:
                    if (
                        candidate not in self._teleop_reserved_keys
                        and candidate not in used
                        and candidate not in (self._start_input_key, self._stop_input_key)
                    ):
                        replacement = candidate
                        break
                if replacement is None:
                    continue
                print(
                    f"[MockInput] remapped subsystem key for '{subsystem}' "
                    f"from '{key}' to '{replacement}' (conflict with teleop/unified keys)"
                )
                key = replacement
            sanitized[subsystem] = key
            used.add(key)
        return sanitized

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()
            if not self._help_printed:
                self._print_help()
                self._help_printed = True

    def shutdown(self):
        with self._lock:
            self._running = False
        if self._thread is not None:
            self._thread.join(timeout=0.5)
            self._thread = None
        with self._lock:
            if self._old_settings is not None:
                try:
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)
                except Exception:
                    pass
                self._old_settings = None

    def pop_key(self, subsystem: Optional[str]):
        if subsystem is None:
            return None
        with self._lock:
            q = self._queues.get(subsystem)
            if not q:
                return None
            if len(q) == 0:
                return None
            return q.popleft()

    def _loop(self):
        while True:
            with self._lock:
                if not self._running:
                    return
            if not select.select([sys.stdin], [], [], 0.05)[0]:
                continue
            try:
                ch = sys.stdin.read(1)
            except Exception:
                continue
            if not ch:
                continue
            self._handle_key(ch)

    def _handle_key(self, ch: str):
        with self._lock:
            if ch == self._start_input_key:
                self._input_enabled = True
                print("[MockInput] input forwarding ENABLED")
                return

            if ch == self._stop_input_key:
                self._input_enabled = False
                print("[MockInput] input forwarding DISABLED")
                return

            if ch in self._inverse_subsystem_start_keys:
                self._active_subsystem = self._inverse_subsystem_start_keys[ch]
                self._input_enabled = True
                print(f"[MockInput] active subsystem -> {self._active_subsystem}")
                return

            if self._input_enabled and self._active_subsystem is not None:
                self._queues[self._active_subsystem].append(ch)

    def _print_help(self):
        print("")
        print("[MockInput] controls:")
        print(f"  enable input forwarding: '{self._start_input_key}'")
        print(f"  disable input forwarding: '{self._stop_input_key}'")
        if self._subsystem_start_keys:
            print("  select subsystem:")
            for subsystem, key in self._subsystem_start_keys.items():
                print(f"    '{key}' -> {subsystem}")
        print(f"  success key for non-teleop mock nodes: {self._success_key}")
        print("")


_controller = MockInputController()
atexit.register(_controller.shutdown)


def get_mock_input_controller() -> MockInputController:
    return _controller
