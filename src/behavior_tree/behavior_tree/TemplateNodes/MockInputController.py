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
        self._broadcast_all_subsystems = False
        self._help_printed = False
        self._teleop_reserved_keys = set(
            [
                "w", "s", "a", "d", "q", "e",
                "j", "l", "i", "k", "u", "o",
                "1", "2", "3", "4", "5", "6", "7", "8", "9", "0", "-", "=", "[", "]",
            ]
        )
        self._dynamic_teleop_reserved_keys = set()
        self._max_queue_per_subsystem = 8
        self._last_key = None
        self._last_key_source = None
        self._tick_index = 0
        self._event_id = 0
        self._last_delivered_event = defaultdict(int)

    def configure(self, cfg: Dict):
        with self._lock:
            if not isinstance(cfg, dict):
                return
            self._start_input_key = cfg.get("start_input_key", "\\")
            self._stop_input_key = cfg.get("stop_input_key", "/")
            subsystem_keys = cfg.get("subsystem_start_keys", {})
            dynamic_reserved = cfg.get("teleop_reserved_keys", [])
            if isinstance(dynamic_reserved, list):
                self._dynamic_teleop_reserved_keys = set(
                    [k for k in dynamic_reserved if isinstance(k, str) and len(k) == 1]
                )
            if isinstance(subsystem_keys, dict):
                sanitized = self._sanitize_subsystem_keys(subsystem_keys)
                self._subsystem_start_keys = sanitized
                self._inverse_subsystem_start_keys = {
                    value: key for key, value in sanitized.items() if isinstance(value, str)
                }
            self._success_key = cfg.get("success_key", "ENTER")

    def _sanitize_subsystem_keys(self, subsystem_keys: Dict) -> Dict:
        """
        Keep configured subsystem activation keys as-is (from mock_config.json).
        Only reject invalid/duplicate or global-control-colliding keys.
        """
        used = set()
        sanitized = {}
        for subsystem, key in subsystem_keys.items():
            if not isinstance(key, str) or len(key) != 1:
                print(f"[MockInput] ignoring invalid subsystem key for '{subsystem}': {key}")
                continue
            if key in used:
                print(f"[MockInput] ignoring duplicate subsystem key '{key}' for '{subsystem}'")
                continue
            if key in (self._start_input_key, self._stop_input_key):
                print(
                    f"[MockInput] ignoring subsystem key '{key}' for '{subsystem}' "
                    f"because it conflicts with start/stop controls"
                )
                continue
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

    def pop_key(
        self,
        subsystem: Optional[str],
        consumer_id: Optional[str] = None,
        consumer_start_tick: Optional[int] = None,
        consumer_start_event: Optional[int] = None,
    ):
        keys = self.pop_keys(
            subsystem,
            consumer_id=consumer_id,
            consumer_start_tick=consumer_start_tick,
            consumer_start_event=consumer_start_event,
            max_keys=1,
        )
        if keys:
            return keys[0]
        return None

    def pop_keys(
        self,
        subsystem: Optional[str],
        consumer_id: Optional[str] = None,
        consumer_start_tick: Optional[int] = None,
        consumer_start_event: Optional[int] = None,
        max_keys: Optional[int] = None,
    ):
        """
        Pop all currently eligible keys for one consumer.
        This is used by teleop to process burst input within the same tick.
        """
        if subsystem is None:
            return []
        with self._lock:
            q = self._queues.get(subsystem)
            if not q:
                return []
            if len(q) == 0:
                return []

            if consumer_id is None:
                # Backward compatibility path (single-consumer semantics)
                result = []
                while q and (max_keys is None or len(result) < max_keys):
                    _, key_value, _ = q.popleft()
                    result.append(key_value)
                return result

            last_event = self._last_delivered_event[consumer_id]
            min_allowed_event = last_event
            if consumer_start_event is not None:
                min_allowed_event = max(min_allowed_event, consumer_start_event)

            selected = []
            for event_id, key_value, event_tick in q:
                if event_id <= min_allowed_event:
                    continue
                if consumer_start_tick is not None and event_tick <= consumer_start_tick:
                    continue
                selected.append((event_id, key_value))
                if max_keys is not None and len(selected) >= max_keys:
                    break

            if selected:
                self._last_delivered_event[consumer_id] = selected[-1][0]
            return [key for _, key in selected]

    def inject_key(self, ch: str, source: str = "gui"):
        """
        Public API for non-stdin input sources (e.g. GUI) to forward keys.
        """
        if not isinstance(ch, str) or len(ch) == 0:
            return
        self._handle_key(ch[0], source=source)

    def get_status_snapshot(self) -> Dict:
        """
        Thread-safe current state for diagnostics/visualization.
        """
        with self._lock:
            queue_sizes = {subsystem: len(queue) for subsystem, queue in self._queues.items()}
            return {
                "running": self._running,
                "input_enabled": self._input_enabled,
                "active_subsystem": self._active_subsystem,
                "broadcast_all_subsystems": self._broadcast_all_subsystems,
                "start_input_key": self._start_input_key,
                "stop_input_key": self._stop_input_key,
                "success_key": self._success_key,
                "subsystem_start_keys": dict(self._subsystem_start_keys),
                "queue_sizes": queue_sizes,
                "last_key": self._last_key,
                "last_key_source": self._last_key_source,
                "tick_index": self._tick_index,
            }

    def get_tick_index(self) -> int:
        with self._lock:
            return self._tick_index

    def get_event_index(self) -> int:
        with self._lock:
            return self._event_id

    def end_tick_cycle(self):
        """
        Called once after each behavior-tree tick.
        Drops any unconsumed keys so they don't leak into later ticks.
        """
        with self._lock:
            for subsystem in list(self._queues.keys()):
                self._queues[subsystem].clear()
            self._last_delivered_event.clear()
            self._tick_index += 1

    def _next_event_id_locked(self) -> int:
        self._event_id += 1
        return self._event_id

    def _is_teleop_motion_key_locked(self, ch: str) -> bool:
        return ch in self._teleop_reserved_keys or ch in self._dynamic_teleop_reserved_keys

    def _enqueue_forward_key_locked(self, subsystem: str, event_id: int, ch: str, event_tick: int):
        q = self._queues[subsystem]
        if self._is_teleop_motion_key_locked(ch):
            # Keep only non-teleop keys + the latest teleop intent.
            # This avoids long "tails" from stale motion key backlog.
            if q:
                retained = deque(item for item in q if not self._is_teleop_motion_key_locked(item[1]))
                if len(retained) != len(q):
                    self._queues[subsystem] = retained
                    q = retained
        q.append((event_id, ch, event_tick))
        # Bound queue growth under burst GUI input.
        while len(q) > self._max_queue_per_subsystem:
            q.popleft()

    def _loop(self):
        while True:
            with self._lock:
                if not self._running:
                    return
            if not select.select([sys.stdin], [], [], 0.001)[0]:
                continue
            # Drain all currently buffered chars in one burst so held-key
            # repeats are forwarded at full rate.
            while True:
                try:
                    ch = sys.stdin.read(1)
                except Exception:
                    break
                if not ch:
                    break
                self._handle_key(ch, source="stdin")
                if not select.select([sys.stdin], [], [], 0.0)[0]:
                    break

    def _handle_key(self, ch: str, source: str = "stdin"):
        with self._lock:
            self._last_key = ch
            self._last_key_source = source
            if ch == self._start_input_key:
                self._input_enabled = True
                self._broadcast_all_subsystems = True
                self._active_subsystem = None
                print("[MockInput] input forwarding ENABLED for ALL subsystems")
                return

            if ch == self._stop_input_key:
                self._input_enabled = False
                self._broadcast_all_subsystems = False
                self._active_subsystem = None
                print("[MockInput] input forwarding DISABLED for ALL subsystems")
                return

            if ch in self._inverse_subsystem_start_keys:
                self._active_subsystem = self._inverse_subsystem_start_keys[ch]
                self._input_enabled = True
                self._broadcast_all_subsystems = False
                print(f"[MockInput] active subsystem -> {self._active_subsystem}")
                return

            if self._input_enabled:
                event_id = self._next_event_id_locked()
                event_tick = self._tick_index
                if self._broadcast_all_subsystems:
                    for subsystem in self._subsystem_start_keys.keys():
                        self._enqueue_forward_key_locked(subsystem, event_id, ch, event_tick)
                elif self._active_subsystem is not None:
                    self._enqueue_forward_key_locked(self._active_subsystem, event_id, ch, event_tick)

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
