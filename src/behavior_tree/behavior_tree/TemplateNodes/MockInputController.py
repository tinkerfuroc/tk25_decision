import atexit
import threading
from collections import defaultdict, deque
from typing import Dict, Iterable, Optional, Tuple


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

        self._start_input_key = "\\"
        self._stop_input_key = "/"
        self._start_input_combo = frozenset({"\\"})
        self._stop_input_combo = frozenset({"/"})
        self._subsystem_start_keys = {}
        self._subsystem_start_combos = {}
        self._inverse_subsystem_start_combos = {}
        self._success_key = "ENTER"
        self._success_combo = frozenset({"enter"})

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
        self._max_queue_per_subsystem = 512
        self._last_key = None
        self._last_keys = ()
        self._last_key_source = None
        self._tick_index = 0
        self._event_id = 0
        self._last_delivered_event = defaultdict(int)
        self._last_injected_combo: Tuple[str, ...] = ()

    def configure(self, cfg: Dict):
        with self._lock:
            if not isinstance(cfg, dict):
                return
            self._start_input_key = cfg.get("start_input_key", "\\")
            self._stop_input_key = cfg.get("stop_input_key", "/")
            self._start_input_combo = self._parse_combo_spec(self._start_input_key, default=frozenset({"\\"}))
            self._stop_input_combo = self._parse_combo_spec(self._stop_input_key, default=frozenset({"/"}))
            subsystem_keys = cfg.get("subsystem_start_keys", {})
            dynamic_reserved = cfg.get("teleop_reserved_keys", [])
            if isinstance(dynamic_reserved, list):
                reserved_tokens = set()
                for spec in dynamic_reserved:
                    combo = self._parse_combo_spec(spec, default=frozenset())
                    reserved_tokens.update(combo)
                self._dynamic_teleop_reserved_keys = reserved_tokens
            if isinstance(subsystem_keys, dict):
                sanitized, combos = self._sanitize_subsystem_keys(subsystem_keys)
                self._subsystem_start_keys = sanitized
                self._subsystem_start_combos = combos
                self._inverse_subsystem_start_combos = {
                    combo: subsystem for subsystem, combo in combos.items()
                }
            self._success_key = cfg.get("success_key", "ENTER")
            self._success_combo = self._parse_combo_spec(self._success_key, default=frozenset({"enter"}))

    def _sanitize_subsystem_keys(self, subsystem_keys: Dict) -> Tuple[Dict, Dict]:
        """
        Keep configured subsystem activation keys as-is (from mock_config.json).
        Only reject invalid/duplicate or global-control-colliding keys.
        """
        used = set()
        sanitized = {}
        combos = {}
        for subsystem, key in subsystem_keys.items():
            combo = self._parse_combo_spec(key, default=frozenset())
            if not combo:
                print(f"[MockInput] ignoring invalid subsystem key for '{subsystem}': {key}")
                continue
            combo_key = tuple(sorted(combo))
            if combo_key in used:
                print(f"[MockInput] ignoring duplicate subsystem key '{key}' for '{subsystem}'")
                continue
            if combo in (self._start_input_combo, self._stop_input_combo):
                print(
                    f"[MockInput] ignoring subsystem key '{key}' for '{subsystem}' "
                    f"because it conflicts with start/stop controls"
                )
                continue
            sanitized[subsystem] = key
            combos[subsystem] = combo
            used.add(combo_key)
        return sanitized, combos

    def start(self):
        with self._lock:
            if self._running:
                return
            self._running = True
            if not self._help_printed:
                self._print_help()
                self._help_printed = True

    def shutdown(self):
        with self._lock:
            self._running = False

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
        Public API for one-key non-stdin input sources (e.g. GUI).
        """
        if not isinstance(ch, str) or len(ch) == 0:
            return
        self.inject_keys((ch[0],), source=source)

    def inject_keys(self, keys: Iterable[str], source: str = "gui"):
        """
        Public API for key-combo input sources (GUI only).
        """
        combo = self._canonicalize_combo(keys)
        if not combo:
            return
        self._handle_combo(combo, source=source)

    def clear_active_combo(self):
        """
        Clear combo edge-detection state (call on GUI key release/focus out).
        """
        with self._lock:
            self._last_injected_combo = ()

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
                "last_keys": self._last_keys,
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

    def _enqueue_forward_key_locked(self, subsystem: str, event_id: int, combo: Tuple[str, ...], event_tick: int):
        q = self._queues[subsystem]
        q.append((event_id, combo, event_tick))
        # Bound queue growth under burst GUI input.
        while len(q) > self._max_queue_per_subsystem:
            q.popleft()

    def _handle_combo(self, combo: Tuple[str, ...], source: str = "gui"):
        with self._lock:
            repeated_combo = (combo == self._last_injected_combo)
            self._last_injected_combo = combo
            self._last_key = combo[-1] if combo else None
            self._last_keys = combo
            self._last_key_source = source
            combo_set = frozenset(combo)
            if combo_set == self._start_input_combo:
                if repeated_combo:
                    return
                self._input_enabled = True
                self._broadcast_all_subsystems = True
                self._active_subsystem = None
                print("[MockInput] input forwarding ENABLED for ALL subsystems")
                return

            if combo_set == self._stop_input_combo:
                if repeated_combo:
                    return
                self._input_enabled = False
                self._broadcast_all_subsystems = False
                self._active_subsystem = None
                print("[MockInput] input forwarding DISABLED for ALL subsystems")
                return

            if combo_set in self._inverse_subsystem_start_combos:
                if repeated_combo:
                    return
                self._active_subsystem = self._inverse_subsystem_start_combos[combo_set]
                self._input_enabled = True
                self._broadcast_all_subsystems = False
                print(f"[MockInput] active subsystem -> {self._active_subsystem}")
                return

            if self._input_enabled:
                event_id = self._next_event_id_locked()
                event_tick = self._tick_index
                if self._broadcast_all_subsystems:
                    for subsystem in self._subsystem_start_keys.keys():
                        self._enqueue_forward_key_locked(subsystem, event_id, combo, event_tick)
                elif self._active_subsystem is not None:
                    self._enqueue_forward_key_locked(self._active_subsystem, event_id, combo, event_tick)

    def is_success_event(self, event) -> bool:
        combo = self._coerce_event_to_frozenset(event)
        return bool(combo) and combo == self._success_combo

    def _coerce_event_to_frozenset(self, event) -> frozenset:
        if event is None:
            return frozenset()
        if isinstance(event, str):
            return self._parse_combo_spec(event, default=frozenset())
        if isinstance(event, (tuple, list, set, frozenset)):
            return frozenset(self._canonicalize_combo(event))
        return frozenset()

    def _parse_combo_spec(self, spec, default: frozenset) -> frozenset:
        if isinstance(spec, str):
            raw = spec.strip()
            if not raw:
                return default
            parts = [p.strip() for p in raw.split("+") if p.strip()]
            return frozenset(self._normalize_token(p) for p in parts if self._normalize_token(p))
        if isinstance(spec, (tuple, list, set, frozenset)):
            normalized = [self._normalize_token(str(p)) for p in spec]
            normalized = [n for n in normalized if n]
            return frozenset(normalized) if normalized else default
        return default

    def _canonicalize_combo(self, keys: Iterable[str]) -> Tuple[str, ...]:
        normalized = []
        for token in keys:
            n = self._normalize_token(token)
            if n:
                normalized.append(n)
        if not normalized:
            return ()
        unique = sorted(set(normalized))
        return tuple(unique)

    def _normalize_token(self, token: str) -> str:
        if token is None:
            return ""
        t = str(token).strip().lower()
        alias = {
            "return": "enter",
            "kp_enter": "enter",
            "\\n": "enter",
            "\n": "enter",
            "\\r": "enter",
            "\r": "enter",
            "control": "ctrl",
            "control_l": "ctrl",
            "control_r": "ctrl",
            "shift_l": "shift",
            "shift_r": "shift",
            "alt_l": "alt",
            "alt_r": "alt",
            "spacebar": "space",
            " ": "space",
        }
        return alias.get(t, t)

    def _print_help(self):
        print("")
        print("[MockInput] controls (GUI input only):")
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
