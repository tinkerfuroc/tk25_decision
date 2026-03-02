from __future__ import annotations

from datetime import datetime
from pathlib import Path
import queue
import threading
import time
import warnings
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import py_trees
from py_trees.blackboard import Blackboard
from behavior_tree.config import get_config, should_use_keyboard_control
from behavior_tree.TemplateNodes.MockInputController import get_mock_input_controller

try:
    import tkinter as tk
    import tkinter.font as tkfont
    from tkinter import ttk
except Exception:  # pragma: no cover - tkinter may not be available
    tk = None
    tkfont = None
    ttk = None


@dataclass
class _NodeSnapshot:
    node_id: str
    parent_id: str
    name: str
    node_type: str
    status: str
    feedback: str


class BehaviorTreeStatusGUI:
    """
    Live tkinter dashboard for behavior tree node status and blackboard content.

    - Call `start()` once.
    - Call `update(root)` from post tick handlers.
    - Call `shutdown()` on exit.
    """

    _STATUS_COLORS = {
        "SUCCESS": "#2E7D32",
        "FAILURE": "#C62828",
        "RUNNING": "#F9A825",
        "INVALID": "#6C757D",
    }

    def __init__(self, title: str = "Behavior Tree Monitor", max_value_length: int = 120):
        self.title = title
        self.max_value_length = max_value_length
        self.enabled = tk is not None and ttk is not None

        self._queue: "queue.Queue[Optional[Tuple[List[_NodeSnapshot], Dict[str, Any]]]]" = queue.Queue(maxsize=1)
        self._thread: Optional[threading.Thread] = None
        self._ready = threading.Event()
        self._closed = threading.Event()
        self._window_error: Optional[str] = None

        self._root_window = None
        self._treeview = None
        self._bb_table = None
        self._mock_static_text = None
        self._mock_dynamic_text = None
        self._first_node_render = True
        self._mock_controller = get_mock_input_controller()
        self._tree_body_font = None
        self._tree_heading_font = None
        self._mock_body_font = None
        self._node_ids: set[str] = set()
        self._node_render_cache: Dict[str, Tuple[str, str, str, str]] = {}
        self._state_log_file = None
        self._state_log_path: Optional[Path] = None
        self._log_queue: "queue.Queue[str]" = queue.Queue(maxsize=4096)
        self._log_thread: Optional[threading.Thread] = None
        self._log_stop = threading.Event()
        self._last_node_state: Dict[str, Tuple[str, str]] = {}
        self._last_bb_state: Dict[str, str] = {}
        self._last_mock_runtime: Optional[str] = None
        self._bb_row_by_key: Dict[str, str] = {}
        self._bb_value_cache: Dict[str, str] = {}
        self._pressed_keys: Dict[str, str] = {}
        self._pressed_key_down_at: Dict[str, float] = {}
        self._pressed_key_seen_at: Dict[str, float] = {}
        self._pending_release_at: Dict[str, float] = {}
        self._gui_key_repeat_ms = 8
        self._gui_stuck_key_timeout_s = 0.8
        self._gui_wait_single_key_timeout_s = 0.6
        self._gui_release_grace_s = 0.04

    def start(self) -> bool:
        if not self.enabled:
            return False
        self._thread = threading.Thread(target=self._run_tk, daemon=True)
        self._thread.start()
        self._ready.wait(timeout=2.0)
        if self._window_error:
            self.enabled = False
            return False
        return True

    def update(self, root: py_trees.behaviour.Behaviour) -> None:
        if not self.enabled or self._closed.is_set():
            return
        if self._queue.full():
            return
        snapshot = self._build_snapshot(root)
        self._queue.put_nowait(snapshot)

    def shutdown(self) -> None:
        if not self.enabled:
            return
        self._closed.set()
        if self._queue.full():
            try:
                self._queue.get_nowait()
            except queue.Empty:
                pass
        self._queue.put_nowait(None)
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        if self._log_thread and self._log_thread.is_alive():
            self._log_stop.set()
            self._log_thread.join(timeout=1.0)
        if self._state_log_file is not None:
            try:
                self._state_log_file.write(f"{self._timestamp()} [INFO] GUI shutdown\n")
                self._state_log_file.flush()
                self._state_log_file.close()
            except Exception:
                pass
            self._state_log_file = None

    def _run_tk(self) -> None:
        try:
            self._open_state_log_file()
            self._root_window = tk.Tk()
            self._root_window.title(self.title)
            self._root_window.geometry("1400x780")
            self._root_window.minsize(900, 520)
            self._root_window.protocol("WM_DELETE_WINDOW", self._on_close)
            self._configure_styles()
            self._root_window.bind_all("<KeyPress>", self._on_keypress)
            self._root_window.bind_all("<KeyRelease>", self._on_keyrelease)
            self._root_window.bind("<FocusOut>", self._on_focus_out)
            self._root_window.bind_all("<Control-c>", self._on_copy_selection)
            self._root_window.bind_all("<Control-C>", self._on_copy_selection)

            root_split = ttk.Panedwindow(self._root_window, orient=tk.VERTICAL)
            root_split.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

            container = ttk.Panedwindow(root_split, orient=tk.HORIZONTAL)
            root_split.add(container, weight=5)

            node_frame = ttk.Labelframe(container, text="Node Status", padding=8)
            bb_frame = ttk.Labelframe(container, text="Blackboard", padding=8)
            container.add(node_frame, weight=3)
            container.add(bb_frame, weight=2)
            mock_frame = ttk.Labelframe(root_split, text="Mock Mode Status / Input", padding=8)
            root_split.add(mock_frame, weight=1)

            node_table = ttk.Frame(node_frame)
            node_table.pack(fill=tk.BOTH, expand=True)

            self._treeview = ttk.Treeview(
                node_table,
                columns=("type", "status", "feedback"),
                show="tree headings",
                selectmode="browse",
            )
            self._treeview.heading("#0", text="Node")
            self._treeview.heading("type", text="Type")
            self._treeview.heading("status", text="Status")
            self._treeview.heading("feedback", text="Feedback")
            self._treeview.column("#0", width=420, minwidth=220, anchor=tk.W, stretch=False)
            self._treeview.column("type", width=240, minwidth=120, anchor=tk.W, stretch=False)
            self._treeview.column("status", width=160, minwidth=100, anchor=tk.CENTER, stretch=False)
            self._treeview.column("feedback", width=1000, minwidth=320, anchor=tk.W, stretch=False)
            self._treeview.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

            node_scroll_y = ttk.Scrollbar(node_table, orient=tk.VERTICAL, command=self._treeview.yview)
            node_scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
            node_scroll_x = ttk.Scrollbar(node_frame, orient=tk.HORIZONTAL, command=self._treeview.xview)
            node_scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
            self._treeview.configure(yscrollcommand=node_scroll_y.set, xscrollcommand=node_scroll_x.set)
            self._treeview.bind("<Shift-MouseWheel>", lambda e: self._treeview.xview_scroll(int(-e.delta / 120), "units"))

            bb_table_frame = ttk.Frame(bb_frame)
            bb_table_frame.pack(fill=tk.BOTH, expand=True)

            self._bb_table = ttk.Treeview(
                bb_table_frame,
                columns=("key", "value"),
                show="headings",
                selectmode="browse",
            )
            self._bb_table.heading("key", text="Key")
            self._bb_table.heading("value", text="Value")
            self._bb_table.column("key", width=360, minwidth=180, anchor=tk.W, stretch=False)
            self._bb_table.column("value", width=1400, minwidth=320, anchor=tk.W, stretch=False)
            self._bb_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

            bb_scroll_y = ttk.Scrollbar(bb_table_frame, orient=tk.VERTICAL, command=self._bb_table.yview)
            bb_scroll_y.pack(side=tk.RIGHT, fill=tk.Y)
            bb_scroll_x = ttk.Scrollbar(bb_frame, orient=tk.HORIZONTAL, command=self._bb_table.xview)
            bb_scroll_x.pack(side=tk.BOTTOM, fill=tk.X)
            self._bb_table.configure(yscrollcommand=bb_scroll_y.set, xscrollcommand=bb_scroll_x.set)
            self._bb_table.bind("<Shift-MouseWheel>", lambda e: self._bb_table.xview_scroll(int(-e.delta / 120), "units"))

            mock_split = ttk.Panedwindow(mock_frame, orient=tk.HORIZONTAL)
            mock_split.pack(fill=tk.BOTH, expand=True)

            mock_static_frame = ttk.Labelframe(mock_split, text="Mock Constants", padding=6)
            mock_dynamic_frame = ttk.Labelframe(mock_split, text="Mock Runtime", padding=6)
            mock_split.add(mock_static_frame, weight=1)
            mock_split.add(mock_dynamic_frame, weight=1)

            self._mock_static_text = tk.Text(
                mock_static_frame,
                wrap=tk.WORD,
                state=tk.DISABLED,
                font=self._mock_body_font,
            )
            self._mock_static_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            static_scroll = ttk.Scrollbar(mock_static_frame, orient=tk.VERTICAL, command=self._mock_static_text.yview)
            static_scroll.pack(side=tk.RIGHT, fill=tk.Y)
            self._mock_static_text.configure(yscrollcommand=static_scroll.set)

            self._mock_dynamic_text = tk.Text(
                mock_dynamic_frame,
                wrap=tk.WORD,
                state=tk.DISABLED,
                font=self._mock_body_font,
            )
            self._mock_dynamic_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            dynamic_scroll = ttk.Scrollbar(mock_dynamic_frame, orient=tk.VERTICAL, command=self._mock_dynamic_text.yview)
            dynamic_scroll.pack(side=tk.RIGHT, fill=tk.Y)
            self._mock_dynamic_text.configure(yscrollcommand=dynamic_scroll.set)

            for status, color in self._STATUS_COLORS.items():
                self._treeview.tag_configure(status, foreground=color)

            self._ready.set()
            self._poll_queue()
            self._repeat_pressed_keys()
            self._root_window.mainloop()
        except Exception as exc:  # pragma: no cover - runtime UI environment issues
            self._window_error = str(exc)
            self._ready.set()
            self._closed.set()
            self.enabled = False

    def _on_close(self) -> None:
        self._closed.set()
        self._pressed_keys.clear()
        self._pressed_key_down_at.clear()
        self._mock_controller.clear_active_combo()
        self._pending_release_at.clear()
        if self._root_window is not None:
            self._root_window.destroy()

    def _event_to_input(self, event) -> Optional[Tuple[str, str]]:
        keysym = getattr(event, "keysym", "")
        ks = keysym.lower() if isinstance(keysym, str) else ""
        token_map = {
            "shift_l": "shift",
            "shift_r": "shift",
            "control_l": "ctrl",
            "control_r": "ctrl",
            "alt_l": "alt",
            "alt_r": "alt",
            "meta_l": "alt",
            "meta_r": "alt",
            "return": "enter",
            "kp_enter": "enter",
            "space": "space",
            # Common non-alnum keysyms where KeyRelease often has empty char.
            "slash": "/",
            "backslash": "\\",
            "minus": "-",
            "equal": "=",
            "bracketleft": "[",
            "bracketright": "]",
            "semicolon": ";",
            "apostrophe": "'",
            "comma": ",",
            "period": ".",
            "grave": "`",
        }
        if ks in token_map:
            return (ks, token_map[ks])
        ch = getattr(event, "char", "")
        if isinstance(ch, str) and len(ch) > 0:
            token = ch[0].lower()
            # Ignore non-ascii IME/composition characters in control channel.
            if ord(token) > 127:
                return None
            return (ks if ks else token, token)
        if isinstance(ks, str) and len(ks) == 1:
            if ord(ks) > 127:
                return None
            return (ks, ks)
        # Last-resort fallback for unhandled keysyms:
        # keep a stable id/token so KeyRelease can clear stuck keys.
        if isinstance(ks, str) and len(ks) > 0:
            return (ks, ks)
        return None

    def _on_keypress(self, event) -> None:
        resolved = self._event_to_input(event)
        if not resolved:
            # Defensive clear to recover quickly from IME/dead-key edge cases.
            self._pressed_keys.clear()
            self._pressed_key_down_at.clear()
            self._pressed_key_seen_at.clear()
            self._pending_release_at.clear()
            return
        key_id, token = resolved
        # Preserve copy shortcut behavior without forwarding to mock input.
        if event.state & 0x4 and token == "c":
            return
        now = time.monotonic()
        self._pending_release_at.pop(key_id, None)
        if key_id not in self._pressed_keys:
            self._pressed_key_down_at[key_id] = now
        self._pressed_keys[key_id] = token
        self._pressed_key_seen_at[key_id] = now
        # Keep combo members alive together when OS repeats only one key.
        for existing_key in list(self._pressed_key_seen_at.keys()):
            self._pressed_key_seen_at[existing_key] = now
        self._mock_controller.inject_keys(tuple(self._pressed_keys.values()), source="gui")

    def _on_keyrelease(self, event) -> None:
        resolved = self._event_to_input(event)
        # print(f"DEBUG: detected key release {event}, mapped to {resolved}")
        if not resolved:
            self._pressed_keys.clear()
            self._pressed_key_down_at.clear()
            self._pressed_key_seen_at.clear()
            self._pending_release_at.clear()
            self._mock_controller.clear_active_combo()
            return
        key_id, token = resolved
        now = time.monotonic()
        # Key autorepeat on some platforms emits rapid release/press pairs.
        # Delay actual removal slightly; a following keypress cancels removal.
        if key_id in self._pressed_keys:
            self._pending_release_at[key_id] = now + self._gui_release_grace_s
        else:
            # Fallback for layout/keysym mismatches: schedule same-token entries.
            for stale_id, value in list(self._pressed_keys.items()):
                if value == token:
                    self._pending_release_at[stale_id] = now + self._gui_release_grace_s

    def _on_focus_out(self, _event=None) -> None:
        # Avoid stuck "held keys" if release happens outside this window.
        self._pressed_keys.clear()
        self._pressed_key_down_at.clear()
        self._pressed_key_seen_at.clear()
        self._pending_release_at.clear()
        self._mock_controller.clear_active_combo()

    def _repeat_pressed_keys(self) -> None:
        if self._closed.is_set():
            return
        now = time.monotonic()
        if self._pressed_keys:
            stale_ids = [
                key_id
                for key_id, seen_at in self._pressed_key_seen_at.items()
                if (now - seen_at) > self._gui_stuck_key_timeout_s
            ]
            for stale_id in stale_ids:
                self._pressed_keys.pop(stale_id, None)
                self._pressed_key_down_at.pop(stale_id, None)
                self._pressed_key_seen_at.pop(stale_id, None)
                self._pending_release_at.pop(stale_id, None)

            due_releases = [
                key_id for key_id, release_at in self._pending_release_at.items() if now >= release_at
            ]
            for key_id in due_releases:
                self._pressed_keys.pop(key_id, None)
                self._pressed_key_down_at.pop(key_id, None)
                self._pressed_key_seen_at.pop(key_id, None)
                self._pending_release_at.pop(key_id, None)

        repeat_tokens = []
        if self._pressed_keys:
            for key_id, token in self._pressed_keys.items():
                down_at = self._pressed_key_down_at.get(key_id, now)
                if (now - down_at) >= self._gui_wait_single_key_timeout_s:
                    repeat_tokens.append(token)
        if repeat_tokens:
            print(f"repeated: {repeat_tokens}")
            self._mock_controller.inject_keys(tuple(repeat_tokens), source="gui")
        else:
            self._mock_controller.clear_active_combo()
        if self._root_window is not None:
            self._root_window.after(self._gui_key_repeat_ms, self._repeat_pressed_keys)

    def _on_copy_selection(self, _event=None):
        if self._root_window is None:
            return "break"
        widget = self._root_window.focus_get()
        text_to_copy = ""

        if widget == self._treeview and self._treeview is not None:
            selected = self._treeview.selection()
            if selected:
                item = self._treeview.item(selected[0])
                values = item.get("values", [])
                text_to_copy = "\t".join(
                    [str(item.get("text", ""))]
                    + [str(v) for v in values]
                )
        elif widget == self._bb_table and self._bb_table is not None:
            selected = self._bb_table.selection()
            if selected:
                item = self._bb_table.item(selected[0])
                values = item.get("values", [])
                text_to_copy = "\t".join([str(v) for v in values])
        elif isinstance(widget, tk.Text):
            try:
                text_to_copy = widget.get("sel.first", "sel.last")
            except Exception:
                text_to_copy = ""

        if text_to_copy:
            self._root_window.clipboard_clear()
            self._root_window.clipboard_append(text_to_copy)
        return "break"

    def _poll_queue(self) -> None:
        if self._closed.is_set():
            return

        latest = None
        while True:
            try:
                latest = self._queue.get_nowait()
            except queue.Empty:
                break

        if latest is None:
            if self._closed.is_set():
                return
        else:
            node_snapshots, blackboard_data = latest
            self._log_state_changes(node_snapshots, blackboard_data)
            self._render_nodes(node_snapshots)
            self._render_blackboard(blackboard_data)
            self._render_mock_status()

        if self._root_window is not None:
            self._root_window.after(33, self._poll_queue)

    def _render_nodes(self, snapshots: List[_NodeSnapshot]) -> None:
        if self._treeview is None:
            return
        current_ids = {snap.node_id for snap in snapshots}
        rebuild_required = (current_ids != self._node_ids) or self._first_node_render

        if rebuild_required:
            open_state = self._collect_open_state()
            for item in self._treeview.get_children():
                self._treeview.delete(item)
            self._node_render_cache = {}
            for snap in snapshots:
                self._treeview.insert(
                    snap.parent_id,
                    tk.END,
                    iid=snap.node_id,
                    text=snap.name,
                    values=(snap.node_type, snap.status, snap.feedback),
                    tags=(snap.status,),
                    open=open_state.get(snap.node_id, True),
                )
                self._node_render_cache[snap.node_id] = (snap.name, snap.node_type, snap.status, snap.feedback)
            self._node_ids = current_ids
            self._first_node_render = False
            return

        # Fast path: update only changed rows on an unchanged tree structure.
        for snap in snapshots:
            new_state = (snap.name, snap.node_type, snap.status, snap.feedback)
            if self._node_render_cache.get(snap.node_id) == new_state:
                continue
            self._treeview.item(snap.node_id, text=snap.name, values=(snap.node_type, snap.status, snap.feedback), tags=(snap.status,))
            self._node_render_cache[snap.node_id] = new_state

    def _render_blackboard(self, bb_data: Dict[str, Any]) -> None:
        if self._bb_table is None:
            return

        current_keys = set(bb_data.keys())
        existing_keys = set(self._bb_row_by_key.keys())
        for key in sorted(existing_keys - current_keys):
            row_id = self._bb_row_by_key.pop(key, None)
            self._bb_value_cache.pop(key, None)
            if row_id:
                try:
                    self._bb_table.delete(row_id)
                except Exception:
                    pass

        for key in sorted(current_keys):
            value_repr = self._format_blackboard_value(bb_data[key])
            previous = self._bb_value_cache.get(key)
            if previous == value_repr:
                continue
            row_id = self._bb_row_by_key.get(key)
            if row_id is None:
                row_id = self._bb_table.insert("", tk.END, values=(key, value_repr))
                self._bb_row_by_key[key] = row_id
            else:
                self._bb_table.item(row_id, values=(key, value_repr))
            self._bb_value_cache[key] = value_repr

    def _build_snapshot(self, root: py_trees.behaviour.Behaviour) -> Tuple[List[_NodeSnapshot], Dict[str, Any]]:
        nodes: List[_NodeSnapshot] = []

        def walk(node: py_trees.behaviour.Behaviour, parent_id: str, local_id: str) -> None:
            status = node.status.name if node.status is not None else "INVALID"
            feedback = getattr(node, "feedback_message", "")
            nodes.append(
                _NodeSnapshot(
                    node_id=local_id,
                    parent_id=parent_id,
                    name=node.name,
                    node_type=type(node).__name__,
                    status=status,
                    feedback=self._safe_repr(feedback),
                )
            )
            for index, child in enumerate(getattr(node, "children", [])):
                walk(child, local_id, f"{local_id}.{index}")

        walk(root, "", "root")
        return nodes, dict(Blackboard.storage)

    def _safe_repr(self, value: Any) -> str:
        text = repr(value)
        text = text.replace("\n", " ")
        if len(text) > self.max_value_length:
            return text[: self.max_value_length - 3] + "..."
        return text

    def _format_blackboard_value(self, value: Any) -> str:
        if isinstance(value, list):
            if len(value) > 6:
                head = ", ".join(self._safe_repr(v) for v in value[:3])
                tail = ", ".join(self._safe_repr(v) for v in value[-2:])
                return f"list(len={len(value)}): [{head}, ..., {tail}]"
            return f"list(len={len(value)}): {self._safe_repr(value)}"
        if isinstance(value, dict):
            keys = list(value.keys())
            if len(keys) > 8:
                return f"dict(len={len(keys)}): keys={keys[:8]}..."
            return f"dict(len={len(keys)}): keys={keys}"
        return self._safe_repr(value)

    def _collect_open_state(self) -> Dict[str, bool]:
        if self._treeview is None:
            return {}
        if self._first_node_render:
            return {}
        state: Dict[str, bool] = {}

        def walk(item_id: str) -> None:
            state[item_id] = bool(self._treeview.item(item_id, "open"))
            for child_id in self._treeview.get_children(item_id):
                walk(child_id)

        for root_id in self._treeview.get_children(""):
            walk(root_id)
        return state

    def _render_mock_status(self) -> None:
        if self._mock_static_text is None or self._mock_dynamic_text is None:
            return

        cfg = get_config()
        controller = self._mock_controller.get_status_snapshot()
        subsystems = cfg._mock_config.get("mock_mode", {}).get("subsystems", {})
        subsystem_names = sorted(subsystems.keys())
        key_map = controller.get("subsystem_start_keys", {})
        active_subsystem = controller.get("active_subsystem")
        input_enabled = bool(controller.get("input_enabled"))
        broadcast_all = bool(controller.get("broadcast_all_subsystems"))
        queue_sizes = controller.get("queue_sizes", {})

        static_lines = [
            f"mock_mode: {'ENABLED' if cfg.is_mock_mode() else 'DISABLED'}",
            f"keyboard_control: {'ENABLED' if should_use_keyboard_control() else 'DISABLED'}",
            (
                f"global_controls: start='{controller['start_input_key']}', "
                f"stop='{controller['stop_input_key']}', success={controller['success_key']}"
            ),
            "subsystems:",
        ]
        for subsystem in subsystem_names:
            subsystem_cfg = subsystems.get(subsystem, {})
            mocked = bool(isinstance(subsystem_cfg, dict) and subsystem_cfg.get("enabled", False))
            key = key_map.get(subsystem, "-")
            static_lines.append(
                f"  - {subsystem}: mocked={'YES' if mocked else 'NO'}, enable_key='{key}'"
            )

        dynamic_lines = [
            (
                f"input_forwarding: {'ENABLED' if input_enabled else 'DISABLED'} "
                f"| mode={'ALL' if broadcast_all else 'SUBSYSTEM'} "
                f"| target={(active_subsystem or 'ALL') if input_enabled else '-'}"
            ),
            (
                f"controller: running={controller['running']} "
                f"| last_key={repr(controller['last_key'])} from {controller['last_key_source'] or '-'} "
                f"| tick={controller.get('tick_index', '-')}"
            ),
            "subsystems (forwarding):",
        ]
        for subsystem in subsystem_names:
            if not input_enabled:
                forwarding = "NO"
            elif broadcast_all:
                forwarding = "YES"
            else:
                forwarding = "YES" if active_subsystem == subsystem else "NO"
            dynamic_lines.append(
                f"  - {subsystem}: forwarding={forwarding}, queue={queue_sizes.get(subsystem, 0)}"
            )
        dynamic_lines.append("GUI input: focus this window and press subsystem/start/stop/success keys.")
        self._mock_static_text.configure(state=tk.NORMAL)
        self._mock_static_text.delete("1.0", tk.END)
        self._mock_static_text.insert(tk.END, "\n".join(static_lines))
        self._mock_static_text.configure(state=tk.DISABLED)
        self._mock_dynamic_text.configure(state=tk.NORMAL)
        self._mock_dynamic_text.delete("1.0", tk.END)
        dynamic_text = "\n".join(dynamic_lines)
        self._mock_dynamic_text.insert(tk.END, dynamic_text)
        self._mock_dynamic_text.configure(state=tk.DISABLED)
        self._log_mock_runtime_change(dynamic_text)

    def _configure_styles(self) -> None:
        if ttk is None or tkfont is None:
            return
        style = ttk.Style()
        self._tree_body_font = tkfont.Font(family="DejaVu Sans Mono", size=12)
        self._tree_heading_font = tkfont.Font(family="DejaVu Sans", size=12, weight="bold")
        self._mock_body_font = tkfont.Font(family="DejaVu Sans Mono", size=11)
        rowheight = max(30, self._tree_body_font.metrics("linespace") + 12)
        style.configure("Treeview", font=self._tree_body_font, rowheight=rowheight)
        style.configure("Treeview.Heading", font=self._tree_heading_font)

    def _timestamp(self) -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def _open_state_log_file(self) -> None:
        try:
            log_dir = Path.cwd() / "bt_visualization_logs"
            log_dir.mkdir(parents=True, exist_ok=True)
            filename = f"bt_gui_state_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
            self._state_log_path = log_dir / filename
            self._state_log_file = self._state_log_path.open("a", encoding="utf-8", buffering=1)
            self._state_log_file.write(f"{self._timestamp()} [INFO] GUI log started: {self._state_log_path}\n")
            print(f"[BT GUI] state log: {self._state_log_path}")
            self._log_stop.clear()
            self._log_thread = threading.Thread(target=self._log_writer_loop, daemon=True)
            self._log_thread.start()
        except Exception:
            self._state_log_file = None
            self._state_log_path = None

    def _write_log_line(self, level: str, text: str) -> None:
        if self._state_log_file is None or self._log_stop.is_set():
            return
        try:
            self._log_queue.put_nowait(f"{self._timestamp()} [{level}] {text}\n")
        except Exception:
            pass

    def _log_writer_loop(self) -> None:
        while not self._log_stop.is_set():
            try:
                line = self._log_queue.get(timeout=0.2)
            except queue.Empty:
                continue
            try:
                if self._state_log_file is not None:
                    self._state_log_file.write(line)
            except Exception:
                pass
        # drain remaining lines on shutdown
        while True:
            try:
                line = self._log_queue.get_nowait()
            except queue.Empty:
                break
            try:
                if self._state_log_file is not None:
                    self._state_log_file.write(line)
            except Exception:
                pass

    def _log_state_changes(self, node_snapshots: List[_NodeSnapshot], blackboard_data: Dict[str, Any]) -> None:
        node_map = {snap.node_id: (snap.status, snap.feedback, snap.name) for snap in node_snapshots}
        for node_id, (status, feedback, name) in node_map.items():
            prev = self._last_node_state.get(node_id)
            if prev is None:
                self._write_log_line("NODE", f"ADDED id={node_id} name={name} status={status} feedback={feedback}")
            elif prev != (status, feedback):
                self._write_log_line(
                    "NODE",
                    (
                        f"UPDATED id={node_id} name={name} "
                        f"status={prev[0]}->{status} feedback={prev[1]}->{feedback}"
                    ),
                )
            self._last_node_state[node_id] = (status, feedback)
        removed_node_ids = set(self._last_node_state.keys()) - set(node_map.keys())
        for node_id in sorted(removed_node_ids):
            prev = self._last_node_state.pop(node_id, None)
            if prev is not None:
                self._write_log_line("NODE", f"REMOVED id={node_id} status={prev[0]} feedback={prev[1]}")

        bb_map = {key: self._format_blackboard_value(value) for key, value in blackboard_data.items()}
        for key, value in bb_map.items():
            prev = self._last_bb_state.get(key)
            if prev is None:
                self._write_log_line("BLACKBOARD", f"SET key={key} value={value}")
            elif prev != value:
                self._write_log_line("BLACKBOARD", f"UPDATED key={key} value={prev}->{value}")
            self._last_bb_state[key] = value
        removed_bb_keys = set(self._last_bb_state.keys()) - set(bb_map.keys())
        for key in sorted(removed_bb_keys):
            prev = self._last_bb_state.pop(key, None)
            if prev is not None:
                self._write_log_line("BLACKBOARD", f"REMOVED key={key} previous={prev}")

    def _log_mock_runtime_change(self, runtime_text: str) -> None:
        if runtime_text != self._last_mock_runtime:
            self._write_log_line("MOCK", runtime_text.replace("\n", " | "))
            self._last_mock_runtime = runtime_text


def create_post_tick_visualizer(
    *,
    title: str = "Behavior Tree Monitor",
    print_blackboard: bool = False,
    enable_gui: bool = True,
    warn_on_fallback: bool = True,
) -> Tuple[Callable[[Any], None], Callable[[], None], bool]:
    """
    Create a unified post tick handler and cleanup hook.

    Returns:
        post_tick_handler: pass to tree.tick_tock(..., post_tick_handler=...)
        shutdown: call in `finally`
        gui_started: True if tkinter dashboard is active
    """
    gui = BehaviorTreeStatusGUI(title=title) if enable_gui else None
    gui_started = gui.start() if gui is not None else False
    mock_controller = get_mock_input_controller()
    if enable_gui and warn_on_fallback and not gui_started:
        warnings.warn("BehaviorTreeStatusGUI unavailable, falling back to console output.")

    def post_tick_handler(tree: Any) -> None:
        if gui_started and gui is not None:
            gui.update(tree.root)
        else:
            print(py_trees.display.unicode_tree(root=tree.root, show_status=True))
            if print_blackboard:
                print(py_trees.display.unicode_blackboard())
        mock_controller.end_tick_cycle()

    def shutdown() -> None:
        if gui is not None:
            gui.shutdown()

    return post_tick_handler, shutdown, gui_started
