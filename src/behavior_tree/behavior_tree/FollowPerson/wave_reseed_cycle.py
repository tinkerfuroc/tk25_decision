# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Shared wave->reseed async machine used by the NEEDS_HELP reactions.

``WaveReseedCycle`` drives a throttled ``DetectWaving`` -> ``ReseedTarget`` cycle
through a small IDLE -> WAVE_PENDING -> RESEED_PENDING state machine, at most one
service call in flight, advanced one step per ``step()`` call. It is ROS-free
(driven by an injected ``bridge``) so it unit-tests with a fake bridge.
``_WaveReseedBridge`` is the live ROS adapter (lazy tk26 msg import).
"""

import time


class _WaveReseedBridge:
    """Live ROS bridge: async DetectWaving + ReseedTarget.

    ``detect_async()`` returns an rclpy Future of a ``DetectWaving.Response``
    (read ``.status`` + ``.waving_boxes``); ``reseed_async(box)`` returns a Future
    of a ``ReseedTarget.Response`` (read ``.success``). ``box`` is
    ``(x0, y0, x1, y1)`` in the current color frame.
    """

    def __init__(self, node, waving_service, reseed_service):
        # Lazy import (live path only) so mock/test never need tk26 msgs.
        from behavior_tree.messages import DetectWaving, ReseedTarget
        self._DetectWaving = DetectWaving
        self._ReseedTarget = ReseedTarget
        self._wave_cli = node.create_client(DetectWaving, waving_service)
        self._reseed_cli = node.create_client(ReseedTarget, reseed_service)

    def detect_async(self):
        # min_waving_persons=0 (default) -> fast MediaPipe-only path (no VLM
        # fallback), well inside the vision budget.
        return self._wave_cli.call_async(self._DetectWaving.Request())

    def reseed_async(self, box):
        req = self._ReseedTarget.Request()
        x0, y0, x1, y1 = box
        req.bbox.x_offset = max(0, int(x0))
        req.bbox.y_offset = max(0, int(y0))
        req.bbox.width = max(0, int(x1 - x0))
        req.bbox.height = max(0, int(y1 - y0))
        req.frame_id = ""
        return self._reseed_cli.call_async(req)


class WaveReseedCycle:
    """Throttled DetectWaving -> ReseedTarget cycle, advanced one step per tick.

    On an UNAMBIGUOUS single waver (status OK, exactly one box) it reseeds the
    tracker onto that box; zero/multiple wavers or a non-OK status -> NO reseed
    (precision: never re-lock onto an ambiguous candidate). DetectWaving is
    throttled to ``throttle_s`` (measured from the start of the previous scan).
    """

    _IDLE, _WAVE_PENDING, _RESEED_PENDING = "idle", "wave_pending", "reseed_pending"

    def __init__(self, bridge=None, throttle_s: float = 3.0, clock=time.monotonic):
        self._bridge = bridge
        self.throttle_s = throttle_s
        self._clock = clock
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._last_detect_time = -1e9

    def set_bridge(self, bridge):
        self._bridge = bridge

    @property
    def has_bridge(self) -> bool:
        return self._bridge is not None

    def reset(self) -> None:
        """Cancel any in-flight cycle (does NOT reset the throttle clock)."""
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None

    def step(self) -> str:
        """Advance one step of IDLE -> WAVE_PENDING -> RESEED_PENDING."""
        now = self._clock()

        if self._phase == self._IDLE:
            if (now - self._last_detect_time) >= self.throttle_s:
                self._future = self._bridge.detect_async()
                self._last_detect_time = now
                self._phase = self._WAVE_PENDING
                return "Scanning for a waving operator"
            return "Waiting to re-scan"

        if self._phase == self._WAVE_PENDING:
            if self._future is None or not self._future.done():
                return "Awaiting waving result"
            resp = self._future.result()
            self._future = None
            box = self._single_waver_box(resp)
            if box is None:
                self._phase = self._IDLE
                return "No unambiguous single waver"
            self._future = self._bridge.reseed_async(box)
            self._pending_box = box
            self._phase = self._RESEED_PENDING
            return "Single waver found; reseeding tracker"

        if self._phase == self._RESEED_PENDING:
            if self._future is None or not self._future.done():
                return "Awaiting reseed result"
            resp = self._future.result()
            ok = bool(getattr(resp, "success", False))
            self._future = None
            self._pending_box = None
            self._phase = self._IDLE
            return "Reseed succeeded" if ok else "Reseed did not match a person"

        return ""

    @staticmethod
    def _single_waver_box(resp):
        """``(x0, y0, x1, y1)`` iff exactly one waver with status OK, else None."""
        if resp is None or int(getattr(resp, "status", -1)) != 0:
            return None
        boxes = list(getattr(resp, "waving_boxes", []) or [])
        if len(boxes) != 1:
            return None
        b = boxes[0]
        x0 = int(b.x_offset)
        y0 = int(b.y_offset)
        return (x0, y0, x0 + int(b.width), y0 + int(b.height))
