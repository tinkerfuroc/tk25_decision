# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License"); see
# http://www.apache.org/licenses/LICENSE-2.0
"""Trigger-driven wave->reseed async machine for NEEDS_HELP recovery.

``WaveReseedCycle`` fires ONE ``DetectWaving`` (gated to ``distance_threshold_m``)
when ``trigger()`` has been called, then walks IDLE -> WAVE_PENDING ->
RESEED_PENDING, at most one request in flight, one transition per ``step()``.
It does NOT free-run: without a ``trigger()`` it stays idle. ROS-free (injected
``bridge``) so it unit-tests with a fake. ``_WaveReseedBridge`` is the live ROS
adapter (lazy tk26 msg import). The recovery FSM fires one detect per scan angle
AFTER the head has settled and waits for the response (``is_idle``) before
advancing.
"""


class _ActionPayloadFuture:
    """Expose an action's payload through the Future contract used by the FSM."""

    def __init__(self, send_future):
        from concurrent.futures import Future

        self._future = Future()
        self._goal_handle = None
        self._cancel_requested = False
        send_future.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            self._goal_handle = future.result()
            if self._goal_handle is None or not self._goal_handle.accepted:
                raise RuntimeError("DetectWaving action goal rejected")
            if self._cancel_requested or self._future.cancelled():
                self._goal_handle.cancel_goal_async()
                return
            result_future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self._result_response)
        except Exception as exc:
            if not self._future.done():
                self._future.set_exception(exc)

    def _result_response(self, future):
        if self._future.done():
            return
        try:
            self._future.set_result(future.result().result)
        except Exception as exc:
            self._future.set_exception(exc)

    def done(self):
        return self._future.done()

    def result(self):
        return self._future.result()

    def cancel(self):
        self._cancel_requested = True
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        return self._future.cancel()


class _WaveReseedBridge:
    """Live ROS bridge: async DetectWaving + ReseedTarget.

    ``detect_async(threshold_m)`` requests wavers within ``threshold_m`` of the
    camera (the server drops any waver whose centroid depth exceeds it) and
    returns a Future of a ``DetectWaving.Result`` payload (read ``.status`` +
    ``.waving_boxes``). ``reseed_async(box)`` returns a Future of a
    ``ReseedTarget.Response`` (read ``.success``). ``box`` is ``(x0,y0,x1,y1)``.
    """

    def __init__(self, node, waving_service, reseed_service):
        # Lazy import (live path only) so mock/test never need tk26 msgs.
        from rclpy.action import ActionClient
        from behavior_tree.interfaces.messages import DetectWavingAction, ReseedTarget
        self._DetectWaving = DetectWavingAction
        self._ReseedTarget = ReseedTarget
        self._wave_cli = ActionClient(
            node, DetectWavingAction, waving_service)
        self._reseed_cli = node.create_client(ReseedTarget, reseed_service)

    def detect_async(self, threshold_m):
        goal = self._DetectWaving.Goal()
        goal.threshold_meters = float(threshold_m)
        # min_waving_persons=0 -> fast MediaPipe-only path (no VLM fallback).
        goal.min_waving_persons = 0
        return _ActionPayloadFuture(self._wave_cli.send_goal_async(goal))

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
    """One-shot, trigger-driven DetectWaving -> ReseedTarget cycle.

    ``trigger()`` arms one detect; the next ``step()`` while IDLE fires it (gated
    to ``distance_threshold_m``). On an UNAMBIGUOUS single waver (status OK,
    exactly one box) it reseeds the tracker; zero/multiple wavers or a non-OK
    status -> NO reseed. ``is_idle`` is False from ``trigger()`` until the
    round-trip completes, so the caller can wait on the response.
    """

    _IDLE, _WAVE_PENDING, _RESEED_PENDING = "idle", "wave_pending", "reseed_pending"

    def __init__(self, bridge=None, distance_threshold_m: float = 3.5):
        self._bridge = bridge
        self.distance_threshold_m = float(distance_threshold_m)
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._armed = False

    def set_bridge(self, bridge):
        self._bridge = bridge

    @property
    def has_bridge(self) -> bool:
        return self._bridge is not None

    @property
    def is_idle(self) -> bool:
        """True iff no detect is armed and no request is in flight."""
        return self._phase == self._IDLE and not self._armed

    def trigger(self) -> None:
        """Arm one detect; the next IDLE ``step()`` fires it."""
        self._armed = True

    def reset(self) -> None:
        """Cancel any armed/in-flight cycle."""
        if self._future is not None and hasattr(self._future, "cancel"):
            self._future.cancel()
        self._phase = self._IDLE
        self._future = None
        self._pending_box = None
        self._armed = False

    def step(self) -> str:
        """Advance one transition of IDLE -> WAVE_PENDING -> RESEED_PENDING."""
        if self._phase == self._IDLE:
            if self._armed:
                self._armed = False
                if self._bridge is not None:
                    self._future = self._bridge.detect_async(
                        self.distance_threshold_m)
                    self._phase = self._WAVE_PENDING
                    return f"Scanning for a waver (<= {self.distance_threshold_m:.1f} m)"
            return "Idle"

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
