# Copyright 2025 Tinker Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#
# CoalescingTTS Module
# ====================
#
# A pure, hardware-free single-slot speaker that guarantees:
#   - At most one utterance is "active" (speaking) at a time (no overlap).
#   - At most one utterance is "pending", and it is always the newest one
#     submitted while another was speaking (latest-wins; older queued texts
#     are dropped).
#
# The class is decoupled from ROS via two injected callables so it can be
# unit-tested with fakes:
#   - ``start(text) -> handle``  begins speaking ``text`` and returns an opaque
#     handle (in production: a Future from ``client.call_async(...)``).
#   - ``is_done(handle) -> bool``  True once that utterance has finished (in
#     production: ``future.done()``).
#
# Call :meth:`submit` to request speech and :meth:`poll` every tick to advance
# the single-slot queue.
#


class CoalescingTTS:
    """Non-overlapping, latest-wins single-slot speaker.

    Attributes:
        _active: Handle of the utterance currently speaking, or ``None``.
        _pending: Newest queued text awaiting the active slot, or ``None``.
    """

    def __init__(self, start, is_done):
        """Construct the coalescing speaker.

        Args:
            start: Callable ``start(text) -> handle`` that begins speaking
                ``text`` and returns an opaque completion handle.
            is_done: Callable ``is_done(handle) -> bool`` that reports whether
                the utterance for ``handle`` has finished.
        """
        self._start = start
        self._is_done = is_done
        self._active = None
        self._pending = None

    def submit(self, text):
        """Request that ``text`` be spoken.

        If nothing is currently speaking, ``text`` starts immediately.
        Otherwise it overwrites any pending text (latest-wins): only the most
        recent submission survives to be spoken next.

        Args:
            text: The utterance to speak.
        """
        if self._active is None:
            self._active = self._start(text)
        else:
            self._pending = text

    def poll(self):
        """Advance the single-slot queue; call once per tick.

        Clears the active slot once its utterance is done, then promotes the
        pending text (if any) into the active slot by starting it.
        """
        if self._active is not None and self._is_done(self._active):
            self._active = None
        if self._active is None and self._pending is not None:
            self._active = self._start(self._pending)
            self._pending = None
