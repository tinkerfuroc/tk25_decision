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

"""Unit tests for CoalescingTTS (non-overlapping latest-wins speaker)."""

from behavior_tree.FollowPerson.coalescing_tts import CoalescingTTS


class _FakeSpeaker:
    """Records started utterances; completion is driven manually per handle."""

    def __init__(self):
        self.started = []

    def start(self, text):
        handle = {"text": text, "done": False}
        self.started.append(handle)
        return handle

    @staticmethod
    def is_done(handle):
        return handle["done"]

    def texts(self):
        return [h["text"] for h in self.started]

    def active_count(self):
        return sum(1 for h in self.started if not h["done"])


def _make():
    fake = _FakeSpeaker()
    return fake, CoalescingTTS(start=fake.start, is_done=fake.is_done)


def test_idle_submit_speaks_immediately():
    fake, tts = _make()
    tts.submit("a")
    assert fake.texts() == ["a"]
    assert tts._pending is None


def test_submit_while_active_queues_as_pending():
    fake, tts = _make()
    tts.submit("a")
    tts.submit("b")
    assert fake.texts() == ["a"]  # b not started yet
    assert tts._pending == "b"


def test_second_submit_while_active_overwrites_pending():
    fake, tts = _make()
    tts.submit("a")
    tts.submit("b")
    tts.submit("c")
    assert fake.texts() == ["a"]  # still only "a" speaking
    assert tts._pending == "c"  # "b" dropped, latest wins


def test_poll_after_completion_starts_pending():
    fake, tts = _make()
    tts.submit("a")
    tts.submit("b")
    tts.submit("c")
    # Mark "a" done, advance the queue.
    fake.started[0]["done"] = True
    tts.poll()
    assert fake.texts() == ["a", "c"]
    assert tts._pending is None


def test_never_two_active_at_once():
    fake, tts = _make()
    tts.submit("a")
    tts.submit("b")
    # While "a" still active, poll() must not start "b".
    tts.poll()
    assert fake.active_count() == 1
    assert fake.texts() == ["a"]
    # Complete "a"; poll starts "b"; still only one active.
    fake.started[0]["done"] = True
    tts.poll()
    assert fake.active_count() == 1
    assert fake.texts() == ["a", "b"]


def test_poll_clears_active_when_done_and_no_pending():
    fake, tts = _make()
    tts.submit("a")
    fake.started[0]["done"] = True
    tts.poll()
    assert tts._active is None
    # A fresh submit speaks immediately again.
    tts.submit("d")
    assert fake.texts() == ["a", "d"]
