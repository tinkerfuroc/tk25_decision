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

"""Unit tests for BtNode_ReacqAnnounce (reacq-driven voice announcements)."""

import py_trees
import pytest

from behavior_tree.components.following.nodes import BtNode_ReacqAnnounce

PASSIVE_TEXT = "Please slow down so I can keep up."


class _FakeClock:
    """Mutable monotonic clock for deterministic throttle tests."""

    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class _FakeCoalescer:
    """Records submit() calls; poll() is a no-op."""

    def __init__(self):
        self.submitted = []

    def submit(self, text):
        self.submitted.append(text)

    def poll(self):
        pass


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _writer(bb_key):
    client = py_trees.blackboard.Client(name="reacq_writer")
    client.register_key(key=bb_key, access=py_trees.common.Access.WRITE)
    return client


def _make_node(clock, fake_tts, bb_key="track/reacquisition_state"):
    node = BtNode_ReacqAnnounce(
        name="ReacqAnnounce",
        passive_text=PASSIVE_TEXT,
        throttle_s=5.0,
        bb_key=bb_key,
        clock=clock,
    )
    # Inject the fake speaker, bypassing the ROS setup() path.
    node.inject_coalescer(fake_tts)
    return node


def test_full_cadence_sequence():
    clock = _FakeClock()
    fake = _FakeCoalescer()
    bb_key = "track/reacquisition_state"
    writer = _writer(bb_key)
    node = _make_node(clock, fake, bb_key)

    # TRACKING (0): no announcement.
    writer.set(bb_key, 0, overwrite=True)
    assert node.tick_once() is None or node.status == py_trees.common.Status.SUCCESS
    assert node.status == py_trees.common.Status.SUCCESS
    assert fake.submitted == []

    # Transition to PASSIVE (1): announce once.
    writer.set(bb_key, 1, overwrite=True)
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert fake.submitted == [PASSIVE_TEXT]

    # Still PASSIVE, advance <5 s: no new announcement.
    clock.advance(3.0)
    writer.set(bb_key, 1, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT]

    # Still PASSIVE, cross >=5 s since last announce: re-announce passive.
    clock.advance(3.0)  # now 6.0, >= 5 since last announce at 0.0
    writer.set(bb_key, 1, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT]

    # Transition to NEEDS_HELP (2): the announcer is now PASSIVE-only and stays
    # silent here — BtNode_RecoveryScan owns all NEEDS_HELP speech.
    writer.set(bb_key, 2, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT]

    # Back to TRACKING (0): no announcement, resets last-announced state.
    writer.set(bb_key, 0, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT]

    # Re-enter PASSIVE immediately (no clock advance): announces right away.
    writer.set(bb_key, 1, overwrite=True)
    node.tick_once()
    assert fake.submitted == [PASSIVE_TEXT, PASSIVE_TEXT, PASSIVE_TEXT]
    assert node.status == py_trees.common.Status.SUCCESS


def test_unset_blackboard_defaults_to_tracking():
    clock = _FakeClock()
    fake = _FakeCoalescer()
    node = _make_node(clock, fake)
    # No write to blackboard at all -> treated as TRACKING (0), no submit.
    node.tick_once()
    assert node.status == py_trees.common.Status.SUCCESS
    assert fake.submitted == []


def test_poll_called_every_tick():
    clock = _FakeClock()

    class _CountingCoalescer(_FakeCoalescer):
        def __init__(self):
            super().__init__()
            self.polls = 0

        def poll(self):
            self.polls += 1

    fake = _CountingCoalescer()
    bb_key = "track/reacquisition_state"
    writer = _writer(bb_key)
    node = _make_node(clock, fake, bb_key)
    writer.set(bb_key, 0, overwrite=True)
    node.tick_once()
    node.tick_once()
    assert fake.polls == 2
