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

"""Unit tests for FeedbackBuffer reacquisition_state (TrackPersonAction)."""

from types import SimpleNamespace

from behavior_tree.nodes.TrackPersonAction import FeedbackBuffer


def _make_feedback(reacquisition_state):
    return SimpleNamespace(
        target_lost=False,
        target_track_id=7,
        is_transformation_successful=True,
        target_position=None,
        reacquisition_state=reacquisition_state,
    )


def test_default_reacquisition_state_is_zero():
    buf = FeedbackBuffer()
    state = buf.get_state()
    # reacquisition_state is the last element of the state tuple.
    assert state[-1] == 0


def test_update_records_reacquisition_state():
    buf = FeedbackBuffer()
    buf.update(_make_feedback(2))
    state = buf.get_state()
    assert state[-1] == 2


def test_clear_resets_reacquisition_state():
    buf = FeedbackBuffer()
    buf.update(_make_feedback(2))
    assert buf.get_state()[-1] == 2
    buf.clear()
    assert buf.get_state()[-1] == 0


def test_missing_reacquisition_state_defaults_to_zero():
    buf = FeedbackBuffer()
    fb = SimpleNamespace(
        target_lost=False,
        target_track_id=3,
        is_transformation_successful=True,
        target_position=None,
    )
    buf.update(fb)
    assert buf.get_state()[-1] == 0
