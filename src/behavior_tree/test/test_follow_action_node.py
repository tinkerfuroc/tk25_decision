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

"""Mock-mode unit tests for BtNode_FollowAction (continuous Follow action)."""

import os

os.environ["BT_MOCK_MODE"] = "true"  # noqa: E402 — force mock before config loads

import py_trees  # noqa: E402
import pytest  # noqa: E402

from behavior_tree.nodes.FollowAction import BtNode_FollowAction  # noqa: E402


@pytest.fixture(autouse=True)
def _clear_blackboard():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()
    yield
    if callable(clear_fn):
        clear_fn()


def _make_node():
    node = BtNode_FollowAction(
        name="Follow",
        use_breadcrumbs=True,
        timeout=0.0,
    )
    # Mock mode skips the action-client setup; pass a dummy node so the
    # blackboard writer is registered.
    node.setup(node=None)
    return node


def test_constructs_in_mock_mode():
    node = BtNode_FollowAction(name="Follow", use_breadcrumbs=True, timeout=0.0)
    assert node.mock_mode is True
    assert node.use_breadcrumbs is True
    assert node.timeout == 0.0


def test_mock_runs_and_reports_state():
    node = _make_node()
    # Continuous action: RUNNING while "following".
    node.tick_once()
    assert node.status == py_trees.common.Status.RUNNING

    # Mock mode seeds synthetic feedback on the blackboard.
    bb = node.attach_blackboard_client(name="reader")
    bb.register_key(
        key=BtNode_FollowAction.DEFAULT_BB_KEY_STATE,
        access=py_trees.common.Access.READ,
    )
    assert bb.get(BtNode_FollowAction.DEFAULT_BB_KEY_STATE) is not None


def test_mock_writes_all_feedback_keys():
    node = _make_node()
    node.tick_once()

    bb = node.attach_blackboard_client(name="reader_all")
    for key in (
        BtNode_FollowAction.DEFAULT_BB_KEY_STATE,
        BtNode_FollowAction.DEFAULT_BB_KEY_DISTANCE,
        BtNode_FollowAction.DEFAULT_BB_KEY_REACQ,
    ):
        bb.register_key(key=key, access=py_trees.common.Access.READ)

    assert bb.get(BtNode_FollowAction.DEFAULT_BB_KEY_STATE) is not None
    assert bb.get(BtNode_FollowAction.DEFAULT_BB_KEY_DISTANCE) is not None
    assert bb.get(BtNode_FollowAction.DEFAULT_BB_KEY_REACQ) is not None


def test_mock_stays_running_across_ticks():
    node = _make_node()
    # KEYPRESS mock mode keeps RUNNING until the parent cancels (no key press).
    for _ in range(5):
        node.tick_once()
        assert node.status == py_trees.common.Status.RUNNING
