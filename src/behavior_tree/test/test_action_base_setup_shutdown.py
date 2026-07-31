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

"""Regression test: ActionHandler.setup() must exit cleanly on context shutdown.

Root cause this guards against
------------------------------
When an action server is missing, ``ActionHandler.setup`` (with the
``wait_for_server_timeout_sec <= 0`` convention used across all action nodes)
retries ``wait_for_server`` indefinitely. rclpy's ``ActionClient.wait_for_server``
loops ``while node.context.ok() and not server_is_ready()`` then calls
``server_is_ready()`` one final time UNCONDITIONALLY. If a shutdown signal
(Ctrl-C / SIGTERM / launcher teardown) invalidates the rclpy context during the
wait, that final call touches a node whose context is gone and raises
``RCLError: rcl node's context is invalid`` instead of a clean shutdown.

The retry loop must detect context shutdown and raise the documented setup
failure (``TimedOutError``) instead of letting the raw ``RCLError`` escape.
"""

import os

os.environ["BT_MOCK_MODE"] = "false"  # noqa: E402  force the real action path

import pytest  # noqa: E402
import rclpy.action  # noqa: E402
from rclpy.impl.implementation_singleton import (  # noqa: E402
    rclpy_implementation as _rclpy,
)
from py_trees_ros import exceptions  # noqa: E402

from behavior_tree.nodes.ActionBase import ActionHandler  # noqa: E402


class _FakeContext:
    """rclpy context stub whose ok() flips to False after N checks."""

    def __init__(self, ok_for_checks):
        self._remaining = ok_for_checks

    def ok(self):
        if self._remaining > 0:
            self._remaining -= 1
            return True
        return False


class _FakeNode:
    """Minimal rclpy node stub exposing only what setup() touches."""

    def __init__(self, context):
        self.context = context

    def get_logger(self):
        return self

    # logger surface used by setup()
    def warning(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _FakeActionClient:
    """Mimics rclpy ActionClient.wait_for_server around a dying context.

    While the context is ok the server is "missing" (returns False). Once the
    context is no longer ok, the real rclpy method's final unconditional
    ``server_is_ready()`` raises ``RCLError``; we reproduce that exactly.
    """

    def __init__(self, node, **_kwargs):
        self._node = node

    def wait_for_server(self, timeout_sec=None):
        if not self._node.context.ok():
            raise _rclpy.RCLError(
                "Failed to check if action server is available: "
                "rcl node's context is invalid, at ./src/rcl/node.c:428"
            )
        return False  # server missing -> caller retries


def _make_handler():
    # action_type=None / key=None is the shape BtNode_TrackPersonAction uses
    # before it injects the real action type in its own setup().
    return ActionHandler(
        name="track",
        action_type=object,
        action_name="track_person",
        key=None,
        wait_for_server_timeout_sec=-1.0,  # infinite-retry convention
    )


def test_setup_raises_timedout_not_rclerror_on_context_shutdown(monkeypatch):
    """A context shutdown mid-wait must surface as a clean TimedOutError."""
    monkeypatch.setenv("BT_MOCK_MODE", "false")
    monkeypatch.setattr(rclpy.action, "ActionClient", _FakeActionClient)

    # Context survives one wait_for_server (server missing), then shuts down.
    node = _FakeNode(_FakeContext(ok_for_checks=1))
    handler = _make_handler()

    with pytest.raises(exceptions.TimedOutError):
        handler.setup(node=node)


def test_setup_does_not_leak_rclerror(monkeypatch):
    """The raw RCLError from rclpy's wait_for_server must never escape setup()."""
    monkeypatch.setattr(rclpy.action, "ActionClient", _FakeActionClient)

    node = _FakeNode(_FakeContext(ok_for_checks=0))  # context already down
    handler = _make_handler()

    try:
        handler.setup(node=node)
    except _rclpy.RCLError as exc:  # pragma: no cover - this is the bug
        pytest.fail(f"setup() leaked raw RCLError instead of handling it: {exc}")
    except exceptions.TimedOutError:
        pass  # expected clean failure
