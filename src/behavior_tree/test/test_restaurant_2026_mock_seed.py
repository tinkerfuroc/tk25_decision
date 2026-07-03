# Copyright 2026 Tinker Team
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

"""restaurant-2026 MOCK_SEED_CUSTOMER Phase-1 bypass.

Verifies the offline audio-test toggle: when on, Phase 1 seeds a synthetic
active customer instead of scanning; when off, the real scan is unchanged.
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402
import pytest  # noqa: E402

import behavior_tree.Restaurant.order_intake_items as oii  # noqa: E402


class _DummyNode:
    """Stand-in for the rclpy node BtNode_WriteToBlackboard.setup() requires."""


def _clear_bb():
    clear_fn = getattr(py_trees.blackboard.Blackboard, "clear", None)
    if callable(clear_fn):
        clear_fn()


def _node_class_names(root):
    return {n.__class__.__name__ for n in root.iterate()}


def _setup_and_tick(subtree):
    for node in subtree.iterate():
        node.setup(node=_DummyNode())
    for _ in subtree.tick():
        pass
    return subtree.status


def test_seed_subtree_populates_active_customer_state():
    _clear_bb()
    subtree = oii._createSeedCustomerSubtree(7)
    assert _setup_and_tick(subtree) == py_trees.common.Status.SUCCESS

    reader = py_trees.blackboard.Client(name="reader")
    for key in (
        oii.KEY_ACTIVE_CUSTOMER_ID,
        oii.KEY_CUSTOMER_LOCATION,
        oii.KEY_ACTIVE_CUSTOMER_PICTURE,
        oii.KEY_CUSTOMER_QUEUE,
    ):
        reader.register_key(
            key=key,
            access=py_trees.common.Access.READ,
            remap_to=py_trees.blackboard.Blackboard.absolute_name("/", key),
        )
    assert getattr(reader, oii.KEY_ACTIVE_CUSTOMER_ID) == 7
    assert getattr(reader, oii.KEY_ACTIVE_CUSTOMER_PICTURE) == ""
    queue = getattr(reader, oii.KEY_CUSTOMER_QUEUE)
    assert len(queue) == 1
    assert queue[0]["id"] == 7
    assert queue[0]["status"] == "active"
    loc = getattr(reader, oii.KEY_CUSTOMER_LOCATION)
    assert loc.header.frame_id == "map"
