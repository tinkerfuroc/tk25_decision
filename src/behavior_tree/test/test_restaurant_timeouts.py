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

"""The restaurant approach subtree must carry a tick-count backstop.

With ``action_timeout_ticks=0`` (the ActionHandler default) and the
GoToApproach feedback callback refreshing ``feedback_timeout`` every frame,
a hung action server blocks the tree forever. The backstop must exceed the
server's own 75 s total timeout (so the server normally terminates first)
at the restaurant tick period of 500 ms.
"""


def _find_approach_nodes(behaviour):
    found = []
    for child in behaviour.iterate():
        if child.__class__.__name__ == "BtNode_Approach":
            found.append(child)
    return found


def test_approach_subtree_has_tick_backstop():
    from behavior_tree.Restaurant.restaurants import _approachCustomerSubtree

    subtree = _approachCustomerSubtree()
    approach_nodes = _find_approach_nodes(subtree)
    assert approach_nodes, "no BtNode_Approach found in approach subtree"
    for node in approach_nodes:
        ticks = node.action_timeout_ticks
        assert ticks > 0, f"{node.name}: action_timeout_ticks=0 disables the backstop"
        # 75 s server total < ticks * 0.5 s period: server terminates first.
        assert ticks * 0.5 > 75.0, (
            f"{node.name}: backstop {ticks * 0.5:.0f}s must exceed the "
            f"server's 75s nav_total_timeout_sec"
        )
