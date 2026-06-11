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

"""Both bar trips drive through ANCHOR_NEAREST_FREE projection (P3-BT).

The two "Go to kitchen bar" navigations (Phase-2 barman trip and the
per-item Phase-3 pickup verification) must first project the raw bar anchor
through the ``find_approach_pose`` service in ANCHOR_NEAREST_FREE mode, then
drive to the projected pose. On projection failure or an unavailable service
the tree degrades gracefully to the raw-anchor ``BtNode_GotoAction``.

Tree shape per bar trip (inside the existing ``Retry`` wrapper)::

    Selector
      Sequence
        BtNode_ProjectPose   (bar -> projected key)
        BtNode_GotoAction    (drive to projected key)
      BtNode_GotoAction      (raw bar-anchor fallback, name "Go to kitchen bar")

These tests walk the constructed subtrees with ``.iterate()`` and never
touch ROS, mirroring ``test_restaurant_timeouts.py``.

``BtNode_GotoAction`` does not retain its blackboard key as an attribute (it
only remaps the ``"goal"`` key), so the raw vs projected Goto are told apart
by name: the raw fallback keeps the historical "Go to kitchen bar" name; the
projected leg uses "Go to kitchen bar (projected)".
"""

import py_trees

RAW_GOTO_NAME = "Go to kitchen bar"
PROJECTED_GOTO_NAME = "Go to kitchen bar (projected)"


def _iter(behaviour):
    return list(behaviour.iterate())


def _find_raw_bar_gotos(subtree):
    """Every BtNode_GotoAction still named "Go to kitchen bar" (the raw fallback)."""
    found = []
    for node in _iter(subtree):
        if (
            node.__class__.__name__ == "BtNode_GotoAction"
            and node.name == RAW_GOTO_NAME
        ):
            found.append(node)
    return found


def _parent_of(subtree, target):
    for node in _iter(subtree):
        if target in getattr(node, "children", []):
            return node
    return None


def _assert_projection_guarded(subtree, bar_key):
    """The raw bar Goto must be the Selector fallback behind a ProjectPose+Goto."""
    raw_gotos = _find_raw_bar_gotos(subtree)
    assert raw_gotos, "no raw 'Go to kitchen bar' BtNode_GotoAction found"
    for raw_goto in raw_gotos:
        selector = _parent_of(subtree, raw_goto)
        assert isinstance(selector, py_trees.composites.Selector), (
            f"raw bar Goto must be a child of a Selector, got "
            f"{type(selector).__name__ if selector else None}"
        )
        children = list(selector.children)
        assert raw_goto is children[-1], (
            "raw bar Goto must be the LAST (fallback) child of the Selector"
        )
        # The preferred branch is a Sequence: ProjectPose then Goto(projected).
        preferred = children[0]
        assert isinstance(preferred, py_trees.composites.Sequence), (
            f"preferred branch must be a Sequence, got {type(preferred).__name__}"
        )
        seq_children = list(preferred.children)
        assert seq_children, "preferred Sequence is empty"
        project_node = seq_children[0]
        assert project_node.__class__.__name__ == "BtNode_ProjectPose", (
            f"first child of preferred Sequence must be BtNode_ProjectPose, "
            f"got {project_node.__class__.__name__}"
        )
        # ProjectPose reads the raw bar key and writes a different out-key.
        assert getattr(project_node, "bb_in_key", None) == bar_key, (
            "ProjectPose must read the raw bar anchor key"
        )
        assert getattr(project_node, "projection_mode", None) == 3, (
            "ProjectPose must default to ANCHOR_NEAREST_FREE (mode 3)"
        )
        out_key = getattr(project_node, "bb_out_key", None)
        assert out_key and out_key != bar_key, (
            "ProjectPose must write a projected key distinct from the raw bar key"
        )
        # The projected Goto drives to the ProjectPose out-key.
        projected_goto = seq_children[-1]
        assert projected_goto.__class__.__name__ == "BtNode_GotoAction", (
            "last child of preferred Sequence must be a BtNode_GotoAction"
        )
        assert projected_goto.name == PROJECTED_GOTO_NAME, (
            "the projected Goto must use the distinct projected name"
        )


def test_pickup_verification_bar_return_is_projection_guarded():
    from behavior_tree.Restaurant.config import KEY_KITCHEN_BAR_POSE
    from behavior_tree.Restaurant.restaurants import createPickupVerification

    subtree = createPickupVerification()
    _assert_projection_guarded(subtree, KEY_KITCHEN_BAR_POSE)


def test_barman_phase_bar_return_is_projection_guarded():
    from behavior_tree.Restaurant.config import KEY_KITCHEN_BAR_POSE
    from behavior_tree.Restaurant.restaurants import createBarmanPhase

    subtree = createBarmanPhase()
    _assert_projection_guarded(subtree, KEY_KITCHEN_BAR_POSE)
