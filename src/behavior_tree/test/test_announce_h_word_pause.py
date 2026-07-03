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

"""TTS pause hack: a comma is inserted before every h-/H-word at speak time.

``add_pause_before_h_words`` is a pure helper (``TemplateNodes/announce_text``)
applied by ``BtNode_Announce.initialise()`` to the final resolved
``announce_msg`` — the exact string handed to ``request.text`` (real) or the
mock print. The raw constructor ``given_msg`` and blackboard values must stay
untouched so message-asserting tests and blackboard consumers see raw text.
"""

import os

os.environ.setdefault("BT_MOCK_MODE", "true")

import py_trees  # noqa: E402

from behavior_tree.TemplateNodes.announce_text import (  # noqa: E402
    add_pause_before_h_words,
)
from behavior_tree.TemplateNodes.Audio import BtNode_Announce  # noqa: E402


# --- pure helper ---


def test_users_example_fragment():
    # Leading "has" is untouched at string start (nothing precedes it).
    assert add_pause_before_h_words("has black hair") == "has black, hair"


def test_every_h_word_mid_sentence():
    assert (
        add_pause_before_h_words("who has black hair")
        == "who, has black, hair"
    )


def test_uppercase_h_words():
    assert (
        add_pause_before_h_words("your name is Henry")
        == "your name is, Henry"
    )


def test_h_word_at_start_gets_no_leading_comma():
    assert add_pause_before_h_words("hello there") == "hello there"


def test_no_double_punctuation():
    assert add_pause_before_h_words("Dear, host") == "Dear, host"
    assert add_pause_before_h_words("stop. Here we go") == "stop. Here we go"


def test_idempotent():
    once = add_pause_before_h_words("who has black hair")
    assert add_pause_before_h_words(once) == once


def test_empty_string():
    assert add_pause_before_h_words("") == ""


def test_digit_before_h_word():
    assert add_pause_before_h_words("wait 3 hours") == "wait 3, hours"


def test_whitespace_preserved():
    assert add_pause_before_h_words("black\nhair") == "black,\nhair"


# --- BtNode_Announce integration (mock mode, no ROS graph) ---


def _initialised_announce(**kwargs):
    node = BtNode_Announce(name="announce under test", **kwargs)
    # Force the mock branch regardless of mock_config subsystem settings so
    # initialise() never builds a real service request.
    node.mock_mode = True
    node.initialise()
    return node


def test_announce_transforms_fixed_message_but_not_given_msg():
    node = _initialised_announce(bb_source=None, message="who has black hair")
    assert node.announce_msg == "who, has black, hair"
    assert node.given_msg == "who has black hair"


def test_announce_transforms_blackboard_and_concatenated_text():
    writer = py_trees.blackboard.Client(name="test writer")
    writer.register_key("h_pause_test_msg", access=py_trees.common.Access.WRITE)
    writer.h_pause_test_msg = "guest one has black hair"

    node = _initialised_announce(
        bb_source="h_pause_test_msg", message="Attention please."
    )
    assert node.announce_msg == "Attention please. guest one, has black, hair"
    # The blackboard value itself stays raw.
    assert writer.h_pause_test_msg == "guest one has black hair"
