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

"""Pure (ROS-free) text helpers for spoken announcements.

Kept free of ROS / message imports so the transforms can be unit-tested
without a running graph (``behavior_tree.messages`` does not import on every
host) — same convention as ``TemplateNodes/pointing_math.py``.
"""

import re

# A word character, then whitespace, then a word starting with h/H. The
# whitespace is captured and re-emitted so original spacing (incl. newlines)
# survives; the h-word itself is a lookahead and never rewritten.
_H_WORD_PAUSE = re.compile(r"(\w)(\s+)(?=[hH])")


def add_pause_before_h_words(text: str) -> str:
    """Insert a comma before every h-/H-word so the TTS takes a micro-pause.

    The speech engine rushes into aspirated h sounds; a comma after the
    preceding word forces a beat that makes h-words clearly audible, e.g.
    ``"who has black hair"`` -> ``"who, has black, hair"``.

    Properties:
    - Case-insensitive: ``"your name is Henry"`` -> ``"your name is, Henry"``.
    - An h-word at the start of the text gets no comma (nothing precedes it).
    - An h-word already preceded by punctuation (``,``/``.``/etc.) is left
      alone — no double punctuation — which also makes the function
      idempotent: ``f(f(x)) == f(x)``.
    """
    return _H_WORD_PAUSE.sub(r"\1,\2", text)
