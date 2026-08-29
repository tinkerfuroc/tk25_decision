"""D4 (battery run 006): `follow` must not be chosen for "locate them in <room>".

The LLM picked `follow` for a command that only asked to find/meet a person
in a room -- there was never a follow/accompany cue. The catalogue entries
for `follow` and `guide` now say explicitly which command phrasing licenses
each action, so the model has an unambiguous disambiguation rule instead of
relying on the surrounding prose.
"""
from __future__ import annotations

import re

from behavior_tree.GPSR.orchestrator import ACTION_CATALOGUE_DESCRIPTION

# Collapse runs of whitespace (including the catalogue's own line-wrapping)
# so the assertion is robust to how the sentence happens to be wrapped.
_NORMALISED = re.sub(r"\s+", " ", ACTION_CATALOGUE_DESCRIPTION)


def test_follow_entry_restricts_to_explicit_follow_accompany_come_with():
    assert (
        "ONLY when the command explicitly says follow / accompany / come with; "
        "\"find/locate/meet <person> in <room>\" is goto + find_person, never follow."
        in _NORMALISED
    )


def test_guide_entry_restricts_to_explicit_guide_lead_take():
    assert (
        "ONLY when the command says guide/lead/take <person> to <location>."
        in _NORMALISED
    )
