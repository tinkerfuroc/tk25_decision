"""Contextual speech matching without microphone, model or robot calls."""
import json
from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from behavior_tree.HomeServiceHangzhou import speech, switch_intent
from behavior_tree.HomeServiceHangzhou.ros_nodes import RosLeaves, HangzhouListen
from behavior_tree.HomeServiceHangzhou.config import load_config
from behavior_tree.HomeServiceHangzhou.state import PREFIX


@pytest.mark.parametrize("text,expected", [
    ("\u63a5\u5ba2", "\u6770\u514b"), ("\u6211\u53eb\u63a5\u5ba2", "\u6770\u514b"), ("\u8857\u53ef", "\u6770\u514b"),
    ("\u5b89\u51ac", "\u5b89\u4e1c"), ("\u68a8\u82b1", "\u674e\u534e"), ("\u674e\u6ed1", "\u674e\u534e"),
    ("\u4f60\u534e", "\u674e\u534e"), ("\u5b89\u706f", "\u5b89\u4e1c"), ("\u4e0d\u662f\u6770\u514b\uff0c\u6211\u53eb\u674e\u534e", "\u674e\u534e"),
])
def test_enrollment_fuzzy_names(text, expected):
    assert speech.match_owner_name(text) == expected


@pytest.mark.parametrize("text", [
    "", "\u5929\u6c14\u5f88\u597d", "\u63a5\u5ba2\u548c\u674e\u534e", "\u6770\u514b\u6216\u8005\u5b89\u51ac", "\u4e0d\u662f\u6770\u514b",
    "\u738b\u4e94", "\u674e \u82b1", "\u674eabc\u82b1", "\u529b\u53d1",
])
def test_unknown_negative_or_ambiguous_names(text):
    with pytest.raises(ValueError):
        speech.match_owner_name(text)


@pytest.mark.parametrize("raw,normalized", [
    ("\u6253\u51ef\u7535\u5668", "\u6253\u5f00\u7535\u5668"), ("\u5e2e\u6211\u5b98\u95ed\u7535\u5668", "\u5e2e\u6211\u5173\u95ed\u7535\u5668"),
    ("\u4e0d\u8981\u6253\u51ef\uff0c\u5b98\u95ed\u5c31\u597d", "\u4e0d\u8981\u6253\u5f00\uff0c\u5173\u95ed\u5c31\u597d"),
    ("\u522b\u5b98\u95ed", "\u522b\u5173\u95ed"), ("\u5173\u767b", "\u5173\u706f"), ("\u4e0d\u8981\u5f00", "\u4e0d\u8981\u5f00"),
    ("\u5927\u6982\u5427", "\u5927\u6982\u5427"), ("\u6253\u5f00\u8fd8\u662f\u5173\u95ed", "\u6253\u5f00\u8fd8\u662f\u5173\u95ed"),
    ("\u5173\u6389\uff0c\u4e0d\u5bf9\uff0c\u5e2e\u6211\u6253\u51ef", "\u5173\u6389\uff0c\u4e0d\u5bf9\uff0c\u5e2e\u6211\u6253\u5f00"),
])
def test_switch_homophones_preserve_meaning(raw, normalized):
    assert speech.normalize_switch_transcript(raw) == normalized


def test_model_receives_both_original_and_corrected_speech(monkeypatch):
    backend = Mock(return_value='{"command":"none"}')
    monkeypatch.setattr(switch_intent, "request_completion", backend)
    assert switch_intent.interpret_switch_request("\u522b\u5b98\u95ed", {}) == "none"
    text = backend.call_args.args[0][1]["content"]
    assert json.loads(text) == {
        "original_transcript": "\u522b\u5b98\u95ed", "phonetic_normalization": "\u522b\u5173\u95ed"}


def test_task_prompts_are_distinct_and_switch_leaf_carries_context():
    assert len({speech.NAME_PROMPT, speech.SWITCH_PROMPT,
                speech.HELP_PROMPT, speech.ASSISTANCE_PROMPT}) == 4
    context = SimpleNamespace(config=load_config(), audio_actions=[])
    root = RosLeaves(context).command()
    leaves = [n for n in root.iterate() if isinstance(n, HangzhouListen)]
    assert len(leaves) == 1
    assert leaves[0].initial_prompt == speech.SWITCH_PROMPT


def test_listen_goal_carries_per_request_prompt():
    leaf = HangzhouListen("name", bb_dest_key=PREFIX+"heard",
                          initial_prompt=speech.NAME_PROMPT)
    leaf.mock_mode = False
    leaf.send_goal_request = Mock()
    leaf.send_goal()
    goal = leaf.send_goal_request.call_args.args[0]
    assert goal.initial_prompt == speech.NAME_PROMPT
    assert goal.timeout == leaf.timeout
