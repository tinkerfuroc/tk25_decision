"""Semantic request adapter checks without model or robot calls."""
from concurrent.futures import Future
from types import SimpleNamespace

import pytest

from behavior_tree.HomeServiceHangzhou import switch_intent as module
from behavior_tree.HomeServiceHangzhou.state import S, get, put


@pytest.mark.parametrize("command", ["on", "off", "none", "unknown"])
def test_complete_utterance_is_sent_unchanged(monkeypatch, command):
    utterance = "\u4e0d\u8981\u5f00\uff0c\u5e2e\u6211\u5173\u6389"
    calls = []

    def complete(messages, config, max_tokens):
        calls.append(messages)
        return '{"command":"' + command + '"}'

    monkeypatch.setattr(module, "request_completion", complete)
    assert module.interpret_switch_request(utterance, {}) == command
    assert calls[0][1] == {"role": "user", "content": utterance}
    assert "negation" in calls[0][0]["content"]


@pytest.mark.parametrize("reply", ['{}', '{"command":"toggle"}', '["on"]',
    '{"command":"on","extra":1}', '{"command":null}', 'not JSON'])
def test_malformed_model_reply_never_becomes_an_operation(monkeypatch, reply):
    monkeypatch.setattr(module, "request_completion", lambda *args: reply)
    with pytest.raises((ValueError, TypeError)):
        module.interpret_switch_request("Please turn it on", {})


@pytest.mark.parametrize("command,expected", [
    ("on", S.SUCCESS), ("off", S.SUCCESS), ("none", S.SUCCESS), ("unknown", S.FAILURE)])
def test_only_explicit_interpretations_reach_the_tree(command, expected):
    import time
    node = module.SwitchIntent(SimpleNamespace(config={"identity_vlm_timeout_seconds": 25}))
    node.started = time.monotonic()
    node.future = Future()
    node.future.set_result(command)
    put("command", None)
    assert node.update() == expected
    assert get("command") == (command if expected == S.SUCCESS else None)


def test_late_worker_result_cannot_overwrite_new_request():
    import threading
    released = threading.Event()
    started = threading.Event()

    def backend(text, config):
        started.set()
        released.wait(2)
        return "on"

    node = module.SwitchIntent(
        SimpleNamespace(config={"identity_vlm_timeout_seconds": 25}, event=lambda *a, **k: None), backend)
    put("heard", "Please turn it on")
    node.initialise()
    assert started.wait(1)
    previous = node.future
    node.stop(S.INVALID)
    put("command", "off")
    released.set()
    assert previous.result(timeout=1) == "on"
    assert get("command") == "off"
