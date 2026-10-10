"""L4 — team self-knowledge in constants -> loader -> prompts.

Sim battery run 008: "tell the person your team's country" had no self-
knowledge anywhere to draw on -- the planner announced a refusal. Nothing in
constants.json / the loader / the prompts ever surfaced the team's own
identity (only the official command generator asks for it; dev_tests.py's
main_tell_info() harness hardcodes a throwaway string instead).

constants.json gains an OPTIONAL top-level ``team_info`` object;
``load_knowledge_from_constants`` parses it into a module-level
``TEAM_INFO: dict`` (empty when absent -- the loader must not break on a
constants file with no such key); when non-empty, an "About yourself" block
is appended to the system prompt at BOTH planning-layer call sites
(TOP_LAYER_SYSTEM_PROMPT / LOWER_LAYER_SYSTEM_PROMPT) so the LLM can answer
questions about itself/its team via announce(). When TEAM_INFO is empty the
assembled prompts are byte-identical to today.

Run with PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 (ROS pytest plugins break
collection).
"""
from __future__ import annotations

import json

import pytest

from behavior_tree.GPSR import orchestrator as orch


# ---------------------------------------------------------------------------
# load_knowledge_from_constants -> TEAM_INFO
# ---------------------------------------------------------------------------

def _write_constants(tmp_path, extra=None):
    data = {
        "possible_poses": {},
        "possible_objects": {},
        "default_locations": {},
        "search_spots": {},
    }
    if extra:
        data.update(extra)
    path = tmp_path / "constants.json"
    path.write_text(json.dumps(data))
    return str(path)


def test_loader_populates_team_info_when_present(tmp_path):
    path = _write_constants(tmp_path, {
        "team_info": {
            "team_name": "Tinker",
            "robot_name": "Tinker",
            "affiliation": "Tsinghua University",
            "country": "China",
        },
    })
    orch.load_knowledge_from_constants(path)
    assert orch.TEAM_INFO == {
        "team_name": "Tinker",
        "robot_name": "Tinker",
        "affiliation": "Tsinghua University",
        "country": "China",
    }


def test_loader_leaves_team_info_empty_when_absent(tmp_path):
    path = _write_constants(tmp_path)
    orch.load_knowledge_from_constants(path)
    assert orch.TEAM_INFO == {}


def test_loader_clears_stale_team_info_on_reload(tmp_path):
    with_info = _write_constants(tmp_path, {"team_info": {"team_name": "Tinker"}})
    orch.load_knowledge_from_constants(with_info)
    assert orch.TEAM_INFO == {"team_name": "Tinker"}

    without_info = tmp_path / "constants2.json"
    data = {
        "possible_poses": {}, "possible_objects": {},
        "default_locations": {}, "search_spots": {},
    }
    without_info.write_text(json.dumps(data))
    orch.load_knowledge_from_constants(str(without_info))
    assert orch.TEAM_INFO == {}


def test_loader_does_not_break_on_non_dict_team_info(tmp_path):
    path = _write_constants(tmp_path, {"team_info": "not a dict"})
    orch.load_knowledge_from_constants(path)  # must not raise
    assert orch.TEAM_INFO == {}


def test_loader_still_populates_known_locations_and_objects(tmp_path):
    """Existing loader behaviour is unbroken by the team_info addition."""
    path = _write_constants(tmp_path, {
        "possible_objects": {"coke": "a red can of coke"},
        "team_info": {"team_name": "Tinker"},
    })
    orch.load_knowledge_from_constants(path)
    assert orch.KNOWN_OBJECT_PROMPTS == {"coke": "a red can of coke"}
    assert "coke" in orch.KNOWN_OBJECT_NAMES


# ---------------------------------------------------------------------------
# Prompt assembly: "About yourself" block, byte-identical when TEAM_INFO empty
# ---------------------------------------------------------------------------

@pytest.fixture(autouse=True)
def _reset_team_info():
    saved = dict(orch.TEAM_INFO)
    orch.TEAM_INFO.clear()
    yield
    orch.TEAM_INFO.clear()
    orch.TEAM_INFO.update(saved)


def test_team_info_block_empty_when_team_info_empty():
    orch.TEAM_INFO.clear()
    assert orch.render_team_info_block() == ""


def test_team_info_block_contains_all_facts_when_present():
    orch.TEAM_INFO.clear()
    orch.TEAM_INFO.update({
        "team_name": "Tinker",
        "robot_name": "Tinker",
        "affiliation": "Tsinghua University",
        "country": "China",
    })
    block = orch.render_team_info_block()
    assert "Tinker" in block
    assert "Tsinghua University" in block
    assert "China" in block


def test_prompts_byte_identical_when_team_info_empty():
    from behavior_tree.GPSR import planner as planner_module

    orch.TEAM_INFO.clear()
    top = planner_module.TOP_LAYER_SYSTEM_PROMPT + orch.render_team_info_block()
    lower = planner_module.LOWER_LAYER_SYSTEM_PROMPT + orch.render_team_info_block()
    assert top == planner_module.TOP_LAYER_SYSTEM_PROMPT
    assert lower == planner_module.LOWER_LAYER_SYSTEM_PROMPT


def test_assembled_prompt_carries_country_line_for_both_layers(monkeypatch):
    """L4d — prompt-assembly unit test (no LLM): a target "tell the person
    your team's country" with TEAM_INFO loaded -> the system prompt
    ACTUALLY SENT to the LLM (via the real split_command / plan_target call
    sites, _call_llm intercepted) contains the country line, at BOTH
    planning-layer usage sites."""
    from behavior_tree.GPSR import planner as planner_module

    orch.TEAM_INFO.clear()
    orch.TEAM_INFO.update({
        "team_name": "Tinker",
        "robot_name": "Tinker",
        "affiliation": "Tsinghua University",
        "country": "China",
    })

    captured_system_prompts = []

    def _fake_call_llm(client, system_prompt, user_prompt, temperature):
        captured_system_prompts.append(system_prompt)
        return {
            "targets": [{
                "id": "a", "desc": "tell the person your team's country",
                "depends_on": [], "preconditions": [], "postconditions": [],
            }],
        }, None

    planner = planner_module.GPSRPlanner(max_attempts=1)
    planner._offline_mock = False
    monkeypatch.setattr(planner, "_new_client", lambda: object())
    monkeypatch.setattr(planner_module, "_call_llm", _fake_call_llm)
    planner.split_command("tell the person your team's country")
    assert len(captured_system_prompts) == 1
    assert "China" in captured_system_prompts[0]

    def _fake_call_llm_lower(client, system_prompt, user_prompt, temperature):
        captured_system_prompts.append(system_prompt)
        return {"plan": [{"action": "announce", "params": {"text": "China"}}]}, None

    monkeypatch.setattr(planner_module, "_call_llm", _fake_call_llm_lower)
    planner.plan_target(0, 0, "tell the person your team's country")
    assert len(captured_system_prompts) == 2
    assert "China" in captured_system_prompts[1]
