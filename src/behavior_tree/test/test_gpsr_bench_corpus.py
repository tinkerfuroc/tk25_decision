import json
from pathlib import Path

import pytest

from behavior_tree.GPSR.bench import corpus

CONSTANTS = Path(__file__).resolve().parents[1] / "behavior_tree" / "GPSR" / "constants.rcw2026.json"
TOP_LEVEL = [
    "goToLoc", "takeObjFromPlcmt", "findPrsInRoom", "findObjInRoom", "meetPrsAtBeac",
    "countObjOnPlcmt", "countPrsInRoom", "tellPrsInfoInLoc", "tellObjPropOnPlcmt",
    "talkInfoToGestPrsInRoom", "followNameFromBeacToRoom", "guideNameFromBeacToBeac",
    "guidePrsFromBeacToBeac", "guideClothPrsFromBeacToBeac", "bringMeObjFromPlcmt",
    "tellCatPropOnPlcmt", "greetClothDscInRm", "greetNameInRm", "meetNameAtLocThenFindInRm",
    "countClothPrsInRoom", "tellPrsInfoAtLocToPrsAtLoc", "followPrsAtLoc",
]


def test_knowledge_comes_from_constants_file():
    kb = corpus.build_knowledge(CONSTANTS)
    raw = json.loads(CONSTANTS.read_text())
    assert set(kb.rooms) == {"kitchen", "laundry_room", "living_room", "bedroom"}
    assert set(kb.locations) == set(raw["possible_poses"]) - {"command_point"}
    assert set(kb.objects) == set(raw["possible_objects"])
    assert kb.placement_locations and set(kb.placement_locations) <= set(kb.locations)


def test_expand_template_returns_text_and_followup_names():
    kb = corpus.build_knowledge(CONSTANTS)
    gen = corpus.make_generator(kb, seed=1)
    text, followups = corpus.expand_template(gen, "goToLoc", "people")
    assert text and "{" not in text and "WARNING" not in text
    assert followups and set(followups) <= set(gen.followup_templates)


def test_generate_corpus_is_deterministic_for_a_seed():
    a = corpus.generate_corpus(CONSTANTS, seed=7, per_template=1)
    b = corpus.generate_corpus(CONSTANTS, seed=7, per_template=1)
    assert [e.text for e in a] == [e.text for e in b]
    c = corpus.generate_corpus(CONSTANTS, seed=8, per_template=1)
    assert [e.text for e in a] != [e.text for e in c]


def test_generate_corpus_covers_every_template_k_times():
    entries = corpus.generate_corpus(CONSTANTS, seed=3, per_template=2)
    counts = {}
    for e in entries:
        counts[e.template] = counts.get(e.template, 0) + 1
    assert set(counts) == set(TOP_LEVEL)
    assert all(n == 2 for n in counts.values())
    assert all(e.feasibility in {"A", "B", "C"} for e in entries)
    assert len({e.id for e in entries}) == len(entries)


def test_corpus_text_stays_inside_arena_vocabulary():
    kb = corpus.build_knowledge(CONSTANTS)
    entries = corpus.generate_corpus(CONSTANTS, seed=5, per_template=1)
    allowed = set(kb.rooms) | set(kb.locations)
    for e in entries:
        for word in ("office", "bathroom", "desk_lamp"):
            assert word not in e.text, (e.template, e.text)


def test_jsonl_roundtrip(tmp_path):
    entries = corpus.generate_corpus(CONSTANTS, seed=2, per_template=1)
    path = tmp_path / "corpus.jsonl"
    corpus.write_jsonl(entries, path)
    assert corpus.read_jsonl(path) == entries


def test_feasibility_map_covers_every_template_and_followup():
    kb = corpus.build_knowledge(CONSTANTS)
    gen = corpus.make_generator(kb, seed=0)
    assert set(corpus.FEASIBILITY) == set(gen.templates) | set(gen.followup_templates)


def test_edge_commands_are_labelled():
    assert corpus.EDGE_COMMANDS
    assert all(e.seed == -1 and e.template == "edge" for e in corpus.EDGE_COMMANDS)
