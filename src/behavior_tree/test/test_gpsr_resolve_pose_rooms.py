"""resolve_pose room fallback: goto(room) must resolve to a real map pose.

Regression for the 2026-08-27 battery, where all four room-targeted runs
"navigated" to the robot's own start pose (goals within 10 cm of spawn,
drifting with AMCL) because rcw2026 constants define rooms only as
``search_spots`` sweep lists, not as ``possible_poses``/``egpsr_rooms``
waypoints, and ``resolve_pose`` returned ``None`` — leaving the previous
``TARGET_POSE`` silently in effect.
"""
from __future__ import annotations

import pytest

from behavior_tree.GPSR import orchestrator as orch


class _EmptyBB:
    """Blackboard stub: no START_POSE, no DYNAMIC_LOCATIONS."""

    def get(self, key):
        raise KeyError(key)


@pytest.fixture()
def _rcw2026_locations(monkeypatch):
    """Pin the exact rcw2026 shape: placements mapped, rooms only in spots."""
    kitchen_table = object()
    sofa = object()
    side_table = object()
    monkeypatch.setattr(
        orch, "KNOWN_LOCATIONS",
        {"kitchen_table": kitchen_table, "sofa": sofa, "side_table": side_table},
    )
    monkeypatch.setattr(
        orch, "ROOM_SEARCH_SPOTS",
        {"living_room": ["sofa", "side_table"], "bedroom": ["side_table_02"]},
    )
    return {"kitchen_table": kitchen_table, "sofa": sofa}


def test_room_resolves_to_first_search_spot_pose(_rcw2026_locations):
    assert orch.resolve_pose(_EmptyBB(), "living_room") is _rcw2026_locations["sofa"]
    # Space spelling (LLM output) matches too, like other locations.
    assert orch.resolve_pose(_EmptyBB(), "living room") is _rcw2026_locations["sofa"]


def test_room_skips_spots_without_poses(_rcw2026_locations):
    # bedroom's only spot (side_table_02) has no pose -> still unresolved.
    assert orch.resolve_pose(_EmptyBB(), "bedroom") is None


def test_placement_resolution_unchanged(_rcw2026_locations):
    assert (
        orch.resolve_pose(_EmptyBB(), "kitchen_table")
        is _rcw2026_locations["kitchen_table"]
    )


def test_unknown_location_still_none(_rcw2026_locations):
    assert orch.resolve_pose(_EmptyBB(), "garage") is None


class _PoisonedBB:
    """Registry stub with a room label registered at the robot's feet —
    the exact workaround the 2026-08-27 LLM plans emitted
    (record_position(label='bedroom') at the start pose)."""

    def __init__(self, registry):
        self._registry = registry

    def get(self, key):
        from behavior_tree.GPSR import bb_keys
        if key == bb_keys.DYNAMIC_LOCATIONS:
            return self._registry
        raise KeyError(key)


def test_known_room_outranks_poisoned_dynamic_registration(_rcw2026_locations, monkeypatch):
    from behavior_tree.GPSR import orchestrator as orch_mod
    # Room aliased into KNOWN_LOCATIONS (as the loader now does).
    monkeypatch.setitem(orch_mod.KNOWN_LOCATIONS, "living_room",
                        _rcw2026_locations["sofa"])
    poisoned = object()
    bb = _PoisonedBB({"living_room": poisoned})
    assert orch_mod.resolve_pose(bb, "living_room") is _rcw2026_locations["sofa"]


def test_loader_aliases_rcw2026_rooms_to_search_spot_poses():
    from pathlib import Path
    from behavior_tree.GPSR import orchestrator as orch_mod
    constants = (
        Path(orch_mod.__file__).parent / "constants.rcw2026.json"
    )
    orch_mod.load_knowledge_from_constants(str(constants))
    try:
        for room in ("kitchen", "living_room", "bedroom", "laundry_room"):
            assert room in orch_mod.KNOWN_LOCATIONS, room
        # Aliased to the first search spot's pose object.
        assert (
            orch_mod.KNOWN_LOCATIONS["living_room"]
            is orch_mod.KNOWN_LOCATIONS["sofa"]
        )
        assert (
            orch_mod.KNOWN_LOCATIONS["kitchen"]
            is orch_mod.KNOWN_LOCATIONS["kitchen_table"]
        )
    finally:
        orch_mod.load_knowledge_from_constants(str(constants))
