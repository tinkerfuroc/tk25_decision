"""resolve_pose room fallback: goto(room) must resolve to a real map pose.

Regression for the 2026-08-27 battery, where room-targeted runs
"navigated" to the robot's own start pose (goals within 10 cm of spawn,
drifting with AMCL) because rcw2026 constants did not provide room
waypoints and ``resolve_pose`` returned ``None`` — leaving the previous
``TARGET_POSE`` silently in effect.

The living-room room anchor is now explicit as well: its sofa search spot is
the correct object-search target, but its heading points at the command-scene
person rather than at the sofa.
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


def test_command_point_resolves_to_constants_pose_not_start_pose():
    """H-1 (round-3 fix review): command_point is a real map waypoint, not
    the pose captured when the operator spoke the command. Loading the
    rcw2026 constants must resolve it via KNOWN_LOCATIONS, never
    START_POSE — even though "command_point" used to be (and, in
    planner_validators.START_LOCATION_WORDS, still is, for gate/coverage
    equivalence only) treated as a start-position alias.
    """
    from pathlib import Path
    from behavior_tree.GPSR import orchestrator as orch_mod

    assert "command_point" not in orch_mod.START_LOCATION_ALIASES

    constants = Path(orch_mod.__file__).parent / "constants.rcw2026.json"
    orch_mod.load_knowledge_from_constants(str(constants))
    try:
        assert "command_point" in orch_mod.KNOWN_LOCATIONS

        class _ExplodingBB:
            """START_POSE must never even be consulted."""

            def get(self, key):
                raise AssertionError(f"resolve_pose consulted the blackboard for {key!r}")

        pose = orch_mod.resolve_pose(_ExplodingBB(), "command_point")
        assert pose is orch_mod.KNOWN_LOCATIONS["command_point"]
    finally:
        orch_mod.load_knowledge_from_constants(str(constants))


def test_resolve_pose_prefers_known_location_over_alias_defensively(_rcw2026_locations, monkeypatch):
    """H-1 defensive branch: if a future constants file ever defines a pose
    for a name that's ALSO in START_LOCATION_ALIASES (e.g. "operator"), the
    map pose wins over the captured start pose.
    """
    from behavior_tree.GPSR import orchestrator as orch_mod

    sentinel = object()
    monkeypatch.setitem(orch_mod.KNOWN_LOCATIONS, "operator", sentinel)
    assert "operator" in orch_mod.START_LOCATION_ALIASES

    class _ExplodingBB:
        def get(self, key):
            raise AssertionError("should not consult START_POSE when a known pose exists")

    assert orch_mod.resolve_pose(_ExplodingBB(), "operator") is sentinel


def test_loader_maps_rooms_to_explicit_or_search_spot_poses():
    from pathlib import Path
    from behavior_tree.GPSR import orchestrator as orch_mod
    constants = (
        Path(orch_mod.__file__).parent / "constants.rcw2026.json"
    )
    orch_mod.load_knowledge_from_constants(str(constants))
    try:
        for room in ("kitchen", "living_room", "bedroom", "laundry_room"):
            assert room in orch_mod.KNOWN_LOCATIONS, room
        # Living-room navigation has a dedicated person-facing anchor; object
        # searches still retain sofa as their first sweep spot.
        assert orch_mod.KNOWN_LOCATIONS["living_room"] is not orch_mod.KNOWN_LOCATIONS["sofa"]
        assert orch_mod.KNOWN_LOCATIONS["living_room"].pose.position.x == pytest.approx(-2.24)
        assert orch_mod.KNOWN_LOCATIONS["living_room"].pose.position.y == pytest.approx(-3.16)
        # The room anchor must face the command-scene person, not the sofa
        # search surface (about -153 degrees in the rcw2026 map frame).
        assert orch_mod.KNOWN_LOCATIONS["living_room"].pose.orientation.z == pytest.approx(-0.9722, abs=1e-4)
        assert orch_mod.KNOWN_LOCATIONS["living_room"].pose.orientation.w == pytest.approx(0.2341, abs=1e-4)
        assert orch_mod.KNOWN_LOCATIONS["kitchen"] is not orch_mod.KNOWN_LOCATIONS["kitchen_table"]
    finally:
        orch_mod.load_knowledge_from_constants(str(constants))


def test_explicit_kitchen_room_anchor_does_not_change_table_search_pose():
    from pathlib import Path
    from behavior_tree.GPSR import orchestrator as orch_mod

    constants = Path(orch_mod.__file__).parent / "constants.rcw2026.json"
    orch_mod.load_knowledge_from_constants(str(constants))
    try:
        room = orch_mod.KNOWN_LOCATIONS["kitchen"]
        table = orch_mod.KNOWN_LOCATIONS["kitchen_table"]
        assert room.pose.position.x == pytest.approx(1.17)
        assert room.pose.position.y == pytest.approx(-3.55)
        assert orch_mod.ROOM_SEARCH_SPOTS["kitchen"] == ["kitchen_table", "shelf_02"]
        assert orch_mod.resolve_pose(_EmptyBB(), "kitchen") is room
        assert orch_mod.resolve_pose(_EmptyBB(), "kitchen_table") is table
        assert table.pose.position.x == pytest.approx(3.412)
        assert table.pose.position.y == pytest.approx(-2.238)
    finally:
        orch_mod.load_knowledge_from_constants(str(constants))


def test_explicit_living_room_anchor_does_not_change_object_search_spots():
    from pathlib import Path
    from behavior_tree.GPSR import orchestrator as orch_mod

    constants = Path(orch_mod.__file__).parent / "constants.rcw2026.json"
    orch_mod.load_knowledge_from_constants(str(constants))
    try:
        room = orch_mod.KNOWN_LOCATIONS["living_room"]
        sofa = orch_mod.KNOWN_LOCATIONS["sofa"]
        assert room is not sofa
        assert room.pose.position.x == pytest.approx(-2.24)
        assert room.pose.position.y == pytest.approx(-3.16)
        assert room.pose.orientation.z == pytest.approx(-0.9722, abs=1e-4)
        assert orch_mod.ROOM_SEARCH_SPOTS["living_room"] == ["sofa", "side_table"]
        assert orch_mod.resolve_pose(_EmptyBB(), "living_room") is room
        assert orch_mod.resolve_pose(_EmptyBB(), "sofa") is sofa
    finally:
        orch_mod.load_knowledge_from_constants(str(constants))
