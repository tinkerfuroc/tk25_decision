"""Ordered inspection route built from Map Studio pose names."""
from dataclasses import dataclass

from .pose_index import PoseIndex, PoseRecord


@dataclass(frozen=True)
class Stop:
    room: str
    kind: str  # "table" | "ground"
    viewpoints: list  # list[PoseRecord]


def build_route(index: PoseIndex, rooms: list) -> list:
    """Per room: ground poses first, then tables (the run's visit order)."""
    route = []
    for room in rooms:
        stops = [Stop(room, "ground", [g]) for g in index.ground(room)]
        stops += [Stop(room, "table", list(t)) for t in index.tables(room)]
        if not stops:
            raise ValueError(f"room {room!r} has no table or ground poses")
        route += stops
    return route
