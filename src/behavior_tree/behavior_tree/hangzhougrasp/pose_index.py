"""Parse ``<room>_<table|ground>_<n>_<m>`` pose names into ordered queries."""
import re
from dataclasses import dataclass

from geometry_msgs.msg import PoseStamped

NAME_RE = re.compile(
    r"(?P<room>[a-z][a-z0-9]*)_(?P<furniture>table|ground)_(?P<n>\d+)_(?P<m>\d+)"
)


@dataclass(frozen=True)
class PoseRecord:
    room: str
    furniture: str
    n: int
    m: int
    name: str
    pose: PoseStamped


class PoseIndex:
    def __init__(self, poses):
        self.unmatched = []
        self._records = {}
        seen = {}
        for name, pose in poses.items():
            match = NAME_RE.fullmatch(name)
            if match is None:
                self.unmatched.append(name)
                continue
            rec = PoseRecord(match["room"], match["furniture"],
                             int(match["n"]), int(match["m"]), name, pose)
            key = (rec.room, rec.furniture, rec.n, rec.m)
            if key in seen:
                raise ValueError(
                    f"pose names {seen[key]!r} and {name!r} parse to the same {key}")
            seen[key] = name
            self._records[key] = rec

    def rooms(self):
        return sorted({k[0] for k in self._records})

    def _select(self, room, furniture):
        return sorted(
            (r for k, r in self._records.items() if k[0] == room and k[1] == furniture),
            key=lambda r: (r.n, r.m))

    def tables(self, room):
        groups = {}
        for rec in self._select(room, "table"):
            groups.setdefault(rec.n, []).append(rec)
        return [groups[n] for n in sorted(groups)]

    def ground(self, room):
        return self._select(room, "ground")
