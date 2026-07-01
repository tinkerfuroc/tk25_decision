from __future__ import annotations

"""Pure destination categorization for the PickAndPlace rulebook tree.

No ROS, no py_trees — deterministic and unit-tested. classify_destination maps
a detected object label to one of three destination classes:
  - wash_staging : cutlery + tableware (routed to the wash-staging surface)
  - trash        : designated trash labels (controlled bin release)
  - cabinet      : everything else, grouped next to similar items

`category_map` is the optional grouping table (label -> group name). v1 is
empty, so each label defaults to itself (its own group; the +20 grouping bonus
is best-effort and improves as clusters grow). For cabinet items the returned
`reference_label` is the group an item should be placed beside.

Bowl disambiguation: "bowl" is in tableware_labels, so during *cleanup* a
detected bowl routes to wash_staging. During *serve-breakfast* the bowl is
handled by the explicit breakfast phase (phase context disambiguates) and never
flows through classify_destination.
"""

from collections import namedtuple

Destination = namedtuple("Destination", ["klass", "reference_label"])


def classify_destination(label, *, cutlery, tableware, trash, category_map):
    norm = (label or "").strip().lower()
    cutlery_set = {c.strip().lower() for c in cutlery}
    tableware_set = {t.strip().lower() for t in tableware}
    trash_set = {t.strip().lower() for t in trash}
    if norm in cutlery_set or norm in tableware_set:
        return Destination("wash_staging", "")
    if norm in trash_set:
        return Destination("trash", "")
    reference_label = category_map.get(norm, norm) if category_map else norm
    return Destination("cabinet", reference_label)
