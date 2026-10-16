# hangzhougrasp: switch between two known maps

Status: decided in chat. Date: 2026-10-16.
Builds on `2026-10-16-hangzhougrasp-multiroom-design.md` and
`2026-10-16-hangzhougrasp-audit-fixes.md`.

## Intent

The task runs on two known maps. Each map has its own rooms, entry rooms and
destination poses, so the enterRoom profiles belong to a map.

## Config shape

`constants.json` replaces the top-level `enter_room` / `enter_profiles` with a
per-map section. `map_name` picks the default map.

```json
"map_name": "robocup_home_2026_measured_5cm",
"maps": {
  "robocup_home_2026_measured_5cm": {
    "enter_room": "livingroom",
    "enter_profiles": {
      "livingroom": {"rooms": ["livingroom", "kitchen", "diningroom", "bedroom"],
                     "destination": "livingroom_destination"},
      "diningroom": {"rooms": ["diningroom", "kitchen", "bedroom", "livingroom"],
                     "destination": "diningroom_destination"}
    }
  },
  "8902_2_edit": {
    "enter_room": "room1",
    "enter_profiles": {"room1": {"rooms": ["room1", "room2"],
                                 "destination": "room1_destination"}}
  }
}
```

- `--map NAME` overrides `map_name`. A map not listed in `maps` is a startup
  error that lists the known maps.
- `--enter-room` overrides the selected map's `enter_room`.
- A map entry accepts only `enter_room` and `enter_profiles`; other keys are
  a startup error.
- No fallback: the old top-level `enter_room` / `enter_profiles` keys are
  rejected as unknown.

## Decisions from you

- Second map: `8902_2_edit`. Its poses were renamed on 2026-10-16 (you
  asked): `grasp1` → `room1_table_0_0`, `grasp2` → `room2_table_0_0`,
  `point_area` → `room1_destination`. You wrote `room1_table0`; I used the
  full `<room>_table_<n>_<m>` form the parser requires. Entry room1,
  order room1 → room2.
- robocup map entry rooms: livingroom and diningroom only.
  - livingroom: livingroom → kitchen → diningroom → bedroom.
  - diningroom: diningroom → kitchen → bedroom → livingroom.
- Destinations: you will label one per entry room. The config names them
  `livingroom_destination` and `diningroom_destination` (my call on the
  names; change them in `constants.json` if you label them differently).
  Until they exist, startup fails naming the missing pose.
- `diningroom_table1_1`: you rename it in Map Studio. Until then startup
  fails naming it.

## Misnamed-pose rule, narrowed (my call)

The robocup map has helper poses such as `bedroom_search`,
`livingroom_switch` and `register`. Under the old rule every name starting
with `<room>_` was "misnamed" and stopped startup.

A pose of a profile room is now misnamed only when it looks like a route
pose but does not parse:
- it starts with `<room>_table` or `<room>_ground`, or
- it ends in `_<digits>_<digits>`.

So `diningroom_table1_1`, `bedroom_tabel_1_0` and `kitchen_table_x_0` still
stop startup; `bedroom_search` and `kitchen_switch` are logged and ignored.
