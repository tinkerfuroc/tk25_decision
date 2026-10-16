# hangzhougrasp: audit fixes

Status: approved in chat. Date: 2026-10-16.
Follows `2026-10-16-hangzhougrasp-multiroom-design.md`; that spec still holds
except where this one changes it.

## Rule for all spoken text

Everything this task says aloud is Mandarin. The detection announcement
already uses `BtNode_AnnounceDetections` ("我看到了…。") with `label_map`
translations, and the drop announcement is Mandarin. New text added by these
fixes, if any, is Mandarin too.

## Fix A: a speech failure no longer costs the grasp (audit item 3)

Today the detection announcement sits between the scan and the grasp. If the
speech service fails after its retries, the viewpoint counts as a miss and
the grasp is skipped without any log line.

Decided (your answer): **log an error and keep going.**

- `createScanAnnounce` wraps the announcement so a failure after its retries
  logs an ERROR naming the announcement, then returns SUCCESS.
- The scan, grasp and route continue as if the announcement had worked.
- A scan with no detections is still a miss, exactly as before.
- Applies to every scan in the task: table scans (with or without grasp) and
  ground scans.

## Fix B: every detectable label has a Mandarin name (new, from the Mandarin rule)

`BtNode_AnnounceDetections` silently skips labels missing from `label_map`,
so an object can be detected and never spoken.

- At startup, `load_config` splits the prompt the way `yolo26_sam_node` does
  (a whole-prompt exact label first, else split on ` . `, commas, semicolons,
  `|` and newlines, full- and half-width) and fails if any label has no
  `label_map` entry (case-insensitive). The error names the labels and says
  a Mandarin name is required.
- An empty prompt already means "all `label_map` keys", so it always passes.

## Fix C: `scan-announce` help text matches the code (audit item 4)

The part always scans with the Orbbec camera and never moves the arm to the
table pose. The `PARTS` text is corrected; the code is unchanged.

## Fix D: `ground` part stows the arm before driving (audit item 5)

With `--grasp`, the arm-to-navigation step now runs first for `ground` as well
as `room`, `visit-table` and any part given `--at`. Without `--grasp` the part
still never commands the arm, as in the full no-grasp tree.

## Fix E: clearer config errors (audit item 6)

- Misnamed poses are checked before the route is built, so a room whose
  only poses are misnamed reports those names.
- Each profile room name must match `[a-z][a-z0-9]*` (the parser's room
  pattern); otherwise startup fails saying so, instead of "no table or
  ground poses".
- A missing or blank `enter_room` fails with "set enter_room in
  constants.json or pass --enter-room".

## Not changed

- Audit item 1 (your uncommitted `Vision.py`, `Audio.py`, `setup.py`,
  `mock_config.json`) is yours to commit.
- Audit item 2 (possible re-detection of a dropped object) needs the robot.
