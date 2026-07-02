# HRI: trim host from feature matching (trim_first_person)

**Date:** 2026-07-02
**Status:** Approved (Approach A, design reviewed in-session)
**Package:** `behavior_tree` (tk25_decision)

## Problem

`createTwoWayIntroduction` feature-matches ALL of `KEY_PERSONS = [host, guest1, guest2]`
(`BtNode_FeatureMatching(trim_last_person=False)`, `HRI/hri.py:897`). The host's
features/reference image are supposed to come from disk via
`BtNode_LoadPersonReference` in `createWriteHostInfo` (`HRI/hri.py:444`), but host
features are NOT known beforehand and the host never needs to be recognized.
Consequences today:

- With no host files on disk, `BtNode_LoadPersonReference` returns FAILURE
  (`TemplateNodes/Vision.py`, existence checks) and the whole hri-2026 task
  fails at `createWriteHostInfo` — before the first guest arrival.
- If files did exist, the host still occupies a matching slot: one more VLM
  reference, plus a forced-match slot that latches onto some bystander
  (the service prompt mandates "EVERY reference MUST be matched").

## Goal

Host stays registered at `KEY_PERSONS[0]` (name + drink from constants — still
used by the seat-recommendation flow and any host-directed speech), but is
excluded from the feature-matching request, and the host reference files are
no longer needed at all.

## Non-goals

- No change to `KEY_PERSONS` layout ([host, guest1, guest2] convention stays).
- No behavior change for any other tree that instantiates
  `BtNode_FeatureMatching` (Receptionist ×3, GPSR, TestTrees/load_then_match,
  hriwithfollow, nested `HRI/HRI` duplicate package) — the new parameter
  defaults off.
- No change to the kimi_api service side.
- `BtNode_LoadPersonReference` itself is kept (TestTrees + potential future
  use); only `createWriteHostInfo` stops using it.

## Design

### 1. `BtNode_FeatureMatching.trim_first_person` (TemplateNodes/Vision.py)

New ctor param `trim_first_person: bool = False`, mirroring the existing
`trim_last_person` pattern:

- `initialise()`: after building `request.features` / `request.comparison_images`
  from persons (and after the existing `trim_last_person` handling), if
  `trim_first_person`: drop index 0 from BOTH lists.
- `update()` success path: if `trim_first_person`, write
  `[None] + list(result.centroids)` to the blackboard instead of
  `result.centroids` — the None pad keeps `KEY_PERSON_CENTROIDS[i]` aligned
  with `KEY_PERSONS[i]`, so downstream `target_id=1/2` consumers need no
  re-indexing. (Composition order with `trim_last_person`: trims are
  independent — first affects index 0, last affects the final entry; both may
  be active simultaneously and the pad only re-inserts the front slot.)
- Mock mode: mirror the trim in the synthetic-centroid count AND prepend the
  same None pad, so mock and real blackboard shapes agree.
- Failure paths keep writing `[]` (unchanged).

Fail-loud rationale: nothing in the active tree reads centroid index 0
(verified sweep below). If a future node does, dereferencing None raises
immediately instead of silently aiming at a bogus origin point.

### 2. `createTwoWayIntroduction` (HRI/hri.py)

Pass `trim_first_person=True` to the matching leaf. Update the stale comment
at ~`hri.py:989` (which references the matching call) to note the host slot is
a None pad.

### 3. `createWriteHostInfo` (HRI/hri.py)

Replace the `BtNode_LoadPersonReference` child with blackboard writes of empty
host media before `BtNode_CombinePerson` (which hard-reads both keys — KeyError
if never written): `KEY_HOST_FEATURES = ''`, `KEY_HOST_IMAGE = None`. Use
`py_trees.behaviours.SetBlackboardVariable` (or the repo's existing
constant-writer pattern from `createConstantWriter`, whichever matches local
idiom). Host `Person` keeps name/drink from constants; its
`comparison_image=None` is already tolerated by `BtNode_CombinePerson`'s
`getattr(img, 'width', 0)` summary guard, and the matching node never sends it
once trimmed.

## Verified dependency sweep (why this doesn't break anything)

| Consumer | Location | Verdict |
|---|---|---|
| `BtNode_TurnTo` intro gaze | `HRI/hri.py:950,1010` (`target_id=2/1`) | bounds-checks length, dereferences only its own index — None pad at 0 never read |
| `BtNode_PointTo` intro arm | `HRI/hri.py` intro blocks (`target_id=1/2`) | same pattern (tolerant of short/missing lists per its docs); plan re-verifies its `persons` read |
| `BtNode_PointTo` seat flow | `HRI/hri.py:760` (`target_id=0`) | reads `KEY_SEAT_POINTS`, not person centroids — unaffected |
| `BtNode_Introduce` | `HRI/hri.py:978,1037` (`introduced_id/target_id` 1,2) | host never introduced; persons[0] untouched |
| `BtNode_MaintainEyeContact` on centroids | commented out (`HRI/hri.py:~993,1051`) | inactive; if revived for target 1/2 it's pad-safe |
| Seat recommendation | `HRI/hri.py:649,757` region | uses persons (host row keeps name/drink; features become `''`) + `KEY_SEAT_POINTS`; plan verifies the request builder tolerates `''` (it appends `persons[i].features` verbatim — empty string is a valid srv payload) |
| "Look at host" (f725dae) | `HRI/hri_2026.py:104` | fixed `TurnPanTilt(0, 35)` — no centroid read |
| Other trees instantiating the node | Receptionist ×3, `GPSR/gpsr.py:113`, `TestTrees/load_then_match.py:89`, `hriwithfollow.py:914`, nested `HRI/HRI/hri.py:878` | default `trim_first_person=False` → byte-identical behavior |
| kimi_api service | n_feats drops 3→2 with 2 refs | still image mode (`n_refs == n_feats`), n_feats ≥ 1 satisfied; one fewer VLM reference |
| Host reference files on disk | no longer read by hri-2026 | operator no longer authors them (kills the old checklist item) |

Known-unrelated: the nested `HRI/HRI` duplicate subpackage stays untouched
(hri_2026 imports the outer `HRI.hri` only); the stale `package_data`
constants.json install issue (memory 2026-07-02) is pre-existing and out of
scope.

## Testing

1. Unit tests (pytest, `src/behavior_tree/test/`) on the trim/pad logic —
   plan defines the exact seams (request-list trimming and centroid padding)
   and covers: trim_first alone, trim_first+trim_last together, pad present on
   success, `[]` on failure, mock-mode shape parity.
2. Mock-mode tree run (`ros2 run behavior_tree test-mock-mode` or the repo's
   existing mock harness) to confirm hri-2026 composes and ticks through
   `createWriteHostInfo` without the reference files present.
3. On-robot behavioral check (operator): hri-2026 intro phase — matching log
   shows 2 refs / `text_only_mode: false`, and TurnTo/PointTo still aim at the
   correct guests.

## Concurrent-work guard

`HRI/hri.py` currently carries unrelated uncommitted modifications from a
concurrent session, and HEAD moves under this checkout (f725dae landed
mid-investigation). Same commit discipline as the kimi_api work: new files
commit freely; before any commit staging `hri.py` or `Vision.py`, ask the user
whether to sweep in-flight work or defer; never `--amend`/rebase; re-verify
`git status` immediately before each commit.
