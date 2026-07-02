# HRI Trim Host From Feature Matching Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Exclude the host (KEY_PERSONS[0]) from the HRI feature-matching request — host features are unknown and never need recognition — without moving anyone's blackboard index and without touching any other tree.

**Architecture:** Add a default-off `trim_first_person` param to `BtNode_FeatureMatching`, implemented via two tiny pure staticmethods (`_apply_trims` for the request lists, `_pad_centroids` for the response) so the logic is unit-testable without ROS; wire it up in `createTwoWayIntroduction`; replace the host file-load in `createWriteHostInfo` with empty blackboard seeds.

**Tech Stack:** Python 3.10, py_trees, pytest.

**Spec:** `docs/superpowers/specs/2026-07-02-hri-trim-host-from-matching-design.md` (committed, bc935bf)

## Global Constraints

- All commands run from `/home/tinker/tk25_ws/src/tk25_decision` unless stated.
- Tests: `python3 -m pytest src/behavior_tree/test/<file> -v` (behavior_tree auto-mocks missing ROS deps per its CLAUDE.md; if an import demands ROS anyway, prepend `source /opt/ros/humble/setup.zsh && source /home/tinker/tk25_ws/install/setup.zsh`).
- ⚠️ **Concurrent-committer guard:** `src/behavior_tree/behavior_tree/HRI/hri.py` and `src/behavior_tree/behavior_tree/TemplateNodes/Vision.py` carry unrelated UNCOMMITTED in-flight changes from another session, and HEAD moves under this checkout. Commit NEW files (tests, docs) freely; before any commit that stages either modified source file, ask the user whether to sweep in-flight work or defer. Never `--amend`, never rebase. Re-verify `git status` immediately before each commit.
- Do not modify: the nested duplicate `HRI/HRI/` subpackage, Receptionist/GPSR/TestTrees/hriwithfollow trees, `BtNode_LoadPersonReference` itself, or anything in tk26_vision.
- `trim_first_person` MUST default to `False` — six other trees instantiate this node and must stay byte-identical in behavior.
- The centroid pad value is `None` (fail-loud), NOT a zeroed PointStamped.

---

### Task 1: `trim_first_person` on BtNode_FeatureMatching (+ pure-helper tests)

**Files:**
- Modify: `src/behavior_tree/behavior_tree/TemplateNodes/Vision.py` (class `BtNode_FeatureMatching`: ctor ~968-1017, mock block ~1022-1041, request build ~1043-1064, update ~1066-1097)
- Create: `src/behavior_tree/test/test_feature_matching_trim.py`

**Interfaces:**
- Produces: `BtNode_FeatureMatching(..., trim_first_person: bool = False)`; staticmethods `BtNode_FeatureMatching._apply_trims(features, images, trim_first, trim_last) -> tuple[list, list]` and `BtNode_FeatureMatching._pad_centroids(centroids, trim_first) -> list`. Task 2 consumes the ctor param only.

- [ ] **Step 1: Write the failing tests**

Create `src/behavior_tree/test/test_feature_matching_trim.py`:

```python
"""Unit tests for BtNode_FeatureMatching's trim_first_person seams.

Spec: docs/superpowers/specs/2026-07-02-hri-trim-host-from-matching-design.md.
The host occupies KEY_PERSONS[0] but must not enter the matching request;
the response centroids are re-padded with None at index 0 so
KEY_PERSON_CENTROIDS stays index-aligned with KEY_PERSONS (TurnTo/PointTo
target_id=1/2 need no re-indexing and never dereference index 0).
"""
from behavior_tree.TemplateNodes.Vision import BtNode_FeatureMatching


F = ['host-features', 'guest1-features', 'guest2-features']
I = ['host-img', 'guest1-img', 'guest2-img']


def test_apply_trims_first_only():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, True, False)
    assert f == ['guest1-features', 'guest2-features']
    assert i == ['guest1-img', 'guest2-img']


def test_apply_trims_last_only_matches_receptionist_behavior():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, False, True)
    assert f == ['host-features', 'guest1-features']
    assert i == ['host-img', 'guest1-img']


def test_apply_trims_first_and_last_compose():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, True, True)
    assert f == ['guest1-features']
    assert i == ['guest1-img']


def test_apply_trims_default_noop():
    f, i = BtNode_FeatureMatching._apply_trims(F, I, False, False)
    assert f == F and i == I


def test_apply_trims_empty_lists_are_safe():
    assert BtNode_FeatureMatching._apply_trims([], [], True, True) == ([], [])


def test_pad_centroids_prepends_none_when_trimmed():
    out = BtNode_FeatureMatching._pad_centroids(['c1', 'c2'], True)
    assert out == [None, 'c1', 'c2']


def test_pad_centroids_noop_when_not_trimmed():
    assert BtNode_FeatureMatching._pad_centroids(['c0', 'c1'], False) == ['c0', 'c1']
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest src/behavior_tree/test/test_feature_matching_trim.py -v`
Expected: collection ERROR or AttributeError — `_apply_trims` does not exist yet. (If the module import itself fails on ROS deps, re-run with the ROS env sourced per Global Constraints; the auto-mock layer normally makes this unnecessary.)

- [ ] **Step 3: Implement the param and helpers**

In `TemplateNodes/Vision.py`, class `BtNode_FeatureMatching`:

(a) Ctor — add the param after `trim_last_person` and document it:

```python
    def __init__(self,
                 name: str,
                 bb_dest_key: str,
                 bb_persons_key: str,
                 service_name: str = "feature_matching_service",
                 use_orbbec: bool = True,
                 max_distance: float = 6.0,
                 target_frame: str = "base_link",
                 trim_last_person: bool = True,
                 trim_first_person: bool = False,
                 ):
```

Docstring addition (below the existing `trim_last_person` arg doc):

```python
            trim_first_person: when True, drop the FIRST registered person
                (HRI's host at KEY_PERSONS[0] — features unknown, never
                recognized) from the request, and re-pad the returned
                centroid list with None at index 0 so it stays
                index-aligned with the persons list. Downstream consumers
                (TurnTo/PointTo target_id=1/2) need no re-indexing; nothing
                reads index 0, and a future reader fails loudly on None
                rather than aiming at a bogus point.
```

And in the ctor body, next to `self.trim_last_person = trim_last_person`:

```python
        self.trim_first_person = trim_first_person
```

(b) Pure helpers — add as staticmethods on the class (directly above `initialise`):

```python
    @staticmethod
    def _apply_trims(features, images, trim_first, trim_last):
        """Trim request lists; first = HRI host slot, last = Receptionist's
        not-yet-seated newest guest. Independent and composable."""
        if trim_last and features:
            features = features[:-1]
            images = images[:-1]
        if trim_first and features:
            features = features[1:]
            images = images[1:]
        return list(features), list(images)

    @staticmethod
    def _pad_centroids(centroids, trim_first):
        """Restore persons↔centroids index alignment after a first-trim.
        None (not a zeroed point) so an unexpected reader fails loudly."""
        out = list(centroids)
        return ([None] + out) if trim_first else out
```

(c) Request build in `initialise()` — replace the existing trim block:

```python
        if self.trim_last_person:
            # Receptionist flow: the newest registered guest has not sat down yet
            # and won't match anything at the sofa scan, so drop them.
            request.features = request.features[:-1]
            request.comparison_images = request.comparison_images[:-1]
```

with:

```python
        # trim_last: Receptionist flow — the newest registered guest has not
        # sat down yet and won't match anything at the sofa scan.
        # trim_first: HRI flow — the host at index 0 has no known features
        # and is never recognized; centroids are re-padded in update().
        request.features, request.comparison_images = self._apply_trims(
            request.features, request.comparison_images,
            self.trim_first_person, self.trim_last_person,
        )
```

Also extend the feedback message on the next lines to include the new flag
(`f"trim_last={self.trim_last_person}, trim_first={self.trim_first_person})"`).

(d) Mock block in `initialise()` — the current code computes
`n = len(persons)`, decrements for `trim_last_person`, floors at 1, then
builds `n` synthetic centroids into `centroids`. Mirror the new trim and the
pad so mock and real blackboard shapes agree — after the existing
`if self.trim_last_person and n > 0: n -= 1` add:

```python
            if self.trim_first_person and n > 0:
                n -= 1
```

and change the final mock write from `self.blackboard.centroids = centroids` to:

```python
            self.blackboard.centroids = self._pad_centroids(
                centroids, self.trim_first_person
            )
```

(e) `update()` success path — replace:

```python
                self.blackboard.centroids = result.centroids
```

with:

```python
                self.blackboard.centroids = self._pad_centroids(
                    result.centroids, self.trim_first_person
                )
```

Failure paths keep writing `[]` — unchanged.

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest src/behavior_tree/test/test_feature_matching_trim.py -v`
Expected: 7 passed.

- [ ] **Step 5: Regression — existing suite unaffected**

Run: `python3 -m pytest src/behavior_tree/test/ -q 2>&1 | tail -5`
Expected: no NEW failures vs the pre-change baseline (record the baseline
first with `git stash list` NOT allowed — instead run the suite once BEFORE
Step 3 and save the tail to compare; pre-existing failures, if any, are noted
in your report).

- [ ] **Step 6: Commit (guarded)**

The new test file commits freely. `Vision.py` carries unrelated in-flight
work — **ask the user** before staging it; if deferred, commit the test file
only and note the pending source hunk in your report.

```bash
git status --short src/behavior_tree/ && \
git add src/behavior_tree/test/test_feature_matching_trim.py && \
git commit -m "test(behavior_tree): trim_first_person seams for FeatureMatching" \
  -- src/behavior_tree/test/test_feature_matching_trim.py
```

---

### Task 2: Wire hri-2026 — trim host in intro, drop the host file-load

**Files:**
- Modify: `src/behavior_tree/behavior_tree/HRI/hri.py` (`createTwoWayIntroduction` ~897-905 + stale comment ~989; `createWriteHostInfo` ~428-476; imports ~42)
- Create: `src/behavior_tree/test/test_hri_host_trim_wiring.py`

**Interfaces:**
- Consumes: `BtNode_FeatureMatching(trim_first_person=...)` from Task 1.
- Produces: nothing downstream — terminal wiring.

- [ ] **Step 1: Write the failing tests**

Create `src/behavior_tree/test/test_hri_host_trim_wiring.py`:

```python
"""Composition locks for HRI host-trim wiring (spec 2026-07-02).

Walks the factory-built subtrees and asserts the wiring, so regressions
are caught without ROS or hardware: the intro's matching leaf must trim
the host; createWriteHostInfo must no longer contain the disk-reference
loader (host features are unknown beforehand) but must still seed the
blackboard keys BtNode_CombinePerson hard-reads.
"""
import behavior_tree.HRI.hri as hri
from behavior_tree.TemplateNodes.Vision import (
    BtNode_FeatureMatching,
    BtNode_LoadPersonReference,
)


def _iter_tree(root):
    yield root
    for child in getattr(root, 'children', []):
        yield from _iter_tree(child)


def test_two_way_introduction_trims_host_from_matching():
    tree = hri.createTwoWayIntroduction()
    matchers = [n for n in _iter_tree(tree)
                if isinstance(n, BtNode_FeatureMatching)]
    assert matchers, 'intro tree lost its FeatureMatching leaf'
    for m in matchers:
        assert m.trim_first_person is True
        assert m.trim_last_person is False


def test_write_host_info_has_no_disk_reference_loader():
    tree = hri.createWriteHostInfo()
    loaders = [n for n in _iter_tree(tree)
               if isinstance(n, BtNode_LoadPersonReference)]
    assert loaders == [], 'host reference files must no longer be required'


def test_write_host_info_seeds_empty_host_media():
    # BtNode_CombinePerson hard-reads KEY_HOST_FEATURES/KEY_HOST_IMAGE
    # (KeyError if never written); the seeds replace the dropped loader.
    tree = hri.createWriteHostInfo()
    names = [n.name for n in _iter_tree(tree)]
    assert 'Write empty host features' in names
    assert 'Write empty host image' in names
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest src/behavior_tree/test/test_hri_host_trim_wiring.py -v`
Expected: 3 failures — trim_first_person is False (param exists but unwired),
a `BtNode_LoadPersonReference` is present, and the seed nodes are absent.

- [ ] **Step 3: Wire it up in hri.py**

(a) `createTwoWayIntroduction` — the matching leaf gains the flag:

```python
                child=BtNode_FeatureMatching(
                    name="Match seated guest features",
                    bb_dest_key=KEY_PERSON_CENTROIDS,
                    bb_persons_key=KEY_PERSONS,
                    trim_last_person=False,
                    # Host at KEY_PERSONS[0]: features unknown, never
                    # recognized. Centroids come back None-padded at index
                    # 0, so guest target_id=1/2 below stay valid.
                    trim_first_person=True,
                ),
```

(b) The stale comment near the commented-out `BtNode_MaintainEyeContact`
(~line 989: "KEY_PERSON_CENTROIDS is populated by the BtNode_FeatureMatching
above (line ~653, trim_last_person=False).") — append: "index 0 (host) is a
None pad since trim_first_person=True; only target_id 1/2 are dereferenced."

(c) `createWriteHostInfo` — replace the `BtNode_LoadPersonReference(...)`
child (the whole `root.add_child(BtNode_LoadPersonReference(...))` block)
with two seeds using the file's existing constant-writer idiom:

```python
    # Host features are not known beforehand and the host is never
    # recognized (feature matching runs with trim_first_person=True), so
    # no reference files are read. Seed the two keys BtNode_CombinePerson
    # hard-reads (KeyError if absent) with empty media instead.
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write empty host features",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_HOST_FEATURES,
            object="",
        )
    )
    root.add_child(
        BtNode_WriteToBlackboard(
            name="Write empty host image",
            bb_namespace="",
            bb_source=None,
            bb_key=KEY_HOST_IMAGE,
            object=None,
        )
    )
```

(d) Imports: remove `BtNode_LoadPersonReference` from the `hri.py` import
list (~line 42) — it becomes unused in this module (`BtNode_WriteToBlackboard`
is already imported; verify with a grep before assuming).

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest src/behavior_tree/test/test_hri_host_trim_wiring.py src/behavior_tree/test/test_feature_matching_trim.py -v`
Expected: 10 passed (3 wiring + 7 helper).

- [ ] **Step 5: Compose-check the full 2026 task factory**

Run: `python3 -c "import behavior_tree.HRI.hri_2026 as h; t = h.createHRITask2026(); print(type(t).__name__, 'OK')"`
(again with ROS env sourced if plain python3 can't import). Expected: prints
`Sequence OK` — the full tree still composes with the rewired factories.

- [ ] **Step 6: Commit (guarded)**

New test file commits freely. `hri.py` carries unrelated in-flight work —
**ask the user** before staging it (same rule as Task 1's `Vision.py`).

```bash
git status --short src/behavior_tree/ && \
git add src/behavior_tree/test/test_hri_host_trim_wiring.py && \
git commit -m "test(behavior_tree): HRI host-trim wiring locks" \
  -- src/behavior_tree/test/test_hri_host_trim_wiring.py
```

---

### Task 3: Verify seat-request tolerance, install mode, changelog, handoff

**Files:**
- Modify: `src/behavior_tree/README.md` if it exists — otherwise create it minimal (title + one-paragraph package description + Changelog section) per the workspace's README+changelog convention.
- No source modifications.

- [ ] **Step 1: Verify the seat-recommend request builder tolerates `''` host features**

Read the two request-build sites `src/behavior_tree/behavior_tree/TemplateNodes/Vision.py:828-833` and `:931-938` (`request.features.append(self.blackboard.persons[i].features)`). Confirm by reading (not assuming): an empty string appended into a ROS `string[]` field is valid; a `None` would not be — and after this change the host's `features` is `''` (seeded), never `None`. Quote the two snippets in your report. If either site can receive `None` (e.g. `Person()` default is `None` and some path skips CombinePerson), flag it as a finding with the exact path — do NOT fix it unprompted.

- [ ] **Step 2: Determine install mode and rebuild if copied**

```bash
ls -la /home/tinker/tk25_ws/install/behavior_tree/lib/python3.10/site-packages/ 2>/dev/null | head -5
```
If it shows an `.egg-link` / `__editable__` pointer → source edits are live, no rebuild. If it is a plain copy of `behavior_tree/` → rebuild with the workspace convention (`tkbuild tk25_decision --packages-select behavior_tree`; if tkbuild refuses due to a root COLCON_IGNORE, STOP and report — do not fall back to raw colcon). Verify after any rebuild:
```bash
grep -c 'trim_first_person' /home/tinker/tk25_ws/install/behavior_tree/lib/python3.10/site-packages/behavior_tree/TemplateNodes/Vision.py 2>/dev/null || echo "egg-link/editable — grep the source instead"
```

- [ ] **Step 3: Changelog entry**

Append to the package README's Changelog (create the README if the package has none):

```markdown
- 2026-07-02 — HRI: host (KEY_PERSONS[0]) excluded from feature matching
  (`BtNode_FeatureMatching(trim_first_person=True)` in the two-way intro;
  centroids None-padded at index 0 to preserve persons↔centroids index
  alignment). `createWriteHostInfo` no longer reads host reference files
  from disk — host features/image are seeded empty; name/drink constants
  unchanged. Spec: docs/superpowers/specs/2026-07-02-hri-trim-host-from-matching-design.md.
```

- [ ] **Step 4: Commit docs (guarded for source files)**

```bash
git status --short && \
git add src/behavior_tree/README.md docs/superpowers/plans/2026-07-02-hri-trim-host-from-matching.md && \
git commit -m "docs(behavior_tree): changelog + plan — trim host from HRI feature matching" \
  -- src/behavior_tree/README.md docs/superpowers/plans/2026-07-02-hri-trim-host-from-matching.md
```

- [ ] **Step 5: Operator handoff (report, do not execute)**

Report to the user:
1. If Step 2 found a copied install: rebuild done, restart the BT process before the next run.
2. On-robot check: run hri-2026 through the intro phase — the kimi feature_matching vision_log should show **2** references (guests only) with `text_only_mode: false`, and TurnTo/PointTo aim at the correct guests.
3. Host reference files (image/description on disk) are now dead — they can be deleted, and the old "re-author the host description file" checklist item is void.
