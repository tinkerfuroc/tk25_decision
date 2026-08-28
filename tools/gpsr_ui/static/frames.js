// tools/gpsr_ui/static/frames.js
//
// Pure lookup/playback logic for the stop-motion camera viewer. Kept
// DOM-free -- no `Image`, no `document`, nothing that only exists in a
// browser -- so node:test can cover it directly. run.js supplies the DOM
// glue (an actual <img>, a real `Image` factory for preloading) around
// these functions.
//
// Frame lookup is always at-or-BEFORE the playhead, never "nearest": a
// frame from after the moment being inspected would misrepresent what
// the robot could actually see at that instant. This exact bug (nearest
// instead of at-or-before) has been caught twice already in this
// project, so it gets its own regression test here too.

// Picks the ref at-or-before `wall`, ignoring any ref with no wall time
// at all (a clock join can leave individual frames unresolved even in
// "exact" mode). Refs are expected sorted ascending by wall, matching
// how list_frames()/the /api/frames payload already sorts them.
//
// Returns:
//   - the last ref whose wall <= the query, if one exists;
//   - otherwise the first ref that HAS a wall time at all (so a query
//     before the run's first frame still shows something, not nothing);
//   - null if refs is empty, or if not one ref carries a wall time.
export function frameAt(refs, wall) {
  let found = null;
  for (const ref of refs) {
    if (ref.wall === null || ref.wall === undefined) continue;
    if (ref.wall <= wall) found = ref;
    else if (found !== null) break;
  }
  if (found === null) {
    for (const ref of refs) {
      if (ref.wall !== null && ref.wall !== undefined) return ref;
    }
  }
  return found;
}

// Fallback for a run with NO clock metadata at all (clock_mode "none" --
// every ref's `wall` is null; a real example lives in t2plus-2026). There
// is no wall time to join frames against, so instead of going blank the
// viewer scrubs by the playhead's FRACTIONAL position within the run's
// own timeline bounds, mapped onto frame INDEX. This is a deliberately
// approximate stand-in, not a claim that the frame shown is exactly
// synchronised with the tree/ribbon at that instant -- see hasNoWallTimes.
export function frameAtFraction(refs, frac) {
  if (refs.length === 0) return null;
  const clamped = Math.min(1, Math.max(0, frac));
  const index = Math.round(clamped * (refs.length - 1));
  return refs[index];
}

// True only if not one ref, in not one label, carries a real wall time --
// i.e. the /api/frames payload gives this whole run no usable frame/wall
// join. Checked positively over every ref rather than trusting a single
// run-level flag, since the join is computed per frame.
export function hasNoWallTimes(labelRefs) {
  for (const refs of Object.values(labelRefs)) {
    for (const ref of refs) {
      if (ref.wall !== null && ref.wall !== undefined) return false;
    }
  }
  return true;
}

// Returns up to 2*radius+1 images, in index order, centred on `index` and
// clamped to the array's bounds, built via the caller-supplied
// `makeImage` factory (real code passes `() => new Image()`; tests pass
// a fake so this module never touches a browser global).
export function preloadWindow(refs, index, radius, urlFor, makeImage) {
  const images = [];
  if (refs.length === 0) return images;
  const lo = Math.max(0, index - radius);
  const hi = Math.min(refs.length - 1, index + radius);
  for (let i = lo; i <= hi; i += 1) {
    const img = makeImage();
    img.src = urlFor(refs[i]);
    images.push(img);
  }
  return images;
}

// Maps a wall time onto its 0..1 fractional position within `bounds`,
// clamped. Used both to render an unaligned (clock_mode "none") run's
// current frame from the shared playhead, and by makeFrameTick below to
// step playback the same way.
export function fractionOf(wall, bounds) {
  const span = bounds.end - bounds.start;
  if (!(span > 0)) return 0;
  return (wall - bounds.start) / span;
}

// Extends event-derived run bounds ({start, end}, from started_wall/
// finished_wall/epoch/milestone/judge walls) to also cover every frame's
// wall time. The recorder keeps capturing briefly after the run's own
// finish/judge events are logged (measured on a real corpus run:
// finished_wall = 1787914730.807, last frame wall = 1787914731.755, ~0.95s
// later) -- bounds built from events alone leave those trailing frames
// permanently unreachable, by playback (nothing beyond `end` can ever be
// scrubbed to) or by direct scrubbing (xOf/fractionOf never reach that
// wall). Refs with no wall time (clock_mode "none") don't contribute,
// matching frameAt's own null-handling -- there's nothing usable to fold
// in from a run with no clock join at all.
export function boundsWithFrames(bounds, labelRefs) {
  let { start, end } = bounds;
  for (const refs of Object.values(labelRefs)) {
    for (const ref of refs) {
      if (ref.wall === null || ref.wall === undefined) continue;
      if (ref.wall < start) start = ref.wall;
      if (ref.wall > end) end = ref.wall;
    }
  }
  return { start, end };
}

// Builds the onTick callback used by the stop-motion player: advances
// `playhead` to the next frame (by wall time when the run has one, or by
// index-fraction for an unaligned run), and reports the outcome via
// `onStop` if playback should stop. Lives here (not inline in run.js) so
// it is exercisable by node:test against a fake `{ get, set }` playhead
// -- no DOM required.
//
// Stops (calls `onStop`, does not move the playhead further) when:
//   - there is no next frame ref at all, OR
//   - the run has a clock but the next ref carries no wall time, OR
//   - (the defensive case) `playhead.set` was called but the playhead's
//     value did not actually change -- e.g. because `bounds` does not
//     yet cover this frame's wall. `playhead.set` is trusted to clamp,
//     but a clamp-to-the-current-value is indistinguishable from "did
//     nothing" to a caller that doesn't check; this is exactly the bug
//     that let the Task 10 review catch playback hanging forever on a
//     real run (bounds.end fell ~0.95s short of the last frame, so `set`
//     kept clamping to a value equal to itself, never notifying
//     subscribers, while `isPlaying()` stayed true for the life of the
//     page). Even once bounds correctly cover every frame (see
//     `boundsWithFrames`), this guard stays as a defensive backstop: a
//     future bounds change must not be able to resurrect an unkillable
//     interval.
export function makeFrameTick({ refs, playhead, bounds, unaligned, onStop }) {
  return () => {
    const before = playhead.get();
    const ref = unaligned
      ? frameAtFraction(refs, fractionOf(before, bounds))
      : frameAt(refs, before);
    const idx = ref ? refs.indexOf(ref) : -1;
    const next = refs[idx + 1];
    const hasUsableNext = next
      && (unaligned || (next.wall !== null && next.wall !== undefined));
    if (!hasUsableNext) {
      onStop();
      return;
    }
    if (unaligned) {
      const frac = refs.length > 1 ? (idx + 1) / (refs.length - 1) : 0;
      playhead.set(bounds.start + frac * (bounds.end - bounds.start));
    } else {
      playhead.set(next.wall);
    }
    if (playhead.get() === before) {
      onStop();
    }
  };
}

// Fetches the run model and frames listing CONCURRENTLY. There is no
// dependency between them until their results are combined into bounds
// (see boundsWithFrames), so awaiting them sequentially -- issue one,
// wait for it, THEN issue the other -- needlessly serializes two
// independent round trips. On the corpus's largest run (3253 frames),
// the frames listing alone measured ~0.6-0.8s; serial awaiting added
// that on top of the model fetch instead of overlapping it, delaying
// first paint of the ribbon/tree, which never depended on frames at all.
//
// Takes the two fetches as injected functions (not `fetch` + a URL) --
// same DI pattern preloadWindow already uses for `makeImage` -- so this
// is node:test-coverable with fake promises whose call/resolution order
// can be recorded and asserted on, no real network or DOM required.
//
// A failed frames listing must not take down the whole page: the
// ribbon/tree render from the run model alone, so `fetchFrames`'s
// rejection is caught HERE (not passed into Promise.all, which would
// reject the whole pair on the first failure) and replaced with an
// empty payload -- mountFrames already renders that as "no frames
// recorded for this run" (identical to a real run with an empty
// frames/ dir). A failed `fetchRun`, by contrast, is NOT caught: the
// page is meaningless without the run model, so that failure still
// propagates to the caller exactly as it did before this change.
export async function fetchRunAndFrames(fetchRun, fetchFrames) {
  const modelPromise = fetchRun();
  const framesPromise = fetchFrames().catch((err) => {
    console.error("frames listing failed; rendering without it", err);
    return { labels: {} };
  });
  const [model, framesPayload] = await Promise.all([modelPromise, framesPromise]);
  return { model, framesPayload };
}

// Frames are one per SIMULATOR second; playing them back at wall-clock
// speed would be a slideshow (a 900-frame run would take 900 real
// seconds). The player instead just ticks at a chosen fps and leaves it
// to the caller's onTick to decide what "next frame" means -- run.js
// steps the shared playhead forward by one frame per tick, which is what
// actually keeps every panel (tree, ribbon, both cameras) in lockstep.
export function createPlayer({ onTick }) {
  let timer = null;
  return {
    isPlaying: () => timer !== null,
    play(fps) {
      if (timer !== null) return; // already playing: not a second interval
      const period = Math.max(1, Math.round(1000 / fps));
      timer = setInterval(onTick, period);
    },
    pause() {
      if (timer === null) return;
      clearInterval(timer);
      timer = null;
    },
  };
}
