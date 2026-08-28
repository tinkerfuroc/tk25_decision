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
