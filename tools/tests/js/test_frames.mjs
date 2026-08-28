// tools/tests/js/test_frames.mjs
import test from "node:test";
import assert from "node:assert/strict";
import {
  createPlayer, frameAt, frameAtFraction, hasNoWallTimes, preloadWindow,
} from "../../gpsr_ui/static/frames.js";

const REFS = [
  { index: 0, stamp_s: 1, file: "0000_1000.jpg", wall: 100 },
  { index: 1, stamp_s: 2, file: "0001_2000.jpg", wall: 105 },
  { index: 2, stamp_s: 3, file: "0002_3000.jpg", wall: 112 },
];

test("frameAt picks the frame at or before the playhead", () => {
  assert.equal(frameAt(REFS, 104).file, "0000_1000.jpg");
  assert.equal(frameAt(REFS, 105).file, "0001_2000.jpg");
  assert.equal(frameAt(REFS, 999).file, "0002_3000.jpg");
});

test("frameAt before the first frame returns the first, not null", () => {
  assert.equal(frameAt(REFS, 1).file, "0000_1000.jpg");
});

test("frameAt on an empty list returns null", () => {
  assert.equal(frameAt([], 100), null);
});

test("frameAt ignores refs with no wall time", () => {
  const mixed = [{ file: "a.jpg", wall: null }, { file: "b.jpg", wall: 50 }];
  assert.equal(frameAt(mixed, 60).file, "b.jpg");
});

// A frame from AFTER the moment being inspected must never be shown --
// this exact bug (nearest instead of at-or-before) has been caught twice
// already in this project.
test("frameAt never returns a frame from after the playhead when an earlier one exists", () => {
  assert.equal(frameAt(REFS, 106).file, "0001_2000.jpg");
  assert.notEqual(frameAt(REFS, 106).file, "0002_3000.jpg");
});

test("frameAt returns null when every ref has no wall time (clock_mode 'none')", () => {
  const noClock = [{ file: "a.jpg", wall: null }, { file: "b.jpg", wall: null }];
  assert.equal(frameAt(noClock, 50), null);
});

test("hasNoWallTimes is true only when not one ref in any label has a wall time", () => {
  assert.equal(hasNoWallTimes({ head: [{ wall: null }, { wall: null }] }), true);
  assert.equal(hasNoWallTimes({ head: [{ wall: null }], arena: [{ wall: 3 }] }), false);
  assert.equal(hasNoWallTimes({ head: REFS }), false);
  assert.equal(hasNoWallTimes({}), true, "no labels at all counts as no wall times");
});

test("frameAtFraction maps a 0..1 fraction onto the nearest frame index", () => {
  assert.equal(frameAtFraction(REFS, 0).file, "0000_1000.jpg");
  assert.equal(frameAtFraction(REFS, 1).file, "0002_3000.jpg");
  assert.equal(frameAtFraction(REFS, 0.5).file, "0001_2000.jpg");
});

test("frameAtFraction clamps out-of-range fractions instead of returning undefined", () => {
  assert.equal(frameAtFraction(REFS, -5).file, "0000_1000.jpg");
  assert.equal(frameAtFraction(REFS, 5).file, "0002_3000.jpg");
});

test("frameAtFraction on an empty list returns null", () => {
  assert.equal(frameAtFraction([], 0.5), null);
});

test("preloadWindow returns a centred window of images built via the injected factory, not a global Image", () => {
  const created = [];
  const makeImage = () => {
    const img = { src: null };
    created.push(img);
    return img;
  };
  const urlFor = (ref) => `/frame/${ref.file}`;
  const images = preloadWindow(REFS, 1, 1, urlFor, makeImage);
  assert.equal(images.length, 3);
  assert.equal(created.length, 3);
  assert.deepEqual(
    images.map((i) => i.src),
    ["/frame/0000_1000.jpg", "/frame/0001_2000.jpg", "/frame/0002_3000.jpg"]);
});

test("preloadWindow clamps its window to the array bounds", () => {
  const created = [];
  const images = preloadWindow(REFS, 0, 5, () => "u", () => {
    const img = {};
    created.push(img);
    return img;
  });
  assert.equal(images.length, 3, "radius overshoot clamps to [0, length-1]");
});

test("preloadWindow on an empty list returns no images and calls the factory zero times", () => {
  let calls = 0;
  const images = preloadWindow([], 0, 4, () => "u", () => { calls += 1; return {}; });
  assert.equal(images.length, 0);
  assert.equal(calls, 0);
});

test("player reports its state and stops cleanly", () => {
  const player = createPlayer({ onTick: () => {} });
  assert.equal(player.isPlaying(), false);
  player.play(10);
  assert.equal(player.isPlaying(), true);
  player.pause();
  assert.equal(player.isPlaying(), false);
});

test("player play() called twice does not lose track of the timer on a single pause", () => {
  const player = createPlayer({ onTick: () => {} });
  player.play(10);
  player.play(10); // second call must be a no-op, not a second interval
  player.pause();
  assert.equal(player.isPlaying(), false);
});

test("pause() before any play() is a harmless no-op", () => {
  const player = createPlayer({ onTick: () => {} });
  assert.doesNotThrow(() => player.pause());
  assert.equal(player.isPlaying(), false);
});

// Confirms the player actually fires on a real timer and, critically,
// that pause() leaves no timer running afterward -- a leaked interval
// here would hang the test process, and would leak once per run switch
// in the real page.
// Generous margins throughout: this only needs to show the timer is
// real and genuinely stops, not measure precise timing, so it must not
// flake under a loaded CI box.
test("player actually ticks while playing and stops ticking after pause", async () => {
  let ticks = 0;
  const player = createPlayer({ onTick: () => { ticks += 1; } });
  player.play(100); // ~10ms period
  await new Promise((resolve) => setTimeout(resolve, 200));
  player.pause();
  const afterPause = ticks;
  assert.ok(ticks >= 2, `expected at least a couple of ticks in 200ms, got ${ticks}`);
  await new Promise((resolve) => setTimeout(resolve, 100));
  assert.equal(ticks, afterPause, "no further ticks once paused");
});
