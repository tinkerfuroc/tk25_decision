// tools/tests/js/test_frames.mjs
import test from "node:test";
import assert from "node:assert/strict";
import {
  boundsWithFrames, createPlayer, frameAt, frameAtFraction, hasNoWallTimes,
  makeFrameTick, preloadWindow,
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

// Regression test for the Task 10 review bug: bounds built from run
// events alone (started_wall/finished_wall/epoch/milestone/judge walls)
// fell short of the recorder's actual last frame on a real corpus run
// (finished_wall = 1787914730.807, last frame wall = 1787914731.755).
// boundsWithFrames must widen the bounds to cover every frame's wall so
// that tail is reachable, both by playback and by direct scrubbing.
test("boundsWithFrames widens bounds to cover frame walls extending past finished_wall", () => {
  const eventBounds = { start: 1787914700.0, end: 1787914730.807 }; // ~ finished_wall
  const labelRefs = {
    head: [
      { wall: 1787914700.5 },
      { wall: 1787914731.755 }, // 0.948s past finished_wall -- the real measurement
    ],
    arena: [
      { wall: 1787914700.5 },
      { wall: 1787914731.755 },
    ],
  };
  const bounds = boundsWithFrames(eventBounds, labelRefs);
  assert.equal(bounds.start, 1787914700.0, "start unaffected: no frame precedes it");
  assert.equal(bounds.end, 1787914731.755, "end widened to the last frame's wall");
});

test("boundsWithFrames leaves bounds untouched when every frame wall is null (clock_mode 'none')", () => {
  const eventBounds = { start: 0, end: 10 };
  const bounds = boundsWithFrames(eventBounds, { head: [{ wall: null }, { wall: null }] });
  assert.deepEqual(bounds, { start: 0, end: 10 });
});

// Regression test for the Task 10 review bug's user-visible symptom:
// playback hangs forever, `isPlaying()` never returns false, because
// `playhead.set` clamps to a value equal to the playhead's current value
// (bounds.end sits short of the run's real last frame) and playhead.js
// only notifies subscribers when the clamped value actually differs.
// makeFrameTick's defensive check (compare playhead value before/after
// `set`) must catch this and stop playback even when bounds are wrong,
// as a backstop independent of the boundsWithFrames fix above.
test("a player whose next frame lies beyond the bounds pauses rather than spinning", async () => {
  const bounds = { start: 100, end: 105 }; // stops short of the run's real last frame
  const refs = [
    { index: 0, file: "a.jpg", wall: 100, stamp_s: 1 },
    { index: 1, file: "b.jpg", wall: 105, stamp_s: 2 }, // sits at bounds.end
    { index: 2, file: "c.jpg", wall: 112, stamp_s: 3 }, // beyond bounds.end: unreachable
  ];
  let value = 105; // already at bounds.end, as playback would have advanced it to
  const playhead = {
    get: () => value,
    set: (v) => { value = Math.min(bounds.end, Math.max(bounds.start, v)); },
  };
  let stopCalls = 0;
  // onStop must itself call player.pause() -- createPlayer has no notion
  // of "stop" on its own, exactly as in run.js's real wiring (mountFrames
  // calls player.pause() from the onStop callback it hands to
  // makeFrameTick).
  const player = createPlayer({
    onTick: makeFrameTick({
      refs, playhead, bounds, unaligned: false,
      onStop: () => { stopCalls += 1; player.pause(); },
    }),
  });
  player.play(200); // ~5ms period: fast, so the assertions below resolve quickly
  await new Promise((resolve) => setTimeout(resolve, 100));
  assert.equal(player.isPlaying(), false, "must pause instead of spinning forever");
  assert.equal(stopCalls, 1, "onStop should fire exactly once, not once per tick");
  const valueAtStop = value;
  await new Promise((resolve) => setTimeout(resolve, 100));
  assert.equal(value, valueAtStop, "no further changes once paused");
  assert.equal(player.isPlaying(), false, "still not playing after the extra wait");
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
