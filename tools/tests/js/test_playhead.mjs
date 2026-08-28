// tools/tests/js/test_playhead.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { createPlayhead } from "../../gpsr_ui/static/playhead.js";

test("clamps to the run bounds", () => {
  const p = createPlayhead({ start: 100, end: 200 });
  assert.equal(p.clamp(50), 100);
  assert.equal(p.clamp(250), 200);
  assert.equal(p.clamp(150), 150);
});

test("notifies subscribers on change and not on a repeat", () => {
  const p = createPlayhead({ start: 0, end: 10 });
  const seen = [];
  p.subscribe((v) => seen.push(v));
  p.set(5);
  p.set(5);
  p.set(7);
  assert.deepEqual(seen, [5, 7]);
});

test("set clamps out-of-range values", () => {
  const p = createPlayhead({ start: 0, end: 10 });
  p.set(99);
  assert.equal(p.get(), 10);
});

test("a throwing subscriber does not starve later subscribers or escape set()", () => {
  // Task 9 (tree panel) and Task 10 (camera viewer) will each subscribe
  // alongside the ribbon's own cursor. A bug in one of them must not stop
  // the others from updating, and must not throw out of set() into
  // whatever triggered it -- a click handler or a slider drag.
  const p = createPlayhead({ start: 0, end: 10 });
  const seen = [];
  const realConsoleError = console.error;
  console.error = () => {}; // expected error report; keep test output quiet
  try {
    p.subscribe(() => {
      throw new Error("boom");
    });
    p.subscribe((v) => seen.push(v));

    assert.doesNotThrow(() => p.set(5));
    assert.deepEqual(seen, [5]);
  } finally {
    console.error = realConsoleError;
  }
});
