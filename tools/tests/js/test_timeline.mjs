// tools/tests/js/test_timeline.mjs
import test from "node:test";
import assert from "node:assert/strict";
import { buildLanes, xOf } from "../../gpsr_ui/static/timeline.js";

test("xOf maps wall time onto pixel width", () => {
  assert.equal(xOf(100, 100, 200, 500), 0);
  assert.equal(xOf(200, 100, 200, 500), 500);
  assert.equal(xOf(150, 100, 200, 500), 250);
});

test("xOf is stable when the run has zero duration", () => {
  assert.equal(xOf(100, 100, 100, 500), 0);
});

test("buildLanes has no separate replan lane", () => {
  // The vendored classifier emits REPLAN as a judge event once a run has
  // 3+ tree generations. A second lane built from epochs would double-count,
  // and would also mislabel the normal 2-epoch pair as a replan.
  const model = {
    started_wall: 0,
    finished_wall: 100,
    epochs: [{ ordinal: 0, wall: 0 }, { ordinal: 1, wall: 40 },
             { ordinal: 2, wall: 70 }],
    milestones: [], judge_events: [],
  };
  const lanes = buildLanes(model);
  assert.equal(lanes.find((l) => l.id === "replan"), undefined);
});

test("buildLanes surfaces classifier REPLAN events in the judge lane", () => {
  const model = {
    started_wall: 0, finished_wall: 100, epochs: [], milestones: [],
    judge_events: [
      { wall: "1970-01-01T00:00:40Z", kind: "REPLAN", status: "SUCCESS",
        name: "replan", info: "tree regenerated #1 (230 nodes)" },
    ],
  };
  const judge = buildLanes(model).find((l) => l.id === "judge");
  assert.equal(judge.items.length, 1);
  assert.equal(judge.items[0].kind, "REPLAN");
});

test("buildLanes separates milestones by kind and marks failures", () => {
  const model = {
    started_wall: 0, finished_wall: 10, epochs: [],
    milestones: [
      { wall: "1970-01-01T00:00:01Z", kind: "NAV", status: "FAILURE",
        name: "goto target", info: "blocked" },
      { wall: "1970-01-01T00:00:02Z", kind: "AUDIO", status: "SUCCESS",
        name: "announce", info: "hi" },
    ],
    judge_events: [],
  };
  const lanes = buildLanes(model);
  const nav = lanes.find((l) => l.id === "NAV");
  assert.equal(nav.items.length, 1);
  assert.equal(nav.items[0].status, "FAILURE");
});
