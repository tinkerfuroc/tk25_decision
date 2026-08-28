// tools/tests/js/test_tree.mjs
import test from "node:test";
import assert from "node:assert/strict";
import {
  activeAncestorIds, edgesFor, historyFor, isBookkeeping, layoutTree, statusAt,
} from "../../gpsr_ui/static/tree.js";

const NODES = [
  { id: "r", name: "root", type: "Sequence", parent_id: null,
    children: ["a", "b"], node_class: "composite" },
  { id: "a", name: "leaf a", type: "BtNode_Announce", parent_id: "r",
    children: [], node_class: "leaf" },
  { id: "b", name: "leaf b", type: "BtNode_GotoAction", parent_id: "r",
    children: [], node_class: "leaf" },
];

test("layout assigns depth by ancestry and separates siblings", () => {
  const { positions, height } = layoutTree(NODES, "r");
  assert.equal(positions.get("r").depth, 0);
  assert.equal(positions.get("a").depth, 1);
  assert.equal(positions.get("b").depth, 1);
  assert.notEqual(positions.get("a").y, positions.get("b").y);
  assert.ok(height > 0);
});

test("layout tolerates a missing root without throwing", () => {
  const { positions } = layoutTree(NODES, "nope");
  assert.equal(positions.size, 0);
});

test("layout tolerates a cycle without hanging", () => {
  const cyclic = [
    { id: "x", parent_id: null, children: ["y"], name: "x", node_class: "c" },
    { id: "y", parent_id: "x", children: ["x"], name: "y", node_class: "c" },
  ];
  const { positions } = layoutTree(cyclic, "x");
  assert.equal(positions.size, 2, "each node placed exactly once");
});

test("layout skips a child id with no matching node instead of crashing", () => {
  const dangling = [
    { id: "r", parent_id: null, children: ["ghost"], name: "r", node_class: "c" },
  ];
  const { positions } = layoutTree(dangling, "r");
  assert.equal(positions.size, 1);
  assert.equal(positions.has("ghost"), false);
});

test("statusAt returns the last transition at or before the playhead", () => {
  const transitions = [
    { wall: 10, node_id: "a", status: "RUNNING", feedback: "go" },
    { wall: 20, node_id: "a", status: "SUCCESS", feedback: "done" },
    { wall: 30, node_id: "b", status: "FAILURE", feedback: "blocked" },
  ];
  assert.equal(statusAt(transitions, 15).get("a").status, "RUNNING");
  assert.equal(statusAt(transitions, 25).get("a").status, "SUCCESS");
  assert.equal(statusAt(transitions, 25).has("b"), false);
  assert.equal(statusAt(transitions, 5).size, 0);
});

test("keepalive and bookkeeping nodes are flagged for collapsing", () => {
  assert.equal(isBookkeeping({ name: "nav keepalive 0" }), true);
  assert.equal(isBookkeeping({ name: "keepalive gap 2" }), true);
  assert.equal(isBookkeeping({ name: "clear plan_index" }), true);
  assert.equal(isBookkeeping({ name: "goto target" }), false);
});

// The real corpus's single most common node name is "reset <thing>"
// (reset state_log, reset plan_index, reset last_failure, ...) -- 886
// occurrences in a 41-run sample, ahead of every keepalive variant. The
// keepalive WRAPPER nodes ("loop nav keepalive", "keep-alive lines" with
// a hyphen) also don't match the anchored "nav keepalive"/"say
// keepalive" patterns their own children use. Both must still be flagged
// as bookkeeping, or the panel dims only a minority of the noise the
// domain notes say dominates the tree by count.
test("bookkeeping catches reset-prefixed nodes and keepalive wrapper nodes", () => {
  assert.equal(isBookkeeping({ name: "reset plan_index" }), true);
  assert.equal(isBookkeeping({ name: "reset last_failure" }), true);
  assert.equal(isBookkeeping({ name: "loop nav keepalive" }), true);
  assert.equal(isBookkeeping({ name: "keep-alive lines" }), true);
  assert.equal(isBookkeeping({ name: "resettle target" }), false,
    "a real node that merely contains 'reset' as a substring must not " +
    "be swept up by the anchored reset pattern");
});

test("historyFor returns every transition for one node, in order, and none for others", () => {
  const transitions = [
    { wall: 10, tick: 0, node_id: "a", status: "RUNNING", feedback: "go" },
    { wall: 20, tick: 1, node_id: "b", status: "FAILURE", feedback: "blocked" },
    { wall: 30, tick: 2, node_id: "a", status: "SUCCESS", feedback: "done" },
  ];
  const history = historyFor(transitions, "a");
  assert.equal(history.length, 2);
  assert.deepEqual(history.map((h) => h.status), ["RUNNING", "SUCCESS"]);
  assert.equal(historyFor(transitions, "nope").length, 0);
});

test("historyFor tolerates an empty or missing transition list", () => {
  assert.equal(historyFor([], "a").length, 0);
  assert.equal(historyFor(undefined, "a").length, 0);
});

test("edgesFor pairs each non-root node with its parent's position", () => {
  const { positions } = layoutTree(NODES, "r");
  const edges = edgesFor(NODES, positions);
  assert.equal(edges.length, 2);
  const toA = edges.find((e) => e.id === "a");
  assert.deepEqual(toA.from, positions.get("r"));
  assert.deepEqual(toA.to, positions.get("a"));
});

test("edgesFor drops edges for nodes missing from the layout (unlaid root, dangling ids)", () => {
  const { positions } = layoutTree(NODES, "nope");
  assert.equal(edgesFor(NODES, positions).length, 0);
});

test("activeAncestorIds marks a RUNNING node and every ancestor up to the root", () => {
  const statuses = statusAt(
    [{ wall: 1, node_id: "a", status: "RUNNING", feedback: "" }], 1);
  const ids = activeAncestorIds(NODES, statuses);
  assert.equal(ids.has("a"), true);
  assert.equal(ids.has("r"), true, "root is an ancestor of the running leaf");
  assert.equal(ids.has("b"), false, "unrelated sibling is not on the path");
});

test("activeAncestorIds is empty when nothing is RUNNING", () => {
  const statuses = statusAt(
    [{ wall: 1, node_id: "a", status: "SUCCESS", feedback: "" }], 1);
  assert.equal(activeAncestorIds(NODES, statuses).size, 0);
});

test("activeAncestorIds tolerates a cycle without hanging", () => {
  const cyclic = [
    { id: "x", parent_id: "y", children: ["y"], name: "x" },
    { id: "y", parent_id: "x", children: ["x"], name: "y" },
  ];
  const statuses = new Map([["x", { status: "RUNNING", feedback: "" }]]);
  const ids = activeAncestorIds(cyclic, statuses);
  assert.equal(ids.has("x"), true);
  assert.equal(ids.has("y"), true);
});
