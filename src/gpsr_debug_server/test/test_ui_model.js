"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const model = require("../webui/ui_model.js");

test("status and event categories are safe and predictable", () => {
  assert.equal(model.statusCategory("RUNNING"), "running");
  assert.equal(model.statusCategory("SUCCESS"), "success");
  assert.equal(model.statusCategory("tree failure"), "failure");
  assert.equal(model.statusCategory("CANCELLED"), "cancelled");
  assert.equal(model.statusCategory("INVALID"), "invalid");
  assert.equal(model.statusClass("INVALID"), "");
  assert.equal(model.eventCategory({ type: "tree.node_states_changed" }), "tree");
  assert.equal(model.eventCategory({ event_type: "run.heartbeat" }), "heartbeat");
  assert.equal(model.eventCategory(null), "event");
});

test("trajectory identifiers remain bounded while retaining their suffix", () => {
  assert.equal(model.shortTrajectoryId("short"), "short");
  const short = model.shortTrajectoryId("gpsr-20260803T102000Z-a1b2c3d4", 15);
  assert.equal(short.length, 15);
  assert.equal(short, "gpsr-20…1b2c3d4");
  assert.equal(model.shortTrajectoryId({ run_id: "run-123" }), "run-123");
});

test("only adjacent high-rate heartbeat and node-state events are grouped", () => {
  const events = [
    { type: "run.heartbeat", occurred_at: "2026-08-03T10:00:00.000Z" },
    { type: "run.heartbeat", occurred_at: "2026-08-03T10:00:00.900Z" },
    { type: "planner.request", occurred_at: "2026-08-03T10:00:00.950Z" },
    { type: "run.heartbeat", occurred_at: "2026-08-03T10:00:01.000Z" },
    { type: "tree.node_states_changed", occurred_at: "2026-08-03T10:00:02.000Z" },
    { type: "tree.node_states_changed", occurred_at: "2026-08-03T10:00:03.000Z" },
  ];
  const groups = model.groupEvents(events);
  assert.deepEqual(groups.map(group => group.count), [2, 1, 1, 2]);
  assert.equal(groups[0].collapsed, true);
  assert.equal(groups[3].events[1], events[5]);
  assert.equal(model.groupEvents(events, { raw: true }).length, events.length);
  assert.equal(model.groupEvents([
    { type: "run.heartbeat", occurred_at: "not-a-date" },
    { type: "run.heartbeat", occurred_at: "not-a-date" },
  ]).length, 2);
});

test("event search and filters include payloads without mutating records", () => {
  const circular = { type: "planner.response", phase: "planning", payload: { agent_id: "planner", message: "Need water" } };
  circular.payload.self = circular.payload;
  const events = [
    circular,
    { type: "tree.node_states_changed", phase: "execution", payload: { status: "RUNNING", agent_id: "executor" } },
    { type: "planner.error", phase: "planning", payload: { message: "provider timeout" } },
  ];
  assert.equal(model.eventSearchText(circular).includes("need water"), true);
  assert.deepEqual(model.filterEvents(events, { category: "tree" }), [events[1]]);
  assert.deepEqual(model.filterEvents(events, { statuses: ["running"] }), [events[1]]);
  assert.deepEqual(model.filterEvents(events, { search: "TIMEOUT", phase: "planning" }), [events[2]]);
  assert.deepEqual(model.filterEvents(events, { agentId: "planner" }), [events[0]]);
  assert.equal(circular.payload.self, circular.payload);
});

test("tree normalisation accepts object nodes and edges without recursion hazards", () => {
  const tree = model.normalizeTree({
    root: "root",
    nodes: {
      root: { name: "Root", children: ["left"] },
      left: { node_id: "left", name: "Left" },
      right: { name: "Right" },
    },
    edges: [{ from: "root", to: "right" }, { from: "right", to: "root" }, { from: "missing", to: "root" }],
  });
  assert.deepEqual(tree.rootIds, ["root"]);
  assert.deepEqual(tree.childrenById.root, ["left", "right"]);
  assert.deepEqual(tree.parentsById.left, ["root"]);
  assert.equal(tree.byId.left.id, "left");
  assert.ok(tree.warnings.some(item => item.includes("unknown tree edge")));
  assert.deepEqual(model.ancestorIds(tree, "left"), ["root"]);
  assert.deepEqual(model.pathToNode(tree, "left"), ["root", "left"]);
  assert.deepEqual(model.subtreeIds(tree, "root"), ["root", "left", "right"]);
});

test("tree visibility preserves an active path and supports collapsed large trees", () => {
  const tree = model.normalizeTree({
    nodes: [
      { id: "root", children: ["a", "b"] },
      { id: "a", children: ["a1"] },
      { id: "a1" },
      { id: "b", children: ["b1"] },
      { id: "b1" },
    ],
  });
  assert.deepEqual(model.visibleTreeIds(tree, { expandedIds: ["root", "a"] }), ["root", "a", "a1", "b"]);
  const focus = model.treeVisibility(tree, { focusId: "b" });
  assert.deepEqual(focus.pathIds, ["root", "b"]);
  assert.deepEqual(focus.subtreeIds, ["b", "b1"]);
  assert.deepEqual(focus.visibleIds, ["root", "b", "b1"]);
  assert.deepEqual(model.subtreeIds(tree, "root", { maxNodes: 3 }), ["root", "a", "a1"]);
});

test("query state round trips deterministic, escaped values and aliases", () => {
  const parsed = model.parseQuery("?trajectory=run%2F42&at=12&view=tree&task=task-1&q=water+%26+tea&status=running&status=failed&category=tree&raw=1&focus=n-1&tree=r2&tree_mode=executor");
  assert.deepEqual(parsed, {
    trajectoryId: "run/42", at: 12, view: "tree", taskId: "task-1", search: "water & tea",
    statuses: ["running", "failed"], categories: ["tree"], raw: true, focusId: "n-1", treeRevision: "r2", treeMode: "executor",
  });
  const query = model.serializeQuery(parsed);
  assert.equal(query, "?trajectory=run%2F42&at=12&view=tree&task=task-1&q=water%20%26%20tea&status=running&status=failed&category=tree&raw=1&focus=n-1&tree=r2&tree_mode=executor");
  assert.equal(model.serializeQuery(model.DEFAULT_QUERY_STATE), "");
  assert.equal(model.parseQuery("?trajectory_id=legacy&sequence=-1").trajectoryId, "legacy");
  assert.equal(model.parseQuery("?trajectory_id=legacy&sequence=-1").at, null);
});

test("node semantics infer legacy py_trees classes and preserve node_class", () => {
  const legacy = model.normalizeTree({
    nodes: [
      { id: "root", type: "py_trees.composites.Sequence", memory: "true", children: ["gate", "decorator"] },
      { id: "gate", node_class: "Selector", children: ["one", "two"] },
      { id: "decorator", type: "FailureIsRunning", children: ["action"] },
      { id: "one", type: "Condition" }, { id: "two", type: "Action" }, { id: "action", type: "BtNode_GotoAction" },
    ],
  });
  assert.equal(legacy.byId.root.node_class, "sequence");
  assert.equal(legacy.byId.gate.node_class, "selector");
  assert.equal(legacy.byId.decorator.node_class, "failure_is_running");
  const rootSemantics = model.nodeSemantics(legacy.byId.root);
  assert.equal(rootSemantics.node_class, "sequence");
  assert.equal(rootSemantics.memory, true);
  assert.equal(rootSemantics.isComposite, true);
  assert.equal(rootSemantics.source, "node_class");
  assert.equal(model.nodeSemantics({ type: "UnknownLegacyNode" }).node_class, "leaf");
  assert.equal(model.edgeActivationLabel(legacy.byId.root, "gate", legacy), "start sequence");
  assert.equal(model.edgeActivationLabel(legacy.byId.root, "decorator", legacy), "after previous succeeds");
  assert.equal(model.edgeActivationLabel(legacy.byId.gate, "two", legacy), "if earlier branches fail");
  assert.equal(model.edgeActivationLabel(legacy.byId.decorator, "action", legacy), "pass through decorator");
});

test("backend v2 nested semantics override category and drive parallel labels", () => {
  const graph = model.normalizeTree({ nodes: [
    {
      id: "parallel", node_class: "composite", type: "Composite", children: ["completed", "selected", "unknown"],
      semantics: {
        category: "composite", kind: "parallel", control_flow: "concurrent", memory: false,
        parallel_policy: "all", synchronise: true, selected_child_ids: ["selected"],
        counters: { success_count: 1, failure_count: 0 },
      },
    },
    { id: "completed", status: "SUCCESS" }, { id: "selected" }, { id: "unknown" },
  ] });
  const semantics = model.nodeSemantics(graph.byId.parallel);
  assert.equal(graph.byId.parallel.node_class, "parallel");
  assert.equal(semantics.node_class, "parallel");
  assert.equal(semantics.category, "composite");
  assert.equal(semantics.raw_node_class, "composite");
  assert.equal(semantics.raw_kind, "parallel");
  assert.equal(semantics.control_flow, "concurrent");
  assert.equal(semantics.memory, false);
  assert.equal(semantics.success_policy, "all");
  assert.equal(
    model.nodeSemantics({
      type: "Retry",
      node_class: "decorator",
      semantics: {category: "decorator", kind: "retry", counter: {limit: 3}},
    }).kind,
    "retry",
  );
  assert.equal(
    model.nodeSemantics({
      type: "Repeat",
      node_class: "decorator",
      semantics: {category: "decorator", kind: "repeat", counter: {limit: 2}},
    }).kind,
    "repeat",
  );
  assert.equal(semantics.synchronise, true);
  assert.deepEqual(semantics.selected_child_ids, ["selected"]);
  assert.deepEqual(semantics.counters, { success_count: 1, failure_count: 0 });
  assert.equal(model.edgeActivationLabel(graph.byId.parallel, "selected", graph), "selected for parallel tick");
  assert.equal(model.edgeActivationLabel(graph.byId.parallel, "completed", graph), "not selected by synchronised parallel");

  const runtime = { currentTick: { nodes: ["parallel", "selected"] } };
  assert.equal(model.activationState(graph, "completed", runtime), "skipped");
  assert.match(model.activationReason(graph, "completed", runtime), /synchronised parallel preserves/i);
  assert.equal(model.activationState(graph, "unknown", runtime), "not-recorded");
});

test("sequence activation differentiates ticked, resumed, blocked, and skipped", () => {
  const sequence = model.normalizeTree({ nodes: [
    { id: "seq", type: "Sequence", children: ["first", "second", "third"] },
    { id: "first", name: "first action" }, { id: "second", name: "second action" }, { id: "third", name: "third action" },
  ] });
  const running = {
    currentTick: { nodes: [{ id: "seq" }, { id: "first", status: "RUNNING" }] },
    previousTick: { nodes: ["seq", "first"] },
  };
  assert.equal(model.activationState(sequence, "seq", running), "resumed");
  assert.equal(model.activationState(sequence, "first", running), "resumed");
  assert.equal(model.activationState(sequence, "second", running), "blocked");
  assert.match(model.activationReason(sequence, "second", running), /waits for first action/i);
  assert.equal(model.activationState(sequence, "third", running), "blocked");

  const failed = { currentTick: { nodes: [{ id: "seq" }, { id: "first", status: "FAILURE" }] } };
  assert.equal(model.activationState(sequence, "first", failed), "ticked");
  assert.equal(model.activationState(sequence, "second", failed), "skipped");
  assert.match(model.activationReason(sequence, "second", failed), /sequence stopped/i);
  assert.equal(model.activationState(sequence, "missing", failed), "not-recorded");
});

test("selector and memory semantics produce non-speculative branch explanations", () => {
  const selector = model.normalizeTree({ nodes: [
    { id: "select", type: "Selector", children: ["primary", "fallback", "last"] },
    { id: "primary", name: "primary path" }, { id: "fallback", name: "fallback path" }, { id: "last", name: "last path" },
  ] });
  const primaryWins = { currentTick: { nodes: [{ id: "select" }, { id: "primary", status: "SUCCESS" }] } };
  assert.equal(model.activationState(selector, "fallback", primaryWins), "skipped");
  assert.match(model.activationReason(selector, "fallback", primaryWins), /selector kept primary path/i);

  const fallbackRuns = { currentTick: { nodes: [{ id: "select" }, { id: "primary", status: "FAILURE" }, { id: "fallback", status: "RUNNING" }] } };
  assert.equal(model.activationState(selector, "fallback", fallbackRuns), "ticked");
  assert.equal(model.activationState(selector, "last", fallbackRuns), "skipped");

  const memorySequence = model.normalizeTree({ nodes: [
    { id: "memory", type: "Sequence", memory: true, children: ["done", "active"] },
    { id: "done", name: "completed step" }, { id: "active", name: "active step" },
  ] });
  const resumed = {
    currentVisitedIds: ["memory", "active"],
    previousVisitedIds: ["memory", "done"],
  };
  assert.equal(model.activationState(memorySequence, "done", resumed), "skipped");
  assert.match(model.activationReason(memorySequence, "done", resumed), /memory sequence resumed at active step/i);
  assert.equal(model.activationState(memorySequence, "active", resumed), "ticked");
});

test("parallel leaves missing branches as not-recorded rather than inventing a gate", () => {
  const parallel = model.normalizeTree({ nodes: [
    { id: "parallel", type: "Parallel", children: ["left", "right", "unseen"] },
    { id: "left" }, { id: "right" }, { id: "unseen" },
  ] });
  const runtime = {
    currentTick: { nodes: [{ id: "parallel" }, { id: "left", status: "RUNNING" }, { id: "right", status: "SUCCESS" }] },
    previousTick: { nodes: ["parallel", "left"] },
  };
  assert.equal(model.activationState(parallel, "left", runtime), "resumed");
  assert.equal(model.activationState(parallel, "right", runtime), "ticked");
  assert.equal(model.activationState(parallel, "unseen", runtime), "not-recorded");
  assert.match(model.activationReason(parallel, "unseen", runtime), /parallel parent was visited/i);
  assert.equal(model.edgeActivationLabel(parallel.byId.parallel, "right", parallel), "tick with siblings");
});

test("an unvisited parent or ancestor blocks descendants on an otherwise recorded tick", () => {
  const tree = model.normalizeTree({ nodes: [
    { id: "root", type: "Sequence", children: ["branch"] },
    { id: "branch", type: "Sequence", children: ["leaf"] },
    { id: "leaf", name: "operation" },
  ] });
  const parentMissing = { currentTick: { nodes: ["root"] } };
  assert.equal(model.activationState(tree, "leaf", parentMissing), "blocked");
  assert.match(model.activationReason(tree, "leaf", parentMissing), /branch was not visited/i);
  const ancestorMissing = { currentTick: { nodes: ["branch"] } };
  assert.equal(model.activationState(tree, "leaf", ancestorMissing), "blocked");
  assert.match(model.activationReason(tree, "leaf", ancestorMissing), /root was not visited/i);
});

test("semantic skeleton supplies collapsed-subtree and edge DTOs for large trees", () => {
  const graph = model.normalizeTree({ nodes: [
    { id: "root", type: "Sequence", children: ["prepare", "choice"] },
    { id: "prepare", name: "Prepare" },
    { id: "choice", type: "Selector", children: ["fast", "slow"] },
    { id: "fast", name: "Fast route" }, { id: "slow", name: "Slow route" },
  ] });
  const skeleton = model.semanticSkeleton(graph, {
    mode: "collapsed", expandedIds: ["root"], runtime: { currentTick: { nodes: ["root", "prepare"] } },
  });
  assert.deepEqual(skeleton.nodes.map(node => node.id), ["root", "prepare", "choice"]);
  assert.deepEqual(skeleton.edges.map(edge => [edge.parentId, edge.childId, edge.label]), [
    ["root", "prepare", "start sequence"], ["root", "choice", "after previous succeeds"],
  ]);
  assert.deepEqual(skeleton.collapsedSubtrees, [{ rootId: "choice", descendantIds: ["fast", "slow"], count: 2, reason: "collapsed" }]);
  assert.deepEqual(model.collapsedSubtree(graph, "choice"), skeleton.collapsedSubtrees[0]);
  assert.equal(skeleton.nodes[0].activation, "ticked");
  assert.equal(skeleton.nodes[2].collapsed, true);

  const focused = model.semanticSkeleton(graph, { mode: "collapsed", focusId: "slow", search: "slow" });
  assert.equal(focused.focusId, "slow");
  assert.ok(focused.nodes.some(node => node.id === "slow" && node.focused && node.matchesSearch));
  const limited = model.semanticSkeleton(graph, { maxNodes: 2 });
  assert.equal(limited.nodes.length, 2);
  assert.equal(limited.truncated, true);
});
