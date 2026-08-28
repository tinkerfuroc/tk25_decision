// tools/gpsr_ui/static/tree.js
//
// Pure, DOM-free tree logic for the behaviour-tree panel (Task 9). Kept
// free of any `document`/SVG code so node:test can exercise layout,
// status-at-time resolution and bookkeeping classification directly; the
// DOM wiring that consumes these lives in run.js's mountTree().
const ROW = 16;
const COL = 168;
// The brief's own list (nav keepalive/keepalive gap/say keepalive/clear/
// set/latch) only flags 23% of nodes in the reference run as bookkeeping.
// Two real, high-volume patterns slip through it entirely:
//   - "reset <thing>" nodes (reset state_log, reset plan_index, reset
//     last_failure, ...) are the single most frequent node name in the
//     whole corpus (886 occurrences in a 41-run sample, well ahead of any
//     keepalive variant) -- the same "clear the blackboard" bookkeeping
//     as `clear plan_index`, just spelled with a different verb.
//   - the keepalive WRAPPERS ("loop nav keepalive", "keep-alive lines")
//     don't start with "nav keepalive"/"keepalive gap"/"say keepalive"
//     the way their own children do, so the anchored regexes miss the
//     composite/decorator that groups them.
// Adding `^reset ` and an unanchored (hyphen-tolerant) `keep-?alive`
// raises coverage to 40-49% on the same run, matching the domain fact
// that these nodes "dominate the tree by count".
const BOOKKEEPING = [/^nav keepalive/i, /^keepalive gap/i,
  /^say keepalive/i, /^clear /i, /^set /i, /^latch /i, /^reset /i,
  /keep-?alive/i];

export function isBookkeeping(node) {
  const name = (node && node.name) || "";
  return BOOKKEEPING.some((re) => re.test(name));
}

// Depth-first layout: depth sets the column, a running counter sets the
// row. Guards against cycles (seen-set) and dangling child ids (skip, do
// not place, do not throw) because the corpus is machine-generated and a
// bad edge must not hang or crash the page -- see the brief's own
// call-out that a cycle/missing-root/dangling-child tree must not hang.
export function layoutTree(nodes, rootId) {
  const byId = new Map(nodes.map((n) => [n.id, n]));
  const positions = new Map();
  if (!byId.has(rootId)) return { positions, width: 0, height: 0 };

  const seen = new Set();
  let row = 0;
  let maxDepth = 0;

  const walk = (id, depth) => {
    if (seen.has(id)) return;
    seen.add(id);
    const node = byId.get(id);
    if (!node) return;
    positions.set(id, { x: depth * COL, y: row * ROW, depth });
    row += 1;
    maxDepth = Math.max(maxDepth, depth);
    for (const child of node.children || []) walk(child, depth + 1);
  };
  walk(rootId, 0);

  return { positions, width: (maxDepth + 1) * COL, height: row * ROW };
}

// Last transition per node at or before `wall`. A node absent from the
// returned map has never ticked at or before this playhead position --
// callers must distinguish that ("never ticked") from a node whose last
// tick actually resolved to a terminal status, rather than defaulting a
// missing entry to some status value.
export function statusAt(transitions, wall) {
  const out = new Map();
  for (const t of transitions) {
    if (t.wall === null || t.wall === undefined || t.wall > wall) continue;
    out.set(t.node_id, { status: t.status, feedback: t.feedback });
  }
  return out;
}

// Every transition recorded for one node, oldest first, unfiltered by
// playhead. This is what lets the detail pane show a node's full
// feedback history rather than only its status at the instant it was
// clicked -- a node that flapped RUNNING/FAILURE/RUNNING across retries
// is exactly the kind of thing triage needs to see in full.
export function historyFor(transitions, nodeId) {
  const out = [];
  for (const t of transitions || []) {
    if (t.node_id === nodeId) out.push(t);
  }
  return out;
}

// Elbow-connector endpoints for every non-root node: (parent position) ->
// (child position). Nodes whose parent (or the node itself) didn't make
// it into `positions` -- an unlaid root, a dangling child id skipped by
// layoutTree -- are silently omitted rather than drawn with an undefined
// endpoint.
export function edgesFor(nodes, positions) {
  const edges = [];
  for (const node of nodes) {
    if (node.parent_id == null) continue;
    const from = positions.get(node.parent_id);
    const to = positions.get(node.id);
    if (!from || !to) continue;
    edges.push({ id: node.id, parentId: node.parent_id, from, to });
  }
  return edges;
}

// Ids of every node that is currently RUNNING, plus all of its ancestors
// up to the root -- "the path to the active node" the domain notes call
// for, computed once per redraw so the DOM layer can add a highlight
// class instead of the bookkeeping dimming being the only visual signal.
// Guards against a cycle in parent_id chasing the same way layoutTree
// guards its child-chasing walk.
export function activeAncestorIds(nodes, statuses) {
  const byId = new Map(nodes.map((n) => [n.id, n]));
  const ids = new Set();
  for (const node of nodes) {
    const st = statuses.get(node.id);
    if (!st || st.status !== "RUNNING") continue;
    let cur = node.id;
    const seen = new Set();
    while (cur != null && !seen.has(cur)) {
      seen.add(cur);
      ids.add(cur);
      const parent = byId.get(cur);
      cur = parent ? parent.parent_id : null;
    }
  }
  return ids;
}
