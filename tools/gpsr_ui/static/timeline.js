// tools/gpsr_ui/static/timeline.js
const MILESTONE_KINDS = ["NAV", "VISION", "AUDIO", "MANIP"];

export function parseWall(value) {
  if (typeof value === "number") return value;
  if (typeof value !== "string") return null;
  const ms = Date.parse(value);
  return Number.isNaN(ms) ? null : ms / 1000;
}

export function xOf(wall, start, end, width) {
  if (end <= start) return 0;
  return ((wall - start) / (end - start)) * width;
}

// tree_revision is 0 everywhere in the corpus, so it is never consulted.
// Replan events arrive already classified in model.judge_events.
export function buildLanes(model) {
  const lanes = [];

  for (const kind of MILESTONE_KINDS) {
    lanes.push({
      id: kind,
      label: kind,
      items: (model.milestones || [])
        .filter((m) => m.kind === kind)
        .map((m) => ({
          wall: parseWall(m.wall), kind, status: m.status,
          name: m.name, info: m.info,
        }))
        .filter((m) => m.wall !== null),
    });
  }

  lanes.push({
    id: "judge",
    label: "judge",
    items: (model.judge_events || [])
      .map((j) => ({
        wall: parseWall(j.wall), kind: j.kind, status: j.status,
        name: j.name, info: j.info,
      }))
      .filter((j) => j.wall !== null),
  });

  // No replan lane: the vendored classifier already emits REPLAN as a judge
  // event when a run has 3+ tree generations. Two generations is the normal
  // skeleton-then-materialise pair, not a replan.
  return lanes;
}

const DEFAULT_CLUSTER_PX = 3;

// Collapses a lane's items for DRAWING ONLY -- buildLanes() above still
// returns every event; this is a pure, DOM-free rendering decision so a
// saturated lane (a real corpus run has one with 1281 milestones) reads as
// discrete marks instead of a solid bar.
//
// FAILURE marks are never merged with each other: each is its own group
// and is always rendered as its own targetable mark, because in triage the
// failures are exactly what a user needs to click. A SUCCESS/RUNNING item
// within `thresholdPx` of a failure is absorbed into that failure's mark
// (the failure's identity, colour and click target win; the absorbed item
// only adds to the count) rather than forming a separate mark that would
// visually collide with it. Remaining non-failure items are clustered
// among themselves by ordinary chain proximity (each item compared to the
// last item added to the current cluster, so a dense run collapses into
// one representative even though the cluster's overall span exceeds the
// threshold).
//
// Every returned mark carries `count` (how many original items it stands
// for) and `collapsed` (count > 1); every input item is accounted for in
// exactly one returned mark's count.
export function collapseLaneItems(items, start, end, width, thresholdPx = DEFAULT_CLUSTER_PX) {
  const withX = items
    .map((item) => ({ item, x: xOf(item.wall, start, end, width) }))
    .sort((a, b) => a.x - b.x);

  const failureGroups = withX
    .filter((e) => e.item.status === "FAILURE")
    .map((anchor) => ({ anchor, members: [anchor] }));

  const others = withX.filter((e) => e.item.status !== "FAILURE");
  const leftover = [];

  for (const entry of others) {
    let nearest = null;
    let nearestDist = Infinity;
    for (const group of failureGroups) {
      const dist = Math.abs(entry.x - group.anchor.x);
      if (dist <= thresholdPx && dist < nearestDist) {
        nearest = group;
        nearestDist = dist;
      }
    }
    if (nearest) {
      nearest.members.push(entry);
    } else {
      leftover.push(entry);
    }
  }

  const successClusters = [];
  for (const entry of leftover) {
    const current = successClusters[successClusters.length - 1];
    if (current && Math.abs(entry.x - current[current.length - 1].x) <= thresholdPx) {
      current.push(entry);
    } else {
      successClusters.push([entry]);
    }
  }

  const rendered = [];
  for (const group of failureGroups) {
    rendered.push({ ...group.anchor.item, count: group.members.length,
      collapsed: group.members.length > 1 });
  }
  for (const cluster of successClusters) {
    rendered.push({ ...cluster[0].item, count: cluster.length,
      collapsed: cluster.length > 1 });
  }

  rendered.sort((a, b) => xOf(a.wall, start, end, width) - xOf(b.wall, start, end, width));
  return rendered;
}
