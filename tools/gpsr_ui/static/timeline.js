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
