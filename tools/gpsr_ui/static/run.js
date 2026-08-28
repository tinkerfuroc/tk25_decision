// tools/gpsr_ui/static/run.js
//
// Imports live at the top of this file, always. Later tasks (9, 10) hang
// the tree panel and stop-motion viewer off the playhead created here by
// appending more function bodies and call sites to boot() -- any new
// import they need must be added up here too, never inline further down.
import { createPlayhead } from "./playhead.js";
import { buildLanes, collapseLaneItems, parseWall, xOf } from "./timeline.js";

const SVG_NS = "http://www.w3.org/2000/svg";
const LANE_HEIGHT = 18;
const MARK_RADIUS = 4;
const STATUS_CLASS = { SUCCESS: "ok", FAILURE: "bad", RUNNING: "run" };
// collapseLaneItems' own default threshold (3px) is a floor for the pure
// function, not what draws the ribbon: two marks whose CENTRES are 3-8px
// apart still visually overlap once each is drawn as an r=4 (8px
// diameter) circle, which is exactly the "solid bar" this collapsing
// exists to fix. Cluster at (at least) the mark's own diameter plus a
// small gap so surviving marks are actually visually discrete.
const CLUSTER_PX = MARK_RADIUS * 2 + 2;

function el(tag, attrs) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  return node;
}

// Builds the /api/run/... URL for a tier[/suffix]/dir_name path. `tier`
// may itself contain an embedded slash (real corpus fact: pseudo-tiers
// like "t2-2026/invalidated-20260826" sit alongside a plain "t2-2026/runs"
// dir) -- encodeURIComponent on the whole tier would turn that slash into
// a literal "%2F" segment and break the {path:path} route match, so each
// segment is encoded individually and rejoined.
function apiRunUrl(tier, dirName) {
  const segments = [...tier.split("/"), dirName].map(encodeURIComponent);
  return `/api/run/${segments.join("/")}`;
}

function runBounds(model) {
  const walls = [];
  // `!= null` on purpose: a wall time of exactly 0 (epoch) is a real,
  // meaningful timestamp and must not be dropped by a truthy check.
  if (model.started_wall != null) walls.push(model.started_wall);
  if (model.finished_wall != null) walls.push(model.finished_wall);
  for (const e of model.epochs || []) {
    const w = parseWall(e.wall);
    if (w !== null) walls.push(w);
  }
  for (const m of model.milestones || []) {
    const w = parseWall(m.wall);
    if (w !== null) walls.push(w);
  }
  for (const j of model.judge_events || []) {
    const w = parseWall(j.wall);
    if (w !== null) walls.push(w);
  }
  if (walls.length === 0) return { start: 0, end: 1 };
  return { start: Math.min(...walls), end: Math.max(...walls) };
}

function renderRibbon(svg, lanes, bounds, playhead) {
  const width = svg.clientWidth || 900;
  svg.setAttribute("height", lanes.length * LANE_HEIGHT + 24);
  svg.replaceChildren();

  lanes.forEach((lane, row) => {
    const y = row * LANE_HEIGHT + 12;
    svg.appendChild(el("line", {
      x1: 0, x2: width, y1: y, y2: y, class: "lane-rule",
    }));
    const label = el("text", { x: 2, y: y - 4, class: "lane-label" });
    label.textContent = lane.label;
    svg.appendChild(label);

    // Collapsing is a draw-time-only decision: buildLanes() above still
    // returns every event untouched, so nothing downstream (Task 9/10, or
    // this lane's own item count) loses data -- only what gets a circle
    // on the ribbon is reduced when marks would land within a few pixels
    // of each other. A FAILURE is never merged away; see timeline.js.
    const collapsed = collapseLaneItems(
      lane.items, bounds.start, bounds.end, width, CLUSTER_PX);
    for (const item of collapsed) {
      const mark = el("circle", {
        cx: xOf(item.wall, bounds.start, bounds.end, width),
        cy: y, r: MARK_RADIUS,
        class: `mark ${STATUS_CLASS[item.status] || "run"}`,
      });
      const title = el("title", {});
      const collapsedNote = item.collapsed
        ? `\n(+${item.count - 1} more nearby)` : "";
      title.textContent = `${item.name}\n${item.info || ""}${collapsedNote}`;
      mark.appendChild(title);
      mark.addEventListener("click", () => playhead.set(item.wall));
      svg.appendChild(mark);
    }
  });

  const cursor = el("line", {
    y1: 0, y2: lanes.length * LANE_HEIGHT + 12, class: "cursor",
    x1: 0, x2: 0,
  });
  svg.appendChild(cursor);

  const move = (wall) => {
    const x = xOf(wall, bounds.start, bounds.end, width);
    cursor.setAttribute("x1", x);
    cursor.setAttribute("x2", x);
  };
  move(playhead.get());
  playhead.subscribe(move);

  svg.addEventListener("click", (event) => {
    if (event.target !== svg) return;
    const rect = svg.getBoundingClientRect();
    const frac = (event.clientX - rect.left) / rect.width;
    playhead.set(bounds.start + frac * (bounds.end - bounds.start));
  });
}

export async function boot({ tier, dirName }) {
  const base = apiRunUrl(tier, dirName);
  const model = await (await fetch(base)).json();

  const badge = document.getElementById("clock-badge");
  badge.textContent = `clock: ${model.clock_mode}`;
  badge.className = `badge clock-${model.clock_mode}`;
  badge.title = model.clock_mode === "approximate"
    ? "Frame times interpolated from recorder-meta.json. Accurate to seconds "
      + "only; RTF varies 0.2-0.5 within a run."
    : model.clock_mode === "exact"
      ? "Frames joined to wall time via frames/index.jsonl."
      : "No frame/wall mapping available for this run.";

  document.getElementById("regen-count").textContent = model.tree_regenerations;
  document.getElementById("gate-failures").textContent = model.gate_failures;
  document.getElementById("verdict").textContent = model.verdict || "in flight";
  document.getElementById("verdict").className =
    `verdict v-${(model.verdict || "none").toLowerCase()}`;

  const bounds = runBounds(model);
  const playhead = createPlayhead(bounds);
  renderRibbon(
    document.getElementById("ribbon"), buildLanes(model), bounds, playhead);

  window.__gpsr = { model, playhead, bounds, base };
  return window.__gpsr;
}
