// tools/gpsr_ui/static/run.js
//
// Imports live at the top of this file, always. Later tasks (9, 10) hang
// the tree panel and stop-motion viewer off the playhead created here by
// appending more function bodies and call sites to boot() -- any new
// import they need must be added up here too, never inline further down.
import { createPlayhead } from "./playhead.js";
import { buildLanes, collapseLaneItems, parseWall, xOf } from "./timeline.js";
import {
  activeAncestorIds, edgesFor, historyFor, isBookkeeping, isHiddenBookkeeping,
  layoutTree, statusAt,
} from "./tree.js";
import {
  createPlayer, frameAt, frameAtFraction, hasNoWallTimes, preloadWindow,
} from "./frames.js";

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

// Shared by every URL builder below (run API, frames API, frame image):
// `tier` may itself contain an embedded slash (real corpus fact:
// pseudo-tiers like "t2-2026/invalidated-20260826" sit alongside a plain
// "t2-2026/runs" dir) -- encodeURIComponent on the whole tier would turn
// that slash into a literal "%2F" segment and break the server's
// {path:path} route match, so each segment (tier's own parts, dir_name,
// and anything else) is encoded individually and rejoined. Task 8 found
// and fixed exactly this bug for the run API; the frame routes need the
// same treatment, not a fresh bare encodeURIComponent(tier).
function pathSegments(tier, dirName, ...rest) {
  return [...tier.split("/"), dirName, ...rest].map(encodeURIComponent).join("/");
}

function apiRunUrl(tier, dirName) {
  return `/api/run/${pathSegments(tier, dirName)}`;
}

// The frames listing and frame-image routes live under their OWN
// top-level prefixes (/api/frames/..., /frame/...), not nested under
// /api/run/..., precisely so a greedy {path:path} match on /api/run/...
// can never swallow them first (see app.py's module docstring).
function apiFramesUrl(tier, dirName) {
  return `/api/frames/${pathSegments(tier, dirName)}`;
}

function frameUrl(tier, dirName, label, file) {
  return `/frame/${pathSegments(tier, dirName, label, file)}`;
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

// Renders the behaviour tree as it stood at the playhead's moment and
// lets a click show a node's blackboard access and full feedback
// history. "The tree" is whichever epoch is latest at-or-before the
// playhead -- normally exactly two (a startup skeleton, then the
// executor's materialised plan), so the panel must redraw the whole
// tree (not just recolour it) whenever the epoch changes shape. Layout,
// status-at-time and bookkeeping classification are pure functions from
// tree.js so they're covered by node:test; only DOM wiring lives here.
function renderNodeDetail(detail, epoch, nodeId, transitions, wall) {
  if (!nodeId) {
    detail.textContent = "click a node";
    return;
  }
  const byId = new Map(epoch.nodes.map((n) => [n.id, n]));
  const node = byId.get(nodeId);
  if (!node) {
    // The tree changed shape (epoch switch) and this id isn't in the new
    // one -- say so rather than showing stale or blank detail.
    detail.textContent = `(node ${nodeId} is not present in epoch ${epoch.ordinal})`;
    return;
  }
  const current = statusAt(transitions, wall).get(nodeId);
  const history = historyFor(transitions, nodeId);
  const lines = [
    `id      : ${node.id}`,
    `name    : ${node.name}`,
    `type    : ${node.type}`,
    `class   : ${node.node_class}`,
    `reads   : ${(node.reads || []).join(", ") || "-"}`,
    `writes  : ${(node.writes || []).join(", ") || "-"}`,
    "",
    `status @ playhead : ${current ? current.status : "(not yet ticked)"}`,
    `feedback          : ${current ? current.feedback || "-" : "-"}`,
    "",
    `full history (${history.length} tick${history.length === 1 ? "" : "s"}):`,
  ];
  if (history.length === 0) {
    lines.push("  (never ticked)");
  } else {
    for (const t of history) {
      lines.push(`  tick ${t.tick} @ ${t.wall} : ${t.status}  ${t.feedback || ""}`);
    }
  }
  detail.textContent = lines.join("\n");
}

function mountTree(container, model, playhead) {
  const panel = document.createElement("div");
  panel.className = "tree-panel";

  const toolbar = document.createElement("div");
  toolbar.className = "tree-toolbar";
  const hideLabel = document.createElement("label");
  const hideInput = document.createElement("input");
  hideInput.type = "checkbox";
  hideInput.checked = true;
  hideLabel.append(hideInput, document.createTextNode(" hide bookkeeping nodes"));
  toolbar.appendChild(hideLabel);

  const svg = document.createElementNS(SVG_NS, "svg");
  svg.setAttribute("class", "tree");
  const detail = document.createElement("pre");
  detail.className = "node-detail";
  detail.textContent = "click a node";
  panel.append(toolbar, svg, detail);
  container.appendChild(panel);

  const applyHideToggle = () => svg.classList.toggle("hide-bookkeeping", hideInput.checked);
  hideInput.addEventListener("change", applyHideToggle);
  applyHideToggle();

  let currentEpoch = null;
  let selectedNodeId = null;
  // Set whenever the epoch (re)draws below; reused every tick so the
  // colour/hide pass doesn't need to rebuild a lookup map on every
  // playhead move.
  let nodesById = new Map();
  // Vertical clearance for the "epoch N -- M nodes" label so it doesn't
  // sit on top of the root node's own dot/text at row 0, column 0.
  const TOP_MARGIN = 14;

  const draw = (wall) => {
    // The tree shown is the latest epoch at or before the playhead.
    let epoch = null;
    for (const e of model.epochs) {
      if (e.wall !== null && e.wall <= wall) epoch = e;
    }
    if (epoch === null) epoch = model.epochs[0];
    if (!epoch) return;

    if (epoch !== currentEpoch) {
      currentEpoch = epoch;
      svg.replaceChildren();
      const { positions, width, height } = layoutTree(epoch.nodes, epoch.root_id);
      svg.setAttribute("width", width + 220);
      svg.setAttribute("height", height + 20 + TOP_MARGIN);

      nodesById = new Map(epoch.nodes.map((n) => [n.id, n]));

      const edgesLayer = el("g", { class: "edges" });
      svg.appendChild(edgesLayer);
      for (const edge of edgesFor(epoch.nodes, positions)) {
        const path = el("path", {
          class: "edge",
          d: `M ${edge.from.x} ${edge.from.y + 10 + TOP_MARGIN} `
            + `V ${edge.to.y + 10 + TOP_MARGIN} H ${edge.to.x}`,
        });
        path.dataset.nodeId = edge.id;
        edgesLayer.appendChild(path);
      }

      for (const [id, pos] of positions) {
        const node = nodesById.get(id);
        const g = document.createElementNS(SVG_NS, "g");
        g.setAttribute("class",
          `node ${isBookkeeping(node) ? "bookkeeping" : ""}`);
        g.setAttribute("transform", `translate(${pos.x},${pos.y + 10 + TOP_MARGIN})`);
        g.dataset.nodeId = id;
        if (id === selectedNodeId) g.classList.add("selected");

        const dot = document.createElementNS(SVG_NS, "circle");
        dot.setAttribute("r", 3.5);
        dot.setAttribute("class", "node-dot");
        const text = document.createElementNS(SVG_NS, "text");
        text.setAttribute("x", 8);
        text.setAttribute("y", 3);
        text.textContent = node.name;
        g.append(dot, text);

        g.addEventListener("click", () => {
          selectedNodeId = id;
          for (const other of svg.querySelectorAll("g.node.selected")) {
            other.classList.remove("selected");
          }
          g.classList.add("selected");
          renderNodeDetail(detail, epoch, id, model.transitions, playhead.get());
        });
        svg.appendChild(g);
      }
      const header = document.createElementNS(SVG_NS, "text");
      header.setAttribute("class", "epoch-label");
      header.setAttribute("x", 0);
      header.setAttribute("y", 8);
      header.textContent =
        `epoch ${epoch.ordinal} — ${epoch.nodes.length} nodes`;
      svg.appendChild(header);
    }

    const states = statusAt(model.transitions, wall);
    // "The path to the active node should be visible" -- every RUNNING
    // node's ancestor chain gets an on-path highlight distinct from the
    // bookkeeping dimming, so triage can trace what's currently ticking.
    const onPath = activeAncestorIds(epoch.nodes, states);
    for (const g of svg.querySelectorAll("g.node")) {
      const id = g.dataset.nodeId;
      const st = states.get(id);
      g.setAttribute("data-status", st ? st.status : "NONE");
      g.classList.toggle("on-path", onPath.has(id));
      // Recomputed every tick (not just on redraw): a node that ticks
      // FAILURE while hide-bookkeeping is on must become visible right
      // then, and re-hide if it later moves off FAILURE. isHiddenBookkeeping
      // is the single source of truth for this -- see tree.js.
      g.classList.toggle("hide-eligible", isHiddenBookkeeping(nodesById.get(id), st));
    }
    for (const path of svg.querySelectorAll("path.edge")) {
      // An edge's own "status" is its child node's status: the edge
      // leading to a FAILURE node must stay drawn along with that node,
      // or the node would render as a dot floating with no visible
      // connection to the rest of the tree.
      const id = path.dataset.nodeId;
      const st = states.get(id);
      path.classList.toggle("on-path", onPath.has(id));
      path.classList.toggle("hide-eligible", isHiddenBookkeeping(nodesById.get(id), st));
    }

    if (selectedNodeId) {
      renderNodeDetail(detail, epoch, selectedNodeId, model.transitions, wall);
    }
  };

  draw(playhead.get());
  playhead.subscribe(draw);
}

// The stop-motion camera viewer: one <img>+<figcaption> track per camera
// label (a run may have only one -- the real s2026-003-findObjInRoom has
// `head` but no `arena`), all driven off the shared playhead so a click
// on a ribbon mark jumps the tree panel AND every camera image to that
// same moment at once. That linked jump is the actual point of this
// tool, so this function's whole job is: never show a blank/broken image
// when a frame genuinely exists, and never show a frame from later than
// the moment being inspected.
function fractionOf(wall, bounds) {
  const span = bounds.end - bounds.start;
  if (!(span > 0)) return 0;
  return (wall - bounds.start) / span;
}

async function mountFrames(container, tier, dirName, playhead, bounds) {
  const payload = await (await fetch(apiFramesUrl(tier, dirName))).json();
  const labelRefs = payload.labels || {};
  const labels = Object.keys(labelRefs);
  const panel = document.createElement("div");
  panel.className = "frames-panel";

  if (labels.length === 0) {
    panel.innerHTML = "<p class='muted'>no frames recorded for this run</p>";
    container.appendChild(panel);
    return;
  }

  // clock_mode "none": not one frame in this run has a wall time to join
  // against (a real example lives in t2plus-2026). Rather than silently
  // showing no image at all (frameAt() would return null for every
  // track, forever), fall back to scrubbing by the playhead's fractional
  // position in the run's own timeline via frameAtFraction, and say so.
  const unaligned = hasNoWallTimes(labelRefs);
  if (unaligned) {
    const note = document.createElement("p");
    note.className = "muted frames-note";
    note.textContent = "no clock metadata for this run — frames are not "
      + "time-aligned; scrubbing by position only.";
    panel.appendChild(note);
  }

  const pick = (refs, wall) => (unaligned
    ? frameAtFraction(refs, fractionOf(wall, bounds))
    : frameAt(refs, wall));

  const tracks = labels.map((label) => {
    const refs = labelRefs[label];
    const wrap = document.createElement("figure");
    const img = document.createElement("img");
    img.loading = "eager";
    img.alt = `${label} camera`;
    const cap = document.createElement("figcaption");
    wrap.append(img, cap);
    panel.appendChild(wrap);
    return { label, refs, img, cap };
  });

  const controls = document.createElement("div");
  controls.className = "frame-controls";
  const button = document.createElement("button");
  button.type = "button";
  button.textContent = "play";
  const speed = document.createElement("select");
  for (const fps of [2, 5, 10, 20, 40]) {
    const option = document.createElement("option");
    option.value = String(fps);
    option.textContent = `${fps} fps`;
    if (fps === 10) option.selected = true;
    speed.appendChild(option);
  }
  controls.append(button, speed);
  panel.appendChild(controls);
  container.appendChild(panel);

  const render = (wall) => {
    for (const track of tracks) {
      const ref = pick(track.refs, wall);
      if (!ref) continue; // empty label: nothing to show, nothing to break
      const url = frameUrl(tier, dirName, track.label, ref.file);
      if (track.img.getAttribute("src") !== url) track.img.src = url;
      track.cap.textContent =
        `${track.label} · frame ${ref.index} · sim ${ref.stamp_s.toFixed(3)}s`;
      const idx = track.refs.indexOf(ref);
      preloadWindow(
        track.refs, idx, 4,
        (r) => frameUrl(tier, dirName, track.label, r.file),
        () => new Image());
    }
  };
  render(playhead.get());
  playhead.subscribe(render);

  // Frames are one per sim-second, so playback advances by frame INDEX
  // at the chosen fps, not by wall-clock rate (see frames.js). Stepping
  // moves the shared playhead itself -- not just the primary image --
  // so the tree/ribbon stay in lockstep with both cameras while playing.
  const primary = tracks[0];
  const player = createPlayer({
    onTick: () => {
      const ref = pick(primary.refs, playhead.get());
      const idx = ref ? primary.refs.indexOf(ref) : -1;
      const next = primary.refs[idx + 1];
      if (!next) {
        player.pause();
        button.textContent = "play";
        return;
      }
      if (unaligned) {
        const frac = primary.refs.length > 1 ? (idx + 1) / (primary.refs.length - 1) : 0;
        playhead.set(bounds.start + frac * (bounds.end - bounds.start));
      } else if (next.wall === null || next.wall === undefined) {
        player.pause();
        button.textContent = "play";
      } else {
        playhead.set(next.wall);
      }
    },
  });
  button.addEventListener("click", () => {
    if (player.isPlaying()) {
      player.pause();
      button.textContent = "play";
    } else {
      player.play(Number(speed.value));
      button.textContent = "pause";
    }
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

  mountTree(document.getElementById("panels"), model, playhead);
  await mountFrames(
    document.getElementById("panels"), tier, dirName, playhead, bounds);

  window.__gpsr = { model, playhead, bounds, base };
  return window.__gpsr;
}
