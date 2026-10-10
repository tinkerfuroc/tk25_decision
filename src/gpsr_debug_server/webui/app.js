/* GPSR Mission Debugger UI. Telemetry and model output are always text nodes. */
"use strict";

const API = "/api/v1";
const Model = window.GpsrUiModel;
const VIEW_IDS = new Set(["overview", "supervisor", "planning", "tree", "agents", "state", "controls"]);
const $ = id => document.getElementById(id);
const state = {
  token: "",
  controller: "",
  lease: null,
  trajectories: [],
  runStatus: "all",
  selectedId: null,
  snapshot: null,
  liveSnapshot: null,
  events: [],
  selectedEvent: null,
  causal: { ancestors: [], descendants: [] },
  selectedState: null,
  view: "overview",
  eventLimit: 250,
  eventCategory: "all",
  eventSearch: "",
  rawEvents: false,
  planRevision: null,
  treeKind: "executor",
  treeRevision: null,
  treeViewMode: "explain",
  expandedTreeBranches: new Set(),
  treeSearch: "",
  treeGraph: null,
  treeModel: null,
  treeVisual: null,
  selectedNodeId: null,
  agentGraph: null,
  historicalAt: null,
  socket: null,
  reconnectTimer: null,
  liveRefreshTimer: null,
  trajectoryRefreshTimer: null,
  historyTimer: null,
  graphResizeTimer: null,
  renderQueued: false,
  loadGeneration: 0,
  pendingCommand: null,
};

function el(tag, className, text) {
  const node = document.createElement(tag);
  if (className) node.className = className;
  if (text !== undefined && text !== null) node.textContent = String(text);
  return node;
}

function button(text, className, handler) {
  const node = el("button", className, text);
  node.type = "button";
  if (handler) node.addEventListener("click", handler);
  return node;
}

function eventType(event) {
  return event && (event.type || event.event_type) || "event";
}

function eventPayload(event) {
  const value = event && (event.payload ?? event.data);
  return value && typeof value === "object" && !Array.isArray(value) ? value : {};
}

function trajectoryId(item) {
  return item && (item.trajectory_id || item.id || item.run_id) || "";
}

function statusClass(value) {
  const category = Model.statusCategory(value);
  if (category === "cancelled") return "failure";
  return ["success", "running", "failure"].includes(category) ? category : "";
}

function humanType(value) {
  return String(value || "event")
    .replaceAll("_", " ")
    .replaceAll(".", " · ")
    .replace(/\b\w/g, match => match.toUpperCase());
}

function formatTime(value, includeDate = false) {
  if (!value) return "—";
  const date = new Date(value);
  if (!Number.isFinite(date.getTime())) return String(value);
  return new Intl.DateTimeFormat(undefined, includeDate
    ? { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit", second: "2-digit" }
    : { hour: "2-digit", minute: "2-digit", second: "2-digit" }).format(date);
}

function formatDuration(started, finished) {
  const start = Date.parse(started || "");
  const end = Date.parse(finished || "") || (started ? Date.now() : NaN);
  if (!Number.isFinite(start) || !Number.isFinite(end)) return "—";
  const seconds = Math.max(0, Math.round((end - start) / 1000));
  if (seconds < 60) return `${seconds}s`;
  const minutes = Math.floor(seconds / 60);
  return seconds < 3600 ? `${minutes}m ${seconds % 60}s` : `${Math.floor(minutes / 60)}h ${minutes % 60}m`;
}

function safeJson(value) {
  try {
    return JSON.stringify(value, null, 2);
  } catch (_) {
    return String(value);
  }
}

function truncate(value, length = 120) {
  const text = String(value ?? "").replace(/\s+/g, " ").trim();
  return text.length > length ? `${text.slice(0, length - 1)}…` : text;
}

function copyText(value) {
  if (navigator.clipboard && window.isSecureContext) return navigator.clipboard.writeText(String(value));
  const area = document.createElement("textarea");
  area.value = String(value);
  area.style.position = "fixed";
  area.style.opacity = "0";
  document.body.append(area);
  area.select();
  document.execCommand("copy");
  area.remove();
  return Promise.resolve();
}

async function request(path, options = {}) {
  const headers = { ...(options.headers || {}), "x-gpsr-session": state.token };
  const response = await fetch(`${API}${path}`, { ...options, headers });
  if (!response.ok) throw new Error(await response.text() || response.statusText);
  return response.json();
}

function setConnection(connected, label) {
  $("connection-dot").className = `dot ${connected ? "on" : "off"}`;
  $("connection-text").textContent = label || (connected ? "Live" : "Offline");
}

function queryState() {
  return {
    trajectoryId: state.selectedId,
    at: state.historicalAt,
    view: state.view,
    search: state.eventSearch,
    categories: state.eventCategory === "all" ? [] : [state.eventCategory],
    raw: state.rawEvents,
    focusId: state.selectedNodeId,
    treeRevision: state.treeRevision,
    treeMode: state.treeKind,
  };
}

function updateUrl() {
  const query = Model.serializeQuery(queryState());
  history.replaceState(null, "", `${location.pathname}${query}`);
}

async function boot() {
  bindInteractions();
  const query = Model.parseQuery(location.search);
  state.view = VIEW_IDS.has(query.view) ? query.view : "overview";
  state.eventSearch = query.search || "";
  state.eventCategory = query.categories[0] || "all";
  state.rawEvents = Boolean(query.raw);
  state.historicalAt = query.at;
  state.treeRevision = query.treeRevision;
  state.treeKind = new URLSearchParams(location.search).has("tree_mode") ? query.treeMode : "executor";
  state.selectedNodeId = query.focusId;
  $("event-search").value = state.eventSearch;
  $("event-category").value = state.eventCategory;
  $("raw-events").checked = state.rawEvents;
  activateView(state.view, false);
  try {
    const sessionResponse = await fetch(`${API}/session`);
    if (!sessionResponse.ok) throw new Error("Debugger session is unavailable");
    const session = await sessionResponse.json();
    state.token = session.token;
    state.lease = session.controller_lease;
    await refreshTrajectories(query.trajectoryId);
    connect();
    setConnection(true, "Live");
  } catch (error) {
    setConnection(false, "Unavailable");
    renderFatal(error);
  }
}

async function refreshTrajectories(preferredId = state.selectedId) {
  const result = await request("/trajectories?limit=500");
  state.trajectories = result.trajectories || [];
  renderTrajectories();
  const target = state.trajectories.some(item => trajectoryId(item) === preferredId)
    ? preferredId
    : trajectoryId(state.trajectories[0]);
  if (target && (target !== state.selectedId || !state.snapshot)) await selectTrajectory(target);
  else if (!target) renderNoTrajectories();
}

async function selectTrajectory(id, options = {}) {
  if (!id) return;
  const generation = ++state.loadGeneration;
  state.selectedId = id;
  state.selectedEvent = null;
  state.selectedState = null;
  state.causal = { ancestors: [], descendants: [] };
  state.eventLimit = 250;
  state.expandedTreeBranches.clear();
  state.treeSearch = "";
  $("tree-search").value = "";
  if (options.keepHistorical !== true && options.fromHistory !== true) state.historicalAt = null;
  renderTrajectories();
  renderLoadingMission();
  try {
    const at = state.historicalAt === null ? "" : `?at=${state.historicalAt}`;
    const [snapshot, eventResult] = await Promise.all([
      request(`/trajectories/${encodeURIComponent(id)}${at}`),
      state.events.length && options.reuseEvents
        ? Promise.resolve({ events: state.events })
        : request(`/trajectories/${encodeURIComponent(id)}/events?limit=10000`),
    ]);
    if (generation !== state.loadGeneration) return;
    state.snapshot = snapshot;
    if (state.historicalAt === null) state.liveSnapshot = snapshot;
    state.events = eventResult.events || [];
    initialiseSelections();
    renderAll();
    updateUrl();
  } catch (error) {
    if (generation === state.loadGeneration) renderFatal(error);
  }
}

function initialiseSelections() {
  const plans = state.snapshot?.plans?.revisions || {};
  if (!state.planRevision || !plans[state.planRevision]) {
    state.planRevision = String(state.snapshot?.plans?.active_revision ?? Object.keys(plans).at(-1) ?? "");
  }
  const revisions = treeRevisions(state.treeKind);
  if (!state.treeRevision || !revisions.some(([id]) => id === String(state.treeRevision))) {
    state.treeRevision = revisions.at(-1)?.[0] || null;
  }
  const lastSequence = Number(state.snapshot?.trajectory?.last_sequence ?? state.snapshot?.sequence ?? 0);
  $("history-range").max = String(Math.max(0, lastSequence));
  $("history-range").value = String(state.historicalAt ?? lastSequence);
}

function renderAll() {
  renderMission();
  renderPhases();
  renderHealth();
  renderTimeline();
  renderEventInspector();
  renderSupervisor();
  renderPlanning();
  renderTreeControls();
  renderAgents();
  renderState();
  renderControls();
  renderHistoryMode();
  if (state.view === "tree") queueGraphRender(renderTreeGraph);
  if (state.view === "agents") queueGraphRender(renderAgentGraph);
}

function renderTrajectories() {
  const root = $("trajectory-list");
  root.replaceChildren();
  const search = $("trajectory-filter").value.trim().toLowerCase();
  const filtered = state.trajectories.filter(item => {
    const status = statusClass(item.status);
    const matchesStatus = state.runStatus === "all"
      || (state.runStatus === "running" && status === "running")
      || (state.runStatus === "succeeded" && status === "success")
      || (state.runStatus === "failed" && status === "failure");
    return matchesStatus && (!search || safeJson(item).toLowerCase().includes(search));
  });
  for (const item of filtered) {
    const id = trajectoryId(item);
    const category = statusClass(item.status);
    const row = el("div", `trajectory ${state.selectedId === id ? "selected" : ""}`);
    row.tabIndex = 0;
    row.title = id;
    row.setAttribute("role", "button");
    row.setAttribute("aria-label", `${item.name || id}, ${item.status || "unknown"}`);
    const top = el("div", "trajectory-top");
    const label = item.name || `GPSR · ${Model.shortTrajectoryId(id, 20)}`;
    top.append(el("div", "name", label));
    const icon = item.pinned || item.name ? el("span", "pin", "◆") : el("span", `run-dot ${category}`);
    icon.title = item.pinned ? "Pinned" : item.name ? "Retained by name" : item.status || "unknown";
    top.append(icon);
    const meta = el("div", "trajectory-meta");
    meta.append(el("span", "", String(item.status || "unknown").replaceAll("_", " ")));
    meta.append(el("span", "", `${item.event_count || 0} events`));
    row.append(top, meta);
    row.addEventListener("click", () => selectTrajectory(id));
    row.addEventListener("keydown", event => {
      if (event.key === "Enter" || event.key === " ") {
        event.preventDefault();
        selectTrajectory(id);
      }
    });
    root.append(row);
  }
  if (!filtered.length) root.append(emptyBlock("No matching runs", "Try another search or status filter.", "⌕"));
}

function renderMission() {
  const root = $("mission-card");
  root.replaceChildren();
  if (!state.snapshot) {
    root.append(emptyBlock("Select a trajectory", "Choose a mission run from the sidebar.", "⌁"));
    return;
  }
  const snapshot = state.snapshot;
  const trajectory = snapshot.trajectory || {};
  const tasks = Object.values(snapshot.tasks || {});
  const agents = Object.values(snapshot.agents || {});
  const lead = el("div", "mission-lead");
  const titleRow = el("div", "mission-title-row");
  const title = snapshot.name || trajectory.name || `GPSR · ${Model.shortTrajectoryId(state.selectedId, 28)}`;
  titleRow.append(el("h2", "mission-title", title));
  titleRow.append(button("Copy ID", "button compact ghost", () => copyText(state.selectedId)));
  lead.append(titleRow);
  lead.append(el("div", "mission-id", state.selectedId));
  lead.append(el("div", "mission-command", snapshot.command || tasks[0]?.command || "No mission command recorded"));
  const actions = el("div", "mission-actions");
  actions.append(button(snapshot.name ? "Rename" : "Name run", "button compact", renameTrajectory));
  actions.append(button(snapshot.pinned ? "Unpin" : "Pin", "button compact ghost", togglePin));
  lead.append(actions);
  const box = el("div", "mission");
  box.append(lead);
  const metrics = [
    ["Status", snapshot.status || trajectory.status || "unknown", "status"],
    ["Duration", formatDuration(snapshot.started_at, snapshot.finished_at)],
    ["Tasks", tasks.length],
    ["Agents", agents.length],
  ];
  for (const [label, value, kind] of metrics) {
    const metric = el("div", `metric ${kind === "status" ? "status-metric" : ""}`);
    if (kind === "status") metric.append(el("strong", `status-label ${statusClass(value)}`, value));
    else metric.append(el("strong", "", value));
    metric.append(el("small", "", label));
    box.append(metric);
  }
  root.append(box);
}

function renderPhases() {
  const root = $("phase-rail");
  root.replaceChildren();
  const events = state.events;
  const snapshot = state.snapshot || {};
  const has = predicate => events.some(predicate);
  const phases = [
    { name: "Intake", detail: "Command received", done: Boolean(snapshot.command) || has(e => eventType(e) === "task.command_received") },
    { name: "Planning", detail: "LLM and validation", done: has(e => eventType(e) === "plan.committed") || Object.keys(snapshot.plans?.revisions || {}).length > 0 },
    { name: "Tree", detail: "Behavior generated", done: Object.keys(snapshot.trees?.revisions || {}).length > 0 },
    { name: "Execution", detail: "Steps dispatched", done: has(e => eventType(e) === "step.finished") || Object.values(snapshot.tasks || {}).some(t => ["succeeded", "failed"].includes(String(t.status).toLowerCase())) },
    { name: "Terminal", detail: snapshot.status || "Awaiting outcome", done: Boolean(snapshot.finished_at) },
  ];
  let firstPending = phases.findIndex(phase => !phase.done);
  if (firstPending < 0) firstPending = phases.length;
  phases.forEach((phase, index) => {
    const failed = index === phases.length - 1 && statusClass(snapshot.status) === "failure";
    const item = el("div", `phase ${phase.done ? "complete" : index === firstPending ? "active" : ""} ${failed ? "failed" : ""}`);
    item.append(el("span", "phase-node", phase.done ? "✓" : String(index + 1)));
    const copy = el("span", "phase-copy");
    copy.append(el("strong", "", phase.name), el("small", "", phase.detail));
    item.append(copy);
    root.append(item);
  });
  $("phase-summary").textContent = snapshot.finished_at
    ? `Completed ${formatTime(snapshot.finished_at, true)}`
    : `${state.events.length} events observed`;
}

function renderHealth() {
  const root = $("health-grid");
  root.replaceChildren();
  const snapshot = state.snapshot || {};
  const tasks = Object.values(snapshot.tasks || {});
  const activeTask = tasks.find(task => task.status === "running") || tasks.at(-1);
  const planRevision = snapshot.plans?.active_revision ?? "—";
  const treeRevision = snapshot.trees?.active_revision ?? "—";
  const warnings = (snapshot.warnings || []).length + (snapshot.unknown_events || []).length;
  const cards = [
    ["Current task", activeTask ? Model.shortTrajectoryId(activeTask.task_id, 24) : "No active task", activeTask?.status || "—"],
    ["Plan revision", `Revision ${planRevision}`, `${Object.keys(snapshot.plans?.revisions || {}).length} committed`],
    ["Tree revision", String(treeRevision), `${Object.keys(snapshot.trees?.revisions || {}).length} documents`],
    ["Attention", warnings ? `${warnings} items` : "No warnings", warnings ? "Warnings or unknown events" : "Projection is clean"],
  ];
  for (const [label, value, detail] of cards) {
    const card = el("div", "surface health-card");
    card.append(el("p", "", label), el("strong", "", value), el("small", "", detail));
    root.append(card);
  }
}

function uiEventCategory(event) {
  const category = Model.eventCategory(event);
  if (["lifecycle", "heartbeat"].includes(category)) return "mission";
  if (category === "intervention") return "control";
  if (category === "failure") return "other";
  return ["planning", "tree", "execution", "agent", "state"].includes(category) ? category : "other";
}

function filteredEventGroups() {
  const filtered = Model.filterEvents(state.events, { search: state.eventSearch });
  const byCategory = state.eventCategory === "all"
    ? filtered
    : filtered.filter(event => uiEventCategory(event) === state.eventCategory);
  return Model.groupEvents(byCategory, { raw: state.rawEvents, windowMs: 1000 });
}

function eventSummary(event) {
  const payload = eventPayload(event);
  const message = payload.message || payload.reason || payload.action || payload.command || payload.status
    || payload.outcome || event.phase || "";
  const actor = event.agent_id || payload.agent_id || "";
  return truncate([actor, message].filter(Boolean).join(" · "), 110);
}

function renderTimeline() {
  const root = $("timeline");
  root.replaceChildren();
  const groups = filteredEventGroups();
  $("event-count").textContent = `${groups.length}${groups.length !== state.events.length ? ` / ${state.events.length}` : ""} rows`;
  const visible = groups.slice(0, state.eventLimit);
  for (const group of visible) {
    const event = group.last;
    const type = eventType(event);
    const category = uiEventCategory(event);
    const selected = state.selectedEvent && state.selectedEvent.event_id === event.event_id;
    const isFuture = state.historicalAt !== null && Number(event.sequence) > state.historicalAt;
    const row = el("div", `event ${category} ${selected ? "selected" : ""} ${isFuture ? "future" : ""}`);
    row.setAttribute("role", "listitem");
    row.tabIndex = 0;
    row.append(el("time", "", formatTime(group.firstOccurredAt)));
    const rail = el("span", "event-rail");
    rail.append(el("i", "event-dot"));
    row.append(rail);
    const main = el("div", "event-main");
    main.append(el("div", "event-label", humanType(type)));
    main.append(el("div", "event-sub", eventSummary(event) || event.event_id || "No summary"));
    row.append(main);
    if (group.count > 1) row.append(el("span", "event-count-pill", `×${group.count}`));
    row.addEventListener("click", () => selectEvent(event));
    row.addEventListener("keydown", keyboardActivate(() => selectEvent(event)));
    root.append(row);
  }
  $("event-more").classList.toggle("hidden", groups.length <= state.eventLimit);
  if (!groups.length) root.append(emptyBlock("No matching events", "Change the category, raw mode, or search query.", "⌕"));
}

async function selectEvent(event) {
  state.selectedEvent = event;
  state.causal = { ancestors: [], descendants: [] };
  renderTimeline();
  renderEventInspector();
  updateUrl();
  if (!event?.event_id || !state.selectedId) return;
  const base = `/trajectories/${encodeURIComponent(state.selectedId)}/events/${encodeURIComponent(event.event_id)}/causal`;
  try {
    const [ancestors, descendants] = await Promise.all([
      request(`${base}?direction=ancestors&limit=200`),
      request(`${base}?direction=descendants&limit=200`),
    ]);
    if (state.selectedEvent?.event_id !== event.event_id) return;
    state.causal = { ancestors: ancestors.events || [], descendants: descendants.events || [] };
    renderEventInspector();
  } catch (_) {
    // The event remains fully inspectable if a legacy server lacks this route.
  }
}

function renderEventInspector() {
  const root = $("event-inspector");
  root.replaceChildren();
  const event = state.selectedEvent;
  if (!event) {
    root.className = "inspector";
    root.append(emptyBlock("Choose an event", "Inspect its payload, timing, and causal chain.", "↳"));
    $("event-kind").textContent = "None";
    return;
  }
  root.className = "inspector";
  $("event-kind").textContent = humanType(eventType(event));
  const envelope = inspectorSection("Envelope");
  const list = el("dl", "definition-list");
  const rows = [
    ["Sequence", event.sequence],
    ["Timestamp", formatTime(event.occurred_at || event.timestamp, true)],
    ["Phase", event.phase || "—"],
    ["Source", event.source_id || "—"],
    ["Task", event.task_id || "—"],
    ["Agent", event.agent_id || eventPayload(event).agent_id || "—"],
    ["Event ID", event.event_id || "—"],
  ];
  for (const [name, value] of rows) list.append(el("dt", "", name), el("dd", "", value));
  envelope.append(list);
  const payload = inspectorSection("Payload");
  const payloadCode = el("pre", "code-block", safeJson(eventPayload(event)));
  payload.append(payloadCode, button("Copy event JSON", "button compact ghost", () => copyText(safeJson(event))));
  const causal = inspectorSection("Causal chain");
  causal.append(causalGroup("Ancestors", state.causal.ancestors), causalGroup("Descendants", state.causal.descendants));
  root.append(envelope, payload, causal);
}

function inspectorSection(title) {
  const section = el("section", "inspector-section");
  section.append(el("h3", "", title));
  return section;
}

function causalGroup(title, events) {
  const block = el("div", "causal-block");
  block.append(el("h3", "", `${title} · ${events.length}`));
  const list = el("div", "causal-list");
  events.slice(0, 20).forEach(event => {
    const link = button(humanType(eventType(event)), "causal-link");
    link.append(el("span", "", `#${event.sequence ?? "—"}`));
    link.addEventListener("click", () => selectEvent(event));
    list.append(link);
  });
  if (!events.length) list.append(el("span", "quiet-label", "None recorded"));
  block.append(list);
  return block;
}

function renderSupervisor() {
  const checkpoints = Model.supervisorCheckpoints(state.events);
  const metrics = $("supervisor-metrics");
  metrics.replaceChildren();
  const local = checkpoints.filter(item => item.recoveries.some(event => event.type === "supervisor.recovery.proposed")).length;
  const global = checkpoints.filter(item => item.global).length;
  const clear = checkpoints.filter(item => item.verdict?.verdict === "all_clear").length;
  [
    ["Checkpoints", checkpoints.length],
    ["All clear", clear],
    ["Local recovery", local],
    ["Global / stop", global],
  ].forEach(([label, value]) => {
    const item = el("div", "supervisor-metric");
    item.append(el("strong", "", value), el("span", "", label));
    metrics.append(item);
  });

  renderSupervisorTestMatrix();
  const root = $("supervisor-checkpoints");
  root.replaceChildren();
  checkpoints.forEach((checkpoint, index) => root.append(renderSupervisorCheckpoint(checkpoint, index)));
  if (!checkpoints.length) {
    root.append(emptyBlock(
      "No supervisor checkpoints",
      "Enable GPSR supervision or load the hardware-free validation replay.",
      "◎",
    ));
  }
}

function renderSupervisorTestMatrix() {
  const root = $("supervisor-test-matrix");
  root.replaceChildren();
  const event = state.events.find(item => eventType(item) === "supervisor.test.summary");
  const payload = eventPayload(event);
  const tests = Array.isArray(payload.tests) ? payload.tests : [];
  if (!tests.length) return;
  const heading = el("div", "supervisor-test-heading");
  heading.append(
    el("div", "", payload.title || "Hardware-free validation"),
    el("span", "badge success", payload.status || "passed"),
  );
  root.append(heading);
  const grid = el("div", "test-role-grid");
  tests.forEach(test => {
    const card = el("article", "surface test-role");
    const top = el("div", "test-role-top");
    top.append(el("strong", "", test.role || test.name || "Test"));
    top.append(el("span", `badge ${statusClass(test.status || "passed")}`, test.status || "passed"));
    card.append(top);
    card.append(el("p", "", test.coverage || test.description || ""));
    const meta = el("div", "test-role-meta");
    [test.model, test.effort ? `${test.effort} reasoning` : "", test.mode].filter(Boolean)
      .forEach(value => meta.append(el("span", "", value)));
    card.append(meta);
    grid.append(card);
  });
  root.append(grid);
}

function renderSupervisorCheckpoint(checkpoint, index) {
  const created = checkpoint.created || {};
  const verdict = checkpoint.verdict || {};
  const terminal = created.node || created.terminal_node || {};
  const card = el("article", "surface supervisor-card");
  const head = el("div", "supervisor-card-head");
  const title = el("div");
  title.append(el("p", "section-kicker", created.test_case || `Checkpoint ${index + 1}`));
  title.append(el("h2", "", terminal.name || terminal.class_name || created.subtask_goal || checkpoint.checkpointId));
  title.append(el("p", "checkpoint-id", checkpoint.checkpointId));
  const route = supervisorRoute(checkpoint);
  head.append(title, el("span", `decision-pill ${route.className}`, route.label));
  card.append(head);

  const flow = el("div", "supervisor-flow");
  flow.append(
    supervisorFlowStep("1", "BT returned", created.reported_status || terminal.reported_status || "recorded", statusClass(created.reported_status || terminal.reported_status)),
    el("span", "supervisor-arrow", "→"),
    supervisorFlowStep("2", "Luna verified", verdict.verdict || (checkpoint.unavailable ? "unavailable" : "pending"), supervisorDecisionClass(verdict.verdict)),
    el("span", "supervisor-arrow", "→"),
    supervisorFlowStep("3", "Runtime action", route.label, route.className),
  );
  card.append(flow);

  const evidence = el("div", "supervisor-evidence-layout");
  const artifacts = el("section", "supervisor-pane");
  artifacts.append(el("h3", "", "Synchronized visual context"));
  const artifactGrid = el("div", "artifact-grid");
  const artifactList = Array.isArray(created.artifacts) ? created.artifacts : [];
  ["front_camera", "wrist_camera", "map", "arm"].forEach(role => {
    artifactGrid.append(renderSupervisorArtifact(artifactList.find(item => item.role === role), role));
  });
  artifacts.append(artifactGrid);

  const context = el("section", "supervisor-pane context-pane");
  context.append(el("h3", "", "Decision context"));
  const contextGrid = el("div", "context-grid");
  contextGrid.append(
    supervisorContextItem("Subtask goal", created.subtask_goal || "Not recorded"),
    supervisorContextItem("Current node", terminal.name || terminal.class_name || safeJson(terminal)),
    supervisorContextItem("Next node", nodeLabel(created.next_node)),
    supervisorContextItem("Tree", treeSummary(created.subtask_tree)),
  );
  context.append(contextGrid);
  const blackboard = el("details", "supervisor-details");
  blackboard.append(el("summary", "", `Associated blackboard · ${Object.keys(created.blackboard || {}).length} keys`));
  blackboard.append(el("pre", "code-block", safeJson(created.blackboard || {})));
  context.append(blackboard);
  evidence.append(artifacts, context);
  card.append(evidence);

  card.append(renderSupervisorDecision(checkpoint));
  return card;
}

function renderSupervisorArtifact(artifact, role) {
  const card = el("figure", `artifact-card ${artifact?.missing ? "missing" : ""}`);
  const source = supervisorArtifactSource(artifact);
  if (source) {
    const image = document.createElement("img");
    image.src = source;
    image.alt = `${humanType(role)} checkpoint evidence`;
    image.loading = "lazy";
    card.append(image);
  } else {
    card.append(el("div", "artifact-placeholder", artifact?.missing ? "Missing" : "Not captured"));
  }
  const caption = document.createElement("figcaption");
  caption.append(el("strong", "", humanType(role)));
  const detail = artifact?.metadata?.robot_pose
    ? `pose ${artifact.metadata.robot_pose.join(", ")}`
    : artifact?.metadata?.joints
      ? `${artifact.metadata.joints.length} joints`
      : artifact?.metadata?.camera || (artifact?.missing ? artifact?.metadata?.reason : "fixture evidence");
  caption.append(el("span", "", detail || ""));
  card.append(caption);
  return card;
}

function supervisorArtifactSource(artifact) {
  const value = artifact && (artifact.url || artifact.href);
  if (!value) return "";
  try {
    const url = new URL(String(value), location.origin);
    if (url.origin !== location.origin || !url.pathname.startsWith(`${API}/artifacts/`)) return "";
    url.searchParams.set("token", state.token);
    return url.toString();
  } catch (_) {
    return "";
  }
}

function renderSupervisorDecision(checkpoint) {
  const created = checkpoint.created || {};
  const verdict = checkpoint.verdict || {};
  const section = el("section", "supervisor-decision");
  const top = el("div", "decision-summary");
  const confidence = Number(verdict.confidence);
  top.append(
    el("div", "", verdict.rationale || checkpoint.unavailable?.error || "No verifier rationale recorded."),
    el("span", "confidence", Number.isFinite(confidence) ? `${Math.round(confidence * 100)}% confidence` : ""),
  );
  section.append(top);
  const tags = el("div", "decision-tags");
  [
    verdict.bt_assessment,
    verdict.subtask_status,
    verdict.world_change,
    verdict.failure_category,
    verdict.escalation,
  ].filter(Boolean).forEach(value => tags.append(el("span", "badge muted", String(value).replaceAll("_", " "))));
  section.append(tags);
  if (Array.isArray(verdict.evidence) && verdict.evidence.length) {
    const list = el("ul", "evidence-list");
    verdict.evidence.forEach(item => list.append(el("li", "", item)));
    section.append(list);
  }

  const interventions = checkpoint.recoveries.filter(item => item.type === "supervisor.recovery.proposed");
  interventions.forEach(recovery => {
    const block = el("div", "intervention-block local");
    const finished = checkpoint.recoveries.find(item =>
      item.type === "supervisor.recovery.finished" && item.strategy_id === recovery.strategy_id);
    block.append(el("strong", "", `Local recovery · ${humanType(recovery.kind)}`));
    block.append(el("span", `badge ${finished ? statusClass(finished.succeeded ? "success" : "failure") : "running"}`,
      finished ? (finished.succeeded ? "succeeded" : "failed") : "proposed"));
    block.append(el("p", "", recovery.rationale || `Strategy ${recovery.strategy_id || "recorded"}`));
    if (recovery.arguments) block.append(el("pre", "mini-code", safeJson(recovery.arguments)));
    section.append(block);
  });
  if (checkpoint.global) {
    const global = checkpoint.global;
    const block = el("div", "intervention-block global");
    block.append(el("strong", "", `Global decision · ${humanType(global.action)}`));
    block.append(el("span", "badge failure", "execution barrier"));
    block.append(el("p", "", global.rationale || "Global escalation requested."));
    if (global.operator_message) block.append(el("blockquote", "", global.operator_message));
    if (Array.isArray(global.replacement_plan) && global.replacement_plan.length) {
      const plan = el("ol", "replacement-plan");
      global.replacement_plan.forEach(step => plan.append(el("li", "", step.action || step.name || safeJson(step))));
      block.append(plan);
    }
    section.append(block);
  }
  const regression = renderSupervisorRegression(created);
  if (regression) section.append(regression);
  return section;
}

function renderSupervisorRegression(created) {
  const expected = created.expected_response;
  if (!expected || typeof expected !== "object") return null;
  const block = el("div", "regression-block");
  const heading = el("div", "regression-heading");
  heading.append(
    el("strong", "", "Three-run regression"),
    el("span", `badge ${created.live_passed ? "success" : "muted"}`, created.live_passed ? "passed" : "not passed"),
  );
  block.append(heading);
  const expectedPlanner = created.expected_planner;
  block.append(el(
    "p",
    "regression-expected",
    `Expected verifier · ${expected.verdict} / ${expected.bt_assessment} / ${expected.escalation}`
      + (expectedPlanner?.action ? ` → ${expectedPlanner.action}` : ""),
  ));
  const samples = [
    ...(Array.isArray(created.observed_responses) ? created.observed_responses : []),
    ...(Array.isArray(created.observed_planners) ? created.observed_planners : []),
  ];
  if (!samples.length) {
    block.append(el("p", "regression-empty", "Live Luna samples have not been attached."));
    return block;
  }
  const rows = el("div", "regression-rows");
  samples.forEach(sample => {
    const observed = sample.observed || {};
    const result = observed.verdict || observed.kind || observed.action || "error";
    const row = el("div", "regression-row");
    row.append(
      el("span", "", `Run ${sample.repetition || "?"}`),
      el("span", "", `${humanType(sample.role || "query")} · ${humanType(result)}`),
      el("span", `badge ${sample.passed ? "success" : "failure"}`, sample.passed ? "pass" : "fail"),
    );
    rows.append(row);
  });
  block.append(rows);
  return block;
}

function supervisorFlowStep(number, label, value, className) {
  const step = el("div", `supervisor-flow-step ${className || ""}`);
  step.append(el("span", "flow-number", number));
  const copy = el("div");
  copy.append(el("small", "", label), el("strong", "", String(value || "unknown").replaceAll("_", " ")));
  step.append(copy);
  return step;
}

function supervisorContextItem(label, value) {
  const item = el("div", "context-item");
  item.append(el("span", "", label), el("strong", "", value));
  return item;
}

function supervisorRoute(checkpoint) {
  if (checkpoint.unavailable) return { label: "stop / fallback", className: "failure" };
  if (checkpoint.global) {
    const action = checkpoint.global.action || "global replan";
    return { label: String(action).replaceAll("_", " "), className: "failure" };
  }
  const finished = checkpoint.recoveries.filter(item => item.type === "supervisor.recovery.finished").at(-1);
  if (finished) {
    return finished.succeeded
      ? { label: "recovered", className: "success" }
      : { label: "recovery failed", className: "failure" };
  }
  if (checkpoint.recoveries.some(item => item.type === "supervisor.recovery.proposed")) {
    return { label: "local recovery", className: "running" };
  }
  if (checkpoint.verdict?.verdict === "all_clear") return { label: "continue", className: "success" };
  return { label: checkpoint.verdict?.escalation || "pending", className: "running" };
}

function supervisorDecisionClass(value) {
  if (value === "all_clear") return "success";
  if (value === "recoverable" || value === "uncertain") return "running";
  if (value === "unrecoverable") return "failure";
  return "";
}

function nodeLabel(node) {
  if (!node || typeof node !== "object") return "End of subtask";
  return node.name || node.class_name || node.node_id || safeJson(node);
}

function treeSummary(tree) {
  if (!tree || typeof tree !== "object") return "Not recorded";
  const nodes = Array.isArray(tree.nodes) ? tree.nodes.length : Object.keys(tree.nodes || {}).length;
  const edges = Array.isArray(tree.edges) ? tree.edges.length : Object.keys(tree.edges || {}).length;
  return `${nodes} nodes · ${edges} edges`;
}

function renderPlanning() {
  const snapshot = state.snapshot || {};
  const attempts = Object.values(snapshot.planning_attempts || {}).sort((a, b) => (a.updated_sequence || 0) - (b.updated_sequence || 0));
  $("attempt-count").textContent = `${attempts.length} attempt${attempts.length === 1 ? "" : "s"}`;
  const root = $("planning-attempts");
  root.replaceChildren();
  attempts.forEach((attempt, index) => root.append(renderAttempt(attempt, index)));
  if (!attempts.length) root.append(emptyBlock("No model call recorded", "Offline mock plans bypass the provider; live runs show requests and responses here.", "◌"));

  const revisions = snapshot.plans?.revisions || {};
  const select = $("plan-revision");
  select.replaceChildren();
  Object.keys(revisions).forEach(id => {
    const option = el("option", "", `Revision ${id}${String(snapshot.plans.active_revision) === id ? " · active" : ""}`);
    option.value = id;
    option.selected = id === String(state.planRevision);
    select.append(option);
  });
  renderPlan();
}

function renderAttempt(attempt, index) {
  const details = el("details", "attempt");
  if (index === 0) details.open = true;
  const summary = el("summary");
  summary.append(el("span", "attempt-index", String(index + 1).padStart(2, "0")));
  const title = el("span");
  title.append(el("span", "attempt-title", attempt.model || attempt.attempt_id || `Attempt ${index + 1}`));
  const outcome = attempt.error ? "Provider error" : attempt.accepted === false ? "Rejected" : attempt.last_type || "Recorded";
  title.append(el("span", "attempt-meta", `${outcome}${attempt.duration_ms ? ` · ${attempt.duration_ms} ms` : ""}`));
  const tokenCount = attempt.usage?.total_tokens;
  const badgeText = tokenCount !== undefined
    ? `${tokenCount} tok`
    : attempt.error
      ? "error"
      : attempt.accepted === false
        ? "rejected"
        : "recorded";
  summary.append(title, el("span", `badge ${attempt.error ? "failure" : ""}`, badgeText));
  details.append(summary);
  const body = el("div", "attempt-body");
  const messages = Array.isArray(attempt.messages) ? attempt.messages : [];
  messages.forEach(message => {
    const item = el("div", `message ${message.role || ""}`);
    item.append(el("div", "message-role", message.role || "message"));
    item.append(el("div", "message-content", message.content || safeJson(message)));
    body.append(item);
  });
  const response = attempt.raw_response || attempt.raw_content || attempt.response || attempt.content || attempt.reasoning;
  if (response) {
    const item = el("div", "message assistant");
    item.append(el("div", "message-role", "Response"));
    item.append(el("div", "message-content", typeof response === "string" ? response : safeJson(response)));
    body.append(item);
  }
  if (attempt.error || attempt.validation_error) {
    const item = el("div", "message");
    item.append(el("div", "message-role", "Failure"));
    item.append(el("div", "message-content", attempt.error || attempt.validation_error));
    body.append(item);
  }
  if (!body.children.length) body.append(el("div", "quiet-label", safeJson(attempt)));
  details.append(body);
  return details;
}

function renderPlan() {
  const root = $("plan-view");
  root.replaceChildren();
  const revisions = state.snapshot?.plans?.revisions || {};
  const revision = revisions[String(state.planRevision)] || revisions[state.snapshot?.plans?.active_revision] || {};
  const steps = Array.isArray(revision.steps) ? revision.steps : [];
  const tasks = Object.values(state.snapshot?.tasks || {});
  const taskSteps = Object.values(tasks[0]?.steps || {});
  steps.forEach((step, index) => {
    const observed = taskSteps.find(item => item.step_index === index || item.step_id === step.step_id) || {};
    const status = step.status || observed.status || observed.outcome || "not visited";
    const item = el("div", `step ${statusClass(status)}`);
    item.append(el("span", "step-index", String(index + 1).padStart(2, "0")));
    const body = el("div");
    body.append(el("div", "step-action", step.action || step.name || "Unnamed action"));
    body.append(el("div", "step-params", safeJson(step.params || step.parameters || {})));
    item.append(body, el("span", `badge ${statusClass(status)}`, status));
    root.append(item);
  });
  if (!steps.length) root.append(emptyBlock("No committed plan", "A committed plan revision will appear here.", "⇢"));
}

function treeRevisions(kind = state.treeKind) {
  const revisions = Object.entries(state.snapshot?.trees?.revisions || {});
  return revisions.filter(([id, tree]) => {
    const inferred = String(tree.kind || "").toLowerCase() === "planned" || id.startsWith("planned-") ? "planned" : "executor";
    return inferred === kind;
  });
}

function renderTreeControls() {
  $("planned-toggle").classList.toggle("active", state.treeKind === "planned");
  $("executor-toggle").classList.toggle("active", state.treeKind === "executor");
  document.querySelectorAll(".tree-mode-button").forEach(control => {
    control.classList.toggle("active", control.dataset.treeView === state.treeViewMode);
  });
  const select = $("tree-revision-select");
  select.replaceChildren();
  const revisions = treeRevisions();
  revisions.forEach(([id, tree]) => {
    const count = Array.isArray(tree.nodes) ? tree.nodes.length : Object.keys(tree.nodes || {}).length;
    const option = el("option", "", `${id} · ${count} nodes`);
    option.value = id;
    option.selected = id === String(state.treeRevision);
    select.append(option);
  });
  if (!revisions.some(([id]) => id === String(state.treeRevision))) state.treeRevision = revisions.at(-1)?.[0] || null;
  renderTreeRuntimeControls();
}

function selectedTreeDocument() {
  return state.snapshot?.trees?.revisions?.[String(state.treeRevision)] || null;
}

function activeTreeNode(graph) {
  const running = graph.nodes.filter(node => Model.statusCategory(node.status || node.state) === "running");
  if (running.length) {
    return running.sort((a, b) =>
      treeDepth(graph, b.id) - treeDepth(graph, a.id)
      || (b.updated_sequence || 0) - (a.updated_sequence || 0)
    )[0];
  }
  return graph.nodes.filter(node => Model.statusCategory(node.status || node.state) === "failure")
    .sort((a, b) => (b.updated_sequence || 0) - (a.updated_sequence || 0))[0] || null;
}

function normalizeTickRecord(raw) {
  const source = raw && typeof raw === "object" ? raw : {};
  const runtimeNodes = source.nodes && typeof source.nodes === "object" && !Array.isArray(source.nodes)
    ? source.nodes
    : {};
  const entries = Array.isArray(source.visited)
    ? source.visited
    : Array.isArray(source.visited_nodes)
      ? source.visited_nodes
      : Array.isArray(source.visited_node_ids)
        ? source.visited_node_ids
        : [];
  const visited = entries.map((entry, index) => {
    if (typeof entry === "string") {
      const detail = runtimeNodes[entry] || {};
      return { node_id: entry, order: Number(detail.visit_order ?? index), status: detail.status || detail.state || "" };
    }
    return {
      node_id: entry?.node_id || entry?.id || "",
      order: Number(entry?.order ?? index),
      status: entry?.status || entry?.state || "",
    };
  }).filter(entry => entry.node_id);
  return {
    recorded: Boolean(source.recorded || source.tick !== undefined || source.tick_id !== undefined || visited.length),
    tick: source.tick ?? source.tick_id ?? source.count ?? null,
    sequence: source.sequence ?? null,
    visited,
    visitedIds: new Set(visited.map(entry => entry.node_id)),
    statuses: Object.fromEntries(visited.map(entry => [entry.node_id, entry.status])),
  };
}

function selectedTreeRuntime(document = selectedTreeDocument()) {
  const raw = document?.runtime || document?.tick_runtime || {};
  const current = normalizeTickRecord(raw.current || raw.current_tick || raw.last_tick || raw);
  const previous = normalizeTickRecord(raw.previous || raw.previous_tick || {});
  return { recorded: current.recorded, current, previous };
}

function fallbackNodeSemantics(node) {
  const type = String(node?.type || node?.node_type || "").toLowerCase();
  const explicit = node?.semantics && typeof node.semantics === "object" ? node.semantics : {};
  let kind = String(explicit.kind || explicit.control_flow || "").toLowerCase();
  if (!kind) {
    if (type.includes("sequence")) kind = "sequence";
    else if (type.includes("selector")) kind = "selector";
    else if (type.includes("parallel")) kind = "parallel";
    else if (type.includes("retry")) kind = "retry";
    else if (type.includes("repeat")) kind = "repeat";
    else if (type.includes("timeout")) kind = "timeout";
    else if (/(issuccess|isfailure|isrunning|inverter)/.test(type)) kind = "status-map";
    else kind = "leaf";
  }
  const declaredClass = ["composite", "decorator", "leaf"].includes(node?.node_class) ? node.node_class : "";
  const nodeClass = declaredClass || (["sequence", "selector", "parallel"].includes(kind)
    ? "composite"
    : kind === "leaf" ? "leaf" : "decorator");
  const symbol = {
    sequence: "→",
    selector: "?",
    parallel: "⇉",
    retry: "↻",
    repeat: "⟳",
    "status-map": "⇄",
    timeout: "◷",
  }[kind] || "•";
  let rule = "Runs when its parent activates it.";
  if (kind === "sequence") rule = `Runs children in order${explicit.memory ? " and resumes the running child" : ""}.`;
  if (kind === "selector") rule = `Tries children until one succeeds${explicit.memory ? " and resumes the running child" : ""}.`;
  const limit = explicit.limit ?? explicit.counter?.limit;
  const progress = explicit.progress ?? node?.counters?.value;
  const successPolicy = explicit.success_policy || explicit.parallel_policy || explicit.policy;
  if (kind === "parallel") rule = `Ticks children together; succeeds on ${successPolicy || "its configured policy"}.`;
  if (kind === "retry") rule = `Retries after failure${limit !== undefined ? ` up to ${limit} failures` : ""}.`;
  if (kind === "repeat") rule = `Repeats after success${limit !== undefined ? ` until ${limit} successes` : ""}.`;
  return { ...explicit, kind, nodeClass, symbol, rule, limit, progress, success_policy: successPolicy };
}

function nodeSemantics(node) {
  const fallback = fallbackNodeSemantics(node);
  if (typeof Model.nodeSemantics !== "function") return fallback;
  const inferred = Model.nodeSemantics(node) || {};
  const inferredKind = inferred.kind || (["sequence", "selector", "parallel"].includes(inferred.node_class) ? inferred.node_class : "");
  return {
    ...fallback,
    ...inferred,
    kind: inferredKind || fallback.kind,
    nodeClass: inferred.nodeClass && ["composite", "decorator", "leaf"].includes(inferred.nodeClass)
      ? inferred.nodeClass
      : fallback.nodeClass,
    symbol: inferred.symbol || fallback.symbol,
    rule: inferred.rule || fallback.rule,
    memory: inferred.memory ?? fallback.memory,
    success_policy: inferred.success_policy ?? fallback.success_policy,
    successPolicy: inferred.successPolicy ?? fallback.success_policy,
    synchronise: inferred.synchronise ?? fallback.synchronise,
    selected_child_ids: inferred.selected_child_ids?.length
      ? inferred.selected_child_ids
      : (fallback.selected_child_ids || []),
    selectedChildIds: inferred.selectedChildIds?.length
      ? inferred.selectedChildIds
      : (fallback.selected_child_ids || []),
    counters: inferred.counters ?? fallback.counters ?? fallback.counter ?? null,
    limit: inferred.counters?.limit ?? fallback.limit,
    progress: inferred.counters?.value ?? fallback.progress,
  };
}

function fallbackActivation(graph, nodeId, runtime) {
  if (!runtime.recorded) return { state: "not-recorded", reason: "This trace does not contain per-tick visitation." };
  const current = runtime.current;
  const previous = runtime.previous;
  if (current.visitedIds.has(nodeId)) {
    const previousStatus = String(previous.statuses[nodeId] || "").toUpperCase();
    const resumed = previous.visitedIds.has(nodeId) && previousStatus === "RUNNING";
    return {
      state: resumed ? "resumed" : "ticked",
      reason: resumed ? "Visited again after running on the previous tick." : "Visited by py_trees on the selected tick.",
    };
  }
  const node = graph.byId[nodeId];
  const parentId = node?.parentId;
  if (!parentId) return { state: "blocked", reason: "The root was not visited on the selected tick." };
  if (!current.visitedIds.has(parentId)) {
    return { state: "blocked", reason: `Its parent, ${graph.byId[parentId]?.name || parentId}, was not visited.` };
  }
  const parent = graph.byId[parentId];
  const siblings = graph.childrenById[parentId] || [];
  const index = siblings.indexOf(nodeId);
  const semantics = nodeSemantics(parent);
  const prior = siblings.slice(0, Math.max(0, index));
  if (semantics.kind === "sequence") {
    const gate = prior.find(id => {
      const status = String(current.statuses[id] || graph.byId[id]?.status || "").toUpperCase();
      return status !== "SUCCESS";
    });
    return { state: "skipped", reason: gate ? `Waiting for ${graph.byId[gate]?.name || gate} to succeed.` : "The sequence did not advance to this child." };
  }
  if (semantics.kind === "selector") {
    const gate = prior.find(id => String(current.statuses[id] || graph.byId[id]?.status || "").toUpperCase() !== "FAILURE");
    return { state: "skipped", reason: gate ? `${graph.byId[gate]?.name || gate} prevented selector fallback.` : "The selector did not reach this child." };
  }
  if (semantics.kind === "parallel" && semantics.synchronise) {
    return { state: "skipped", reason: "Already successful and parallel synchronization is enabled." };
  }
  return { state: "skipped", reason: "Its parent was ticked but did not select this child." };
}

function nodeActivation(graph, nodeId, runtime) {
  if (typeof Model.activationState === "function") {
    const result = Model.activationState(graph, nodeId, runtime);
    if (result && typeof result === "object") return result;
    if (typeof result === "string") {
      const reason = typeof Model.activationReason === "function" ? Model.activationReason(graph, nodeId, runtime) : "";
      return { state: result, reason };
    }
  }
  return fallbackActivation(graph, nodeId, runtime);
}

function edgeGate(parent, childId, graph) {
  if (typeof Model.edgeActivationLabel === "function") return Model.edgeActivationLabel(parent, childId, graph);
  const semantics = nodeSemantics(parent);
  const children = graph.childrenById[parent.id] || [];
  const index = children.indexOf(childId);
  if (semantics.kind === "sequence") return index === 0 ? "start" : "after previous succeeds";
  if (semantics.kind === "selector") return index === 0 ? "start" : "if previous fails";
  if (semantics.kind === "parallel") {
    const selected = new Set(semantics.selected_child_ids || []);
    return selected.has(childId) ? "tick together · counts for success" : "tick together";
  }
  if (semantics.nodeClass === "decorator") return "decorates";
  return children.length > 1 ? `child ${index + 1}` : "";
}

function treeDepth(graph, id) {
  return typeof Model.ancestorIds === "function" ? Model.ancestorIds(graph, id).length : 0;
}

function treeVisibleIds(graph, runtime) {
  if (state.treeViewMode === "full") return graph.nodes.map(node => node.id);
  const query = state.treeSearch.trim().toLowerCase();
  const match = query && graph.nodes.find(node => `${node.name || ""} ${node.id} ${node.type || ""}`.toLowerCase().includes(query));
  const focus = match || (state.selectedNodeId && graph.byId[state.selectedNodeId] ? graph.byId[state.selectedNodeId] : null);
  const visible = new Set();
  const addPath = id => {
    if (!id || !graph.byId[id]) return;
    Model.pathToNode(graph, id).forEach(pathId => visible.add(pathId));
  };
  graph.rootIds.forEach(id => visible.add(id));
  if (state.treeViewMode === "runtime") {
    runtime.current.visited.forEach(entry => {
      visible.add(entry.node_id);
      addPath(entry.node_id);
      (graph.childrenById[entry.node_id] || []).forEach(id => visible.add(id));
    });
    if (!runtime.current.visited.length) addPath(activeTreeNode(graph)?.id);
  } else {
    graph.nodes.filter(node => treeDepth(graph, node.id) <= 2).forEach(node => visible.add(node.id));
    addPath(activeTreeNode(graph)?.id);
    graph.nodes.filter(node => node.action_context?.boundary === true || node.context?.role === "action_boundary")
      .forEach(node => visible.add(node.id));
  }
  addPath(focus?.id);
  for (const id of state.expandedTreeBranches) {
    if (!graph.byId[id]) continue;
    visible.add(id);
    (graph.childrenById[id] || []).forEach(childId => visible.add(childId));
  }
  return Array.from(visible);
}

function collapsedBranchSummary(graph, rootIds) {
  const roots = Array.isArray(rootIds) ? rootIds : [rootIds];
  const ids = Array.from(new Set(roots.flatMap(rootId => Model.subtreeIds(graph, rootId))));
  const counts = { composite: 0, decorator: 0, leaf: 0 };
  ids.forEach(id => {
    const classification = nodeSemantics(graph.byId[id]).nodeClass;
    counts[classification] = (counts[classification] || 0) + 1;
  });
  return { total: ids.length, counts };
}

function activationDisplay(value) {
  return {
    ticked: "TICKED",
    resumed: "RESUMED",
    skipped: "SKIPPED",
    blocked: "BLOCKED",
    "not-recorded": "NOT RECORDED",
  }[value] || String(value || "NOT RECORDED").toUpperCase();
}

function treeNodeLabel(node, semantics, activation) {
  const kind = semantics.kind === "leaf" ? (node.type || node.node_type || "Leaf") : semantics.kind;
  let modifier = "";
  if (semantics.kind === "parallel" && semantics.success_policy) modifier = ` · ${humanType(semantics.success_policy)}`;
  else if (["sequence", "selector"].includes(semantics.kind) && semantics.memory) modifier = " · memory";
  else if (["retry", "repeat"].includes(semantics.kind) && semantics.limit !== undefined) modifier = ` · ${semantics.limit}`;
  return `${semantics.symbol || "•"} ${truncate(`${humanType(kind)}${modifier}`, 28)}\n${truncate(node.name || node.id, 34)}\n${activationDisplay(activation.state)}`;
}

function renderTreeGraph() {
  const container = $("tree-canvas");
  const priorWidth = state.treeVisual?.viewportWidth;
  const priorHeight = state.treeVisual?.viewportHeight;
  const dimensionsChanged = Number.isFinite(priorWidth) && Number.isFinite(priorHeight)
    && (Math.abs(container.clientWidth - priorWidth) > 24 || Math.abs(container.clientHeight - priorHeight) > 24);
  const previousViewport = state.treeGraph && !dimensionsChanged
    ? { zoom: state.treeGraph.zoom(), pan: state.treeGraph.pan() }
    : null;
  if (state.treeGraph) {
    state.treeGraph.destroy();
    state.treeGraph = null;
  }
  container.replaceChildren();
  const document = selectedTreeDocument();
  if (!document) {
    container.append(emptyBlock("No tree in this mode", "Choose another tree kind or trajectory.", "◇"));
    renderNodeDetail(null);
    return;
  }
  const graph = Model.normalizeTree(document);
  const runtime = selectedTreeRuntime(document);
  state.treeModel = graph;
  const visible = new Set(treeVisibleIds(graph, runtime));
  const elements = [];
  graph.nodes.filter(node => visible.has(node.id)).forEach(node => {
    const semantics = nodeSemantics(node);
    const activation = nodeActivation(graph, node.id, runtime);
    elements.push({
      group: "nodes",
      data: {
        id: node.id,
        label: treeNodeLabel(node, semantics, activation),
        status: Model.statusCategory(node.status || node.state),
        type: node.type || node.node_type || "",
        nodeClass: semantics.nodeClass,
        semanticKind: semantics.kind,
        activation: activation.state,
        activationReason: activation.reason,
        collapsed: false,
      },
    });
  });
  graph.nodes.forEach(node => {
    if (!visible.has(node.id)) return;
    const hiddenChildren = [];
    (graph.childrenById[node.id] || []).forEach((child, index) => {
      if (visible.has(child)) {
        elements.push({
          group: "edges",
          data: {
            id: `${node.id}->${child}`,
            source: node.id,
            target: child,
            order: index + 1,
            gate: `${index + 1} · ${edgeGate(node, child, graph)}`,
          },
        });
      } else {
        hiddenChildren.push({ child, index });
      }
    });
    if (hiddenChildren.length) {
      const branchRoots = hiddenChildren.map(item => item.child);
      const summary = collapsedBranchSummary(graph, branchRoots);
      const id = `__collapsed__:${node.id}`;
      elements.push({
        group: "nodes",
        data: {
          id,
          branchRoots: JSON.stringify(branchRoots),
          label: `▸ ${summary.total} hidden nodes\n${hiddenChildren.length} branches · ${summary.counts.decorator} decorators`,
          status: "unknown",
          nodeClass: "collapsed",
          semanticKind: "collapsed",
          activation: hiddenChildren.some(({ child }) => runtime.current.visitedIds.has(child)) ? "ticked" : "blocked",
          collapsed: true,
        },
      });
      elements.push({
        group: "edges",
        data: {
          id: `${node.id}->${id}`,
          source: node.id,
          target: id,
          order: hiddenChildren[0].index + 1,
          gate: `${hiddenChildren.length} collapsed branches`,
        },
      });
    }
  });
  if (!window.cytoscape) {
    container.append(emptyBlock("Graph renderer unavailable", "The locally vendored Cytoscape asset did not load.", "!"));
    return;
  }
  state.treeGraph = window.cytoscape({
    container,
    elements,
    wheelSensitivity: .18,
    minZoom: .08,
    maxZoom: 2.5,
    selectionType: "single",
    boxSelectionEnabled: false,
    style: treeGraphStyle(),
    layout: {
      name: "breadthfirst",
      directed: true,
      direction: "rightward",
      spacingFactor: 1.25,
      padding: 48,
      roots: graph.rootIds.filter(id => visible.has(id)).map(id => `#${cssEscape(id)}`).join(", "),
      nodeDimensionsIncludeLabels: true,
      fit: false,
      animate: false,
      transform: (_node, position) => ({ x: position.x, y: -position.y }),
    },
  });
  state.treeGraph.on("tap", "node", event => {
    if (event.target.data("collapsed")) {
      JSON.parse(event.target.data("branchRoots") || "[]").forEach(id => state.expandedTreeBranches.add(id));
      renderTreeGraph();
      return;
    }
    state.selectedNodeId = event.target.id();
    renderNodeDetail(graph.byId[state.selectedNodeId]);
    highlightTreeSelection(state.selectedNodeId);
    updateUrl();
  });
  const autoSelected = !state.selectedNodeId;
  if (autoSelected) {
    const initial = activeTreeNode(graph);
    if (initial && visible.has(initial.id)) state.selectedNodeId = initial.id;
  }
  if (state.selectedNodeId && state.treeGraph.getElementById(state.selectedNodeId).length) {
    const node = state.treeGraph.getElementById(state.selectedNodeId);
    node.select();
    if (!autoSelected && !previousViewport) {
      state.treeGraph.animate({ center: { eles: node }, zoom: Math.max(state.treeGraph.zoom(), .7) }, { duration: 180 });
    }
    renderNodeDetail(graph.byId[state.selectedNodeId]);
    highlightTreeSelection(state.selectedNodeId);
  } else {
    renderNodeDetail(null);
  }
  if (previousViewport) {
    state.treeGraph.zoom(previousViewport.zoom);
    state.treeGraph.pan(previousViewport.pan);
  } else {
    const initialiseViewport = () => {
      if (!state.treeGraph || state.treeGraph.destroyed()) return;
      const rootNode = state.treeGraph.getElementById(graph.rootIds[0]);
      if (!rootNode.length) return;
      const zoom = .82;
      const position = rootNode.position();
      state.treeGraph.zoom(zoom);
      state.treeGraph.pan({
        x: 135 - position.x * zoom,
        y: container.clientHeight / 2 - position.y * zoom,
      });
    };
    initialiseViewport();
    requestAnimationFrame(initialiseViewport);
  }
  state.treeVisual = {
    runtime,
    visibleIds: visible,
    signature: Array.from(visible).sort().join("\u0000"),
    viewportWidth: container.clientWidth,
    viewportHeight: container.clientHeight,
  };
  renderTreeRuntimeControls();
}

function updateTreeGraphRuntime() {
  if (!state.treeGraph || !state.treeVisual) return false;
  const document = selectedTreeDocument();
  if (!document) return false;
  const graph = Model.normalizeTree(document);
  const runtime = selectedTreeRuntime(document);
  const visible = new Set(treeVisibleIds(graph, runtime));
  const signature = Array.from(visible).sort().join("\u0000");
  if (signature !== state.treeVisual.signature) return false;
  visible.forEach(id => {
    const node = graph.byId[id];
    const element = state.treeGraph.getElementById(id);
    if (!node || !element.length) return;
    const semantics = nodeSemantics(node);
    const activation = nodeActivation(graph, id, runtime);
    element.data({
      label: treeNodeLabel(node, semantics, activation),
      status: Model.statusCategory(node.status || node.state),
      activation: activation.state,
      activationReason: activation.reason,
    });
  });
  state.treeGraph.nodes("[collapsed]").forEach(element => {
    const branchRoots = JSON.parse(element.data("branchRoots") || "[]");
    element.data("activation", branchRoots.some(id => runtime.current.visitedIds.has(id)) ? "ticked" : "blocked");
  });
  state.treeModel = graph;
  state.treeVisual = {
    runtime,
    visibleIds: visible,
    signature,
    viewportWidth: state.treeVisual.viewportWidth,
    viewportHeight: state.treeVisual.viewportHeight,
  };
  if (state.selectedNodeId && graph.byId[state.selectedNodeId]) {
    renderNodeDetail(graph.byId[state.selectedNodeId]);
    highlightTreeSelection(state.selectedNodeId);
  }
  renderTreeRuntimeControls();
  return true;
}

function treeGraphStyle() {
  return [
    {
      selector: "node",
      style: {
        width: 188,
        height: 70,
        shape: "round-rectangle",
        "background-color": "#ffffff",
        "border-width": 2,
        "border-color": "#8795a8",
        label: "data(label)",
        color: "#263244",
        "font-size": 11,
        "font-family": "Inter, Segoe UI, sans-serif",
        "text-wrap": "wrap",
        "text-max-width": 168,
        "text-valign": "center",
        "text-halign": "center",
        "line-height": 1.35,
      },
    },
    { selector: "node[nodeClass = 'composite']", style: { "background-color": "#eaf0fb", shape: "round-rectangle" } },
    { selector: "node[semanticKind = 'parallel']", style: { "background-color": "#ddf3f4", shape: "round-rectangle" } },
    { selector: "node[nodeClass = 'decorator']", style: { "background-color": "#f0eafb", shape: "round-rectangle", "border-style": "dashed" } },
    { selector: "node[nodeClass = 'leaf']", style: { "background-color": "#ffffff" } },
    { selector: "node[nodeClass = 'collapsed']", style: { "background-color": "#edf1f6", "border-style": "dashed", shape: "round-rectangle", color: "#526173" } },
    { selector: "node[status = 'success']", style: { "border-color": "#16805c" } },
    { selector: "node[status = 'running']", style: { "border-color": "#a86516", "border-width": 4 } },
    { selector: "node[status = 'failure']", style: { "border-color": "#c23b50", "border-width": 3 } },
    { selector: "node[activation = 'ticked']", style: { "underlay-color": "#5266d8", "underlay-opacity": .13, "underlay-padding": 5 } },
    { selector: "node[activation = 'resumed']", style: { "underlay-color": "#7553b7", "underlay-opacity": .15, "underlay-padding": 6 } },
    { selector: "node[activation = 'skipped']", style: { opacity: .68 } },
    { selector: "node[activation = 'blocked']", style: { opacity: .38 } },
    { selector: "node:selected", style: { "overlay-opacity": 0, "border-color": "#5266d8", "border-width": 4, "background-color": "#e2e8ff", opacity: 1 } },
    { selector: "node.path-context", style: { opacity: 1, "underlay-color": "#5266d8", "underlay-opacity": .1, "underlay-padding": 4 } },
    {
      selector: "edge",
      style: {
        width: 2,
        "line-color": "#7c8a9e",
        "target-arrow-color": "#65758a",
        "target-arrow-shape": "triangle",
        "arrow-scale": .9,
        "curve-style": "taxi",
        "taxi-direction": "rightward",
        "taxi-turn": 24,
        label: "data(gate)",
        color: "#526173",
        "font-size": 9,
        "font-weight": 600,
        "text-background-color": "#f5f7fb",
        "text-background-opacity": .92,
        "text-background-padding": 4,
        "text-rotation": "none",
      },
    },
    { selector: "edge.path-context", style: { width: 4, "line-color": "#5266d8", "target-arrow-color": "#3548ad", "z-index": 10 } },
  ];
}

function highlightTreeSelection(nodeId) {
  if (!state.treeGraph || !state.treeModel) return;
  state.treeGraph.elements().removeClass("path-context");
  const path = Model.pathToNode(state.treeModel, nodeId);
  path.forEach(id => state.treeGraph.getElementById(id).addClass("path-context"));
  for (let index = 0; index < path.length - 1; index += 1) {
    state.treeGraph.getElementById(`${path[index]}->${path[index + 1]}`).addClass("path-context");
  }
}

function treeTickEvents() {
  return state.events.filter(event => {
    if (eventType(event) !== "tree.tick_observed") return false;
    const payload = eventPayload(event);
    const kind = String(payload.tree_kind || payload.kind || "executor");
    const revision = String(payload.tree_revision ?? payload.revision ?? "0");
    return kind === state.treeKind && (!state.treeRevision || revision === String(state.treeRevision));
  });
}

function renderTreeRuntimeControls() {
  const ticks = treeTickEvents();
  const runtime = selectedTreeRuntime();
  let index = ticks.length - 1;
  if (state.historicalAt !== null) {
    const candidate = ticks.findLastIndex(event => Number(event.sequence) <= Number(state.historicalAt));
    index = candidate;
  }
  const currentEvent = index >= 0 ? ticks[index] : null;
  const tick = eventPayload(currentEvent).tick ?? runtime.current.tick;
  $("tree-tick-label").textContent = tick === null || tick === undefined
    ? "No tick visitation recorded"
    : `${state.historicalAt === null ? "Live" : "Historical"} tick #${tick}`;
  $("tree-tick-prev").disabled = index <= 0;
  $("tree-tick-next").disabled = index < 0 || index >= ticks.length - 1;
  $("tree-tick-live").classList.toggle("hidden", state.historicalAt === null);
}

function navigateTreeTick(delta) {
  const ticks = treeTickEvents();
  if (!ticks.length) return;
  let index = state.historicalAt === null
    ? ticks.length - 1
    : ticks.findLastIndex(event => Number(event.sequence) <= Number(state.historicalAt));
  index = Math.min(ticks.length - 1, Math.max(0, index + delta));
  const target = ticks[index];
  if (target) loadHistorical(target.sequence);
}

function returnTreeToLive() {
  state.historicalAt = null;
  selectTrajectory(state.selectedId, { keepHistorical: true, reuseEvents: true });
}

function renderNodeDetail(node) {
  const root = $("node-detail");
  root.replaceChildren();
  if (!node) {
    root.className = "node-inspector";
    root.append(emptyBlock("Select a node", "Explain its activation rule, selected-tick state, and blackboard access.", "◇"));
    return;
  }
  root.className = "node-inspector";
  const semantics = nodeSemantics(node);
  const runtime = state.treeVisual?.runtime || selectedTreeRuntime();
  const activation = nodeActivation(state.treeModel, node.id, runtime);
  const heading = el("div", "node-heading");
  const badges = el("div", "node-badges");
  badges.append(el("span", `badge activation ${activation.state}`, activationDisplay(activation.state)));
  badges.append(el("span", `badge ${statusClass(node.status || node.state)}`, node.status || node.state || "INVALID"));
  heading.append(badges);
  heading.append(el("h3", "", node.name || node.type || node.id));
  heading.append(el("div", "node-path", node.id));
  const activationSection = inspectorSection("Activation on selected tick");
  activationSection.append(el("p", "activation-reason", activation.reason || "No activation reason recorded."));
  activationSection.append(el("p", "control-rule", semantics.rule || "Runs when its parent activates it."));
  const details = inspectorSection("Node");
  const list = el("dl", "definition-list");
  const semanticDetail = [
    semantics.memory !== undefined ? `memory: ${semantics.memory ? "enabled" : "disabled"}` : "",
    semantics.success_policy ? `success: ${semantics.success_policy}` : "",
    semantics.synchronise !== undefined ? `synchronise: ${semantics.synchronise ? "yes" : "no"}` : "",
    semantics.limit !== undefined ? `limit: ${semantics.limit}` : "",
    semantics.progress !== undefined ? `progress: ${semantics.progress}` : "",
  ].filter(Boolean).join(" · ") || "—";
  [["Type", node.type || node.node_type || "—"], ["Semantic kind", semantics.kind], ["Configuration", semanticDetail], ["Parent", node.parentId || "root"], ["Child position", node.parentId ? `${(state.treeModel.childrenById[node.parentId] || []).indexOf(node.id) + 1}` : "root"], ["Children", (node.children || []).length], ["Updated", node.updated_at ? formatTime(node.updated_at, true) : "—"]]
    .forEach(([key, value]) => list.append(el("dt", "", key), el("dd", "", value)));
  details.append(list);
  const access = node.blackboard_access || node.blackboard || {};
  const blackboard = inspectorSection("Blackboard access");
  const accessList = el("dl", "definition-list");
  const formatKeys = value => Array.isArray(value) && value.length ? value.join("\n") : "None";
  [["Read", formatKeys(access.read)], ["Write", formatKeys(access.write)], ["Exclusive", formatKeys(access.exclusive)]]
    .forEach(([key, value]) => accessList.append(el("dt", "", key), el("dd", "blackboard-keys", value)));
  blackboard.append(accessList);
  const feedback = inspectorSection("Feedback");
  feedback.append(el("pre", "code-block", node.feedback || node.message || "No feedback"));
  root.append(heading, activationSection, details, blackboard, feedback);
}

function renderAgents() {
  const agents = Object.values(state.snapshot?.agents || {});
  const root = $("agent-lanes");
  root.replaceChildren();
  agents.forEach(agent => {
    const card = el("article", "surface agent-card");
    const title = el("div", "agent-title");
    title.append(el("strong", "", agent.name || agent.agent_id || agent.id || "Agent"));
    title.append(el("span", `badge ${statusClass(agent.status)}`, agent.status || "unknown"));
    card.append(title);
    card.append(el("div", "agent-role", `${agent.role || "agent"}${agent.model ? ` · ${agent.model}` : ""}`));
    card.append(el("div", "agent-updated", agent.updated_at ? `Updated ${formatTime(agent.updated_at)}` : "No activity timestamp"));
    root.append(card);
  });
  if (!agents.length) root.append(emptyBlock("No background agents", "Agent lifecycle events will create lanes here.", "◎"));

  const proposals = state.snapshot?.proposals || [];
  $("proposal-count").textContent = String(proposals.length);
  const proposalRoot = $("proposal-list");
  proposalRoot.replaceChildren();
  proposals.forEach(proposal => proposalRoot.append(renderProposal(proposal)));
  if (!proposals.length) proposalRoot.append(emptyBlock("No adaptive proposals", "On-the-fly tree changes, votes, and decisions will appear here.", "◇"));
  if (state.view === "agents") queueGraphRender(renderAgentGraph);
}

function renderProposal(proposal) {
  const item = el("article", "proposal");
  const title = el("div", "proposal-title");
  title.append(el("span", "", proposal.summary || proposal.proposal_id || "Proposal"));
  title.append(el("span", "badge", humanType(proposal.last_type || proposal.type || "recorded")));
  item.append(title);
  item.append(el("div", "proposal-copy", `Proposer: ${proposal.proposer_agent_id || proposal.agent_id || "unknown"} · Base tree: ${proposal.base_tree_version ?? "—"} · Result: ${proposal.resulting_tree_version ?? "pending"}`));
  const votes = el("div", "vote-list");
  (proposal.votes || []).forEach(vote => votes.append(el("span", `badge ${String(vote.vote).includes("reject") ? "failure" : "success"}`, `${vote.agent_id || "agent"}: ${vote.vote || "voted"}`)));
  item.append(votes);
  return item;
}

function renderAgentGraph() {
  const container = $("agent-canvas");
  if (state.agentGraph) {
    state.agentGraph.destroy();
    state.agentGraph = null;
  }
  container.replaceChildren();
  const agents = Object.values(state.snapshot?.agents || {});
  const proposals = state.snapshot?.proposals || [];
  const elements = [];
  const agentIds = new Set();
  agents.forEach(agent => {
    const id = `agent:${agent.agent_id || agent.id || agent.name}`;
    agentIds.add(String(agent.agent_id || agent.id || agent.name));
    elements.push({ data: { id, label: truncate(agent.name || agent.agent_id || agent.id, 25), kind: "agent", status: Model.statusCategory(agent.status) } });
  });
  proposals.forEach(proposal => {
    const id = `proposal:${proposal.proposal_id || proposal.id}`;
    elements.push({ data: { id, label: truncate(proposal.summary || proposal.proposal_id || "Proposal", 28), kind: "proposal", status: "unknown" } });
    const proposer = proposal.proposer_agent_id || proposal.agent_id;
    if (proposer && agentIds.has(String(proposer))) elements.push({ data: { id: `${proposer}->${id}`, source: `agent:${proposer}`, target: id } });
    (proposal.votes || []).forEach((vote, index) => {
      if (vote.agent_id && agentIds.has(String(vote.agent_id))) elements.push({ data: { id: `${vote.agent_id}->${id}:vote${index}`, source: `agent:${vote.agent_id}`, target: id, vote: vote.vote || "" } });
    });
    if (proposal.resulting_tree_version !== undefined) {
      const revision = `tree:${proposal.resulting_tree_version}`;
      if (!elements.some(element => element.data.id === revision)) elements.push({ data: { id: revision, label: `Tree r${proposal.resulting_tree_version}`, kind: "tree" } });
      elements.push({ data: { id: `${id}->${revision}`, source: id, target: revision } });
    }
  });
  if (!elements.length || !window.cytoscape) {
    container.append(emptyBlock("No collaboration graph", "Agents and proposals will form a causal graph here.", "◎"));
    return;
  }
  state.agentGraph = window.cytoscape({
    container,
    elements,
    wheelSensitivity: .18,
    minZoom: .2,
    maxZoom: 2.5,
    style: [
      { selector: "node", style: { width: 120, height: 38, shape: "round-rectangle", label: "data(label)", color: "#344054", "font-size": 9, "background-color": "#ffffff", "border-width": 1, "border-color": "#aebaca", "text-wrap": "ellipsis", "text-max-width": 105 } },
      { selector: "node[kind = 'agent']", style: { "border-color": "#9479c9", "background-color": "#f2edfb" } },
      { selector: "node[kind = 'proposal']", style: { "border-color": "#7384dd", "background-color": "#e8edff" } },
      { selector: "node[kind = 'tree']", style: { "border-color": "#55a482", "background-color": "#e6f6ee" } },
      { selector: "edge", style: { width: 1, "curve-style": "bezier", "line-color": "#98a5b6", "target-arrow-color": "#738196", "target-arrow-shape": "triangle", "arrow-scale": .7 } },
    ],
    layout: { name: "breadthfirst", directed: true, direction: "rightward", spacingFactor: 1.3, padding: 35, animate: false },
  });
}

function renderState() {
  const history = state.snapshot?.state_history || [];
  $("state-count").textContent = `${history.length} change${history.length === 1 ? "" : "s"}`;
  const root = $("state-history");
  root.replaceChildren();
  history.forEach(item => {
    const row = el("article", `state-row ${state.selectedState?.event_id === item.event_id ? "selected" : ""}`);
    const head = el("div", "state-row-head");
    head.append(el("strong", "", humanType(item.type || item.event_type || "state")));
    head.append(el("span", "", `#${item.sequence ?? "—"}`));
    row.append(head);
    row.append(el("div", "state-row-copy", truncate(safeJson(eventPayload(item)), 150)));
    row.addEventListener("click", () => {
      state.selectedState = item;
      renderState();
    });
    root.append(row);
  });
  if (!history.length) root.append(emptyBlock("No state deltas", "Checkpoint, blackboard, and shared-memory changes appear here.", "{ }"));
  renderStateDetail();

  const lastSequence = Number(state.liveSnapshot?.trajectory?.last_sequence ?? state.snapshot?.trajectory?.last_sequence ?? state.snapshot?.sequence ?? 0);
  $("history-range").max = String(Math.max(0, lastSequence));
  $("history-range").value = String(state.historicalAt ?? lastSequence);
  $("history-sequence").textContent = state.historicalAt === null ? `Live · #${lastSequence}` : `Snapshot · #${state.historicalAt}`;
}

function renderStateDetail() {
  const root = $("state-detail");
  root.replaceChildren();
  if (!state.selectedState) {
    root.className = "inspector";
    root.append(emptyBlock("Select a state change", "Changed keys and values appear here.", "{ }"));
    return;
  }
  root.className = "inspector";
  const section = inspectorSection(humanType(state.selectedState.type));
  section.append(el("pre", "code-block", safeJson(eventPayload(state.selectedState))));
  root.append(section);
}

function renderHistoryMode() {
  const historical = state.historicalAt !== null;
  $("history-mode").classList.toggle("hidden", !historical);
  $("return-live").classList.toggle("hidden", !historical);
  $("control-mode").textContent = historical ? "Disabled in historical mode" : "Live revision required";
}

async function loadHistorical(sequence) {
  const liveLast = Number(state.liveSnapshot?.trajectory?.last_sequence ?? state.snapshot?.trajectory?.last_sequence ?? 0);
  state.historicalAt = Number(sequence) >= liveLast ? null : Math.max(0, Number(sequence));
  await selectTrajectory(state.selectedId, { keepHistorical: true, fromHistory: true, reuseEvents: true });
}

function renderControls() {
  const lease = state.lease || {};
  const ours = Boolean(lease.owner && lease.owner === state.controller);
  $("lease-state").className = `badge ${ours ? "success" : "warning"}`;
  $("lease-state").textContent = lease.owner ? (ours ? "Lease held" : "Controlled elsewhere") : "Lease required";
  $("lease-button").textContent = ours ? "Renew control" : "Acquire control";
  const details = $("lease-detail");
  details.replaceChildren();
  [["Owner", lease.owner || "None"], ["Expires", lease.owner ? `${Math.ceil(lease.expires_in_s || 0)} seconds` : "—"], ["Tree revision", state.snapshot?.trees?.active_revision ?? "—"]]
    .forEach(([key, value]) => details.append(el("dt", "", key), el("dd", "", value)));
  const disabled = state.historicalAt !== null || !state.selectedId;
  $("send-command").disabled = disabled;
  document.querySelectorAll(".control").forEach(control => { control.disabled = disabled; });
  validatePayload();
}

async function acquireLease() {
  try {
    const path = state.lease?.owner === state.controller ? "/control/lease/renew" : "/control/lease";
    const result = await request(path, { method: "POST", headers: { "x-gpsr-controller": state.controller || "" } });
    state.controller = result.controller_id;
    state.lease = result.lease;
    renderControls();
  } catch (error) {
    showCommandResult(error.message, true);
  }
}

function validatePayload() {
  const root = $("command-validation");
  try {
    const value = JSON.parse($("command-payload").value || "{}");
    if (!value || typeof value !== "object" || Array.isArray(value)) throw new Error("Payload must be a JSON object");
    root.textContent = "";
    return value;
  } catch (error) {
    root.textContent = error.message;
    return null;
  }
}

function reviewCommand() {
  if (state.historicalAt !== null) return showCommandResult("Return to live before sending a command.", true);
  const payload = validatePayload();
  if (payload === null) return;
  const command = $("command-select").value;
  const justification = $("justification").value.trim();
  state.pendingCommand = { command, payload, justification };
  $("confirm-title").textContent = `${humanType(command)}?`;
  const destructive = ["cancel", "rollback", "apply_patch", "edit_state", "skip_step"].includes(command);
  $("confirm-copy").textContent = destructive
    ? "This command can alter or stop the running mission. It will be sent with the active tree revision and recorded in the audit trail."
    : "The typed command will be sent to the GPSR debug gateway and recorded in the audit trail.";
  $("confirm-send").className = `button ${destructive ? "danger" : "primary"}`;
  $("confirm-dialog").showModal();
}

async function submitPendingCommand() {
  const pending = state.pendingCommand;
  state.pendingCommand = null;
  if (!pending) return;
  try {
    const result = await request(`/trajectories/${encodeURIComponent(state.selectedId)}/commands`, {
      method: "POST",
      headers: { "x-gpsr-controller": state.controller, "Content-Type": "application/json" },
      body: JSON.stringify({
        command: pending.command,
        expected_revision: state.snapshot?.trees?.active_revision,
        payload: pending.payload,
        justification: pending.justification,
      }),
    });
    showCommandResult(safeJson(result), false);
  } catch (error) {
    showCommandResult(error.message, true);
  }
}

function showCommandResult(message, error) {
  const root = $("command-result");
  root.className = `result ${error ? "error" : ""}`;
  root.textContent = String(message);
}

async function renameTrajectory() {
  if (!state.selectedId) return;
  const current = state.snapshot?.name || "";
  const name = window.prompt("Trajectory name (blank clears it)", current);
  if (name === null) return;
  try {
    await request(`/trajectories/${encodeURIComponent(state.selectedId)}`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
    await refreshTrajectories(state.selectedId);
  } catch (error) {
    window.alert(error.message);
  }
}

async function togglePin() {
  if (!state.selectedId) return;
  try {
    await request(`/trajectories/${encodeURIComponent(state.selectedId)}`, {
      method: "PATCH",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ pinned: !state.snapshot?.pinned }),
    });
    await refreshTrajectories(state.selectedId);
  } catch (error) {
    window.alert(error.message);
  }
}

function activateView(view, update = true) {
  state.view = VIEW_IDS.has(view) ? view : "overview";
  document.querySelectorAll(".tab").forEach(tab => {
    const active = tab.dataset.view === state.view;
    tab.classList.toggle("active", active);
    tab.setAttribute("aria-selected", String(active));
  });
  document.querySelectorAll(".view").forEach(panel => panel.classList.toggle("active", panel.id === `view-${state.view}`));
  if (state.view === "tree") queueGraphRender(renderTreeGraph);
  if (state.view === "agents") queueGraphRender(renderAgentGraph);
  if (update) updateUrl();
}

function setTreeKind(kind) {
  state.treeKind = kind;
  state.treeRevision = treeRevisions(kind).at(-1)?.[0] || null;
  state.expandedTreeBranches.clear();
  state.selectedNodeId = null;
  renderTreeControls();
  renderTreeGraph();
  updateUrl();
}

function queueGraphRender(callback) {
  requestAnimationFrame(() => requestAnimationFrame(callback));
}

function queueSelectedRender() {
  if (state.renderQueued) return;
  state.renderQueued = true;
  requestAnimationFrame(() => {
    state.renderQueued = false;
    renderTimeline();
    renderMission();
    renderPhases();
    renderSupervisor();
  });
}

function scheduleLiveSnapshotRefresh() {
  if (state.liveRefreshTimer || state.historicalAt !== null || !state.selectedId) return;
  state.liveRefreshTimer = setTimeout(async () => {
    state.liveRefreshTimer = null;
    try {
      const snapshot = await request(`/trajectories/${encodeURIComponent(state.selectedId)}`);
      if (state.historicalAt !== null) return;
      state.snapshot = snapshot;
      state.liveSnapshot = snapshot;
      initialiseSelections();
      if (state.view === "tree" && state.treeGraph) {
        renderMission();
        renderTreeControls();
        renderHistoryMode();
        if (!updateTreeGraphRuntime()) renderTreeGraph();
      } else {
        renderAll();
      }
    } catch (_) {}
  }, 250);
}

function scheduleTrajectoryRefresh() {
  if (state.trajectoryRefreshTimer) return;
  state.trajectoryRefreshTimer = setTimeout(async () => {
    state.trajectoryRefreshTimer = null;
    try {
      const result = await request("/trajectories?limit=500");
      state.trajectories = result.trajectories || [];
      renderTrajectories();
    } catch (_) {}
  }, 1000);
}

function connect() {
  if (state.socket) state.socket.close();
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${scheme}://${location.host}${API}/stream?token=${encodeURIComponent(state.token)}`);
  state.socket = socket;
  socket.onopen = () => setConnection(true, "Live");
  socket.onerror = () => setConnection(false, "Stream error");
  socket.onclose = () => {
    setConnection(false, "Reconnecting");
    clearTimeout(state.reconnectTimer);
    state.reconnectTimer = setTimeout(connect, 1800);
  };
  socket.onmessage = message => {
    try {
      const envelope = JSON.parse(message.data);
      if (envelope.type === "heartbeat") {
        state.lease = envelope.lease;
        renderControls();
        return;
      }
      if (envelope.type !== "gpsr.event" || !envelope.event) return;
      const event = envelope.event;
      const incomingId = trajectoryId(event) || event.trace_id;
      scheduleTrajectoryRefresh();
      if (incomingId !== state.selectedId) return;
      if (!state.events.some(item => item.event_id && item.event_id === event.event_id)) {
        state.events.push(event);
        state.events.sort((a, b) => Number(a.sequence || 0) - Number(b.sequence || 0));
      }
      queueSelectedRender();
      scheduleLiveSnapshotRefresh();
    } catch (_) {}
  };
}

function bindInteractions() {
  document.querySelectorAll(".tab").forEach(tab => tab.addEventListener("click", () => activateView(tab.dataset.view)));
  document.querySelectorAll(".filter-chip").forEach(chip => chip.addEventListener("click", () => {
    state.runStatus = chip.dataset.runStatus;
    document.querySelectorAll(".filter-chip").forEach(item => item.classList.toggle("active", item === chip));
    renderTrajectories();
  }));
  $("trajectory-filter").addEventListener("input", renderTrajectories);
  $("refresh").addEventListener("click", () => refreshTrajectories().catch(error => renderFatal(error)));
  $("sidebar-toggle").addEventListener("click", () => document.querySelector(".app-shell").classList.toggle("sidebar-collapsed"));
  $("event-search").addEventListener("input", event => {
    state.eventSearch = event.target.value;
    state.eventLimit = 250;
    renderTimeline();
    updateUrl();
  });
  $("event-category").addEventListener("change", event => {
    state.eventCategory = event.target.value;
    state.eventLimit = 250;
    renderTimeline();
    updateUrl();
  });
  $("raw-events").addEventListener("change", event => {
    state.rawEvents = event.target.checked;
    renderTimeline();
    updateUrl();
  });
  $("event-more").addEventListener("click", () => {
    state.eventLimit += 250;
    renderTimeline();
  });
  $("plan-revision").addEventListener("change", event => {
    state.planRevision = event.target.value;
    renderPlan();
  });
  $("planned-toggle").addEventListener("click", () => setTreeKind("planned"));
  $("executor-toggle").addEventListener("click", () => setTreeKind("executor"));
  $("tree-revision-select").addEventListener("change", event => {
    state.treeRevision = event.target.value;
    state.expandedTreeBranches.clear();
    state.selectedNodeId = null;
    renderTreeGraph();
    updateUrl();
  });
  $("tree-search").addEventListener("input", event => {
    state.treeSearch = event.target.value;
    clearTimeout(state.treeSearchTimer);
    state.treeSearchTimer = setTimeout(() => {
      renderTreeGraph();
      updateUrl();
    }, 140);
  });
  $("tree-fit").addEventListener("click", () => state.treeGraph?.fit(undefined, 35));
  document.querySelectorAll(".tree-mode-button").forEach(control => control.addEventListener("click", () => {
    state.treeViewMode = control.dataset.treeView;
    state.expandedTreeBranches.clear();
    document.querySelectorAll(".tree-mode-button").forEach(item => item.classList.toggle("active", item === control));
    renderTreeGraph();
  }));
  $("tree-tick-prev").addEventListener("click", () => navigateTreeTick(-1));
  $("tree-tick-next").addEventListener("click", () => navigateTreeTick(1));
  $("tree-tick-live").addEventListener("click", returnTreeToLive);
  $("agent-fit").addEventListener("click", () => state.agentGraph?.fit(undefined, 30));
  $("history-range").addEventListener("input", event => {
    $("history-sequence").textContent = `Sequence · #${event.target.value}`;
    clearTimeout(state.historyTimer);
    state.historyTimer = setTimeout(() => loadHistorical(event.target.value), 180);
  });
  $("return-live").addEventListener("click", () => {
    state.historicalAt = null;
    selectTrajectory(state.selectedId, { keepHistorical: true, reuseEvents: true });
  });
  $("lease-button").addEventListener("click", acquireLease);
  document.querySelectorAll(".control").forEach(control => control.addEventListener("click", () => {
    $("command-select").value = control.dataset.command;
    document.querySelectorAll(".control").forEach(item => item.classList.toggle("primary", item === control));
  }));
  $("command-select").addEventListener("change", event => {
    document.querySelectorAll(".control").forEach(item => item.classList.toggle("primary", item.dataset.command === event.target.value));
  });
  $("command-payload").addEventListener("input", validatePayload);
  $("send-command").addEventListener("click", reviewCommand);
  $("confirm-dialog").addEventListener("close", () => {
    if ($("confirm-dialog").returnValue === "confirm") submitPendingCommand();
    else state.pendingCommand = null;
  });
  window.addEventListener("popstate", async () => {
    const query = Model.parseQuery(location.search);
    state.historicalAt = query.at;
    state.eventSearch = query.search;
    state.eventCategory = query.categories[0] || "all";
    state.rawEvents = query.raw;
    state.treeKind = query.treeMode;
    state.treeRevision = query.treeRevision;
    state.selectedNodeId = query.focusId;
    activateView(query.view, false);
    if (query.trajectoryId) await selectTrajectory(query.trajectoryId, { keepHistorical: true });
  });
  window.addEventListener("resize", () => {
    clearTimeout(state.graphResizeTimer);
    state.graphResizeTimer = setTimeout(() => {
      if (state.view === "tree" && state.treeGraph && !state.treeGraph.destroyed()) {
        state.treeGraph.resize();
        state.treeGraph.fit(undefined, 35);
      }
      if (state.view === "agents" && state.agentGraph && !state.agentGraph.destroyed()) {
        state.agentGraph.resize();
        state.agentGraph.fit(undefined, 30);
      }
    }, 120);
  });
}

function keyboardActivate(handler) {
  return event => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      handler();
    }
  };
}

function cssEscape(value) {
  if (window.CSS && CSS.escape) return CSS.escape(String(value));
  return String(value).replace(/[^a-zA-Z0-9_-]/g, "\\$&");
}

function emptyBlock(title, copy, icon) {
  const block = el("div", "empty-state");
  block.append(el("span", "empty-icon", icon));
  const text = el("div");
  text.append(el("strong", "", title), el("p", "", copy));
  block.append(text);
  return block;
}

function renderLoadingMission() {
  const root = $("mission-card");
  root.replaceChildren(emptyBlock("Loading trajectory", "Reconstructing projection and immutable events.", "↻"));
}

function renderNoTrajectories() {
  state.selectedId = null;
  state.snapshot = null;
  $("mission-card").replaceChildren(emptyBlock("No trajectories yet", "Start GPSR or send a mock trace to populate the debugger.", "⌁"));
  renderAll();
}

function renderFatal(error) {
  const message = error instanceof Error ? error.message : String(error);
  $("mission-card").replaceChildren(emptyBlock("Debugger unavailable", message, "!"));
}

boot();
