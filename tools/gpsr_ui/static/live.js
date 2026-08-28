// tools/gpsr_ui/static/live.js
//
// The dashboard for a battery run in progress. Polls /api/live/stream
// (server-sent events, one line roughly every 2s) and re-renders every
// in-flight run as a card.
//
// URL construction is deliberately NOT reinvented here: tier names can
// embed a slash (a real pseudo-tier, e.g. "t2-2026/invalidated-20260826"),
// so a bare `/run/${tier}/${dirName}` template literal or a fresh
// `encodeURIComponent(tier)` would mis-split or double-encode it -- that
// exact bug has already been fixed twice in run.js. Reuse its per-segment
// builders instead of a third copy of the same fix.
import { frameUrl, runPageUrl } from "./run.js";

const HTML_ESCAPES = { "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" };

export function escapeHtml(text) {
  return String(text).replace(/[&<>"]/g, (c) => HTML_ESCAPES[c]);
}

export function fmtDuration(seconds) {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) {
    return "-";
  }
  const clamped = Math.max(0, seconds);
  const m = Math.floor(clamped / 60);
  const s = Math.floor(clamped % 60);
  return `${m}m${String(s).padStart(2, "0")}s`;
}

// Fraction of the run's 900s hard timeout elapsed so far, clamped to
// [0, 100] -- the budget bar's fill width. Independent of whether the
// run is already over its 6-minute *target*, which is a separate,
// softer signal (see overBudget below).
export function timeoutPct(summary) {
  if (summary.elapsed_s === null || summary.elapsed_s === undefined) return 0;
  if (!summary.timeout_s) return 0;
  return Math.min(100, Math.max(0, (summary.elapsed_s / summary.timeout_s) * 100));
}

export function overBudget(summary) {
  return summary.elapsed_s !== null && summary.elapsed_s !== undefined
    && summary.budget_s !== null && summary.budget_s !== undefined
    && summary.elapsed_s > summary.budget_s;
}

function frameFigures(tier, dirName, newestFrames) {
  const labels = Object.keys(newestFrames || {}).sort();
  if (labels.length === 0) return "";
  const figures = labels.map((label) => {
    const file = newestFrames[label];
    const src = frameUrl(tier, dirName, label, file);
    return `<figure><img src="${src}" alt="${escapeHtml(label)} camera">`
      + `<figcaption>${escapeHtml(label)}</figcaption></figure>`;
  }).join("");
  return `<div class="live-frames">${figures}</div>`;
}

function announceBlock(lines) {
  if (!lines || lines.length === 0) return "";
  const text = lines.map(escapeHtml).join("\n");
  return `<pre class="announce">${text}</pre>`;
}

// Pure: builds one card's HTML from one /api/live(/stream) item. Kept
// free of DOM/EventSource so it can be unit tested directly.
export function renderCard(item) {
  const s = item.summary;
  const over = overBudget(s);
  const pct = timeoutPct(s);
  const lastNav = s.last_nav
    ? `${escapeHtml(s.last_nav.name)} — ${escapeHtml(s.last_nav.status)}`
    : "-";
  const lastFailure = s.last_failure ? escapeHtml(s.last_failure.feedback) : "none";
  return `
    <article class="live-card">
      <h3><a href="${runPageUrl(item.tier, item.dir_name)}">${escapeHtml(item.dir_name)}</a></h3>
      <div class="budget"><div class="budget-fill ${over ? "over" : ""}" style="width:${pct}%"></div></div>
      <dl>
        <dt>elapsed</dt><dd class="${over ? "bad" : ""}">${
          fmtDuration(s.elapsed_s)} / ${fmtDuration(s.budget_s)} target
          (timeout ${fmtDuration(s.timeout_s)})</dd>
        <dt>plan step</dt><dd>${s.plan_step ? escapeHtml(s.plan_step) : "-"}</dd>
        <dt>last nav</dt><dd>${lastNav}</dd>
        <dt>tree regens</dt><dd>${s.tree_regenerations}</dd>
        <dt>gate failures</dt><dd class="${s.gate_failures ? "bad" : ""}">${s.gate_failures}</dd>
        <dt>last failure</dt><dd class="bad">${lastFailure}</dd>
      </dl>
      ${frameFigures(item.tier, item.dir_name, s.newest_frames)}
      ${announceBlock(s.announcements)}
    </article>`;
}

// The count is over a bounded tail window of the log, never the whole
// run (see live.py's find_progress_failures for why: the log can reach
// tens to ~100MB and grows throughout a battery, so reading it whole on
// every poll would stall the server). That MUST be visible here, not
// just in the API shape -- an undisclosed partial count is worse than
// an honest one, since it would silently understate a long battery's
// real failure count with no indication anything was cut off.
function windowLabel(progressFailures) {
  const kb = Math.round(progressFailures.window_bytes / 1024);
  const size = kb >= 1024 ? `${(kb / 1024).toFixed(1)}MB` : `${kb}KB`;
  return progressFailures.truncated
    ? `last ${size} of the log`
    : `the whole log (${size})`;
}

export function renderProgressPanel(progressFailures) {
  if (!progressFailures) return "";
  return `<p class="muted progress-panel">"failed to make progress" in `
    + `${windowLabel(progressFailures)} (${escapeHtml(progressFailures.path)}): `
    + `<b>${progressFailures.recent_count}</b></p>`;
}

// Pure: the whole page body's HTML from one /api/live(/stream) payload.
export function renderDashboard(data) {
  const cards = (data.in_flight || []).length === 0
    ? "<p class='muted'>no run in flight</p>"
    : data.in_flight.map(renderCard).join("");
  return cards + renderProgressPanel(data.progress_failures);
}

export function boot() {
  const root = document.getElementById("live");
  const source = new EventSource("/api/live/stream");
  source.onmessage = (event) => {
    const data = JSON.parse(event.data);
    root.innerHTML = renderDashboard(data);
  };
  source.onerror = () => {
    root.insertAdjacentHTML("afterbegin",
      "<p class='muted'>stream interrupted; retrying…</p>");
  };
  return source;
}
