// tools/tests/js/test_live.mjs
import test from "node:test";
import assert from "node:assert/strict";
import {
  escapeHtml, fmtDuration, overBudget, renderCard, renderDashboard,
  renderProgressPanel, timeoutPct,
} from "../../gpsr_ui/static/live.js";

function summary(overrides = {}) {
  return {
    elapsed_s: 30, budget_s: 360, timeout_s: 900,
    plan_step: null, last_nav: null, last_failure: null,
    tree_regenerations: 0, gate_failures: 0,
    announcements: [], newest_frames: {},
    ...overrides,
  };
}

test("fmtDuration renders minutes and zero-padded seconds", () => {
  assert.equal(fmtDuration(65), "1m05s");
  assert.equal(fmtDuration(0), "0m00s");
  assert.equal(fmtDuration(null), "-");
  assert.equal(fmtDuration(undefined), "-");
});

test("timeoutPct is the fraction of the 900s hard timeout, clamped", () => {
  assert.equal(timeoutPct(summary({ elapsed_s: 450, timeout_s: 900 })), 50);
  assert.equal(timeoutPct(summary({ elapsed_s: 9000, timeout_s: 900 })), 100);
  assert.equal(timeoutPct(summary({ elapsed_s: null })), 0);
});

test("overBudget compares elapsed against the 6-minute target, not the timeout", () => {
  assert.equal(overBudget(summary({ elapsed_s: 400, budget_s: 360 })), true);
  assert.equal(overBudget(summary({ elapsed_s: 100, budget_s: 360 })), false);
  assert.equal(overBudget(summary({ elapsed_s: null, budget_s: 360 })), false);
});

test("escapeHtml neutralises markup so an announcement can never inject a tag", () => {
  assert.equal(escapeHtml('<img src=x onerror="1">'),
    "&lt;img src=x onerror=&quot;1&quot;&gt;");
});

test("renderCard links to the run page via the shared per-segment builder", () => {
  const html = renderCard({
    tier: "t2-2026/invalidated-20260826",
    dir_name: "s9999-070-x",
    summary: summary(),
  });
  // A bare `/run/${tier}/${dirName}` would leave the pseudo-tier's own
  // embedded slash un-encoded in a way indistinguishable from the run's
  // own path separator, or (with a naive whole-string encodeURIComponent)
  // would turn it into a literal "%2F" that breaks the server's
  // {path:path} route match -- this asserts the per-segment builder
  // (imported straight from run.js) was actually used.
  assert.ok(html.includes(
    "/run/t2-2026/invalidated-20260826/s9999-070-x"));
});

test("renderCard shows last nav's name and status when present", () => {
  const html = renderCard({
    tier: "t9", dir_name: "s9999-071-x",
    summary: summary({ last_nav: { name: "goto target kitchen", status: "SUCCESS" } }),
  });
  assert.ok(html.includes("goto target kitchen"));
  assert.ok(html.includes("SUCCESS"));
});

test("renderCard escapes an announcement instead of injecting it as markup", () => {
  const html = renderCard({
    tier: "t9", dir_name: "s9999-072-x",
    summary: summary({ announcements: ["<script>alert(1)</script>"] }),
  });
  assert.ok(!html.includes("<script>alert(1)</script>"));
  assert.ok(html.includes("&lt;script&gt;"));
});

test("renderCard includes a figure per camera, using the shared frame URL builder", () => {
  const html = renderCard({
    tier: "t9", dir_name: "s9999-073-x",
    summary: summary({ newest_frames: { head: "0005_3000.jpg", arena: "0002_1000.jpg" } }),
  });
  assert.ok(html.includes("/frame/t9/s9999-073-x/head/0005_3000.jpg"));
  assert.ok(html.includes("/frame/t9/s9999-073-x/arena/0002_1000.jpg"));
});

test("renderCard omits the frames block entirely when no camera has a frame yet", () => {
  const html = renderCard({
    tier: "t9", dir_name: "s9999-074-x", summary: summary({ newest_frames: {} }),
  });
  assert.ok(!html.includes("live-frames"));
});

test("renderDashboard shows the empty state when nothing is in flight", () => {
  const html = renderDashboard({ in_flight: [], progress_failures: null });
  assert.ok(html.includes("no run in flight"));
});

test("renderDashboard renders one card per in-flight run", () => {
  const html = renderDashboard({
    in_flight: [
      { tier: "t9", dir_name: "a", summary: summary() },
      { tier: "t9", dir_name: "b", summary: summary() },
    ],
    progress_failures: null,
  });
  assert.ok(html.includes(">a<"));
  assert.ok(html.includes(">b<"));
});

test("renderProgressPanel is empty when the bridge log could not be found", () => {
  assert.equal(renderProgressPanel(null), "");
});

test("renderProgressPanel surfaces the count when the bridge log was found", () => {
  const html = renderProgressPanel({ path: "/x/02-bridge.log", count: 3 });
  assert.ok(html.includes("<b>3</b>"));
});
