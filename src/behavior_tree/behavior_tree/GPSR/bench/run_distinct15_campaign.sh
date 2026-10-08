#!/usr/bin/env bash
# Run the explicit diversity corpus until all fifteen unique command IDs have
# a strict planner=PASS + physical=PASS result.
#
# This is a setup/runner wrapper.  It does not start anything when sourced; run
# it explicitly after the location/constants fixes and after checking the
# simulator is idle.  One command gets its own isolated tier-2 batch, so a
# failure can be retried without overwriting its recording or truth report.
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BENCH="$ROOT/gpsr_runs/bench"
CORPUS_DIR="${GPSR_DISTINCT15_CORPUS_DIR:-$BENCH/t2-15-distinct}"
CORPUS="$CORPUS_DIR/corpus.jsonl"
CAMPAIGN="$BENCH/t2-15-distinct-campaign.json"
TEMPLATE="${GPSR_DISTINCT15_BATTERY_TEMPLATE:-$BENCH/t2-20-live-rerun109/battery-natural.sh}"
TARGET=15

mkdir -p "$CORPUS_DIR"
  python3 "$ROOT/bench/distinct15.py" --out "$CORPUS" >/dev/null

campaign_json() {
  python3 - "$CAMPAIGN" <<'PY'
import json, pathlib, sys
p = pathlib.Path(sys.argv[1])
try:
    d = json.loads(p.read_text())
except (OSError, json.JSONDecodeError):
    d = {}
if not isinstance(d, dict):
    d = {}
d.setdefault("name", "gpsr-distinct15")
d.setdefault("target", 15)
d.setdefault("batches", [])
print(json.dumps(d))
PY
}

strict_ids() {
  # Print command IDs that have already passed, once per ID.  A retry batch is
  # allowed to coexist with an earlier failure; only a dual PASS contributes.
  python3 - "$CAMPAIGN" "$BENCH" "$CORPUS" <<'PY'
import json, pathlib, sys
campaign_path, bench, corpus_path = (pathlib.Path(sys.argv[1]),
                                     pathlib.Path(sys.argv[2]),
                                     pathlib.Path(sys.argv[3]))
try:
    campaign = json.loads(campaign_path.read_text())
except (OSError, json.JSONDecodeError):
    campaign = {}
# A batch can outlive a corpus correction. Only count a pass when the command
# text recorded in that batch still matches the current canonical corpus.
try:
    current_text = {
        row["id"]: row.get("text", "")
        for line in corpus_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and isinstance((row := json.loads(line)).get("id"), str)
    }
except (OSError, json.JSONDecodeError):
    current_text = {}
passed = set()
for batch in campaign.get("batches", []) if isinstance(campaign, dict) else []:
    batch_dir = bench / str(batch)
    try:
        batch_text = {
            row["id"]: row.get("text", "")
            for line in (batch_dir / "corpus.jsonl").read_text(encoding="utf-8").splitlines()
            if line.strip() and isinstance((row := json.loads(line)).get("id"), str)
        }
    except (OSError, json.JSONDecodeError):
        batch_text = {}
    try:
        report = json.loads((batch_dir / "report.json").read_text())
    except (OSError, json.JSONDecodeError):
        continue
    for result in report.get("results", []) if isinstance(report, dict) else []:
        if (isinstance(result, dict)
                and result.get("planner_verdict") == "PASS"
                and result.get("physical_verdict") == "PASS"):
            entry_id = result.get("entry_id")
            recorded = batch_text.get(entry_id) if isinstance(entry_id, str) else None
            if (isinstance(entry_id, str) and entry_id in current_text
                    and recorded == current_text[entry_id]
                    and (not result.get("text") or result.get("text") == recorded)):
                passed.add(entry_id)
for entry_id in sorted(passed):
    print(entry_id)
PY
}

strict_count() { strict_ids | wc -l | tr -d ' '; }

next_entry() {
  python3 - "$CORPUS" <(strict_ids) <<'PY'
import json, pathlib, sys
corpus = pathlib.Path(sys.argv[1])
passed = set(pathlib.Path(sys.argv[2]).read_text().splitlines())
for line in corpus.read_text(encoding="utf-8").splitlines():
    if not line.strip():
        continue
    raw = json.loads(line)
    entry_id = raw.get("id")
    if isinstance(entry_id, str) and entry_id not in passed:
        print(entry_id)
        break
PY
}

entry_index() {
  python3 - "$CORPUS" "$1" <<'PY'
import json, pathlib, sys
needle = sys.argv[2]
entries = []
for line in pathlib.Path(sys.argv[1]).read_text().splitlines():
    if not line.strip():
        continue
    raw = json.loads(line)
    if isinstance(raw.get("id"), str):
        entries.append(raw)
for i, entry in enumerate(entries):
    if entry.get("id") == needle:
        print(i)
        break
else:
    raise SystemExit(f"entry not in corpus: {needle}")
PY
}

next_batch() {
  python3 - "$BENCH" <<'PY'
import pathlib, re, sys
values = []
for p in pathlib.Path(sys.argv[1]).glob("t2-15-distinct-rerun*"):
    m = re.search(r"rerun(\d+)$", p.name)
    if m:
        values.append(int(m.group(1)))
print(max(values, default=0) + 1)
PY
}

record_batch() {
  python3 - "$CAMPAIGN" "$1" <<'PY'
import json, os, pathlib, sys, tempfile
p, batch = pathlib.Path(sys.argv[1]), sys.argv[2]
try:
    d = json.loads(p.read_text())
except (OSError, json.JSONDecodeError):
    d = {}
if not isinstance(d, dict):
    d = {}
d.setdefault("name", "gpsr-distinct15")
d["target"] = 15
names = d.setdefault("batches", [])
if batch not in names:
    names.append(batch)
p.parent.mkdir(parents=True, exist_ok=True)
fd, tmp = tempfile.mkstemp(prefix=".distinct15-campaign-", dir=str(p.parent))
with os.fdopen(fd, "w", encoding="utf-8") as fh:
    json.dump(d, fh, indent=2)
    fh.write("\n")
os.replace(tmp, p)
PY
}

while [ "$(strict_count)" -lt "$TARGET" ]; do
  entry="$(next_entry)"
  [ -n "$entry" ] || { echo "no unpassed entry remains" >&2; exit 2; }
  index="$(entry_index "$entry")"
  batch_index="$(next_batch)"
  batch="t2-15-distinct-rerun${batch_index}"
  out="$BENCH/$batch"
  mkdir -p "$out"
  cp "$TEMPLATE" "$out/battery-natural.sh"
  # A one-entry batch needs a one-entry streak requirement.  Do not alter the
  # source template in place; every batch remains a self-contained artifact.
  sed -i 's/--require-streak 20/--require-streak 1/g' "$out/battery-natural.sh"
  chmod +x "$out/battery-natural.sh"
  cat > "$out/battery.conf" <<EOF
CORPUS_DIR=$CORPUS_DIR
OUT=$out
  # Fast DDS maps ROS domain IDs into UDP ports; values above 232 overflow
  # that mapping and make camera_compat_bridge abort during vision startup.
  # Keep isolated domains in the validated low range and wrap if retries are
  # numerous.
GPSR_ROS_DOMAIN_ID=$((100 + (batch_index % 100)))
# The copied live template predates the nested GPSR package directory. Set the
# controller profile explicitly so manipulation launch cannot resolve the
# path as <workspace>/behavior_tree/gpsr_runs/....
GPSR_CONTROLLERS_FILE=$ROOT/gpsr_runs/bench/t2-20-live-controllers.yaml
LIMIT=1
STARTS="$index"
EVIDENCE=1
GPU_WAIT_S=14400
TIMEOUT=1500
EOF
  echo "[distinct15] launching $batch entry=$entry index=$index strict=$(strict_count)/$TARGET $(date -Is)"
  (cd "$ROOT" && bash "$out/battery-natural.sh" 1) >"$out/campaign.log" 2>&1 || true
  status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("status"))' "$out/progress.json" 2>/dev/null || true)"
  if grep -q '^BATTERY DONE up_rc=0' "$out/battery.log" 2>/dev/null \
      && grep -q '^gpsr-stack up rc=0' "$out/battery.log" 2>/dev/null \
      && [ "$status" = complete ]; then
    record_batch "$batch"
    echo "[distinct15] completed $batch strict=$(strict_count)/$TARGET $(date -Is)"
  else
    echo "[distinct15] $batch did not reach normal teardown; preserving artifacts and retrying" >&2
  fi
done

echo "[distinct15] target reached: $(strict_count)/$TARGET unique strict dual passes $(date -Is)"
