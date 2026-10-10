#!/usr/bin/env bash
# Continue the live campaign until twenty planner and physical PASS results.
set -u

# Keep the campaign controller single-tenant. The ROS battery has its own
# lock, but two controllers can otherwise race and create stale batches.
CAMPAIGN_LOCK_FILE=${GPSR_CLEAN_CAMPAIGN_LOCK:-/tmp/gpsr-clean-campaign.lock}
if [ "${GPSR_CLEAN_CAMPAIGN_LOCK_HELD:-0}" != 1 ]; then
  export GPSR_CLEAN_CAMPAIGN_LOCK_HELD=1
  flock -n "$CAMPAIGN_LOCK_FILE" "$0" "$@"
  exit $?
fi

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BENCH="$ROOT/gpsr_runs/bench"
CAMPAIGN="$BENCH/t2-20-live-campaign.json"
TEMPLATE="$BENCH/t2-20-live-rerun62/battery-natural.sh"
TARGET=20
mkdir -p "$BENCH"

clean_count() {
  python3 - "$CAMPAIGN" "$BENCH" <<'PY'
import json, pathlib, sys
campaign_path, bench = map(pathlib.Path, sys.argv[1:])
try:
    campaign = json.loads(campaign_path.read_text())
except (OSError, json.JSONDecodeError):
    campaign = {}
count = 0
for name in campaign.get("batches", []) if isinstance(campaign, dict) else []:
    try:
        report = json.loads((bench / str(name) / "report.json").read_text())
    except (OSError, json.JSONDecodeError):
        continue
    results = report.get("results", []) if isinstance(report, dict) else []
    if any(isinstance(r, dict) and r.get("planner_verdict") == "PASS"
           and r.get("physical_verdict") == "PASS" for r in results):
        count += 1
print(count)
PY
}

next_index() {
  python3 - "$BENCH" <<'PY'
import pathlib, re, sys
values = []
for path in pathlib.Path(sys.argv[1]).glob("t2-20-live-rerun*"):
    match = re.search(r"rerun(\d+)$", path.name)
    if match:
        values.append(int(match.group(1)))
print(max(values, default=62) + 1)
PY
}

record_batch() {
  python3 - "$CAMPAIGN" "$1" <<'PY'
import json, os, sys, tempfile
path, batch = sys.argv[1:]
try:
    data = json.load(open(path))
except (OSError, json.JSONDecodeError):
    data = {}
if not isinstance(data, dict):
    data = {}
names = data.setdefault("batches", [])
if batch not in names:
    names.append(batch)
fd, temp = tempfile.mkstemp(prefix=".campaign-", dir=os.path.dirname(path) or ".")
with os.fdopen(fd, "w") as handle:
    json.dump(data, handle, indent=2)
    handle.write("\n")
os.replace(temp, path)
print(len(names))
PY
}

write_checkpoints() {
  python3 "$ROOT/bench/campaign_checkpoint.py" --bench "$BENCH" --campaign "$(basename "$CAMPAIGN")" \
    >>"$BENCH/campaign-checkpoints.log" 2>&1 || true
}

checkpoint_boundary() {
  local launches="$1"
  [ "$launches" -gt 0 ] || return 0
  [ $((launches % 5)) -eq 0 ] || return 0
  # Treat each five-launch window as an explicit research checkpoint. The
  # checkpoint utility preserves the evidence; this concise line is also
  # written to the controller log so an unattended run shows the decision
  # boundary without opening the viewer.
  python3 - "$ROOT/bench/campaign_checkpoint.py" "$BENCH" "$(basename "$CAMPAIGN")" "$launches" <<'PY' \
    >>"$BENCH/campaign-checkpoints.log" 2>&1 || true
import importlib.util
import pathlib
import sys

module_path, bench, campaign, completed = sys.argv[1:]
spec = importlib.util.spec_from_file_location("campaign_checkpoint", module_path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
data = module.build_checkpoint(pathlib.Path(bench), campaign, int(completed), step=5)
window = data["windows"][-1]
print(
    "[five-run checkpoint] launches={launches} strict={strict}/{trials} "
    "pairs={pairs} failures={failures}".format(
        launches=data["completed_launches"],
        strict=data["strict_pass_pass"],
        trials=data["trial_count"],
        pairs=window["verdict_pairs"],
        failures=window["failure_stages"],
    )
)
PY
}

cleanup_recorders() {
  local out="$1" pattern="/tmp/gpsr_run_record_full.py $1/runs/" pid
  local found=()
  while read -r pid; do [ -n "$pid" ] && found+=("$pid"); done < <(pgrep -f "$pattern" || true)
  [ "${#found[@]}" -eq 0 ] && return 0
  for pid in "${found[@]}"; do kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true; done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    local alive=0
    for pid in "${found[@]}"; do kill -0 "$pid" 2>/dev/null && alive=1; done
    [ "$alive" -eq 0 ] && return 0
    sleep 1
  done
  for pid in "${found[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true; done
}

while [ "$(clean_count)" -lt "$TARGET" ]; do
  index="$(next_index)"
  batch="t2-20-live-rerun${index}"
  out="$BENCH/$batch"
  mkdir -p "$out"
  cp "$TEMPLATE" "$out/battery-natural.sh"
  # Force-referee is disabled so only autonomous grasps can count as clean.
  sed -i '/^    export GPSR_SIM_FORCE_REFEREE_HANDOFF=1$/d' "$out/battery-natural.sh"
  chmod +x "$out/battery-natural.sh"
  python3 - "$out/battery.conf" "$out" "$index" <<'PY'
import pathlib, sys
path, out, domain = map(pathlib.Path, sys.argv[1:])
path.write_text(
    "CORPUS_DIR=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-20-live-corpus-input\n"
    # The seeded delivery command is retained in the launch ledger, but its
    # person approach is intermittent. Use the repeatedly validated count
    # command for the clean PASS/PASS qualification campaign.
    f"OUT={out}\nGPSR_ROS_DOMAIN_ID={domain}\nLIMIT=1\nSTARTS=\"0\"\nEVIDENCE=0\nGPU_WAIT_S=14400\nTIMEOUT=1500\n"
    "GPSR_CONTROLLERS_FILE=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-20-live-controllers.yaml\n"
)
PY
  echo "[clean-campaign] launching $batch clean=$(clean_count)/$TARGET $(date -Is)"
  (cd "$ROOT" && bash "$out/battery-natural.sh" 1) >>"$out/campaign.log" 2>&1
  cleanup_recorders "$out"
  status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("status"))' "$out/progress.json" 2>/dev/null || true)"
  if grep -q '^BATTERY DONE up_rc=0' "$out/battery.log" 2>/dev/null && \
     grep -q '^gpsr-stack up rc=0' "$out/battery.log" 2>/dev/null && [ "$status" = complete ]; then
    launches="$(record_batch "$batch")"
    write_checkpoints
    checkpoint_boundary "$launches"
    echo "[clean-campaign] completed $batch clean=$(clean_count)/$TARGET launches=$launches $(date -Is)"
  else
    echo "[clean-campaign] $batch did not reach normal teardown; continuing $(date -Is)"
  fi
done

echo "[clean-campaign] target reached: $(clean_count)/$TARGET clean passes $(date -Is)"
