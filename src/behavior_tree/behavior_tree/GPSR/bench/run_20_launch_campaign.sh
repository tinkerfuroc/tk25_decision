#!/usr/bin/env bash
# Run isolated one-command GPSR batteries until 20 launches finish cleanly.
# A launch counts when the live stack starts and the battery reaches normal
# terminal teardown. Planner and physical verdicts remain visible separately.
set -u

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BENCH="$ROOT/gpsr_runs/bench"
CAMPAIGN="$BENCH/t2-20-live-campaign.json"
TEMPLATE="$BENCH/t2-20-live-rerun62/battery-natural.sh"
TARGET=20

mkdir -p "$BENCH"

campaign_count() {
  python3 - "$CAMPAIGN" <<'PY'
import json, sys
try:
    data = json.load(open(sys.argv[1]))
except (OSError, json.JSONDecodeError):
    data = {}
print(len(data.get("batches", [])) if isinstance(data, dict) else 0)
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
  python3 - "$CAMPAIGN" "$1" "$TARGET" <<'PY'
import json, os, sys, tempfile
path, batch, target = sys.argv[1], sys.argv[2], int(sys.argv[3])
try:
    data = json.load(open(path))
except (OSError, json.JSONDecodeError):
    data = {}
if not isinstance(data, dict):
    data = {}
data.setdefault("name", "gpsr-20-launches")
data["target"] = target
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

cleanup_recorders() {
  local out="$1"
  local pattern="/tmp/gpsr_run_record_full.py $out/runs/"
  local pid
  local found=()
  while read -r pid; do
    [ -n "$pid" ] && found+=("$pid")
  done < <(pgrep -f "$pattern" || true)
  [ "${#found[@]}" -eq 0 ] && return 0

  echo "[campaign] stopping recorder groups for $(basename "$out")" >>"$out/campaign.log"
  for pid in "${found[@]}"; do
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  done
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    local alive=0
    for pid in "${found[@]}"; do
      kill -0 "$pid" 2>/dev/null && alive=1
    done
    [ "$alive" -eq 0 ] && return 0
    sleep 1
  done
  for pid in "${found[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
}

while [ "$(campaign_count)" -lt "$TARGET" ]; do
  index="$(next_index)"
  batch="t2-20-live-rerun${index}"
  out="$BENCH/$batch"
  domain="$index"
  mkdir -p "$out"
  cp "$TEMPLATE" "$out/battery-natural.sh"
  chmod +x "$out/battery-natural.sh"
  python3 - "$out/battery.conf" "$out" "$domain" <<'PY'
import pathlib, sys
path, out, domain = map(pathlib.Path, sys.argv[1:])
path.write_text(
    "CORPUS_DIR=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-20-live-corpus-input\n"
    f"OUT={out}\n"
    f"GPSR_ROS_DOMAIN_ID={domain}\n"
    "LIMIT=1\nSTARTS=\"2\"\nEVIDENCE=0\nGPU_WAIT_S=14400\nTIMEOUT=1500\n"
    "GPSR_CONTROLLERS_FILE=/home/tinker/tk25_ws/src/tk25_decision/src/behavior_tree/behavior_tree/GPSR/gpsr_runs/bench/t2-20-live-controllers.yaml\n"
)
PY
  echo "[campaign] launching $batch domain=$domain $(date -Is)"
  (
    cd "$ROOT"
    bash "$out/battery-natural.sh" 1
  ) >>"$out/campaign.log" 2>&1
  cleanup_recorders "$out"
  status="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("status"))' "$out/progress.json" 2>/dev/null || true)"
  if grep -q '^BATTERY DONE up_rc=0' "$out/battery.log" 2>/dev/null \
      && grep -q '^gpsr-stack up rc=0' "$out/battery.log" 2>/dev/null \
      && [ "$status" = complete ]; then
    count="$(record_batch "$batch")"
    write_checkpoints
    echo "[campaign] counted $batch ($count/$TARGET launches) $(date -Is)"
  else
    echo "[campaign] $batch did not reach a normal terminal launch; continuing $(date -Is)"
  fi
done

echo "[campaign] target reached: $(campaign_count)/$TARGET launches $(date -Is)"
