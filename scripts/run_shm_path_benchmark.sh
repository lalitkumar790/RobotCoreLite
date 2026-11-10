#!/usr/bin/env bash
# SHM-only copied vs loaned comparison; never describe as a DDS result.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNS="${RUNS:-4}"
SAMPLES="${SAMPLES:-1000}"
WARMUP="${WARMUP:-100}"
PAYLOADS="${PAYLOADS:-64 4096 65536}"
for value in "$RUNS" "$SAMPLES" "$WARMUP"; do
  [[ "$value" =~ ^[1-9][0-9]*$ ]] || { echo "RUNS/SAMPLES/WARMUP must be positive" >&2; exit 1; }
done
for bytes in $PAYLOADS; do
  [[ "$bytes" =~ ^[1-9][0-9]*$ ]] && (( bytes <= 16777216 )) || {
    echo "Invalid payload size: $bytes" >&2; exit 1;
  }
done
cmake -S "$ROOT/robotcore_transports/shared_memory" -B "$ROOT/build/shm-path-bench" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$ROOT/build/shm-path-bench" --target shm_path_bench --parallel
BIN="$ROOT/build/shm-path-bench/shm_path_bench"
OUT="$ROOT/results/raw/shm-path-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$OUT"
{
  printf 'git_commit=%s\n' "$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'uname=%s\n' "$(uname -a)"
  printf 'cpu=%s\n' "$(lscpu | grep '^Model name:' || true)"
  printf 'compiler=%s\n' "$(c++ --version | head -1)"
  printf 'runs=%s\nsamples=%s\nwarmup=%s\npayloads=%s\n' "$RUNS" "$SAMPLES" "$WARMUP" "$PAYLOADS"
  printf 'cpu_governor=%s\n' "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unavailable)"
} > "$OUT/environment.txt"
server=""
cleanup() {
  if [[ -n "$server" ]]; then
    kill "$server" 2>/dev/null || true
    wait "$server" 2>/dev/null || true
    server=""
  fi
}
trap cleanup EXIT
for ((run=1; run<=RUNS; run++)); do
  for bytes in $PAYLOADS; do
    if (( run % 2 == 0 )); then modes=(loaned copied); else modes=(copied loaned); fi
    for mode in "${modes[@]}"; do
      label="r${run}_${bytes}_${mode}"
      name="/robotcore-path-$$-${run}-${bytes}-${mode}"
      "$BIN" responder "$mode" "$name" "$bytes" "$WARMUP" "$SAMPLES" > "$OUT/$label.server.log" 2>&1 &
      server=$!
      ready=0
      for ((attempt=0;attempt<100;attempt++)); do
        if grep -qx ready "$OUT/$label.server.log"; then ready=1; break; fi
        if ! kill -0 "$server" 2>/dev/null; then break; fi
        sleep 0.05
      done
      if (( !ready )); then echo "Responder failed: $OUT/$label.server.log" >&2; exit 1; fi
      status=0
      timeout 180 "$BIN" initiator "$mode" "$name" "$bytes" "$WARMUP" "$SAMPLES" \
        "$OUT/$label.csv" > "$OUT/$label.client.log" 2>&1 || status=$?
      cleanup
      if (( status != 0 )); then
        echo "Benchmark failed ($status): $OUT/$label.client.log" >&2
        exit 1
      fi
      if [[ "$(wc -l < "$OUT/$label.csv")" -ne "$((SAMPLES+1))" ]]; then
        echo "Incomplete benchmark: $label" >&2; exit 1
      fi
      echo "completed $label"
    done
  done
done
python3 "$ROOT/scripts/summarize_shm_paths.py" "$OUT"
echo "Raw results and descriptive report: $OUT"
