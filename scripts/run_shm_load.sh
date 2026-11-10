#!/usr/bin/env bash
# Functional CI smoke or dedicated-host exploratory load characterization.
# NEVER claim superiority to Fast DDS based on this experiment.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUNS="${RUNS:-4}"
FRAMES="${FRAMES:-1000}"
PAYLOADS="${PAYLOADS:-64 4096 65536}"
DELAYS_US="${DELAYS_US:-0 250}"
for n in "$RUNS" "$FRAMES"; do
  [[ "$n" =~ ^[1-9][0-9]*$ ]] || { echo "RUNS and FRAMES must be positive integers" >&2; exit 1; }
done
for bytes in $PAYLOADS; do
  [[ "$bytes" =~ ^[0-9]+$ ]] && (( bytes >= 16 && bytes <= 10000000 )) || {
    echo "Invalid payload size: $bytes" >&2; exit 1;
  }
done
for delay in $DELAYS_US; do
  [[ "$delay" =~ ^[0-9]+$ ]] && (( delay <= 1000000 )) || {
    echo "Invalid delay: $delay" >&2; exit 1;
  }
done
cmake -S "$ROOT/robotcore_transports/shared_memory" -B "$ROOT/build/shm-load" \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
cmake --build "$ROOT/build/shm-load" --target shm_load_bench --parallel
BIN="$ROOT/build/shm-load/shm_load_bench"
OUT="$ROOT/results/raw/shm-load-$(date -u +%Y%m%dT%H%M%SZ)-$$"
mkdir -p "$OUT"
{
  printf 'git_commit=%s\n' "$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'uname=%s\n' "$(uname -a)"
  printf 'compiler=%s\n' "$(c++ --version | head -1)"
  printf 'cpu=%s\n' "$(lscpu | grep '^Model name:' || true)"
  printf 'runs=%s\nframes=%s\npayloads=%s\ndelays_us=%s\n' "$RUNS" "$FRAMES" "$PAYLOADS" "$DELAYS_US"
  printf 'governor=%s\n' "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || echo unknown)"
} > "$OUT/environment.txt"
receiver=""
cleanup() {
  if [[ -n "$receiver" ]]; then
    kill "$receiver" 2>/dev/null || true
    wait "$receiver" 2>/dev/null || true
    receiver=""
  fi
}
trap cleanup EXIT
for ((run=1; run<=RUNS; run++)); do
  for bytes in $PAYLOADS; do
    for delay in $DELAYS_US; do
      if (( run % 2 == 0 )); then modes=(loaned copied); else modes=(copied loaned); fi
      for mode in "${modes[@]}"; do
        label="r${run}_b${bytes}_d${delay}_${mode}"
        name="/robotcore-load-$$-${run}-${bytes}-${delay}-${mode}"
        "$BIN" receiver "$mode" "$name" "$bytes" "$FRAMES" "$delay" \
          "$OUT/$label.receiver.csv" > "$OUT/$label.receiver.log" 2>&1 &
        receiver=$!
        ready=0
        for ((attempt=0; attempt<100; attempt++)); do
          if grep -qx ready "$OUT/$label.receiver.log"; then ready=1; break; fi
          if ! kill -0 "$receiver" 2>/dev/null; then break; fi
          sleep 0.05
        done
        if (( !ready )); then echo "Receiver failed: $OUT/$label.receiver.log" >&2; exit 1; fi
        status=0
        timeout 110s "$BIN" sender "$mode" "$name" "$bytes" "$FRAMES" "$delay" \
          "$OUT/$label.sender.csv" > "$OUT/$label.sender.log" 2>&1 || status=$?
        if (( status != 0 )); then
          echo "Sender failed ($status): $OUT/$label.sender.log" >&2; exit 1;
        fi
        # Reap the receiver naturally to ensure all frames and metrics are persisted.
        # Deadline is enforced inside receiver and by the runner's outer CI timeout.
        receiver_status=0
        wait "$receiver" || receiver_status=$?
        receiver=""
        if (( receiver_status != 0 )); then
          echo "Receiver failed ($receiver_status): $OUT/$label.receiver.log" >&2; exit 1;
        fi
        echo "completed $label"
      done
    done
  done
done
python3 "$ROOT/scripts/summarize_shm_load.py" "$OUT" "$FRAMES"
echo "Observations: $OUT"
