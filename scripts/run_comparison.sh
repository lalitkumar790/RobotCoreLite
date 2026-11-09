#!/usr/bin/env bash
# Controlled cross-process DDS-versus-IPC runner. CI smoke is not research evidence.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EXPECTED="$(printenv EXPECTED_ROS_DISTRO || echo jazzy)"
if [[ "$(printenv ROS_DISTRO || true)" != "$EXPECTED" ]]; then
  echo "Expected ROS_DISTRO=$EXPECTED. Source the installed ROS environment first." >&2
  exit 1
fi
for tool in ros2 colcon cmake python3 timeout; do
  if ! command -v "$tool" >/dev/null; then echo "Missing prerequisite: $tool" >&2; exit 1; fi
done
RUNS="$(printenv RUNS || echo 3)"
SAMPLES="$(printenv SAMPLES || echo 1000)"
WARMUP="$(printenv WARMUP || echo 100)"
TIMEOUT_MS="$(printenv TIMEOUT_MS || echo 1000)"
DISCOVERY_TIMEOUT_MS="$(printenv DISCOVERY_TIMEOUT_MS || echo 15000)"
MAX_RUN_SECS="$(printenv MAX_RUN_SECS || echo 180)"
PAYLOADS="$(printenv PAYLOADS || echo '64 1024 65536')"
RELIABILITY="$(printenv RELIABILITY || echo reliable)"
QOS_DEPTH="$(printenv QOS_DEPTH || echo 10)"
for value in "$RUNS" "$SAMPLES" "$WARMUP" "$TIMEOUT_MS" "$DISCOVERY_TIMEOUT_MS" "$MAX_RUN_SECS" "$QOS_DEPTH"; do
  if [[ ! "$value" =~ ^[1-9][0-9]*$ ]]; then
    echo "RUNS/SAMPLES/WARMUP/TIMEOUT_MS/MAX_RUN_SECS/QOS_DEPTH must be positive integers" >&2
    exit 1
  fi
done
if [[ "$RELIABILITY" != "reliable" ]]; then
  echo "IPC only implements reliable stream delivery. Set RELIABILITY=reliable." >&2
  exit 1
fi
for payload in $PAYLOADS; do
  if [[ ! "$payload" =~ ^[1-9][0-9]*$ ]] || (( payload > 16777216 )); then
    echo "Invalid payload: $payload" >&2; exit 1
  fi
done
(
 cd "$ROOT"
 colcon build --base-paths robotcore_ros --packages-select robotcore_baseline --cmake-args -DCMAKE_BUILD_TYPE=Release
 cmake -S robotcore_transports/ipc -B build/comparison-ipc -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
 cmake --build build/comparison-ipc --parallel
)
set +u
source "$ROOT/install/setup.bash"
set -u
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$ROOT/results/raw/comparison-$STAMP"
REPORT="$ROOT/results/reports/comparison-$STAMP"
mkdir -p "$OUT" "$REPORT"
SOCKDIR="$(mktemp -d "${TMPDIR:-/tmp}/robotcore-compare.XXXXXX")"
SOCKET="$SOCKDIR/transport.sock"
responder=""
cleanup() {
  if [[ -n "$responder" ]]; then
    kill "$responder" 2>/dev/null || true
    wait "$responder" 2>/dev/null || true
    responder=""
  fi
  rm -f "$SOCKET"
}
finish() { cleanup; rmdir "$SOCKDIR" 2>/dev/null || true; }
trap finish EXIT
{
  printf 'timestamp_utc=%s\n' "$STAMP"
  printf 'ros_distro=%s\n' "$ROS_DISTRO"
  printf 'rmw_requested=%s\n' "$(printenv RMW_IMPLEMENTATION || echo default)"
  printf 'ros_domain_id=%s\n' "$(printenv ROS_DOMAIN_ID || echo default)"
  printf 'os=%s\n' "$(grep '^PRETTY_NAME=' /etc/os-release || true)"
  printf 'kernel=%s\n' "$(uname -a)"
  printf 'cpu=%s\n' "$(lscpu | grep '^Model name:' || true)"
  printf 'compiler=%s\n' "$(c++ --version | head -1)"
  printf 'git_sha=%s\n' "$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'build_type=Release\nruns=%s\nsamples=%s\nwarmup=%s\n' "$RUNS" "$SAMPLES" "$WARMUP"
  printf 'payloads=%s\ntimeout_ms=%s\ndiscovery_timeout_ms=%s\nmax_run_secs=%s\n' "$PAYLOADS" "$TIMEOUT_MS" "$DISCOVERY_TIMEOUT_MS" "$MAX_RUN_SECS"
  printf 'reliability=%s\nqos_depth=%s\n' "$RELIABILITY" "$QOS_DEPTH"
  if command -v dpkg-query >/dev/null; then
    dpkg-query -W "ros-$EXPECTED-rclcpp" "ros-$EXPECTED-rmw-*" 2>/dev/null || true
  fi
} > "$OUT/environment.txt"
for ((run=1; run<=RUNS; ++run)); do
  if (( run % 2 == 0 )); then transports=(ipc dds); else transports=(dds ipc); fi
  for payload in $PAYLOADS; do
    for mode in "${transports[@]}"; do
      if [[ "$mode" == dds ]]; then
        cmd=("$ROOT/install/robotcore_baseline/lib/robotcore_baseline/transport_bench_dds")
      else
        cmd=("$ROOT/build/comparison-ipc/transport_bench_ipc")
      fi
      label="${STAMP}_${mode}_${payload}_r${run}"
      csv="$OUT/$label.csv"
      common=(--payload-bytes "$payload" --qos-depth "$QOS_DEPTH" --reliability "$RELIABILITY")
      extra=()
      if [[ "$mode" == ipc ]]; then extra=(--socket "$SOCKET"); fi
      "${cmd[@]}" --role responder "${common[@]}" "${extra[@]}" > "$OUT/$label.server.log" 2>&1 &
      responder=$!
      if [[ "$mode" == dds ]]; then sleep 3; else
        for ((i=0; i<100; ++i)); do
          [[ -S "$SOCKET" ]] && break
          sleep 0.05
        done
        if [[ ! -S "$SOCKET" ]]; then
          echo "IPC server failed to bind: $OUT/$label.server.log" >&2; exit 1
        fi
      fi
      if ! kill -0 "$responder" 2>/dev/null; then
        echo "Responder exited prematurely: $OUT/$label.server.log" >&2; exit 1
      fi
      set +e
      if [[ -x /usr/bin/time ]]; then
        /usr/bin/time -f 'wall_seconds=%e\nuser_seconds=%U\nsystem_seconds=%S\nmax_rss_kb=%M' \
          -o "$OUT/$label.resources.txt" timeout "${MAX_RUN_SECS}s" \
          "${cmd[@]}" --role initiator "${common[@]}" "${extra[@]}" \
          --samples "$SAMPLES" --warmup "$WARMUP" --timeout-ms "$TIMEOUT_MS" --discovery-timeout-ms "$DISCOVERY_TIMEOUT_MS" \
          --run-id "$label" --csv "$csv" > "$OUT/$label.client.log" 2>&1
      else
        timeout "${MAX_RUN_SECS}s" "${cmd[@]}" --role initiator "${common[@]}" "${extra[@]}" \
          --samples "$SAMPLES" --warmup "$WARMUP" --timeout-ms "$TIMEOUT_MS" --discovery-timeout-ms "$DISCOVERY_TIMEOUT_MS" \
          --run-id "$label" --csv "$csv" > "$OUT/$label.client.log" 2>&1
      fi
      exit_code=$?
      set -e
      cleanup
      if [[ "$exit_code" -ne 0 && "$exit_code" -ne 2 ]]; then
        echo "Invalid run $label (exit $exit_code), inspect $OUT/$label.client.log" >&2
        exit 1
      fi
      if [[ ! -f "$csv" ]] || [[ "$(wc -l < "$csv")" -ne "$((SAMPLES + 1))" ]]; then
        echo "Unexpected CSV sample count: $csv" >&2; exit 1
      fi
      printf '%s exit=%s\n' "$label" "$exit_code"
    done
  done
done
python3 "$ROOT/scripts/analyze_results.py" "$OUT" "$REPORT"
echo "Raw measurements: $OUT"
echo "Summary: $REPORT/report.md"
