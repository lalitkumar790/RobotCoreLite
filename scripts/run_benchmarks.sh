#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EXPECTED_ROS_DISTRO="${EXPECTED_ROS_DISTRO:-jazzy}"
if ! command -v ros2 >/dev/null || ! command -v colcon >/dev/null; then
  echo "ros2 and colcon required: source installed ROS 2 and install colcon." >&2; exit 1
fi
if [[ "${ROS_DISTRO:-}" != "$EXPECTED_ROS_DISTRO" ]]; then
  echo "ROS_DISTRO='${ROS_DISTRO:-unset}', expected '$EXPECTED_ROS_DISTRO'; source the correct ROS environment." >&2; exit 1
fi
if [[ ! -f "$ROOT/install/robotcore_baseline/share/robotcore_baseline/package.xml" ]]; then
  echo "Build first: colcon build --base-paths robotcore_ros --packages-select robotcore_baseline" >&2; exit 1
fi
source "$ROOT/install/setup.bash"
RUN_TAG="$(date -u +%Y%m%dT%H%M%SZ)"
OUTPUT="$ROOT/results/raw/$RUN_TAG"
mkdir -p "$OUTPUT"
SAMPLES="${SAMPLES:-1000}"
WARMUP="${WARMUP:-100}"
TIMEOUT_MS="${TIMEOUT_MS:-1000}"
RUNS="${RUNS:-3}"
PAYLOADS="${PAYLOADS:-64 1024 65536}"
RELIABILITY="${RELIABILITY:-reliable}"
QOS_DEPTH="${QOS_DEPTH:-10}"
if [[ ! "$RUNS" =~ ^[1-9][0-9]*$ ]]; then echo "RUNS must be positive" >&2; exit 1; fi
{
  printf 'utc=%s\\n' "$RUN_TAG"
  printf 'ros_distro=%s\\nrmw_implementation=%s\\n' "$ROS_DISTRO" "${RMW_IMPLEMENTATION:-default}"
  printf 'ros_domain_id=%s\\n' "${ROS_DOMAIN_ID:-default}"
  printf 'kernel=%s\\n' "$(uname -a)"
  printf 'os=%s\\n' "$(cat /etc/os-release | grep '^PRETTY_NAME=' || true)"
  printf 'compiler=%s\\n' "$(c++ --version | head -1)"
  printf 'cpu=%s\\n' "$(lscpu | grep '^Model name:' | head -1 || true)"
  printf 'git_commit=%s\\n' "$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'samples=%s\\nwarmup=%s\\ntimeout_ms=%s\\nqos_depth=%s\\nreliability=%s\\n' "$SAMPLES" "$WARMUP" "$TIMEOUT_MS" "$QOS_DEPTH" "$RELIABILITY"
} > "$OUTPUT/environment.txt"
responder_pid=""
cleanup() { if [[ -n "$responder_pid" ]]; then kill "$responder_pid" 2>/dev/null || true; wait "$responder_pid" 2>/dev/null || true; fi; }
trap cleanup EXIT INT TERM
for payload in $PAYLOADS; do
  for ((i=1; i<=RUNS; i++)); do
    run_id="${RUN_TAG}_${payload}_${i}"
    csv="$OUTPUT/${run_id}.csv"
    ros2 run robotcore_baseline dds_pingpong --ros-args -p role:=responder -p payload_bytes:="$payload" -p reliability:="$RELIABILITY" -p qos_depth:="$QOS_DEPTH" > "$OUTPUT/${run_id}_responder.log" 2>&1 &
    responder_pid=$!
    # Allow discovery, but warmup is still required and results include timeouts.
    sleep 2
    set +e
    ros2 run robotcore_baseline dds_pingpong --ros-args -p role:=initiator -p payload_bytes:="$payload" -p samples:="$SAMPLES" -p warmup:="$WARMUP" -p timeout_ms:="$TIMEOUT_MS" -p reliability:="$RELIABILITY" -p qos_depth:="$QOS_DEPTH" -p csv:="$csv" -p run_id:="$run_id" > "$OUTPUT/${run_id}_initiator.log" 2>&1
    status=$?
    set -e
    cleanup
    responder_pid=""
    echo "$run_id exit=$status csv=$csv"
    # Exit 2 means one or more timeouts; continue to preserve those findings.
    if [[ "$status" -ne 0 && "$status" -ne 2 ]]; then exit "$status"; fi
  done
done
python3 "$ROOT/scripts/analyze_results.py" "$OUTPUT" "$ROOT/results/reports/$RUN_TAG"
