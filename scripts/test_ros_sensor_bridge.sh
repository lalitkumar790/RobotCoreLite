#!/usr/bin/env bash
# Cross-process ROS 2 LaserScan CDR delivery via each native transport.
# Functional test only: not a benchmark or evidence of latency advantage.
set -euo pipefail
if (( $# < 1 || $# > 2 )); then
  echo "usage: test_ros_sensor_bridge.sh ipc|shm|persistent_shm [frames]" >&2
  exit 2
fi
mode="$1"
frames="${2:-20}"
if [[ ! "$frames" =~ ^[1-9][0-9]*$ ]] || (( frames > 1000 )); then
  echo "frames must be between 1 and 1000" >&2; exit 2
fi
case "$mode" in
  ipc)
    binary="sensor_ipc_demo"
    flag="--socket"
    endpoint="/tmp/robotcore-native-$$.sock" ;;
  shm)
    binary="sensor_shm_demo"
    flag="--shm-name"
    endpoint="/robotcore-native-shm-$$" ;;
  persistent_shm)
    binary="sensor_persistent_shm_demo"
    flag="--shm-name"
    endpoint="/robotcore-native-pshm-$$" ;;
  *)
    echo "Unsupported mode: $mode" >&2; exit 2 ;;
esac
shm_file="/dev/shm/$(basename "$endpoint")"
root="$(cd "$(dirname "$0")/.." && pwd)"
bin="$root/install/robotcore_baseline/lib/robotcore_baseline/$binary"
if [[ ! -x "$bin" ]]; then
  echo "Build the ROS Jazzy robotcore_baseline package first: $bin" >&2
  exit 1
fi
logdir="$(mktemp -d /tmp/robotcore-native-XXXXXX)"
sink=""
bridge=""
source_pid=""
manager=""
cleanup() {
  for pid in "$source_pid" "$bridge" "$sink"; do
    if [[ -n "$pid" ]]; then
      kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  if [[ -n "$manager" ]]; then
    kill -TERM "$manager" 2>/dev/null || true
    wait "$manager" 2>/dev/null || true
  fi
  rm -rf "$logdir"
}
trap cleanup EXIT
if [[ "$mode" == persistent_shm ]]; then
  "$bin" --role manager "$flag" "$endpoint" > "$logdir/manager.log" 2>&1 &
  manager=$!
  for i in $(seq 1 100); do
    grep -q '^manager_ready_epoch=1$' "$logdir/manager.log" && break
    if ! kill -0 "$manager" 2>/dev/null; then break; fi
    sleep 0.05
  done
  if ! grep -q '^manager_ready_epoch=1$' "$logdir/manager.log"; then
    echo "Persistent manager failed:" >&2
    cat "$logdir/manager.log" >&2
    exit 1
  fi
fi
"$bin" --role sink "$flag" "$endpoint" --samples "$frames" > "$logdir/sink.log" 2>&1 &
sink=$!
for i in $(seq 1 100); do
  if [[ "$mode" == ipc && -S "$endpoint" ]]; then break; fi
  if [[ "$mode" != ipc && -e "$shm_file" ]]; then break; fi
  if ! kill -0 "$sink" 2>/dev/null; then break; fi
  sleep 0.05
done
if [[ "$mode" == ipc && ! -S "$endpoint" ]] ||
   [[ "$mode" != ipc && ! -e "$shm_file" ]]; then
  echo "Native scan sink failed to initialize:" >&2
  cat "$logdir/sink.log" >&2
  exit 1
fi
"$bin" --role bridge "$flag" "$endpoint" > "$logdir/bridge.log" 2>&1 &
bridge=$!
sleep 2
if ! kill -0 "$bridge" 2>/dev/null; then
  echo "Native scan bridge exited unexpectedly:" >&2
  cat "$logdir/bridge.log" >&2
  exit 1
fi
"$bin" --role source --samples 500 > "$logdir/source.log" 2>&1 &
source_pid=$!
if ! wait "$sink"; then
  echo "Native LaserScan delivery failed ($mode):" >&2
  cat "$logdir/sink.log" "$logdir/bridge.log" "$logdir/source.log" >&2
  if [[ -f "$logdir/manager.log" ]]; then cat "$logdir/manager.log" >&2; fi
  exit 1
fi
sink=""
if ! grep -q "^validated_native_scan_frames=$frames invalid=0$" "$logdir/sink.log"; then
  echo "Sink did not verify all expected frames ($mode)" >&2
  cat "$logdir/sink.log" >&2
  exit 1
fi
# The manager must outlive both attached persistent endpoints.
kill -TERM "$bridge" 2>/dev/null || true
wait "$bridge" 2>/dev/null || true
bridge=""
kill -TERM "$source_pid" 2>/dev/null || true
wait "$source_pid" 2>/dev/null || true
source_pid=""
if [[ -n "$manager" ]]; then
  kill -TERM "$manager" 2>/dev/null || true
  if ! wait "$manager"; then
    echo "Persistent manager shutdown failed:" >&2
    cat "$logdir/manager.log" >&2
    exit 1
  fi
  manager=""
fi
if [[ "$mode" != ipc && -e "$shm_file" ]]; then
  echo "Shared-memory name was not released: $shm_file" >&2
  exit 1
fi
cat "$logdir/sink.log"
echo "transport=$mode full_laserscan_frames=$frames valid=1"
