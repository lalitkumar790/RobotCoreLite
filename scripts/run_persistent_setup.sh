#!/usr/bin/env bash
# Reproducible manager-only setup experiment. No message RTT is measured.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SAMPLES="$(printenv SAMPLES || echo 1000)"
WARMUP="$(printenv WARMUP || echo 50)"
CAPACITY="$(printenv CAPACITY || echo 1024)"
for x in "$SAMPLES" "$WARMUP" "$CAPACITY"; do
  if [[ ! "$x" =~ ^[1-9][0-9]*$ ]]; then
    echo "SAMPLES/WARMUP/CAPACITY must be positive integers" >&2; exit 1
  fi
done
if (( SAMPLES > 100000 || WARMUP > 10000 || CAPACITY > 16777216 )); then
  echo "Experiment configuration exceeds supported limits" >&2; exit 1
fi
(
  cd "$ROOT"
  cmake -S robotcore_transports/persistent_shm -B build/persistent-setup \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
  cmake --build build/persistent-setup --parallel
)
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
OUT="$ROOT/results/raw/persistent-setup-$STAMP"
REPORT="$ROOT/results/reports/persistent-setup-$STAMP"
mkdir -p "$OUT" "$REPORT"
{
  printf 'timestamp_utc=%s\n' "$STAMP"
  printf 'git_sha=%s\n' "$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
  printf 'kernel=%s\n' "$(uname -a)"
  printf 'os=%s\n' "$(grep '^PRETTY_NAME=' /etc/os-release || true)"
  printf 'cpu=%s\n' "$(lscpu | grep '^Model name:' || true)"
  printf 'compiler=%s\n' "$(c++ --version | head -1)"
  printf 'build_type=Release\nsamples=%s\nwarmup=%s\ncapacity_bytes=%s\n' \
    "$SAMPLES" "$WARMUP" "$CAPACITY"
  printf 'experiment=manager_setup_only_no_endpoints_no_rtt\n'
} > "$OUT/environment.txt"
"$ROOT/build/persistent-setup/persistent_region_setup" \
  --samples "$SAMPLES" --warmup "$WARMUP" --capacity "$CAPACITY" \
  --csv "$OUT/setup.csv"
python3 "$ROOT/scripts/analyze_persistent_setup.py" "$OUT/setup.csv" "$REPORT/report.md"
echo "Raw observations: $OUT/setup.csv"
echo "Environment metadata: $OUT/environment.txt"
echo "Descriptive report: $REPORT/report.md"
