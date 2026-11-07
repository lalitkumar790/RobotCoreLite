#!/usr/bin/env python3
"""Pure standard-library analysis of measured RTT, never synthetic samples."""
import csv
import json
import math
import pathlib
import statistics
import sys


def percentile(sorted_data, p):
    if not sorted_data:
        return None
    k = (len(sorted_data) - 1) * p / 100
    lo, hi = math.floor(k), math.ceil(k)
    return sorted_data[lo] + (sorted_data[hi] - sorted_data[lo]) * (k - lo)


def analyze(folder):
    results = []
    for path in sorted(pathlib.Path(folder).rglob('*.csv')):
        with path.open(newline='') as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        statuses = [r['status'] for r in rows]
        if any(x not in ('ok', 'timeout') for x in statuses):
            raise ValueError(f'Unexpected status in {path}')
        latencies = sorted(float(r['rtt_us']) for r in rows if r['status'] == 'ok')
        if not all(math.isfinite(x) and x >= 0 for x in latencies):
            raise ValueError(f'Invalid RTT in {path}')
        success = len(latencies)
        result = dict(file=str(path), run_id=rows[0]['run_id'],
                      payload_bytes=int(rows[0]['payload_bytes']),
                      attempted=len(rows), received=success,
                      timed_out=len(rows) - success,
                      loss_rate=(len(rows) - success) / len(rows),
                      mean_rtt_us=statistics.mean(latencies) if latencies else None,
                      median_rtt_us=statistics.median(latencies) if latencies else None,
                      p95_rtt_us=percentile(latencies, 95),
                      p99_rtt_us=percentile(latencies, 99),
                      min_rtt_us=latencies[0] if latencies else None,
                      max_rtt_us=latencies[-1] if latencies else None)
        results.append(result)
    return results


def main(argv):
    if len(argv) != 3:
        raise SystemExit('usage: analyze_results.py INPUT_CSV_DIR OUTPUT_REPORT_DIR')
    results = analyze(argv[1])
    if not results:
        raise SystemExit('No measured CSV samples found; no report generated.')
    dest = pathlib.Path(argv[2]); dest.mkdir(parents=True, exist_ok=True)
    (dest / 'summary.json').write_text(json.dumps(results, indent=2) + '\\n')
    fields = list(results[0].keys())
    with (dest / 'summary.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader(); writer.writerows(results)
    lines = ['# DDS RTT baseline report', '',
             'RTT is measured using one steady clock. RTT/2 is **not** measured one-way latency.',
             'Timeout is a missing reply in this benchmark, not necessarily network packet loss.', '',
             '| Run | Bytes | Success / Attempts | Mean RTT (us) | P95 | P99 | Timeout rate |',
             '|---|---:|---:|---:|---:|---:|---:|']
    def fmt(v): return f'{v:.2f}' if v is not None else 'N/A'
    for r in results:
        lines.append(f"| {r['run_id']} | {r['payload_bytes']} | {r['received']} / {r['attempted']} | {fmt(r['mean_rtt_us'])} | {fmt(r['p95_rtt_us'])} | {fmt(r['p99_rtt_us'])} | {r['loss_rate']:.2%} |")
    (dest / 'report.md').write_text('\\n'.join(lines) + '\\n')
    print('Created', dest / 'report.md')


if __name__ == '__main__':
    main(sys.argv)
