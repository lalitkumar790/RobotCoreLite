import csv
import json
import os
import pathlib
import tempfile
import unittest

from scripts.run_sensor_experiment import (
    MODES, _validated_run, describe_completeness, plan_rounds, run_experiment,
)


class SensorExperimentRunnerTests(unittest.TestCase):
    def test_order_is_reproducible_and_position_balanced(self):
        first = plan_rounds(MODES, 8, seed=790)
        self.assertEqual(first, plan_rounds(MODES, 8, seed=790))
        self.assertNotEqual(first, plan_rounds(MODES, 8, seed=791))
        self.assertEqual(len(first), 8)
        for order in first:
            self.assertEqual(set(order), set(MODES))
        for block in (first[:4], first[4:8]):
            for mode in MODES:
                self.assertEqual(
                    sorted(order.index(mode) for order in block),
                    [0, 1, 2, 3])

    def test_invalid_plan(self):
        with self.assertRaises(ValueError):
            plan_rounds(("ipc", "ipc"), 4, 3)
        with self.assertRaises(ValueError):
            plan_rounds(MODES, 0, 3)

    def write_fixture(self, folder, mode="ipc", count=2, missing=False):
        folder = pathlib.Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        for name, offset in (("source_events.csv", 0),
                             ("sink_events.csv", 2000)):
            with (folder / name).open("w", newline="") as out:
                writer = csv.writer(out)
                writer.writerow(["sequence", "monotonic_ns"])
                for seq in range(1, count + 1):
                    if missing and name == "sink_events.csv" and seq == count:
                        continue
                    writer.writerow([seq, 1000000 * seq + offset])
        with (folder / "paired_events.csv").open("w", newline="") as out:
            writer = csv.writer(out)
            writer.writerow(["sequence", "source_monotonic_ns",
                             "sink_monotonic_ns", "observed_delivery_us",
                             "source_interval_us"])
            for seq in range(1, count + 1):
                writer.writerow([seq, 1000000 * seq, 1000000 * seq + 2000,
                                 "2.000", "" if seq == 1 else "1000.000"])
        (folder / "summary.json").write_text(json.dumps({
            "mode": mode, "source_frames": count, "sink_frames": count,
            "requested_period_ms": 10, "complete_contiguous_delivery": True,
            "performance_claim": False, "observed_source_interval_us_mean": 1000.0,
        }))
        (folder / "environment.txt").write_text("measurement_kind=test_fixture\n")

    def test_artifact_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            self.write_fixture(directory)
            result = _validated_run(pathlib.Path(directory), "ipc", 2, 10)
            self.assertEqual(result["run_median_application_visible_us"], 2.0)
            self.assertEqual(result["frames"], 2)
            # Malformed event pairing is a fatal integrity failure.
            csv_path = pathlib.Path(directory) / "paired_events.csv"
            csv_path.write_text(csv_path.read_text().replace("2.000", "-1.000", 1))
            with self.assertRaises(ValueError):
                _validated_run(pathlib.Path(directory), "ipc", 2, 10)

    def test_incomplete_run_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            self.write_fixture(directory, count=2, missing=True)
            # Paired output alone is not sufficient: the source/sink raw
            # streams must also match and contain the expected count.
            with self.assertRaises(ValueError):
                _validated_run(pathlib.Path(directory), "ipc", 2, 10)

    def test_report_has_no_speed_claim(self):
        output = describe_completeness([], 4)
        self.assertIn("0/4", output)
        self.assertIn("No speedup", output)
        self.assertIn("dds_bridge", output)

    def test_run_collection_retains_outputs_and_manifest(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = pathlib.Path(directory)
            fake = folder / "fake_runner.sh"
            # The fixture creates deterministic test-only observations; it does
            # not simulate hardware, ROS, or any performance measurement.
            fake.write_text("""#!/usr/bin/env bash
set -euo pipefail
python3 - "$1" "$2" "$3" <<'PY'
import csv, json, os, pathlib, sys
mode, count, period = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
out = pathlib.Path(os.environ['SENSOR_EVENTS_DIR'])
for name, delta in (('source_events.csv', 0), ('sink_events.csv', 2000)):
    with (out/name).open('w', newline='') as fd:
        writer = csv.writer(fd)
        writer.writerow(['sequence','monotonic_ns'])
        writer.writerows([[i, i*1000000+delta] for i in range(1,count+1)])
with (out/'paired_events.csv').open('w', newline='') as fd:
    writer = csv.writer(fd)
    writer.writerow(['sequence','source_monotonic_ns','sink_monotonic_ns',
                     'observed_delivery_us','source_interval_us'])
    writer.writerows([[i,i*1000000,i*1000000+2000,'2.000',
                      '' if i==1 else '1000.000'] for i in range(1,count+1)])
(out/'summary.json').write_text(json.dumps(dict(
    mode=mode, source_frames=count, sink_frames=count,
    requested_period_ms=period, complete_contiguous_delivery=True,
    performance_claim=False,observed_source_interval_us_mean=1000.0)))
(out/'environment.txt').write_text('fixture_only=1\\n')
PY
""")
            result = run_experiment(folder / "runs", fake, repetitions=2,
                                    frames=2, period_ms=10, seed=3,
                                    timeout_seconds=10, modes=("ipc", "shm"))
            manifest = json.loads((result / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "completed")
            self.assertEqual(manifest["successful_runs"], 4)
            self.assertEqual(len(list(result.glob("round*/paired_events.csv"))), 4)
            with (result / "run_summary.csv").open() as f:
                self.assertEqual(len(list(csv.DictReader(f))), 4)
            self.assertIn("Validated runs: **4/4**",
                          (result / "report.md").read_text())

    def test_failure_is_not_hidden(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = pathlib.Path(directory)
            fake = folder / "failed_runner.sh"
            fake.write_text("#!/bin/bash\nexit 5\n")
            with self.assertRaises(RuntimeError):
                run_experiment(folder / "runs", fake, repetitions=1, frames=2,
                               period_ms=10, seed=3, timeout_seconds=10,
                               modes=("ipc",))
            generated = list((folder / "runs").iterdir())
            self.assertEqual(len(generated), 1)
            manifest = json.loads((generated[0] / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["successful_runs"], 0)
            self.assertEqual(len(list(generated[0].glob("round*/runner.log"))), 1)


if __name__ == "__main__":
    unittest.main()
