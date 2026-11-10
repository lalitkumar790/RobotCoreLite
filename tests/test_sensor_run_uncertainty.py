import csv
import json
import pathlib
import tempfile
import unittest

from scripts.analyze_sensor_runs import (
    analyze_collection, load_completed_collection, paired_bootstrap,
    quantile, render_report, write_analysis, RUN_HEADER,
)


MODES = ("dds_bridge", "ipc", "shm", "persistent_shm")


def fixture(root, rounds=4, modes=MODES):
    root = pathlib.Path(root)
    root.mkdir(parents=True, exist_ok=True)
    plan = [list(modes[round_idx % len(modes):] +
                 modes[:round_idx % len(modes)]) for round_idx in range(rounds)]
    manifest = {
        "schema_version": 1,
        "status": "completed",
        "modes": list(modes),
        "order": plan,
        "repetitions": rounds,
        "frames_per_run": 2,
        "requested_period_ms": 10,
        "successful_runs": rounds * len(modes),
        "expected_runs": rounds * len(modes),
    }
    (root / "manifest.json").write_text(json.dumps(manifest))
    baseline = {"dds_bridge": 20, "ipc": 10, "shm": 15, "persistent_shm": 12}
    rows = []
    for round_id, order in enumerate(plan, 1):
        for position, mode in enumerate(order, 1):
            us = baseline[mode] + round_id
            run = root / f"round{round_id:03d}_pos{position}_{mode}"
            run.mkdir()
            for filename, delta in (("source_events.csv", 0),
                                     ("sink_events.csv", us * 1000)):
                with (run / filename).open("w", newline="") as stream:
                    writer = csv.writer(stream)
                    writer.writerow(["sequence", "monotonic_ns"])
                    writer.writerows([[seq, seq * 1000000 + delta] for seq in (1, 2)])
            with (run / "paired_events.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["sequence", "source_monotonic_ns",
                                 "sink_monotonic_ns", "observed_delivery_us",
                                 "source_interval_us"])
                writer.writerows([
                    [seq, seq * 1000000, seq * 1000000 + us * 1000,
                     f"{us:.3f}", "" if seq == 1 else "1000.000"]
                    for seq in (1, 2)
                ])
            (run / "summary.json").write_text(json.dumps({
                "mode": mode, "source_frames": 2, "sink_frames": 2,
                "requested_period_ms": 10, "performance_claim": False,
                "complete_contiguous_delivery": True,
                "observed_source_interval_us_mean": 1000.0,
            }))
            (run / "environment.txt").write_text("fixture_only=1\n")
            (run / "run_status.json").write_text(json.dumps({
                "status": "success", "round": round_id, "position": position,
                "mode": mode, "frames": 2,
            }))
            rows.append([round_id, position, mode, 2, us, us, us, 1000, 0.1])
    with (root / "run_summary.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(RUN_HEADER)
        writer.writerows(rows)
    return root


class RunUncertaintyTests(unittest.TestCase):
    def test_complete_paired_rounds_and_deterministic_bootstrap(self):
        with tempfile.TemporaryDirectory() as d:
            root = fixture(d)
            report = analyze_collection(root, seed=123, resamples=1000)
            self.assertFalse(report["performance_claim"])
            self.assertFalse(report["controlled_host_evidence"])
            self.assertEqual(report["rounds"], 4)
            ipc = report["paired_differences"]["ipc"]
            self.assertEqual(ipc["per_round_median_difference_us"], [-10] * 4)
            self.assertEqual(ipc["exploratory_resampling"]["mean_difference_us"], -10)
            self.assertEqual(
                ipc["exploratory_resampling"]["resampling_interval_95_us"],
                [-10, -10])
            self.assertEqual(report, analyze_collection(root, seed=123, resamples=1000))
            saved = write_analysis(root, seed=123, resamples=1000)
            self.assertEqual(saved["rounds"], 4)
            self.assertEqual(json.loads((root / "run_uncertainty.json").read_text()),
                             saved)
            markdown = (root / "run_uncertainty.md").read_text()
            self.assertIn("round", markdown)
            self.assertIn("not", markdown.lower())
            self.assertIn("No p-values", markdown)

    def test_under_four_runs_does_not_invent_uncertainty(self):
        with tempfile.TemporaryDirectory() as d:
            root = fixture(d, rounds=3)
            report = analyze_collection(root, seed=12, resamples=1000)
            self.assertTrue(report["paired_differences"]["ipc"][
                "insufficient_runs_for_resampling"])
            self.assertIsNone(report["paired_differences"]["ipc"][
                "exploratory_resampling"])
            self.assertIn("fewer than 4", render_report(report))

    def test_incomplete_or_failed_trials_cannot_be_compared(self):
        with tempfile.TemporaryDirectory() as d:
            root = fixture(d)
            manifest_path = root / "manifest.json"
            manifest = json.loads(manifest_path.read_text())
            manifest["status"] = "failed"
            manifest_path.write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, "completed"):
                analyze_collection(root)
            manifest["status"] = "completed"
            manifest_path.write_text(json.dumps(manifest))
            missing = root / "round001_pos1_dds_bridge" / "source_events.csv"
            missing.unlink()
            with self.assertRaises((ValueError, FileNotFoundError)):
                analyze_collection(root)

    def test_reject_summary_tampering_and_pid_wrong_mode(self):
        with tempfile.TemporaryDirectory() as d:
            root = fixture(d)
            csv_file = root / "run_summary.csv"
            original = csv_file.read_text()
            csv_file.write_text(original.replace(",ipc,2,", ",ipc,99,", 1))
            with self.assertRaisesRegex(ValueError, "count"):
                load_completed_collection(root)
            csv_file.write_text(original)
            first = root / "round001_pos1_dds_bridge" / "run_status.json"
            record = json.loads(first.read_text())
            record["mode"] = "ipc"
            first.write_text(json.dumps(record))
            with self.assertRaisesRegex(ValueError, "status mismatch"):
                load_completed_collection(root)

    def test_reject_invalid_or_nonfinite_metrics(self):
        with tempfile.TemporaryDirectory() as d:
            root = fixture(d)
            csv_file = root / "run_summary.csv"
            text = csv_file.read_text()
            # Replace a run median without altering raw event measurements.
            csv_file.write_text(text.replace(",21,21,21,", ",nan,21,21,", 1))
            with self.assertRaisesRegex(ValueError, "invalid/nonfinite"):
                load_completed_collection(root)
            csv_file.write_text(text.replace(",21,21,21,", ",999,21,21,", 1))
            with self.assertRaises(ValueError):
                load_completed_collection(root)

    def test_sampling_never_uses_individual_frames(self):
        self.assertIsNone(paired_bootstrap([1, 2, 3], seed=1))
        draw = paired_bootstrap([1, -1, 3, -3, 4], seed=6, resamples=200)
        self.assertEqual(draw["rounds"], 5)
        self.assertIn("round", draw["resampling_unit"])
        self.assertTrue(draw["exploratory_only"])
        self.assertEqual(quantile([1, 2, 3], .5), 2)
        with self.assertRaises(ValueError):
            quantile([], .5)


if __name__ == "__main__":
    unittest.main()
