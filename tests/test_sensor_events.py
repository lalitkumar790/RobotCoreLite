import csv
import json
import pathlib
import tempfile
import unittest
from scripts.analyze_sensor_events import analyze, read_events, percentile_nearest_rank


def write_events(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["sequence", "monotonic_ns"])
        w.writerows(rows)


class SensorEventAnalysisTest(unittest.TestCase):
    def test_valid_ordered_same_host_events_and_summary(self):
        with tempfile.TemporaryDirectory() as folder:
            root = pathlib.Path(folder)
            source, sink = root / "source.csv", root / "sink.csv"
            write_events(source, [(1, 1000000), (2, 2000000), (3, 3000000), (4, 4000000)])
            write_events(sink, [(1, 1020000), (2, 2050000), (3, 3080000), (4, 4120000)])
            output = root / "out"
            report = analyze(source, sink, 4, "dds_bridge", 1, output)
            self.assertEqual(report["delivery_delay_us_p50"], 65.0)
            self.assertEqual(report["delivery_delay_us_p95_nearest_rank"], 120.0)
            self.assertEqual(report["observed_source_interval_us_mean"], 1000.0)
            self.assertTrue((output / "paired_events.csv").is_file())
            self.assertTrue((output / "summary.json").is_file())
            self.assertFalse(json.loads((output / "summary.json").read_text())["performance_claim"])

    def test_reject_gap_duplicate_reordering(self):
        with tempfile.TemporaryDirectory() as folder:
            file = pathlib.Path(folder) / "events.csv"
            for rows in (
                [(1, 10), (3, 20)],
                [(1, 10), (1, 20)],
                [(2, 10)],
                [(1, 10), (2, 10)],
                [(1, 20), (2, 10)],
            ):
                write_events(file, rows)
                with self.assertRaises(ValueError):
                    read_events(file)

    def test_reject_missing_delivery_negative_delay_and_bad_mode(self):
        with tempfile.TemporaryDirectory() as folder:
            root = pathlib.Path(folder)
            source, sink = root / "source.csv", root / "sink.csv"
            write_events(source, [(1, 100), (2, 200)])
            write_events(sink, [(1, 110)])
            with self.assertRaises(ValueError):
                analyze(source, sink, 2, "shm", 10, root / "out")
            write_events(sink, [(1, 90), (2, 210)])
            with self.assertRaises(ValueError):
                analyze(source, sink, 2, "shm", 10, root / "out")
            write_events(sink, [(1, 120), (2, 220)])
            with self.assertRaises(ValueError):
                analyze(source, sink, 2, "unsupported", 10, root / "out")

    def test_one_frame_has_no_interval_estimate(self):
        with tempfile.TemporaryDirectory() as folder:
            root = pathlib.Path(folder)
            source, sink = root / "source.csv", root / "sink.csv"
            write_events(source, [(1, 100)])
            write_events(sink, [(1, 145)])
            report = analyze(source, sink, 1, "ipc", 10, root / "out")
            self.assertIsNone(report["observed_source_interval_us_mean"])
            self.assertEqual(report["delivery_delay_us_mean"], 0.045)

    def test_nearest_rank(self):
        self.assertEqual(percentile_nearest_rank([5, 3, 4, 1, 2], 0.95), 5)


if __name__ == "__main__":
    unittest.main()
