import csv
import json
import os
import pathlib
import tempfile
import time
import unittest

from scripts.sample_sensor_resources import (
    ResourceSampler, read_roles, summarize_samples,
)


class SensorResourceSamplerTests(unittest.TestCase):
    def test_real_linux_process_samples_and_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            root = pathlib.Path(directory)
            with (root / "process_ids.csv").open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["role", "pid"])
                writer.writerows([
                    ["source", os.getpid()],
                    ["bridge", os.getpid()],
                    ["sink", os.getpid()],
                ])
            sampler = ResourceSampler(root, "ipc", interval_ms=30)
            sampler.start()
            time.sleep(.16)
            report = sampler.stop()
            self.assertFalse(report["performance_claim"])
            self.assertEqual(set(report["process_roles"]), {"source", "bridge", "sink"})
            self.assertGreaterEqual(report["process_roles"]["source"]["samples"], 2)
            self.assertEqual(report["process_roles"]["source"]["status"], "sampled")
            self.assertGreater(report["process_roles"]["sink"]["sampled_max_rss_kib"], 0)
            raw = root / "resource_samples.csv"
            with raw.open() as f:
                self.assertGreater(len(list(csv.DictReader(f))), 3)
            self.assertIn("not exact peak", " ".join(report["warnings"]))
            self.assertEqual(json.loads((root / "resource_summary.json").read_text()),
                             report)

    def test_missing_or_short_processes_do_not_invent_cpu_usage(self):
        samples = [{
            "role": "source", "elapsed_ms": 1.0, "pid": 123,
            "start_ticks": 45, "cpu_user_seconds": .01,
            "cpu_system_seconds": .01, "rss_kib": 333,
        }]
        result = summarize_samples(samples, ("source", "bridge", "sink"))
        self.assertEqual(result["source"]["sampled_max_rss_kib"], 333)
        self.assertIsNone(result["source"]["cpu_core_equivalents"])
        self.assertIsNone(result["sink"]["sampled_max_rss_kib"])
        self.assertEqual(result["bridge"]["status"], "insufficient_samples")

    def test_cpu_core_equivalents_observed_window_only(self):
        original = {
            "role": "source", "elapsed_ms": 100.0, "pid": 12, "start_ticks": 100,
            "cpu_user_seconds": 1.0, "cpu_system_seconds": .1, "rss_kib": 100,
        }
        end = dict(original, elapsed_ms=1100.0, cpu_user_seconds=1.5,
                   cpu_system_seconds=.2, rss_kib=200)
        value = summarize_samples([original, end], ("source",))
        self.assertAlmostEqual(value["source"]["cpu_core_equivalents"], .6)
        self.assertEqual(value["source"]["sampled_max_rss_kib"], 200)
        with self.assertRaises(ValueError):
            summarize_samples([original, dict(end, start_ticks=101)], ("source",))
        with self.assertRaises(ValueError):
            summarize_samples([original, dict(end, cpu_user_seconds=.1)], ("source",))

    def test_reject_duplicate_pid_roles(self):
        with tempfile.TemporaryDirectory() as directory:
            file = pathlib.Path(directory) / "process_ids.csv"
            file.write_text("role,pid\nsource,100\nsource,101\n")
            with self.assertRaises(ValueError):
                read_roles(file)
            file.write_text("role,pid\nsource,0\n")
            with self.assertRaises(ValueError):
                read_roles(file)
            file.write_text("")
            self.assertEqual(read_roles(file), {})

    def test_invalid_sampling_arguments(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(ValueError):
                ResourceSampler(directory, "dds", 100)
            with self.assertRaises(ValueError):
                ResourceSampler(directory, "ipc", 1)


if __name__ == "__main__":
    unittest.main()
