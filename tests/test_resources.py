import csv
import pathlib
import subprocess
import sys
import tempfile
import unittest

from scripts.profile_processes import read_proc, run
from scripts.summarize_resources import (
    HEADER, read_profile, summarize_directory, summarize_profile,
)


class ProcessResourceTests(unittest.TestCase):
    def test_sample_real_separate_processes_and_report(self):
        with tempfile.TemporaryDirectory() as directory:
            dest = pathlib.Path(directory)
            responder = subprocess.Popen(
                [sys.executable, "-c", "import time; time.sleep(3)"])
            try:
                self.assertIsNotNone(read_proc(responder.pid))
                path = dest / "test_ipc_128_r1.resources.csv"
                status = run(
                    responder.pid, 0,
                    [sys.executable, "-c", "import time; time.sleep(.45)"],
                    path, dest / "initiator.log", 3, 50)
                self.assertEqual(status, 0)
                readings = read_profile(path)
                self.assertGreaterEqual(len(readings["responder"]), 2)
                self.assertGreaterEqual(len(readings["initiator"]), 2)
                summary = summarize_profile(path)
                self.assertEqual({r[1] for r in summary}, {"responder", "initiator"})
                self.assertTrue(all(row[-1] == "sampled" for row in summary))
                self.assertTrue(all(float(row[4]) > 0 for row in summary))
                rows = summarize_directory(dest, dest / "reports")
                self.assertEqual(len(rows), 2)
                self.assertTrue((dest / "reports/resources.csv").exists())
                self.assertIn("descriptive", (dest / "reports/resources.md").read_text())
            finally:
                responder.terminate()
                responder.wait(timeout=5)

    def test_insufficient_sampling_is_not_presented_as_cpu_estimate(self):
        with tempfile.TemporaryDirectory() as directory:
            dest = pathlib.Path(directory)
            file = dest / "test_persistent_shared_memory_64_r1.resources.csv"
            with file.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(HEADER)
                writer.writerow([0, "initiator", 123, 12345, 1, 2, 400])
                writer.writerow([0, "manager", 456, 23456, 1, 1, 900])
                writer.writerow([100, "manager", 456, 23456, 1.01, 1.02, 950])
            rows = summarize_profile(file)
            self.assertEqual(len(rows), 3)
            self.assertEqual({r[1] for r in rows}, {"initiator", "responder", "manager"})
            self.assertEqual(next(r for r in rows if r[1] == "initiator")[-1],
                             "insufficient_samples")
            self.assertEqual(next(r for r in rows if r[1] == "responder")[5], "")
            self.assertEqual(next(r for r in rows if r[1] == "manager")[6], 950)

    def test_reject_pid_reuse_and_decreasing_cpu_counters(self):
        with tempfile.TemporaryDirectory() as directory:
            file = pathlib.Path(directory) / "incorrect.resources.csv"
            with file.open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(HEADER)
                writer.writerow([0, "responder", 123, 999, 2, 3, 400])
                writer.writerow([100, "responder", 123, 1000, 2.1, 3.1, 400])
            with self.assertRaises(ValueError):
                read_profile(file)

    def test_timeout_terminates_initiator(self):
        with tempfile.TemporaryDirectory() as directory:
            dest = pathlib.Path(directory)
            responder = subprocess.Popen(
                [sys.executable, "-c", "import time; time.sleep(4)"])
            try:
                status = run(
                    responder.pid, 0,
                    [sys.executable, "-c", "import time; time.sleep(10)"],
                    dest / "timeout.resources.csv", dest / "timeout.log",
                    1, 40)
                self.assertEqual(status, 124)
            finally:
                responder.terminate()
                responder.wait(timeout=5)


if __name__ == "__main__":
    unittest.main()
