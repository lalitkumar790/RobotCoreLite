"""Synthetic fixtures test report validation only; NEVER latency evidence."""
import csv
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from summarize_shm_paths import summarize


class ShmPathSummaryTests(unittest.TestCase):
    def write_run(self, root, mode, n=5, corrupt=False, gap=False):
        path = root / f"r1_64_{mode}.csv"
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["mode", "seq", "payload_bytes", "status", "rtt_us"])
            for i in range(n):
                w.writerow([mode, (i + 6 + (1 if gap and i > 2 else 0)),
                            64, "timeout" if corrupt and i == 2 else "ok", i + 1])
        return path

    def test_complete_paired_report(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            self.write_run(root, "copied")
            self.write_run(root, "loaned")
            report = summarize(root).read_text()
            self.assertIn("Copied P99", report)
            self.assertIn("NOT a DDS comparison", report)
            self.assertIn("| 1 | 64 |", report)

    def test_reject_unpaired(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            self.write_run(root, "copied")
            with self.assertRaisesRegex(ValueError, "unpaired"):
                summarize(root)

    def test_reject_noncontiguous(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            self.write_run(root, "copied", gap=True)
            self.write_run(root, "loaned")
            with self.assertRaisesRegex(ValueError, "noncontiguous"):
                summarize(root)

    def test_reject_timeout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            self.write_run(root, "copied", corrupt=True)
            self.write_run(root, "loaned")
            with self.assertRaisesRegex(ValueError, "failed sample"):
                summarize(root)

    def test_reject_unequal_samples(self):
        with tempfile.TemporaryDirectory() as temp:
            root = pathlib.Path(temp)
            self.write_run(root, "copied", n=4)
            self.write_run(root, "loaned", n=5)
            with self.assertRaisesRegex(ValueError, "sample counts differ"):
                summarize(root)


if __name__ == "__main__":
    unittest.main()
