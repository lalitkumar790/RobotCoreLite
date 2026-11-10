"""Synthetic report fixtures only; NOT performance measurements."""
import csv
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "scripts"))
from summarize_shm_load import summarize


class ShmLoadReportTests(unittest.TestCase):
    def fixture(self, directory, mode, count=4, gap=False, bad_clock=False):
        tag = f"r1_b64_d250_{mode}"
        with (directory / (tag + ".sender.csv")).open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["mode", "seq", "payload_bytes", "source_monotonic_ns",
                             "publish_duration_us", "status"])
            for i in range(1, count + 1):
                writer.writerow([mode, i + (1 if gap and i == 3 else 0),
                                 64, i * 100000, 25, "ok"])
        with (directory / (tag + ".receiver.csv")).open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["mode", "seq", "source_monotonic_ns",
                             "callback_monotonic_ns", "delivery_us", "valid"])
            for i in range(1, count + 1):
                origin = i * 100000
                writer.writerow([mode, i, origin + (1000 if bad_clock and i == 2 else 0),
                                 origin + 25000, 25, "true"])

    def test_valid_complete_pair(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = pathlib.Path(temp)
            for mode in ("copied", "loaned"):
                self.fixture(directory, mode)
            report = summarize(directory, 4).read_text()
            self.assertIn("Publish P99", report)
            self.assertIn("NOT Fast DDS-equivalent", report)

    def test_reject_missing_mode(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = pathlib.Path(temp)
            self.fixture(directory, "copied")
            with self.assertRaisesRegex(ValueError, "unpaired"):
                summarize(directory, 4)

    def test_reject_missing_frames(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = pathlib.Path(temp)
            self.fixture(directory, "copied", count=3)
            self.fixture(directory, "loaned")
            with self.assertRaisesRegex(ValueError, "incomplete"):
                summarize(directory, 4)

    def test_reject_sequence_discontinuity(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = pathlib.Path(temp)
            self.fixture(directory, "copied", gap=True)
            self.fixture(directory, "loaned")
            with self.assertRaisesRegex(ValueError, "invalid sequence"):
                summarize(directory, 4)

    def test_reject_unpaired_source_clock(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = pathlib.Path(temp)
            self.fixture(directory, "copied", bad_clock=True)
            self.fixture(directory, "loaned")
            with self.assertRaisesRegex(ValueError, "clock pairing"):
                summarize(directory, 4)


if __name__ == "__main__":
    unittest.main()
