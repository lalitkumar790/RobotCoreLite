import csv
import pathlib
import tempfile
import unittest
from scripts.analyze_results import analyze, percentile

class ComparisonAnalyzerTests(unittest.TestCase):
    def test_transport_field_and_timeout_rates(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / "fixture.csv"
            with path.open("w", newline="") as out:
                writer = csv.writer(out)
                writer.writerow(["run_id", "transport", "seq", "payload_bytes",
                                 "status", "rtt_us", "rtt_half_proxy_us"])
                writer.writerow(["fixture", "ipc", 11, 128, "ok", "120.0", "60.0"])
                writer.writerow(["fixture", "ipc", 12, 128, "timeout", "", ""])
                writer.writerow(["fixture", "ipc", 13, 128, "ok", "80.0", "40.0"])
            result = analyze(folder)[0]
            self.assertEqual(result["transport"], "ipc")
            self.assertEqual(result["attempted"], 3)
            self.assertEqual(result["timed_out"], 1)
            self.assertEqual(result["mean_rtt_us"], 100.0)
            self.assertAlmostEqual(result["loss_rate"], 1 / 3)
            self.assertEqual(percentile([80, 120], 95), 118.0)

    def test_empty_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(analyze(folder), [])

if __name__ == "__main__":
    unittest.main()
