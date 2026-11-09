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

    def test_resource_observations_do_not_enter_rtt_analysis(self):
        with tempfile.TemporaryDirectory() as folder:
            resource = pathlib.Path(folder) / "fixture.resources.csv"
            resource.write_text(
                "elapsed_ms,role,pid,start_ticks,cpu_user_seconds,cpu_system_seconds,rss_kib\n"
                "0,responder,100,999,0.0,0.0,2048\n")
            self.assertEqual(analyze(folder), [])

    def test_invalid_payload_is_not_misreported_as_timeout(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / "invalid.csv"
            with path.open("w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["run_id", "transport", "payload_profile", "seq",
                                 "payload_bytes", "status", "rtt_us", "rtt_half_proxy_us"])
                writer.writerow(["bad", "ipc", "laser360", 2, 1440, "invalid_payload", "", ""])
                writer.writerow(["bad", "ipc", "laser360", 3, 1440, "timeout", "", ""])
                writer.writerow(["bad", "ipc", "laser360", 4, 1440, "ok", "100", "50"])
            record = analyze(folder)[0]
            self.assertEqual(record["payload_profile"], "laser360")
            self.assertEqual(record["invalid_payload"], 1)
            self.assertEqual(record["timed_out"], 1)
            self.assertAlmostEqual(record["timeout_rate"], 1 / 3)

    def test_inconsistent_profile_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / "mixed.csv"
            path.write_text(
                "run_id,transport,payload_profile,seq,payload_bytes,status,rtt_us,rtt_half_proxy_us\n"
                "bad,ipc,laser360,1,1440,ok,100,50\n"
                "bad,ipc,uniform,2,1440,ok,100,50\n")
            with self.assertRaises(ValueError):
                analyze(folder)

    def test_empty_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            self.assertEqual(analyze(folder), [])

if __name__ == "__main__":
    unittest.main()
