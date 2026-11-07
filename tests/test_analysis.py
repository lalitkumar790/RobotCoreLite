import csv
import pathlib
import tempfile
import unittest
from scripts.analyze_results import analyze, percentile


class AnalysisTests(unittest.TestCase):
    def test_percentile(self):
        self.assertEqual(percentile([10, 20, 30, 40, 50], 95), 48)
        self.assertIsNone(percentile([], 99))

    def test_latency_and_timeouts(self):
        with tempfile.TemporaryDirectory() as folder:
            path = pathlib.Path(folder) / 'measured.csv'
            with path.open('w', newline='') as out:
                w = csv.writer(out)
                w.writerow(['run_id', 'seq', 'payload_bytes', 'status', 'rtt_us', 'rtt_half_proxy_us'])
                w.writerow(['test', 1, 64, 'ok', 100, 50])
                w.writerow(['test', 2, 64, 'timeout', '', ''])
                w.writerow(['test', 3, 64, 'ok', 200, 100])
            r = analyze(folder)[0]
            self.assertEqual(r['received'], 2)
            self.assertEqual(r['timed_out'], 1)
            self.assertAlmostEqual(r['mean_rtt_us'], 150)
            self.assertAlmostEqual(r['loss_rate'], 1 / 3)


if __name__ == '__main__':
    unittest.main()
