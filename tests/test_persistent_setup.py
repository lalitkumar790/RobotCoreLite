import csv
import pathlib
import tempfile
import unittest
from scripts.analyze_persistent_setup import COLD, WARM, parse_samples, report

class PersistentSetupAnalyzerTests(unittest.TestCase):
    def fixture(self, folder, rows):
        path = pathlib.Path(folder) / "setup.csv"
        with path.open("w", newline="") as out:
            writer = csv.writer(out)
            writer.writerow(["iteration", "phase", "capacity_bytes", "duration_us", "epoch"])
            writer.writerows(rows)
        return path

    def test_complete_pairs_and_report(self):
        rows = [[1,COLD,64,100,1],[1,WARM,64,30,2],
                [2,COLD,64,80,1],[2,WARM,64,20,2]]
        with tempfile.TemporaryDirectory() as folder:
            pairs, capacity = parse_samples(self.fixture(folder, rows))
            self.assertEqual(capacity, 64)
            self.assertEqual(len(pairs), 2)
            result = report(pairs, capacity)
            self.assertIn("| Cold create + initial session | 90.000 | 90.000 | 80.000 | 100.000 |", result)
            self.assertIn("not interchangeable transport message paths", result)

    def test_reject_missing_or_duplicate_pair(self):
        with tempfile.TemporaryDirectory() as folder:
            rows = [[1,COLD,64,100,1]]
            with self.assertRaises(ValueError):
                parse_samples(self.fixture(folder, rows))
            rows = [[1,COLD,64,100,1],[1,WARM,64,20,2],[1,WARM,64,22,2]]
            with self.assertRaises(ValueError):
                parse_samples(self.fixture(folder, rows))

    def test_reject_nonfinite_measurement(self):
        with tempfile.TemporaryDirectory() as folder:
            rows = [[1,COLD,64,"nan",1],[1,WARM,64,2,2]]
            with self.assertRaises(ValueError):
                parse_samples(self.fixture(folder, rows))

if __name__ == "__main__":
    unittest.main()
