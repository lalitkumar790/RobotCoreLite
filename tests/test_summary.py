import unittest
from scripts.summarize_comparison import render

def entry(mode, run, payload=128, value=100.0, attempts=10, received=10):
    return {"transport": mode, "run_id": run, "payload_bytes": payload,
            "mean_rtt_us": value, "p95_rtt_us": value * 1.5,
            "attempted": attempts, "received": received}

class SummaryTests(unittest.TestCase):
    def test_four_transports_and_run_distribution(self):
        values = [entry("dds", "d1", value=160), entry("dds", "d2", value=200),
                  entry("dds", "d3", value=180), entry("ipc", "i1", value=100),
                  entry("ipc", "i2", value=140), entry("ipc", "i3", value=120),
                  entry("shared_memory", "s1", value=140), entry("shared_memory", "s2", value=180),
                  entry("shared_memory", "s3", value=160),
                  entry("persistent_shared_memory", "p1", value=140),
                  entry("persistent_shared_memory", "p2", value=180),
                  entry("persistent_shared_memory", "p3", value=160)]
        report = render(values)
        self.assertIn("| 128 | dds | 3 | 30/30 | 0.00% | 180.00 | 160.00–200.00 |", report)
        self.assertIn("| 128 | ipc | 3 | 30/30 | 0.00% | 120.00 | 100.00–140.00 |", report)
        self.assertIn("| 128 | shared_memory | 3 | 30/30 | 0.00% | 160.00 | 140.00–180.00 |", report)
        self.assertIn("| 128 | persistent_shared_memory | 3 | 30/30 | 0.00% | 160.00 | 140.00–180.00 |", report)
        self.assertNotIn("insufficient repetition", report)

    def test_single_run_not_declared_conclusive(self):
        report = render([entry("ipc", "i", received=9)])
        self.assertIn("insufficient repetition", report)
        self.assertIn("10.00%", report)
        self.assertIn("N/A", report)

if __name__ == "__main__":
    unittest.main()
