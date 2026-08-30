"""Source-contract regression for the deliberately retained ROS1 threshold mismatch."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class TestCentralOccupancyContract(unittest.TestCase):
    def test_effective_threshold_remains_central_threshold(self):
        yaml = (ROOT / "config" / "decision.yaml").read_text(encoding="utf-8")
        xml = (ROOT / "config" / "strategy_tree.xml").read_text(encoding="utf-8")
        strategy = (ROOT / "src" / "strategy_node.cpp").read_text(encoding="utf-8")
        self.assertIn("central_threshold: 20", yaml)
        self.assertIn("occupy_threshold: 30", yaml)
        self.assertIn('threshold="{occupy_threshold}"', xml)
        self.assertIn('declare_parameter<int>("central_threshold", 20)', strategy)
        self.assertIn("++central_count_>=central_threshold_", strategy)
        # The incoming XML threshold port is retained for tree compatibility,
        # but must not replace the ROS1-effective private parameter.
        accumulator = strategy[strategy.index('action("AccumulateCentralOccupiable"'):strategy.index('action("TriggerOnThreshold"')]
        self.assertNotIn('getInput<int>("threshold")', accumulator)


if __name__ == "__main__":
    unittest.main()
