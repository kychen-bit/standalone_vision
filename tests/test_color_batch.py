"""Round-batch assignment: three blobs, three different colours."""

import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from jetson_recognition.color_batch import assign_distinct, assignment_margin


class AssignDistinctTests(unittest.TestCase):
    def test_bijection_beats_independent_argmax(self):
        # Blob 2 is ambiguous between blue and light blue; blob 1 is clearly
        # green, so light blue must stay with blob 2.
        scores = [
            {"4": 0.9, "6": 0.4},
            {"4": 0.85, "6": 0.86},
        ]
        assignment, total = assign_distinct(scores, ["4", "6"])
        self.assertEqual(assignment, ["4", "6"])
        self.assertAlmostEqual(total, 0.9 + 0.86, places=6)

    def test_independent_labels_would_have_duplicated_a_colour(self):
        scores = [
            {"3": 0.95, "6": 0.90},
            {"3": 0.94, "6": 0.60},
        ]
        assignment, _total = assign_distinct(scores, ["3", "6"])
        self.assertEqual(assignment, ["6", "3"])

    def test_counts_without_a_bijection_are_rejected(self):
        self.assertEqual(assign_distinct([{"1": 1.0}], ["1", "2"]), (None, 0.0))
        self.assertEqual(assign_distinct([], []), (None, 0.0))
        self.assertEqual(
            assign_distinct([{"1": 1.0}, {"1": 1.0}], ["1", "1"]), (None, 0.0)
        )

    def test_ties_resolve_deterministically(self):
        scores = [{"1": 0.5, "2": 0.5}, {"1": 0.5, "2": 0.5}]
        first = assign_distinct(scores, ["1", "2"])
        second = assign_distinct(scores, ["1", "2"])
        self.assertEqual(first, second)
        self.assertEqual(assignment_margin(scores, ["1", "2"]), 0.0)

    def test_margin_is_positive_for_a_clear_assignment(self):
        scores = [{"1": 0.9, "2": 0.1}, {"1": 0.1, "2": 0.9}]
        margin = assignment_margin(scores, ["1", "2"])
        self.assertAlmostEqual(margin, 1.6, places=6)


if __name__ == "__main__":
    unittest.main()
