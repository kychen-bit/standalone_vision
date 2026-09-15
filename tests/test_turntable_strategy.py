import unittest

from jetson_recognition.turntable_strategy import TurntablePickStrategy


class TurntablePickStrategyTests(unittest.TestCase):
    def test_pick_first_matching_material(self):
        strategy = TurntablePickStrategy("3", ["1", "3", "5"])
        self.assertEqual(strategy.observe("3")["action"], "PICK_CURRENT")

    def test_move_then_pick_second_material(self):
        strategy = TurntablePickStrategy("3", ["1", "3", "5"])
        self.assertEqual(
            strategy.observe("1")["action"], "MOVE_NEXT_AND_CLASSIFY"
        )
        self.assertEqual(strategy.observe("3")["action"], "PICK_CURRENT")

    def test_infer_target_after_two_non_matches(self):
        strategy = TurntablePickStrategy("3", ["1", "3", "5"])
        strategy.observe("1")
        result = strategy.observe("5")
        self.assertEqual(result["action"], "RETURN_START_AND_WAIT_TARGET")
        self.assertEqual(result["inferred_color"], "3")

    def test_invalid_round_is_rejected(self):
        with self.assertRaises(ValueError):
            TurntablePickStrategy("1", ["1", "1", "2"])
