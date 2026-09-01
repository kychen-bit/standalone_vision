import unittest

from tools.capture_camera_profile import locked_profile, parse_controls


class CameraProfileTests(unittest.TestCase):
    def test_parse_and_lock_profile(self):
        output = """
white_balance_temperature: 4875
exposure_time_absolute: 100
auto_exposure: 1 (Manual Mode)
focus_absolute: 392
saturation: 60
gain: 3
power_line_frequency: 1
"""
        values = parse_controls(output)
        profile = locked_profile(values)
        self.assertEqual(profile["white_balance_temperature"], 4875)
        self.assertEqual(profile["exposure_time_absolute"], 100)
        self.assertEqual(values["auto_exposure"], 1)
        self.assertEqual(profile["focus_absolute"], 392)
        self.assertEqual(profile["white_balance_automatic"], 0)
        self.assertEqual(profile["auto_exposure"], 1)
        self.assertEqual(profile["focus_automatic_continuous"], 0)
        self.assertTrue(profile["_replace_base"])


if __name__ == "__main__":
    unittest.main()
