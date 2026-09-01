import unittest

from jetson_recognition.model import Measurement
from jetson_recognition.output import measurement_dict


class OutputTests(unittest.TestCase):
    def test_color_console_result_adds_name_and_center_aliases(self):
        measurement = Measurement(
            "COLOR", "1", 320.0, 240.0, 0.0, 0.9, "PX", 0.0, 320.0, 240.0
        )
        result = measurement_dict(measurement, "STABLE", 3, {"1": "RED"})

        self.assertEqual(result["color"], "RED")
        self.assertEqual(result["center_x"], 320.0)
        self.assertEqual(result["center_y"], 240.0)
        self.assertEqual(result["stable_frames"], 3)


if __name__ == "__main__":
    unittest.main()
