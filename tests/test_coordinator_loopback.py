import unittest

from tools.simulate_color_control_loop import run_simulation


class CoordinatorLoopbackTests(unittest.TestCase):
    def test_crc_color_request_grant_exec_done_loop(self):
        transcript = run_simulation(emit=False)
        outbound = [
            item for item in transcript if item["direction"] == "JETSON_TO_MCU"
        ]
        self.assertEqual(
            [item["name"] for item in outbound],
            [
                "READY",
                "TASK_PLAN",
                "ACCEPTED",
                "GRASP_READY",
                "EXEC_ACK",
                "DONE_ACK",
            ],
        )
        grasp = next(item for item in outbound if item["name"] == "GRASP_READY")
        self.assertEqual(grasp["fields"][0:2], ["SIM001", "1"])
        self.assertEqual(grasp["fields"][8], "5")
        self.assertTrue(grasp["wire"].startswith("@GRASP_READY,"))


if __name__ == "__main__":
    unittest.main()
