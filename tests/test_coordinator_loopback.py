import unittest

from tools.simulate_color_control_loop import run_simulation
from tools.simulate_full_control_loop import run_simulation as run_full_simulation
from tools.simulate_ring_control_loop import run_simulation as run_ring_simulation


class CoordinatorLoopbackTests(unittest.TestCase):
    def test_complete_two_batch_color_ring_and_stack_loop(self):
        transcript = run_full_simulation(emit=False)
        outbound = [
            item for item in transcript if item["direction"] == "JETSON_TO_MCU"
        ]
        names = [item["name"] for item in outbound]
        self.assertEqual(names.count("TASK_PLAN"), 1)
        self.assertEqual(names.count("ACCEPTED"), 24)
        self.assertEqual(names.count("GRASP_READY"), 12)
        self.assertEqual(names.count("EXEC_ACK"), 12)
        self.assertEqual(names.count("ALIGN_READY"), 12)
        self.assertEqual(names.count("DONE_ACK"), 24)
        stack_results = [
            item
            for item in outbound
            if item["name"] == "ALIGN_READY" and item["fields"][1] == "STACK"
        ]
        self.assertEqual(len(stack_results), 3)
        self.assertEqual(
            [item["fields"][2] for item in stack_results],
            ["5", "1", "6"],
        )

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
        self.assertGreaterEqual(int(grasp["fields"][8]), 8)
        self.assertTrue(grasp["wire"].startswith("@GRASP_READY,"))

    def test_crc_ring_request_alignment_done_loop(self):
        transcript = run_ring_simulation(emit=False)
        outbound = [
            item for item in transcript if item["direction"] == "JETSON_TO_MCU"
        ]
        self.assertEqual(
            [item["name"] for item in outbound],
            ["READY", "TASK_PLAN", "ACCEPTED", "ALIGN_READY", "DONE_ACK"],
        )
        alignment = next(
            item for item in outbound if item["name"] == "ALIGN_READY"
        )
        self.assertEqual(alignment["fields"][0:3], ["RING001", "PLACE", "2"])
        self.assertAlmostEqual(float(alignment["fields"][3]), 1000.0, delta=3.0)


if __name__ == "__main__":
    unittest.main()
