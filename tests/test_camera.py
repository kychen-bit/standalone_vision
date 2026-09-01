import json
from pathlib import Path
import time
import unittest
from unittest import mock

import cv2
import numpy as np

from jetson_recognition.camera import UVCCamera


ROOT = Path(__file__).resolve().parents[1]


class FakeCapture:
    def __init__(self, source, backend):
        self.properties = {}
        self.released = False
        self.sequence = 0

    def isOpened(self):
        return True

    def set(self, key, value):
        self.properties[key] = value
        return True

    def get(self, key):
        return self.properties.get(key, 0)

    def read(self):
        if self.released:
            return False, None
        self.sequence += 1
        time.sleep(0.001)
        height = int(self.properties.get(cv2.CAP_PROP_FRAME_HEIGHT, 720))
        width = int(self.properties.get(cv2.CAP_PROP_FRAME_WIDTH, 1280))
        return True, np.full((height, width, 3), self.sequence % 255, np.uint8)

    def release(self):
        self.released = True


class ClosedCapture(FakeCapture):
    def isOpened(self):
        return False


class CameraTests(unittest.TestCase):
    def test_jetson_gstreamer_pipeline_requests_mjpg_60_and_drops_old_frames(self):
        config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))["camera"]
        pipeline = UVCCamera._build_gstreamer_pipeline(
            config, "/dev/v4l/by-id/test-camera"
        )
        self.assertIn("image/jpeg,width=1280,height=720,framerate=60/1", pipeline)
        self.assertIn("! nvv4l2decoder mjpeg=1", pipeline)
        self.assertIn("appsink drop=1 max-buffers=1 sync=0", pipeline)

    def test_failed_gstreamer_backend_falls_back_to_opencv(self):
        config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))["camera"]
        config["threaded_capture"] = False
        config["v4l2_controls"] = {}
        config["v4l2_control_profile"] = None
        captures = [ClosedCapture("pipeline", cv2.CAP_GSTREAMER), FakeCapture(0, cv2.CAP_V4L2)]
        with mock.patch(
            "jetson_recognition.camera.cv2.VideoCapture",
            side_effect=captures,
        ):
            camera = UVCCamera(config, ROOT, "0")
            self.assertEqual(camera.backend_name, "opencv_v4l2")
            self.assertEqual(camera.actual_settings["backend"], "opencv_v4l2")
            camera.close()

    def test_threaded_camera_returns_newest_frames_and_closes(self):
        config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))["camera"]
        config["backend"] = "opencv_v4l2"
        config["threaded_capture"] = True
        config["v4l2_controls"] = {}
        config["v4l2_control_profile"] = None
        with mock.patch("jetson_recognition.camera.cv2.VideoCapture", FakeCapture):
            camera = UVCCamera(config, ROOT, "0")
            first = camera.read()
            second = camera.read()
            self.assertEqual(first.shape, (720, 1280, 3))
            self.assertEqual(second.shape, (720, 1280, 3))
            self.assertNotEqual(int(first[0, 0, 0]), int(second[0, 0, 0]))
            self.assertGreater(camera.capture_fps, 0.0)
            camera.close()
            self.assertFalse(camera._reader_thread.is_alive())

    def test_v4l2_automatic_controls_are_applied_before_manual_values(self):
        config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))["camera"]
        config["backend"] = "opencv_v4l2"
        config["threaded_capture"] = False
        config["v4l2_control_profile"] = None
        config["v4l2_controls"] = {
            "exposure_time_absolute": 156,
            "auto_exposure": 1,
            "white_balance_temperature": 4600,
            "white_balance_automatic": 0,
        }
        completed = mock.Mock(returncode=0, stdout="", stderr="")
        with mock.patch("jetson_recognition.camera.subprocess.run", return_value=completed) as run:
            with mock.patch("jetson_recognition.camera.cv2.VideoCapture", FakeCapture):
                camera = UVCCamera(config, ROOT, "/dev/video0")
                camera.close()
        settings = [call.args[0][-1] for call in run.call_args_list]
        self.assertEqual(settings[0], "--set-ctrl=white_balance_automatic=0")
        self.assertEqual(settings[1], "--set-ctrl=auto_exposure=1")
        self.assertLess(
            settings.index("--set-ctrl=auto_exposure=1"),
            settings.index("--set-ctrl=exposure_time_absolute=156"),
        )

    def test_selected_control_profile_overrides_base_manual_values(self):
        config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))["camera"]
        config["backend"] = "opencv_v4l2"
        config["threaded_capture"] = False
        config["v4l2_control_profiles"]["test_manual"] = {
            "white_balance_temperature": 6500,
            "exposure_time_absolute": 50,
            "saturation": 70,
        }
        config["v4l2_control_profile"] = "test_manual"
        completed = mock.Mock(returncode=0, stdout="", stderr="")
        with mock.patch("jetson_recognition.camera.subprocess.run", return_value=completed) as run:
            with mock.patch("jetson_recognition.camera.cv2.VideoCapture", FakeCapture):
                camera = UVCCamera(config, ROOT, "/dev/video0")
                camera.close()

        settings = [call.args[0][-1] for call in run.call_args_list]
        self.assertIn("--set-ctrl=white_balance_temperature=6500", settings)
        self.assertIn("--set-ctrl=exposure_time_absolute=50", settings)
        self.assertIn("--set-ctrl=saturation=70", settings)
        self.assertNotIn("--set-ctrl=white_balance_temperature=4600", settings)

    def test_default_auto_profile_skips_inactive_manual_controls(self):
        config = json.loads((ROOT / "config" / "jetson.json").read_text(encoding="utf-8"))["camera"]
        config["backend"] = "opencv_v4l2"
        config["threaded_capture"] = False
        config["v4l2_control_profile"] = "camera_default_auto"
        completed = mock.Mock(returncode=0, stdout="", stderr="")
        with mock.patch("jetson_recognition.camera.subprocess.run", return_value=completed) as run:
            with mock.patch("jetson_recognition.camera.cv2.VideoCapture", FakeCapture):
                camera = UVCCamera(config, ROOT, "/dev/video0")
                camera.close()

        settings = [call.args[0][-1] for call in run.call_args_list]
        self.assertIn("--set-ctrl=white_balance_automatic=1", settings)
        self.assertIn("--set-ctrl=auto_exposure=3", settings)
        self.assertIn("--set-ctrl=focus_automatic_continuous=1", settings)
        self.assertNotIn("--set-ctrl=white_balance_temperature=4600", settings)
        self.assertNotIn("--set-ctrl=exposure_time_absolute=156", settings)
        self.assertNotIn("--set-ctrl=focus_absolute=414", settings)
        self.assertNotIn("--set-ctrl=sharpness=2000", settings)

if __name__ == "__main__":
    unittest.main()
