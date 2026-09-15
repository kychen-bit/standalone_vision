"""OpenCV camera adapter; importing it does not open hardware."""

from collections import deque
import json
import os
from pathlib import Path
import platform
import subprocess
import threading
import time

import cv2
import numpy as np


class UVCCamera:
    def __init__(self, config, project_root, device_override=None):
        self.config = dict(config)
        source = config.get("device", 0) if device_override is None else device_override
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        self.source = source
        self._apply_v4l2_controls(config, source)
        self.backend_name = str(config.get("backend", "opencv_v4l2")).lower()
        if self.backend_name not in ("opencv_v4l2", "gstreamer_nv"):
            raise ValueError(
                "camera.backend must be opencv_v4l2 or gstreamer_nv"
            )
        self.lock_handle = None
        self._lock_device(config, source)
        # A camera that is mid re-enumeration answers an open with EBUSY/EIO.
        # Retrying with a delay instead of exiting immediately avoids turning a
        # 2 second USB hiccup into a service restart loop that re-opens and
        # re-configures the device again and again.
        attempts = max(1, int(config.get("open_retries", 3)))
        retry_delay_s = max(0.0, float(config.get("open_retry_delay_s", 1.0)))
        for attempt in range(attempts):
            self.gstreamer_pipeline = None
            self.capture = self._open_capture(config, source)
            if self.capture.isOpened():
                break
            self.capture.release()
            if attempt + 1 >= attempts:
                raise RuntimeError(self._open_failure_message(config, source))
            print(
                json.dumps(
                    {
                        "state": "CAMERA_OPEN_RETRY",
                        "device": str(source),
                        "attempt": attempt + 1,
                        "attempts": attempts,
                        "message": (
                            "camera did not open; a UVC device that is "
                            "re-enumerating answers open() with an error"
                        ),
                    }
                ),
                flush=True,
            )
            time.sleep(retry_delay_s)
        fourcc = config.get("fourcc", "MJPG")
        if self.backend_name == "opencv_v4l2":
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(config["width"]))
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(config["height"]))
            self.capture.set(cv2.CAP_PROP_FPS, float(config["fps"]))
            self.capture.set(
                cv2.CAP_PROP_BUFFERSIZE, int(config.get("buffer_size", 1))
            )
        self.actual_settings = self._read_actual_settings()
        self.startup_controls = self._read_current_controls(source)
        self._check_actual_settings(config, fourcc)
        self.map_x = None
        self.map_y = None

        calibration_path = project_root / config.get(
            "calibration_file", "config/calibration.json"
        )
        if calibration_path.exists():
            calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
            requested_size = [int(config["width"]), int(config["height"])]
            calibrated_size = calibration.get("image_size")
            if calibrated_size and list(calibrated_size) != requested_size:
                raise RuntimeError("calibration resolution does not match camera config")
            matrix = calibration.get("camera_matrix")
            distortion = calibration.get("dist_coeffs")
            if matrix and distortion:
                camera_matrix = np.asarray(matrix, dtype=np.float64)
                dist_coeffs = np.asarray(distortion, dtype=np.float64)
                self.map_x, self.map_y = cv2.initUndistortRectifyMap(
                    camera_matrix,
                    dist_coeffs,
                    None,
                    camera_matrix,
                    tuple(requested_size),
                    cv2.CV_32FC1,
                )

        # V4L2 may retain more than one decoded frame even when buffer size is
        # requested as one. A producer thread continuously drains the device;
        # consumers receive only the newest frame and never work through a
        # stale backlog. It can be disabled for driver troubleshooting.
        self.threaded = bool(config.get("threaded_capture", False))
        self.read_timeout_s = max(0.01, float(config.get("read_timeout_s", 0.25)))
        self._condition = threading.Condition()
        self._latest_frame = None
        self._latest_sequence = 0
        self._read_sequence = 0
        self._capture_timestamps = deque(maxlen=120)
        self._running = False
        self._reader_thread = None
        if self.threaded:
            self._running = True
            self._reader_thread = threading.Thread(
                target=self._capture_latest,
                name="uvc-latest-frame",
            )
            self._reader_thread.daemon = True
            self._reader_thread.start()

    def _open_capture(self, config, source):
        """Open the configured backend, honouring backend_fallback."""
        if self.backend_name == "gstreamer_nv":
            self.gstreamer_pipeline = self._build_gstreamer_pipeline(config, source)
            capture = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                return capture
            if str(config.get("backend_fallback", "")).lower() != "opencv_v4l2":
                return capture
            capture.release()
            print(
                json.dumps(
                    {
                        "state": "CAMERA_BACKEND_WARNING",
                        "message": (
                            "gstreamer_nv negotiation failed; falling back to "
                            "opencv_v4l2 (capture may remain near 30 FPS)"
                        ),
                    }
                ),
                flush=True,
            )
            self.backend_name = "opencv_v4l2"
            self.gstreamer_pipeline = None
        if platform.system() == "Linux":
            backend = cv2.CAP_V4L2
        elif platform.system() == "Windows":
            backend = cv2.CAP_DSHOW
        else:
            backend = cv2.CAP_ANY
        return cv2.VideoCapture(source, backend)

    def _open_failure_message(self, config, source):
        if self.backend_name == "gstreamer_nv":
            return (
                "cannot open camera %r through gstreamer_nv; "
                "retry with --camera-backend opencv_v4l2" % (source,)
            )
        return "cannot open camera %r" % (source,)

    @staticmethod
    def _lock_path(config, source):
        name = str(source).replace("/", "_").strip("_") or "0"
        return Path(
            config.get("lock_file")
            or "/tmp/standalone-vision-camera-%s.lock" % name
        )

    def _lock_device(self, config, source):
        """Warn (never fail) when a second process opened the same camera.

        Two readers of one UVC device wedge the stream and push the camera into
        the USB re-enumeration loop that produced the 196 GB udev log.
        """
        if not config.get("exclusive_lock", True) or platform.system() != "Linux":
            return
        try:
            import fcntl
        except ImportError:
            return
        path = self._lock_path(config, source)
        try:
            handle = open(path, "w")
        except OSError:
            return
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            print(
                json.dumps(
                    {
                        "state": "CAMERA_BUSY_WARNING",
                        "device": str(source),
                        "message": (
                            "another process already uses this camera; stop it "
                            "(systemd service or a test script) before "
                            "streaming, two readers wedge a UVC device"
                        ),
                    }
                ),
                flush=True,
            )
            return
        handle.seek(0)
        handle.truncate()
        handle.write("%d\n" % os.getpid())
        handle.flush()
        self.lock_handle = handle

    def _release_device_lock(self):
        handle = getattr(self, "lock_handle", None)
        if handle is None:
            return
        self.lock_handle = None
        try:
            import fcntl

            fcntl.flock(handle, fcntl.LOCK_UN)
        except (ImportError, OSError):
            pass
        handle.close()

    @staticmethod
    def _gstreamer_quote(value):
        value = str(value).replace("\\", "\\\\").replace('"', '\\"')
        return '"%s"' % value

    @classmethod
    def _build_gstreamer_pipeline(cls, config, source):
        """Use Jetson hardware only for MJPEG decode and colour conversion."""
        if platform.system() != "Linux":
            raise RuntimeError("gstreamer_nv camera backend requires Linux")
        if "GStreamer:                   YES" not in cv2.getBuildInformation():
            raise RuntimeError("this OpenCV build has no GStreamer support")
        device = cls._v4l2_device(source)
        if device is None:
            raise RuntimeError("gstreamer_nv requires a /dev/video* camera")
        if str(config.get("fourcc", "MJPG")).upper() != "MJPG":
            raise RuntimeError("gstreamer_nv currently supports MJPG input only")
        width = int(config["width"])
        height = int(config["height"])
        fps = int(round(float(config["fps"])))
        return " ".join(
            (
                "v4l2src device=%s io-mode=2" % cls._gstreamer_quote(device),
                "! image/jpeg,width=%d,height=%d,framerate=%d/1"
                % (width, height, fps),
                "! jpegparse",
                "! nvv4l2decoder mjpeg=1 enable-max-performance=1",
                "! nvvidconv",
                "! video/x-raw,format=BGRx",
                "! videoconvert n-threads=4",
                "! video/x-raw,format=BGR",
                "! appsink drop=1 max-buffers=1 sync=0",
            )
        )

    @staticmethod
    def _v4l2_device(source):
        if isinstance(source, int):
            return "/dev/video%d" % source
        if isinstance(source, str) and source.startswith("/dev/"):
            return source
        return None

    def _apply_v4l2_controls(self, config, source):
        profile_name = config.get("v4l2_control_profile")
        profiles = config.get("v4l2_control_profiles", {})
        controls = dict(config.get("v4l2_controls", {}))
        if profile_name:
            if profile_name not in profiles:
                raise RuntimeError(
                    "unknown camera.v4l2_control_profile: %s" % profile_name
                )
            profile = dict(profiles[profile_name])
            # Automatic profiles should be able to reproduce a normal direct
            # camera open without inheriting unrelated manual tuning (for
            # example a stale absolute focus or extreme sharpness value).
            if profile.pop("_replace_base", False):
                controls = {}
            controls.update(profile)
        if not controls:
            return
        strict = bool(config.get("v4l2_controls_strict", True))
        device = self._v4l2_device(source)
        if platform.system() != "Linux" or device is None:
            message = "V4L2 controls require a Linux /dev/video* device"
            if strict:
                raise RuntimeError(message)
            print(
                json.dumps({"state": "CAMERA_CONTROL_WARNING", "message": message}),
                flush=True,
            )
            return

        # Disable automatic algorithms before setting their dependent manual
        # values. Do not rely on JSON object ordering for this safety rule.
        automatic_controls = (
            "white_balance_automatic",
            "auto_exposure",
            "focus_automatic_continuous",
        )
        ordered_names = [name for name in automatic_controls if name in controls]
        ordered_names.extend(
            name for name in controls if name not in automatic_controls
        )
        for name in ordered_names:
            value = controls[name]
            if value is None:
                continue
            command = [
                "v4l2-ctl",
                "-d",
                device,
                "--set-ctrl=%s=%s" % (name, value),
            ]
            try:
                completed = subprocess.run(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    universal_newlines=True,
                )
            except OSError as error:
                message = "cannot execute v4l2-ctl: %s" % error
                if strict:
                    raise RuntimeError(message)
                print(
                    json.dumps(
                        {"state": "CAMERA_CONTROL_WARNING", "message": message}
                    ),
                    flush=True,
                )
                return
            if completed.returncode != 0:
                detail = (completed.stderr or completed.stdout).strip()
                message = "failed to set %s=%s: %s" % (name, value, detail)
                if strict:
                    raise RuntimeError(message)
                print(
                    json.dumps(
                        {"state": "CAMERA_CONTROL_WARNING", "message": message}
                    ),
                    flush=True,
                )

    @staticmethod
    def _fourcc_text(value):
        value = int(value)
        return "".join(chr((value >> (8 * index)) & 0xFF) for index in range(4))

    def _read_current_controls(self, source):
        device = self._v4l2_device(source)
        if platform.system() != "Linux" or device is None:
            return {}
        names = (
            "white_balance_automatic",
            "white_balance_temperature",
            "auto_exposure",
            "exposure_time_absolute",
            "gain",
            "focus_automatic_continuous",
            "focus_absolute",
        )
        try:
            completed = subprocess.run(
                ["v4l2-ctl", "-d", device, "--get-ctrl=" + ",".join(names)],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
            )
        except OSError:
            return {}
        if completed.returncode != 0:
            return {}
        controls = {}
        for line in completed.stdout.splitlines():
            if ":" not in line:
                continue
            name, value = line.split(":", 1)
            first = value.strip().split(" ", 1)[0]
            try:
                controls[name.strip()] = int(first)
            except ValueError:
                continue
        return controls

    def _read_actual_settings(self):
        return {
            "backend": self.backend_name,
            "width": int(round(self.capture.get(cv2.CAP_PROP_FRAME_WIDTH))),
            "height": int(round(self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))),
            "fps": float(self.capture.get(cv2.CAP_PROP_FPS)),
            "fourcc": self._fourcc_text(self.capture.get(cv2.CAP_PROP_FOURCC)),
            "buffer_size": int(round(self.capture.get(cv2.CAP_PROP_BUFFERSIZE))),
        }

    def _check_actual_settings(self, config, requested_fourcc):
        actual = self.actual_settings
        problems = []
        if actual["width"] != int(config["width"]):
            problems.append("width=%s" % actual["width"])
        if actual["height"] != int(config["height"]):
            problems.append("height=%s" % actual["height"])
        actual_fps = actual["fps"]
        if actual_fps > 0 and abs(actual_fps - float(config["fps"])) > 1.0:
            problems.append("fps=%.3f" % actual_fps)
        if (
            self.backend_name == "opencv_v4l2"
            and actual["fourcc"].strip("\x00")
            and actual["fourcc"] != requested_fourcc
        ):
            problems.append("fourcc=%s" % actual["fourcc"])
        if not problems:
            return
        message = "camera did not negotiate requested settings: " + ", ".join(problems)
        if config.get("strict_settings", False):
            self.capture.release()
            raise RuntimeError(message)
        print(json.dumps({"state": "CAMERA_SETTINGS_WARNING", "message": message}), flush=True)

    def _capture_latest(self):
        while self._running:
            ok, frame = self.capture.read()
            if not self._running:
                break
            if not ok or frame is None:
                time.sleep(0.005)
                continue
            with self._condition:
                self._latest_frame = frame
                self._latest_sequence += 1
                self._capture_timestamps.append(time.monotonic())
                self._condition.notify_all()

    def _undistort(self, frame):
        if frame is not None and self.map_x is not None:
            return cv2.remap(frame, self.map_x, self.map_y, cv2.INTER_LINEAR)
        return frame

    def read(self):
        if self.threaded:
            with self._condition:
                ready = self._condition.wait_for(
                    lambda: (
                        self._latest_sequence != self._read_sequence
                        or not self._running
                    ),
                    timeout=self.read_timeout_s,
                )
                if not ready or self._latest_sequence == self._read_sequence:
                    return None
                frame = self._latest_frame
                self._read_sequence = self._latest_sequence
            return self._undistort(frame)
        ok, frame = self.capture.read()
        if not ok or frame is None:
            return None
        with self._condition:
            self._capture_timestamps.append(time.monotonic())
        return self._undistort(frame)

    @property
    def capture_fps(self):
        with self._condition:
            if len(self._capture_timestamps) < 2:
                return 0.0
            elapsed = self._capture_timestamps[-1] - self._capture_timestamps[0]
            if elapsed <= 0:
                return 0.0
            return (len(self._capture_timestamps) - 1) / elapsed

    def close(self):
        if self.threaded:
            self._running = False
            with self._condition:
                self._condition.notify_all()
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.0)
        # Releasing VideoCapture while the producer is inside capture.read()
        # can crash inside OpenCV/uvcvideo. A healthy UVC read returns within
        # one frame, so stop and join first; release only after that attempt.
        self.capture.release()
        if self._reader_thread is not None and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._release_device_lock()
