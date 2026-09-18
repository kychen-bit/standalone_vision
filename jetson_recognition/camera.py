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


def apply_capture_profile(camera_config, override=None):
    """Resolve ``camera.capture_profiles`` into width/height/fps.

    Switching the capture mode is the reliable way to change the field of view
    on this camera: the modes are enumerated by the device itself. Measured on
    the DECXIN module, 1280x720 is a 16:9 vertical crop of the 4:3 sensor, so
    1280x960 keeps the horizontal field and adds 33% vertically at the same
    pixel density. The ``zoom_absolute`` control can only ever make the field
    narrower and answers EIO before the first frame, so it is not a substitute.

    Idempotent: the values come from the profile, not from the current size.
    """
    profiles = camera_config.get("capture_profiles") or {}
    name = override or camera_config.get("capture_profile")
    width = int(camera_config.get("width", 1280))
    height = int(camera_config.get("height", 720))
    fps = float(camera_config.get("fps", 30))
    if name:
        if name not in profiles:
            raise ValueError(
                "unknown camera.capture_profile: %s (available: %s)"
                % (name, ", ".join(sorted(profiles)) or "none")
            )
        profile = dict(profiles[name])
        if profile.get("width"):
            width = int(profile["width"])
        if profile.get("height"):
            height = int(profile["height"])
        if profile.get("fps"):
            fps = float(profile["fps"])
    camera_config["width"] = width
    camera_config["height"] = height
    camera_config["fps"] = fps
    reference = str(camera_config.get("roi_reference_frame") or "")
    summary = {
        "state": "CAPTURE_MODE",
        "profile": name,
        "width": width,
        "height": height,
        "fps": fps,
        "roi_reference_frame": reference or None,
        "roi_needs_redraw": bool(
            reference and reference != "%dx%d" % (width, height)
        ),
    }
    if summary["roi_needs_redraw"]:
        summary["warning"] = (
            "scenes.* ROI were drawn for %s; redraw them with tools/pick_roi.py "
            "before trusting the boxes" % reference
        )
    if name:
        print(json.dumps(summary, ensure_ascii=False), flush=True)
    return summary


class UVCCamera:
    def __init__(self, config, project_root, device_override=None):
        # Resolve the capture profile first so everything downstream only ever
        # sees one frame size.
        self.capture_mode = apply_capture_profile(config)
        self.config = dict(config)
        source = config.get("device", 0) if device_override is None else device_override
        if isinstance(source, str) and source.isdigit():
            source = int(source)
        self.source = source
        self.lock_handle = None
        self.control_failures = {}
        self._lock_device(config, source)
        try:
            self.control_failures = self._apply_v4l2_controls(config, source)
        except Exception:
            self._release_device_lock()
            raise
        self.backend_name = str(config.get("backend", "opencv_v4l2")).lower()
        if self.backend_name not in ("opencv_v4l2", "gstreamer_nv"):
            raise ValueError(
                "camera.backend must be opencv_v4l2 or gstreamer_nv"
            )
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
        # Retry the controls that failed before the device had a stream, then
        # decide: a control listed in camera.v4l2_controls_optional only warns,
        # everything else still refuses to start in strict mode.
        if self.control_failures:
            self.control_failures = self._apply_v4l2_controls(
                config, source, names=list(self.control_failures)
            )
        if self.control_failures:
            optional = set(
                config.get("v4l2_controls_optional", ("zoom_absolute",))
            )
            hard = {
                name: error
                for name, error in sorted(self.control_failures.items())
                if name not in optional
            }
            print(
                json.dumps(
                    {
                        "state": "CAMERA_CONTROL_WARNING",
                        "failed": self.control_failures,
                        "optional": sorted(optional),
                    },
                    ensure_ascii=False,
                ),
                flush=True,
            )
            if hard and bool(config.get("v4l2_controls_strict", True)):
                self.capture.release()
                self._release_device_lock()
                raise RuntimeError(
                    "failed to set "
                    + ", ".join(
                        "%s (%s)" % (name, error) for name, error in hard.items()
                    )
                )
        self.map_x = None
        self.map_y = None

        calibration_path = project_root / config.get(
            "calibration_file", "config/calibration.json"
        )
        if calibration_path.exists():
            calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
            requested_size = [int(config["width"]), int(config["height"])]
            calibrated_size = calibration.get("image_size")
            size_mismatch = bool(
                calibrated_size and list(calibrated_size) != requested_size
            )
            matrix = calibration.get("camera_matrix")
            distortion = calibration.get("dist_coeffs")
            if matrix and distortion:
                # The undistortion maps are built for one resolution: refuse to
                # silently apply them to another one.
                if size_mismatch:
                    raise RuntimeError(
                        "calibration resolution does not match camera config"
                    )
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
            elif size_mismatch and any(
                plane.get("calibrated") for plane in calibration.get("planes", {}).values()
            ):
                # Nothing to undistort, but a plane homography was measured for
                # the other resolution, so the PX/MM mapping is now wrong.
                print(
                    json.dumps(
                        {
                            "state": "CALIBRATION_SIZE_MISMATCH",
                            "calibration_image_size": calibrated_size,
                            "active_frame": requested_size,
                            "message": (
                                "plane calibration was measured for another "
                                "capture mode; output in MM will be wrong"
                            ),
                        },
                        ensure_ascii=False,
                    ),
                    flush=True,
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
        # /dev/v4l/by-id/... and /dev/videoN can name the same camera. Use
        # the resolved device path so independently launched tools contend on
        # one lock instead of opening one UVC stream twice and wedging it.
        canonical_source = source
        if isinstance(source, str) and source.startswith("/dev/"):
            canonical_source = os.path.realpath(source)
        name = str(canonical_source).replace("/", "_").strip("_") or "0"
        return Path(
            config.get("lock_file")
            or "/tmp/standalone-vision-camera-%s.lock" % name
        )

    def _lock_device(self, config, source):
        """Refuse a second process opening the same physical camera.

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
            raise RuntimeError(
                "camera %s is already used by another standalone-vision "
                "process; stop standalone-vision.service or the other test "
                "before opening it again" % source
            )
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

    @staticmethod
    def _set_control(device, name, value):
        """Return None on success, else the v4l2-ctl error text."""
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
            return "cannot execute v4l2-ctl: %s" % error
        if completed.returncode != 0:
            return (completed.stderr or completed.stdout).strip().replace("\n", " ")
        return None

    def _apply_v4l2_controls(self, config, source, names=None):
        """Apply the configured controls; return {name: error} for failures.

        Every control is attempted: one unhappy control must not stop the
        others, and the caller decides whether a failure is fatal. The zoom
        extension unit on this DECXIN camera answers EIO until the device has
        produced a frame, which is why the caller retries after opening.
        """
        profile_name = config.get("v4l2_control_profile")
        profiles = config.get("v4l2_control_profiles", {})
        controls = dict(config.get("v4l2_controls", {}))
        if profile_name:
            if profile_name not in profiles:
                raise RuntimeError(
                    "unknown camera.v4l2_control_profile: %s" % profile_name
                )
            profile = dict(profiles[profile_name])
            # Read the replace flag before stripping documentation keys, or a
            # profile that means "ignore the base block" would silently inherit
            # it instead.
            replace_base = bool(profile.get("_replace_base", False))
            # A profile may document itself; those keys are not controls and
            # sending them to v4l2-ctl would abort startup in strict mode.
            for name in list(profile):
                if name == "notice" or name.startswith("_") or name.endswith("_notice"):
                    profile.pop(name)
            # Automatic profiles should be able to reproduce a normal direct
            # camera open without inheriting unrelated manual tuning (for
            # example a stale absolute focus or extreme sharpness value).
            if replace_base:
                controls = {}
            controls.update(profile)
        if not controls:
            return {}
        device = self._v4l2_device(source)
        if platform.system() != "Linux" or device is None:
            message = "V4L2 controls require a Linux /dev/video* device"
            if bool(config.get("v4l2_controls_strict", True)):
                raise RuntimeError(message)
            print(
                json.dumps({"state": "CAMERA_CONTROL_WARNING", "message": message}),
                flush=True,
            )
            return {}

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
        failures = {}
        for name in ordered_names:
            if names is not None and name not in names:
                continue
            value = controls.get(name)
            if value is None:
                continue
            error = self._set_control(device, name, value)
            if error:
                failures[name] = error
        return failures

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
