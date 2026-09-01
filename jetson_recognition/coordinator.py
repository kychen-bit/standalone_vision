"""Electronics-driven vision coordinator.

Navigation, obstacle avoidance and gripper motion belong to the electronics
(MCU). This module provides everything recognition-related and the
communication glue:

- receives the task code from MaixCAM Pro (UART A, ``@MAIX_QR`` frames);
- answers MCU requests (``REQ PICK / PLACE / STACK``) with recognition;
- emits ``GRASP_READY`` / ``ALIGN_READY`` with coordinates after the simple
  recognition session becomes stable; navigation and motion remain MCU-owned;
- advances the auto pickup queue after ``DONE OK``;
- reports ``ERROR`` on timeout, bad arguments or giving up after retries.

Frame names and the ``@body*CRC16\\n`` framing are aligned with the protocol in
``new/AUTONOMY_PROTOCOL.md`` so the MCU side only has to implement one framing.
"""

import json
import time

from .task_code import decode_task_code, flat_color_plan


class VisionCoordinator:
    """One MCU-driven recognition session at a time.

    Pure logic: the camera, the two UARTs and the real time source are injected
    by ``run_coordinator`` so this class stays unit-testable without hardware.
    """

    def __init__(self, engine, config):
        self.engine = engine
        self.config = dict(config)
        self.scene = str(config.get("scene", "TURNTABLE")).upper()
        self.max_attempts = max(1, int(config.get("max_attempts", 3)))
        self.timeout_s = float(config.get("timeout_s", 20.0))
        self.action_timeout_s = float(config.get("action_timeout_s", 30.0))
        self.result_valid_ms = max(1, int(config.get("result_valid_ms", 250)))
        self.revoke_after_misses = max(
            1, int(config.get("revoke_after_misses", 2))
        )
        self.require_exec_ack = bool(config.get("require_exec_ack", True))
        self.target_report_every = int(config.get("target_report_every", 0))
        # task queue from the QR payload (auto target for REQ PICK)
        self.queue = []
        self.queue_index = 0
        self.task_raw = None
        # current request
        self.state = "IDLE"
        self.seq = None
        self.kind = None
        self.zone = None
        self.target = None
        self.ring = "1"
        self.attempts = 0
        self.pickup = None
        self.session = None
        self.deadline = None
        self.last_state = None
        self.report_counter = 0
        self.target_from_queue = False
        self.grant_deadline = None
        self.grant_misses = 0
        self.outbox = []

    # -- output -------------------------------------------------------------
    def _emit(self, name, *fields):
        self.outbox.append((name, [str(field) for field in fields]))

    def drain(self):
        frames = self.outbox
        self.outbox = []
        return frames

    def _error(self, seq, reason, *details):
        self._emit("ERROR", seq or "NONE", reason, *details)

    def _clear_plan(self):
        self.queue = []
        self.queue_index = 0
        self.task_raw = None

    # -- task code ----------------------------------------------------------
    def accept_task_code(self, payload):
        """Store the parsed task plan and report it to the MCU."""
        if self.state not in ("WAIT_TASK", "TASK_READY"):
            self._error("NONE", "BAD_STATE", self.state)
            return
        try:
            plan = decode_task_code(payload)
        except ValueError:
            self._error("NONE", "BAD_TASK_CODE")
            return
        self.queue = flat_color_plan(payload)
        self.queue_index = 0
        self.task_raw = plan["raw"]
        self.state = "TASK_READY"
        # Every queue item is an independent protocol field. Embedding commas
        # in one field is forbidden by protocol._field and used to crash the
        # real serial writer even though the coordinator unit test passed.
        self._emit("TASK_PLAN", self.task_raw, *self.queue)

    def _peek_queue_color(self):
        if self.queue_index >= len(self.queue):
            return None
        return self.queue[self.queue_index]

    # -- MCU frames ---------------------------------------------------------
    def on_mcu_frame(self, name, fields):
        name = str(name).upper()
        if name == "START":
            if not fields or not str(fields[0]).strip():
                self._error("NONE", "BAD_START")
                return
            self._reset_request()
            self._clear_plan()
            self.state = "WAIT_TASK"
            self._emit("READY", fields[0])
        elif name == "REQ":
            self._handle_req(fields)
        elif name == "EXEC":
            self._handle_exec(fields)
        elif name == "DONE":
            self._handle_done(fields)
        elif name == "ABORT":
            token = fields[0] if fields else (self.seq or "NONE")
            self._reset_request()
            self._clear_plan()
            self.state = "IDLE"
            self._emit("ABORTED", token)
        # unknown frames are ignored, never acknowledged

    def _handle_req(self, fields):
        if len(fields) < 2:
            self._error(fields[0] if fields else "NONE", "BAD_REQ")
            return
        seq = fields[0]
        kind = fields[1].upper()
        zone = fields[2].upper() if len(fields) > 2 else self.scene
        if self.state != "TASK_READY":
            self._error(seq, "BAD_STATE", self.state)
            return
        target_from_queue = False
        if kind == "PICK":
            target = fields[3] if len(fields) > 3 else None
            if target is None:
                target = self._peek_queue_color()
                if target is None:
                    self._error(seq, "NO_MORE_TARGET")
                    return
                target_from_queue = True
            mode = "COLOR"
            mode_args = (target, zone)
        elif kind == "PLACE":
            if len(fields) < 4:
                self._error(seq, "MISSING_RING")
                return
            target = fields[3]
            mode = "RING"
            mode_args = (target, zone)
        elif kind == "STACK":
            if len(fields) < 4:
                self._error(seq, "MISSING_COLOR")
                return
            target = fields[3]
            self.ring = fields[4] if len(fields) > 4 else "1"
            mode = "STACK"
            mode_args = (target, zone, self.ring)
        else:
            self._error(seq, "BAD_KIND")
            return
        try:
            self.session = self.engine.new_session(mode, mode_args)
        except (KeyError, ValueError) as error:
            self._error(seq, "BAD_REQ", type(error).__name__)
            return
        self.pickup = None
        self.seq = seq
        self.kind = kind
        self.zone = zone
        self.target = target
        self.attempts = 0
        self.deadline = None
        self.last_state = None
        self.report_counter = 0
        self.target_from_queue = target_from_queue
        self.grant_deadline = None
        self.grant_misses = 0
        self.state = "BUSY"
        self._emit("ACCEPTED", seq, kind)

    def _handle_exec(self, fields):
        seq = fields[0] if fields else "NONE"
        if self.state != "WAIT_EXEC":
            self._error(seq, "BAD_STATE", self.state)
            return
        if seq != self.seq:
            self._error(seq, "STALE_SEQ")
            return
        self.state = "WAIT_DONE"
        self.session = None
        self.deadline = None
        self.grant_deadline = None
        self._emit("EXEC_ACK", self.seq)

    def _handle_done(self, fields):
        # ``DONE OK`` means the MCU finished the WHOLE action. For PICK that is
        # "grasped AND placed on the on-board carrier platform", for
        # PLACE/STACK it is "placed on the target ring / stacked" — not merely
        # the grasp. Only then is the target considered complete.
        seq = fields[0] if fields else "NONE"
        if len(fields) < 2:
            self._error(seq, "BAD_DONE")
            return
        if self.state != "WAIT_DONE":
            reason = "EXEC_REQUIRED" if self.state == "WAIT_EXEC" else "BAD_STATE"
            self._error(seq, reason, self.state)
            return
        if seq != self.seq:
            self._error(seq, "STALE_SEQ")
            return
        result = str(fields[1]).upper()
        if result not in ("OK", "FAIL"):
            self._error(seq, "BAD_DONE_RESULT")
            return
        self.deadline = None
        if result == "OK":
            if self.target_from_queue:
                self.queue_index += 1
            self._emit("DONE_ACK", self.seq, "OK")
            self._reset_request()
            self.state = "TASK_READY"
            return
        # FAIL: retry the same target up to max_attempts
        self.attempts += 1
        if self.attempts >= self.max_attempts:
            self._error(self.seq, "GIVE_UP")
            self._reset_request()
            self.state = "TASK_READY"
            return
        self._emit("DONE_ACK", self.seq, "RETRY")
        self.state = "BUSY"
        self.deadline = None
        self.last_state = None
        self.report_counter = 0
        self.grant_deadline = None
        self.grant_misses = 0
        self._restart_session()

    def _restart_session(self):
        if self.kind == "PICK":
            self.session = self.engine.new_session("COLOR", (self.target, self.zone))
        elif self.kind == "PLACE":
            self.session = self.engine.new_session("RING", (self.target, self.zone))
        elif self.kind == "STACK":
            self.session = self.engine.new_session(
                "STACK", (self.target, self.zone, self.ring)
            )

    def _revoke_grasp(self, reason):
        self._emit("GRASP_REVOKED", self.seq, reason)
        self.state = "BUSY"
        self.deadline = None
        self.grant_deadline = None
        self.grant_misses = 0
        self._restart_session()

    def _reset_request(self):
        self.pickup = None
        self.session = None
        self.seq = None
        self.kind = None
        self.zone = None
        self.target = None
        self.attempts = 0
        self.deadline = None
        self.last_state = None
        self.report_counter = 0
        self.target_from_queue = False
        self.grant_deadline = None
        self.grant_misses = 0

    # -- per-frame update ---------------------------------------------------
    def update(self, frame, now_s=None):
        """Feed one camera frame while busy. ``now_s`` is monotonic seconds."""
        if now_s is None:
            now_s = time.monotonic()
        if self.state == "WAIT_DONE":
            if self.deadline is None:
                self.deadline = now_s + self.action_timeout_s
            elif now_s > self.deadline:
                self._error(self.seq, "ACTION_TIMEOUT")
                self._reset_request()
                self.state = "TASK_READY"
            return
        if self.state == "WAIT_EXEC":
            if self.grant_deadline is not None and now_s > self.grant_deadline:
                self._revoke_grasp("EXPIRED")
                return
            self._monitor_grasp(frame)
            return
        if self.state != "BUSY":
            return
        if self.deadline is None:
            self.deadline = now_s + self.timeout_s
        elif now_s > self.deadline:
            self._error(self.seq, "TIMEOUT")
            self._reset_request()
            self.state = "TASK_READY"
            return
        if self.session is not None:
            self._update_session(frame, now_s)

    def _monitor_grasp(self, frame):
        measurement, stable, _count = self.session.update(frame)
        if measurement is None:
            self.grant_misses += 1
            if self.grant_misses >= self.revoke_after_misses:
                self._revoke_grasp("LOST")
            return
        self.grant_misses = 0
        if stable is None:
            self._revoke_grasp("UNSTABLE")

    def _update_session(self, frame, now_s):
        result = self.session.update(frame)
        if result is None:
            return
        measurement, stable, count = result
        if measurement is not None and self.target_report_every > 0:
            self.report_counter += 1
            if self.report_counter % self.target_report_every == 0:
                px = measurement.pixel_x if measurement.pixel_x is not None else measurement.x
                py = measurement.pixel_y if measurement.pixel_y is not None else measurement.y
                self._emit(
                    "TARGET",
                    self.seq,
                    self.kind,
                    self.target,
                    "%.3f" % measurement.x,
                    "%.3f" % measurement.y,
                    measurement.unit,
                    "%.3f" % px,
                    "%.3f" % py,
                    "%.3f" % measurement.confidence,
                    count,
                )
        if measurement is None or stable is None:
            return
        if self.kind == "PICK":
            px = measurement.pixel_x if measurement.pixel_x is not None else measurement.x
            py = measurement.pixel_y if measurement.pixel_y is not None else measurement.y
            self._emit(
                "GRASP_READY",
                self.seq,
                self.target,
                "%.3f" % measurement.x,
                "%.3f" % measurement.y,
                measurement.unit,
                "%.3f" % px,
                "%.3f" % py,
                "%.3f" % measurement.confidence,
                count,
                self.result_valid_ms,
            )
            if self.require_exec_ack:
                self.state = "WAIT_EXEC"
                self.grant_deadline = now_s + self.result_valid_ms / 1000.0
                self.grant_misses = 0
            else:
                self.state = "WAIT_DONE"
                self.session = None
                self.deadline = None
            return
        self._emit(
            "ALIGN_READY",
            self.seq,
            self.kind,
            self.target,
            "%.3f" % measurement.x,
            "%.3f" % measurement.y,
            "%.3f" % measurement.yaw,
            measurement.unit,
            "%.3f" % measurement.confidence,
            count,
        )
        self.state = "WAIT_DONE"
        self.session = None
        self.deadline = None


class FrameReader:
    """Read and CRC-check frames from a pyserial port."""

    def __init__(self, port):
        from .protocol import decode_frame

        self.port = port
        self.buffer = b""
        self._decode = decode_frame

    def read_frames(self):
        chunk = self.port.read(256)
        if chunk:
            self.buffer += chunk
        frames = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            decoded = self._decode(line)
            if decoded is not None:
                frames.append(decoded)
        return frames


def run_coordinator(arguments, config, engine, project_root):
    """IO adapter: Maix UART (in), MCU UART (bidirectional), camera."""
    import serial

    from .camera import UVCCamera
    from .protocol import encode_frame

    coord = config.get("coordinator", {})
    maix_device = arguments.maix_serial or coord.get("maix_serial")
    mcu_device = arguments.mcu_serial or coord.get("mcu_serial")
    baud = int(coord.get("baud", arguments.baud))
    if not maix_device or not mcu_device:
        raise SystemExit(
            "coordinator needs maix_serial and mcu_serial "
            "(config coordinator.maix_serial / mcu_serial or --maix-serial / --mcu-serial)"
        )
    # Both readers are polled in the same camera loop. Blocking timeouts here
    # used to stall recognition by up to 0.4 s per iteration.
    maix = serial.Serial(maix_device, baudrate=baud, timeout=0)
    mcu = serial.Serial(mcu_device, baudrate=baud, timeout=0, write_timeout=0.2)
    camera = UVCCamera(config["camera"], project_root, arguments.camera)
    coordinator = VisionCoordinator(engine, coord)
    maix_reader = FrameReader(maix)
    mcu_reader = FrameReader(mcu)
    print(
        '{"state":"COORDINATOR_READY","maix":"%s","mcu":"%s"}'
        % (maix_device, mcu_device),
        flush=True,
    )
    try:
        while True:
            now_s = time.monotonic()
            for name, fields in maix_reader.read_frames():
                if name == "MAIX_QR" and fields:
                    coordinator.accept_task_code(fields[0])
            for name, fields in mcu_reader.read_frames():
                coordinator.on_mcu_frame(name, fields)
            frame = camera.read()
            if frame is not None:
                coordinator.update(frame, now_s)
            for name, fields in coordinator.drain():
                try:
                    mcu.write(encode_frame(name, *fields))
                except (OSError, UnicodeError, ValueError) as error:
                    print(
                        json.dumps(
                            {
                                "state": "COORDINATOR_TX_ERROR",
                                "frame": name,
                                "error": str(error),
                            }
                        ),
                        flush=True,
                    )
            time.sleep(0.005)
    finally:
        camera.close()
        maix.close()
        mcu.close()
    return 0
