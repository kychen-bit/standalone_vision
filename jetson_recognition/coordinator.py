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
        self.outbox = []

    # -- output -------------------------------------------------------------
    def _emit(self, name, *fields):
        self.outbox.append((name, [str(field) for field in fields]))

    def drain(self):
        frames = self.outbox
        self.outbox = []
        return frames

    # -- task code ----------------------------------------------------------
    def accept_task_code(self, payload):
        """Store the parsed task plan and report it to the MCU."""
        try:
            plan = decode_task_code(payload)
        except ValueError:
            self._emit("ERROR", "", "BAD_TASK_CODE")
            return
        self.queue = flat_color_plan(payload)
        self.queue_index = 0
        self.task_raw = plan["raw"]
        if self.state == "WAIT_TASK":
            self.state = "TASK_READY"
        self._emit("TASK_PLAN", self.task_raw, ",".join(self.queue))

    def _next_queue_color(self):
        if self.queue_index >= len(self.queue):
            return None
        color = self.queue[self.queue_index]
        self.queue_index += 1
        return color

    # -- MCU frames ---------------------------------------------------------
    def on_mcu_frame(self, name, fields):
        name = str(name).upper()
        if name == "START":
            self.state = "WAIT_TASK"
            self._emit("READY", fields[0] if fields else "")
        elif name == "REQ":
            self._handle_req(fields)
        elif name == "DONE":
            self._handle_done(fields)
        elif name == "ABORT":
            self._reset_request()
            self.state = "IDLE"
        # unknown frames are ignored, never acknowledged

    def _handle_req(self, fields):
        if len(fields) < 2:
            self._emit("ERROR", "", "BAD_REQ")
            return
        seq = fields[0]
        kind = fields[1].upper()
        zone = fields[2].upper() if len(fields) > 2 else self.scene
        if self.state == "BUSY":
            self._emit("ERROR", seq, "BUSY")
            return
        if kind == "PICK":
            target = fields[3] if len(fields) > 3 else None
            if target is None:
                target = self._next_queue_color()
                if target is None:
                    self._emit("ERROR", seq, "NO_MORE_TARGET")
                    return
            self.session = self.engine.new_session("COLOR", (target, zone))
            self.pickup = None
        elif kind == "PLACE":
            if len(fields) < 4:
                self._emit("ERROR", seq, "MISSING_RING")
                return
            target = fields[3]
            self.session = self.engine.new_session("RING", (target, zone))
            self.pickup = None
        elif kind == "STACK":
            if len(fields) < 4:
                self._emit("ERROR", seq, "MISSING_COLOR")
                return
            target = fields[3]
            self.ring = fields[4] if len(fields) > 4 else "1"
            self.session = self.engine.new_session(
                "STACK", (target, zone, self.ring)
            )
            self.pickup = None
        else:
            self._emit("ERROR", seq, "BAD_KIND")
            return
        self.seq = seq
        self.kind = kind
        self.zone = zone
        self.target = target
        self.attempts = 0
        self.deadline = None
        self.last_state = None
        self.report_counter = 0
        self.state = "BUSY"
        self._emit("ACCEPTED", seq, kind)

    def _handle_done(self, fields):
        # ``DONE OK`` means the MCU finished the WHOLE action. For PICK that is
        # "grasped AND placed on the on-board carrier platform", for
        # PLACE/STACK it is "placed on the target ring / stacked" — not merely
        # the grasp. Only then is the target considered complete.
        if len(fields) < 2 or self.state != "WAIT_DONE" or fields[0] != self.seq:
            return  # stale or unknown request: ignore
        result = str(fields[1]).upper()
        self.state = "TASK_READY"
        self.deadline = None
        if result == "OK":
            self.pickup = None
            self.session = None
            self._emit("DONE_ACK", self.seq, "OK")
            return
        # FAIL: retry the same target up to max_attempts
        self.attempts += 1
        if self.attempts >= self.max_attempts:
            self.pickup = None
            self.session = None
            self._emit("ERROR", self.seq, "GIVE_UP")
            return
        self._emit("DONE_ACK", self.seq, "RETRY")
        self.state = "BUSY"
        self.deadline = None
        self.last_state = None
        self.report_counter = 0
        if self.kind == "PICK":
            self.session = self.engine.new_session(
                "COLOR", (self.target, self.zone)
            )
        elif self.kind == "PLACE":
            self.session = self.engine.new_session("RING", (self.target, self.zone))
        elif self.kind == "STACK":
            self.session = self.engine.new_session(
                "STACK", (self.target, self.zone, self.ring)
            )

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

    # -- per-frame update ---------------------------------------------------
    def update(self, frame, now_s=None):
        """Feed one camera frame while busy. ``now_s`` is monotonic seconds."""
        if now_s is None:
            now_s = time.monotonic()
        if self.state != "BUSY":
            return
        if self.deadline is None:
            self.deadline = now_s + self.timeout_s
        elif now_s > self.deadline:
            self._emit("ERROR", self.seq, "TIMEOUT")
            self._reset_request()
            self.state = "TASK_READY"
            return
        if self.session is not None:
            self._update_session(frame)

    def _update_session(self, frame):
        result = self.session.update(frame)
        if result is None:
            return
        measurement, stable, _count = result
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
            )
            self.state = "WAIT_DONE"
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
        )
        self.state = "WAIT_DONE"


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
    maix = serial.Serial(maix_device, baudrate=baud, timeout=0.2)
    mcu = serial.Serial(mcu_device, baudrate=baud, timeout=0.2)
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
                mcu.write(encode_frame(name, *fields))
            time.sleep(0.005)
    finally:
        camera.close()
        maix.close()
        mcu.close()
    return 0
