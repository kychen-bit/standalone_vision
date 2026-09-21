"""Electronics-driven vision coordinator.

Navigation, obstacle avoidance and gripper motion belong to the electronics
(MCU). This module provides everything recognition-related and the
communication glue:

- receives the task code from MaixCAM Pro (TCP or UART, ``@MAIX_QR`` frames);
- answers MCU requests (``REQ PICK / PLACE / STACK / LOCATE / CLASSIFY``) with recognition;
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
        # CLASSIFY answers "which colour is in view", so it carries a candidate
        # list instead of a single target colour.
        self.color_candidates = ""
        self.grant_deadline = None
        self.grant_misses = 0
        # Latest per-frame result is retained only for the optional coordinator
        # GUI. It does not participate in protocol decisions.
        self.last_measurement = None
        self.last_stable = None
        self.last_count = 0
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
        payload = str(payload).strip()
        # One START represents one competition run. Once its first valid QR is
        # accepted, a reconnect or a second visible QR must not replace it.
        # ABORT + START is the explicit way to request a fresh scan.
        if self.task_raw is not None:
            return "DUPLICATE" if payload == self.task_raw else "LOCKED"
        if self.state not in ("WAIT_TASK", "TASK_READY"):
            self._error("NONE", "BAD_STATE", self.state)
            return "BAD_STATE"
        try:
            plan = decode_task_code(payload)
        except ValueError:
            self._error("NONE", "BAD_TASK_CODE")
            return "INVALID"
        self.queue = flat_color_plan(payload)
        self.queue_index = 0
        self.task_raw = plan["raw"]
        self.state = "TASK_READY"
        # Every queue item is an independent protocol field. Embedding commas
        # in one field is forbidden by protocol._field and used to crash the
        # real serial writer even though the coordinator unit test passed.
        self._emit("TASK_PLAN", self.task_raw, *self.queue)
        return "ACCEPTED"

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
        elif kind == "LOCATE":
            # Car-body correction in the ring areas: report where the anchor
            # ring is so the MCU can turn it into the car's offset. The scene
            # must be explicit because the ring scenes are not the default
            # TURNTABLE, and the ring defaults to 2 (the slot the car stops in
            # front of).
            if len(fields) < 3:
                self._error(seq, "MISSING_SCENE")
                return
            target = fields[3] if len(fields) > 3 else "2"
            mode = "RING"
            mode_args = (target, zone)
        elif kind == "CLASSIFY":
            # Turntable inspection: report which colour is actually in view so
            # the MCU can decide grasp / probe the next slot / wait for the
            # turntable. Optional trailing fields restrict the candidates to
            # this round's three colours.
            self.color_candidates = ",".join(
                str(value) for value in fields[3:]
            )
            target = "ANY"
            mode = "ANY_COLOR"
            mode_args = (zone, self.color_candidates)
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
        self.last_measurement = None
        self.last_stable = None
        self.last_count = 0
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
        self.last_measurement = None
        self.last_stable = None
        self.last_count = 0
        self._restart_session()

    def _restart_session(self):
        self.last_measurement = None
        self.last_stable = None
        self.last_count = 0
        if self.kind == "PICK":
            self.session = self.engine.new_session("COLOR", (self.target, self.zone))
        elif self.kind in ("PLACE", "LOCATE"):
            self.session = self.engine.new_session("RING", (self.target, self.zone))
        elif self.kind == "CLASSIFY":
            self.session = self.engine.new_session(
                "ANY_COLOR", (self.zone, self.color_candidates)
            )
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
        self.last_measurement = None
        self.last_stable = None
        self.last_count = 0

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
            self._monitor_grasp(frame, now_s)
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

    def _monitor_grasp(self, frame, now_s):
        measurement, stable, count = self.session.update(frame, now_s)
        self.last_measurement = measurement
        self.last_stable = stable
        self.last_count = count
        if measurement is None:
            self.grant_misses += 1
            if self.grant_misses >= self.revoke_after_misses:
                self._revoke_grasp("LOST")
            return
        self.grant_misses = 0
        if stable is None:
            self._revoke_grasp("UNSTABLE")

    def _update_session(self, frame, now_s):
        result = self.session.update(frame, now_s)
        if result is None:
            return
        measurement, stable, count = result
        self.last_measurement = measurement
        self.last_stable = stable
        self.last_count = count
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
        stable_x, stable_y, stable_yaw, stable_confidence, stable_count = stable
        if self.kind == "PICK":
            # Send the stable-window aggregate, not the possibly noisy final
            # frame. With the current PX calibration these are also pixel
            # coordinates; metric calibration keeps the latest raw pixel pair.
            if measurement.unit == "PX":
                px, py = stable_x, stable_y
            else:
                px = (
                    measurement.pixel_x
                    if measurement.pixel_x is not None
                    else measurement.x
                )
                py = (
                    measurement.pixel_y
                    if measurement.pixel_y is not None
                    else measurement.y
                )
            self._emit(
                "GRASP_READY",
                self.seq,
                self.target,
                "%.3f" % stable_x,
                "%.3f" % stable_y,
                measurement.unit,
                "%.3f" % px,
                "%.3f" % py,
                "%.3f" % stable_confidence,
                stable_count,
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
            (measurement.target_id if self.kind == "CLASSIFY" else self.target),
            "%.3f" % stable_x,
            "%.3f" % stable_y,
            "%.3f" % stable_yaw,
            measurement.unit,
            "%.3f" % stable_confidence,
            stable_count,
        )
        self.state = "WAIT_DONE"
        self.session = None
        self.deadline = None


class FrameBuffer:
    """Incrementally split and CRC-check newline-delimited protocol frames."""

    def __init__(self, max_bytes=4096):
        from .protocol import decode_frame

        self.buffer = b""
        self.max_bytes = max(256, int(max_bytes))
        self._decode = decode_frame

    def feed(self, chunk):
        if chunk:
            self.buffer += chunk
        if len(self.buffer) > self.max_bytes and b"\n" not in self.buffer:
            self.buffer = b""
            return []
        frames = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            decoded = self._decode(line)
            if decoded is not None:
                frames.append(decoded)
        return frames


class FrameReader:
    """Read and CRC-check frames from a non-blocking pyserial port."""

    def __init__(self, port):
        self.port = port
        self.decoder = FrameBuffer()

    def read_frames(self):
        return self.decoder.feed(self.port.read(256))


class TcpFrameServer:
    """Non-blocking single-client TCP input for Maix protocol frames."""

    def __init__(self, host, port):
        import socket

        self.host = str(host)
        self.port = int(port)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind((self.host, self.port))
        self.server.listen(1)
        self.server.setblocking(False)
        self.connection = None
        self.peer = None
        self.decoder = FrameBuffer()
        print(
            json.dumps(
                {
                    "state": "MAIX_TCP_LISTENING",
                    "host": self.host,
                    "port": self.port,
                }
            ),
            flush=True,
        )

    def _disconnect(self, reason=None):
        peer = self.peer
        if self.connection is not None:
            try:
                self.connection.close()
            except OSError:
                pass
        self.connection = None
        self.peer = None
        self.decoder = FrameBuffer()
        if peer is not None:
            message = {"state": "MAIX_TCP_DISCONNECTED", "peer": peer[0]}
            if reason:
                message["reason"] = str(reason)
            print(json.dumps(message, ensure_ascii=False), flush=True)

    def _accept_if_ready(self):
        if self.connection is not None:
            return
        try:
            connection, peer = self.server.accept()
        except BlockingIOError:
            return
        connection.setblocking(False)
        self.connection = connection
        self.peer = peer
        self.decoder = FrameBuffer()
        print(
            json.dumps(
                {
                    "state": "MAIX_TCP_CONNECTED",
                    "peer": "%s:%s" % peer,
                }
            ),
            flush=True,
        )

    def read_frames(self):
        self._accept_if_ready()
        if self.connection is None:
            return []
        frames = []
        while True:
            try:
                chunk = self.connection.recv(1024)
            except BlockingIOError:
                break
            except (ConnectionError, OSError) as error:
                self._disconnect(error)
                break
            if not chunk:
                self._disconnect()
                break
            frames.extend(self.decoder.feed(chunk))
        return frames

    def close(self):
        self._disconnect()
        self.server.close()


def run_coordinator(arguments, config, engine, project_root, draw_result_fn=None):
    """IO adapter: Maix TCP/UART input, MCU UART, and camera."""
    import cv2
    import serial

    from .camera import UVCCamera
    from .protocol import encode_frame

    coord = config.get("coordinator", {})
    maix_transport = str(
        getattr(arguments, "maix_transport", None)
        or coord.get("maix_transport", "serial")
    ).lower()
    maix_device = arguments.maix_serial or coord.get("maix_serial")
    maix_host = (
        getattr(arguments, "maix_tcp_host", None)
        or coord.get("maix_tcp_host", "0.0.0.0")
    )
    maix_port = int(
        getattr(arguments, "maix_tcp_port", None)
        or coord.get("maix_tcp_port", 5000)
    )
    mcu_device = arguments.mcu_serial or coord.get("mcu_serial")
    baud = int(coord.get("baud", arguments.baud))
    if maix_transport not in ("tcp", "serial"):
        raise SystemExit("coordinator maix_transport must be tcp or serial")
    if not mcu_device or (maix_transport == "serial" and not maix_device):
        raise SystemExit(
            "coordinator needs mcu_serial and, for serial Maix input, maix_serial"
        )
    # Both readers are polled in the same camera loop. Blocking timeouts here
    # used to stall recognition by up to 0.4 s per iteration.
    if maix_transport == "tcp":
        maix = TcpFrameServer(maix_host, maix_port)
        maix_reader = maix
        maix_label = "%s:%d" % (maix_host, maix_port)
    else:
        maix = serial.Serial(maix_device, baudrate=baud, timeout=0)
        maix_reader = FrameReader(maix)
        maix_label = maix_device
    mcu = serial.Serial(mcu_device, baudrate=baud, timeout=0, write_timeout=0.2)
    camera = UVCCamera(config["camera"], project_root, arguments.camera)
    coordinator = VisionCoordinator(engine, coord)
    mcu_reader = FrameReader(mcu)
    repeat_interval_s = max(
        0.0, float(getattr(arguments, "mcu_repeat_last_frame_seconds", 0.0))
    )
    last_mcu_frame = None
    next_mcu_repeat_s = None
    gui_enabled = bool(getattr(arguments, "gui", False))
    if gui_enabled and draw_result_fn is None:
        raise SystemExit("coordinator GUI drawing function is unavailable")
    if gui_enabled:
        engine.detector.set_color_debug(True)
        engine.detector.set_circle_debug(True)
        cv2.namedWindow("Detection", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    print(
        json.dumps(
            {
                "state": "COORDINATOR_READY",
                "maix_transport": maix_transport,
                "maix": maix_label,
                "mcu": mcu_device,
            }
        ),
        flush=True,
    )
    try:
        while True:
            now_s = time.monotonic()
            # Apply MCU control state first. If START and a queued TCP QR arrive
            # in the same loop, START must establish WAIT_TASK before MAIX_QR.
            for name, fields in mcu_reader.read_frames():
                print(
                    json.dumps(
                        {"state": "MCU_RX", "frame": name, "fields": fields},
                        ensure_ascii=False,
                    ),
                    flush=True,
                )
                coordinator.on_mcu_frame(name, fields)
            for name, fields in maix_reader.read_frames():
                if name == "MAIX_QR" and fields:
                    task_status = coordinator.accept_task_code(fields[0])
                    print(
                        json.dumps(
                            {
                                "state": (
                                    "MAIX_TASK_CODE_ACCEPTED"
                                    if task_status == "ACCEPTED"
                                    else "MAIX_TASK_CODE_IGNORED"
                                ),
                                "reason": task_status,
                                "received": fields[0],
                                "active": coordinator.task_raw,
                                "coordinator_state": coordinator.state,
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
            frame = camera.read()
            if frame is not None:
                coordinator.update(frame, now_s)
                if gui_enabled:
                    if coordinator.kind in ("PICK", "STACK"):
                        color_debug = engine.detector.last_color_debug
                        circle_debug = None
                    elif coordinator.kind == "PLACE":
                        color_debug = None
                        circle_debug = engine.detector.last_circle_debug
                    else:
                        color_debug = None
                        circle_debug = None

                    if coordinator.last_stable is not None:
                        recognition_state = "STABLE"
                    elif coordinator.last_measurement is not None:
                        recognition_state = "UNSTABLE"
                    elif coordinator.state == "BUSY":
                        recognition_state = "LOST"
                    else:
                        recognition_state = "IDLE"
                    display_state = "%s/%s" % (
                        coordinator.state,
                        recognition_state,
                    )
                    canvas = draw_result_fn(
                        frame,
                        coordinator.last_measurement,
                        display_state,
                        coordinator.last_count,
                        color_debug,
                        circle_debug,
                        draw_search_roi=True,
                    )
                    if coordinator.target is not None:
                        cv2.putText(
                            canvas,
                            "request=%s seq=%s target=%s zone=%s"
                            % (
                                coordinator.kind,
                                coordinator.seq,
                                coordinator.target,
                                coordinator.zone,
                            ),
                            (10, 62),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (255, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
                    cv2.imshow("Detection", canvas)

                    mask = None
                    if color_debug is not None:
                        mask = color_debug.get("mask")
                    elif circle_debug is not None:
                        mask = circle_debug.get("processed")
                    if mask is None:
                        mask = frame[:, :, 0] * 0
                    cv2.imshow("Mask", mask)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        print(
                            json.dumps({"state": "COORDINATOR_GUI_STOPPED"}),
                            flush=True,
                        )
                        break
            for name, fields in coordinator.drain():
                try:
                    wire_frame = encode_frame(name, *fields)
                    written = mcu.write(wire_frame)
                    if written != len(wire_frame):
                        raise OSError(
                            "short serial write: %d/%d bytes"
                            % (written, len(wire_frame))
                        )
                    mcu.flush()
                    last_mcu_frame = (wire_frame, name, list(fields))
                    next_mcu_repeat_s = now_s + repeat_interval_s
                    print(
                        json.dumps(
                            {
                                "state": "MCU_TX",
                                "frame": name,
                                "fields": fields,
                                "bytes": written,
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
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
            if (
                repeat_interval_s > 0.0
                and last_mcu_frame is not None
                and next_mcu_repeat_s is not None
                and now_s >= next_mcu_repeat_s
            ):
                wire_frame, name, fields = last_mcu_frame
                try:
                    written = mcu.write(wire_frame)
                    if written != len(wire_frame):
                        raise OSError(
                            "short serial write: %d/%d bytes"
                            % (written, len(wire_frame))
                        )
                    mcu.flush()
                    next_mcu_repeat_s = now_s + repeat_interval_s
                    print(
                        json.dumps(
                            {
                                "state": "MCU_TX_REPEAT",
                                "frame": name,
                                "fields": fields,
                                "bytes": written,
                                "interval_s": repeat_interval_s,
                            },
                            ensure_ascii=False,
                        ),
                        flush=True,
                    )
                except OSError as error:
                    print(
                        json.dumps(
                            {
                                "state": "COORDINATOR_TX_ERROR",
                                "frame": name,
                                "repeat": True,
                                "error": str(error),
                            }
                        ),
                        flush=True,
                    )
            time.sleep(0.005)
    finally:
        if gui_enabled:
            cv2.destroyAllWindows()
        camera.close()
        maix.close()
        mcu.close()
    return 0
