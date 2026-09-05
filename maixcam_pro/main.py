"""MaixCAM Pro QR recognition test; Jetson and UART are optional."""

from maix import app, camera, display, image, pinmap, uart

import socket
import time

import config


def is_valid_task_code(payload):
    groups = str(payload).strip().split("+")
    if len(groups) != 4:
        return False
    first_colors, first_positions, second_colors, second_positions = groups
    return (
        len(first_colors) == 3
        and len(second_colors) == 3
        and all(value in "123456" for value in first_colors + second_colors)
        and len(set(first_colors)) == 3
        and len(set(second_colors)) == 3
        and sorted(first_positions) == ["1", "2", "3"]
        and sorted(second_positions) == ["1", "2", "3"]
    )


def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in bytes(data):
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(name, *fields):
    body = ",".join([str(name).upper()] + [str(field) for field in fields]).encode("ascii")
    return b"@" + body + ("*%04X\n" % crc16_ccitt(body)).encode("ascii")


def qr_geometry(qr):
    points = [(float(point[0]), float(point[1])) for point in qr.corners()]
    if len(points) != 4:
        raise ValueError("QR_CORNERS")
    center_x = sum(point[0] for point in points) / 4.0
    center_y = sum(point[1] for point in points) / 4.0
    sides = []
    for index in range(4):
        next_index = (index + 1) % 4
        dx = points[index][0] - points[next_index][0]
        dy = points[index][1] - points[next_index][1]
        sides.append((dx * dx + dy * dy) ** 0.5)
    return center_x, center_y, sum(sides) / 4.0, points


def qr_bounds(qr):
    """Return (x, y, w, h) bounding box of a detected QR, or None on failure."""
    try:
        points = [(float(point[0]), float(point[1])) for point in qr.corners()]
    except Exception:
        return None
    if len(points) != 4:
        return None
    min_x = int(min(point[0] for point in points))
    min_y = int(min(point[1] for point in points))
    max_x = int(max(point[0] for point in points))
    max_y = int(max(point[1] for point in points))
    return min_x, min_y, max_x - min_x, max_y - min_y


def open_serial_if_enabled():
    if not config.SERIAL_ENABLED:
        return None
    pinmap.set_pin_function(config.UART_TX_PIN, "UART1_TX")
    pinmap.set_pin_function(config.UART_RX_PIN, "UART1_RX")
    return uart.UART(config.UART_DEVICE, config.UART_BAUD)


class TcpFrameSender:
    """Small reconnecting TCP client; recognition continues while disconnected."""

    def __init__(
        self,
        host,
        port,
        timeout_s=1.0,
        retry_interval_s=1.0,
        heartbeat_interval_s=2.0,
    ):
        self.address = (str(host), int(port))
        self.timeout_s = float(timeout_s)
        self.retry_interval_s = float(retry_interval_s)
        self.heartbeat_interval_s = float(heartbeat_interval_s)
        self.socket = None
        self.next_retry_s = 0.0
        self.next_heartbeat_s = 0.0
        self.connection_generation = 0

    def close(self):
        if self.socket is not None:
            try:
                self.socket.close()
            except Exception:
                pass
            self.socket = None

    def _connect(self):
        now_s = time.monotonic()
        if now_s < self.next_retry_s:
            return False
        self.close()
        try:
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.settimeout(self.timeout_s)
            connection.connect(self.address)
            self.socket = connection
            self.connection_generation += 1
            self.next_heartbeat_s = now_s + self.heartbeat_interval_s
            print(
                "TCP_CONNECTED host=%s port=%d"
                % (self.address[0], self.address[1])
            )
            return True
        except Exception as error:
            self.close()
            self.next_retry_s = now_s + self.retry_interval_s
            print("TCP_CONNECT_FAILED error=%s" % error)
            return False

    def write(self, data):
        if self.socket is None and not self._connect():
            return False
        try:
            self.socket.sendall(data)
            return True
        except Exception as error:
            print("TCP_SEND_FAILED error=%s" % error)
            self.close()
            self.next_retry_s = time.monotonic() + self.retry_interval_s
            return False

    def maintain(self):
        """Reconnect while idle and use a low-rate frame to detect stale sockets."""
        now_s = time.monotonic()
        if self.socket is None:
            return self._connect()
        if now_s < self.next_heartbeat_s:
            return True
        self.next_heartbeat_s = now_s + self.heartbeat_interval_s
        return self.write(encode_frame("MAIX_HEARTBEAT"))


def open_tcp_if_enabled():
    if not getattr(config, "TCP_ENABLED", False):
        return None
    sender = TcpFrameSender(
        config.TCP_SERVER_HOST,
        config.TCP_SERVER_PORT,
        getattr(config, "TCP_CONNECT_TIMEOUT_S", 1.0),
        getattr(config, "TCP_RETRY_INTERVAL_S", 1.0),
        getattr(config, "TCP_HEARTBEAT_INTERVAL_S", 2.0),
    )
    # Connect once at startup so the Jetson can verify the link before a QR is
    # shown. A failure is non-fatal; write() keeps retrying in the background.
    sender._connect()
    return sender


def main():
    cam = camera.Camera(config.CAMERA_WIDTH, config.CAMERA_HEIGHT, fps=config.CAMERA_FPS)
    detector_mode = getattr(config, "QR_DETECTOR_MODE", "cpu").lower()
    if detector_mode == "hw":
        # 硬件加速，需 MaixPy >= v4.7.9，否则可能静默返回空
        detector = image.QRCodeDetector()
    else:
        detector = None  # 使用 img.find_qrcodes()（CPU，兼容老固件）
    screen = display.Display() if config.DISPLAY_ENABLED else None
    serial_port = open_serial_if_enabled()
    tcp_sender = open_tcp_if_enabled()
    last_payload = ""
    confirmed = 0
    missed = 0
    emitted_payload = ""
    emitted_connection_generation = 0

    print(
        "MAIX_STANDALONE_QR_READY serial=%s tcp=%s tcp_server=%s:%s detector_mode=%s"
        % (
            bool(serial_port),
            bool(tcp_sender),
            getattr(config, "TCP_SERVER_HOST", "-"),
            getattr(config, "TCP_SERVER_PORT", "-"),
            detector_mode,
        )
    )
    frame_number = 0
    log_every = int(getattr(config, "DEBUG_LOG_EVERY_FRAMES", 30))
    while not app.need_exit():
        frame = cam.read()
        if frame is None:
            print("CAMERA_EMPTY")
            continue
        frame_number += 1
        if tcp_sender is not None:
            tcp_sender.maintain()

        valid_qr = None
        invalid_qr = None
        invalid_payload = ""
        first_payload = ""
        qr_count = 0
        qr_codes = detector.detect(frame) if detector is not None else frame.find_qrcodes()
        for qr in qr_codes:
            qr_count += 1
            payload = qr.payload().strip()
            if first_payload == "":
                first_payload = payload
            if is_valid_task_code(payload):
                valid_qr = qr
                break
            invalid_qr = qr
            invalid_payload = payload

        if frame_number % log_every == 0:
            # 实时调试日志：区分"根本没检测到"(qr_count=0) 和
            # "检测到但内容无效"(invalid 非空)
            print(
                "QR_DEBUG frame=%d qr_count=%d first=%r invalid=%r "
                "confirmed=%d missed=%d" % (
                    frame_number,
                    qr_count,
                    first_payload,
                    invalid_payload,
                    confirmed,
                    missed,
                ),
                flush=True,
            )

        if valid_qr is None:
            missed += 1
            if missed >= config.RESET_AFTER_MISSED_FRAMES:
                last_payload = ""
                confirmed = 0
                emitted_payload = ""
            label = "INVALID QR" if invalid_payload else "SEARCHING QR"
            if getattr(config, "DRAW_BOX", True) and invalid_qr is not None:
                box = qr_bounds(invalid_qr)
                if box is not None:
                    frame.draw_rect(box[0], box[1], box[2], box[3], image.COLOR_RED, thickness=3)
            frame.draw_string(10, 10, label, image.COLOR_RED, scale=2)
        else:
            missed = 0
            payload = valid_qr.payload().strip()
            if payload == last_payload:
                confirmed += 1
            else:
                last_payload = payload
                confirmed = 1
                emitted_payload = ""
            center_x, center_y, side, points = qr_geometry(valid_qr)
            min_x = int(min(point[0] for point in points))
            min_y = int(min(point[1] for point in points))
            max_x = int(max(point[0] for point in points))
            max_y = int(max(point[1] for point in points))
            stable = confirmed >= config.CONFIRM_FRAMES
            color = image.COLOR_GREEN if stable else image.Color.from_rgb(255, 255, 0)
            if getattr(config, "DRAW_BOX", True):
                frame.draw_rect(min_x, min_y, max_x - min_x, max_y - min_y, color, thickness=3)
            frame.draw_string(10, 10, "%s %d/%d" % (payload, confirmed, config.CONFIRM_FRAMES), color, scale=2)
            connection_changed = (
                tcp_sender is not None
                and tcp_sender.connection_generation != emitted_connection_generation
            )
            if stable and (payload != emitted_payload or connection_changed):
                if confirmed == config.CONFIRM_FRAMES:
                    print(
                        "QR_STABLE payload=%s x=%.1f y=%.1f side=%.1f" % (
                            payload,
                            center_x,
                            center_y,
                            side,
                        )
                    )
                output_frame = encode_frame(
                    "MAIX_QR",
                    payload,
                    "%.1f" % center_x,
                    "%.1f" % center_y,
                    "%.1f" % side,
                    confirmed,
                    "STABLE",
                )
                delivered = serial_port is None and tcp_sender is None
                if serial_port is not None:
                    try:
                        serial_port.write(output_frame)
                        delivered = True
                    except Exception as error:
                        print("UART_SEND_FAILED error=%s" % error)
                if tcp_sender is not None and tcp_sender.write(output_frame):
                    delivered = True
                if delivered:
                    emitted_payload = payload
                    if tcp_sender is not None:
                        emitted_connection_generation = tcp_sender.connection_generation
                    print("QR_SENT payload=%s" % payload)
        if screen is not None:
            screen.show(frame)

    if tcp_sender is not None:
        tcp_sender.close()


if __name__ == "__main__":
    main()
