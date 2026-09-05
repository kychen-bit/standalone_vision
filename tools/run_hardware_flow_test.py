"""Interactive Maix TCP + real camera + simulated MCU flow test with GUI.

The script creates a local pseudo-terminal for the MCU side and launches the
real coordinator. No laptop or serial cable is required for this stage.
"""

import argparse
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import time
import tty


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from jetson_recognition.coordinator import FrameBuffer
from jetson_recognition.protocol import encode_frame
from jetson_recognition.run import load_config
from jetson_recognition.task_code import COLOR_NAMES, decode_task_code
from tools.configure_maix_network import ensure_maix_network


def arguments_parser():
    parser = argparse.ArgumentParser(
        description=(
            "launch the real coordinator, receive a real Maix QR over TCP, "
            "and simulate the complete two-batch competition workflow"
        )
    )
    parser.add_argument("--config", default="config/jetson.json")
    parser.add_argument("--camera", help="camera index or /dev/v4l/by-id path")
    parser.add_argument("--host", help="Jetson Maix TCP listen address")
    parser.add_argument("--port", type=int, help="Jetson Maix TCP listen port")
    parser.add_argument("--color-scene", default="TURNTABLE")
    parser.add_argument("--ring-scene", default="ROUGH")
    parser.add_argument(
        "--timeout",
        type=float,
        default=120.0,
        help="seconds to wait for each physical recognition stage",
    )
    parser.add_argument(
        "--auto",
        action="store_true",
        help="do not pause before REQ; targets must already be visible",
    )
    parser.add_argument(
        "--skip-network-config",
        action="store_true",
        help="do not detect the Maix USB NIC or activate its static profile",
    )
    return parser


class SimulatedMCU:
    def __init__(self, fd):
        self.fd = fd
        self.decoder = FrameBuffer()
        flags = fcntl.fcntl(fd, fcntl.F_GETFL)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags | os.O_NONBLOCK)

    def send(self, name, *fields):
        wire = encode_frame(name, *fields)
        os.write(self.fd, wire)
        print(
            "[SIM_MCU -> JETSON] %s" % wire.decode("ascii").strip(),
            flush=True,
        )

    def read(self, timeout_s=0.2):
        readable, _, _ = select.select([self.fd], [], [], max(0.0, timeout_s))
        if not readable:
            return []
        try:
            chunk = os.read(self.fd, 4096)
        except BlockingIOError:
            return []
        frames = self.decoder.feed(chunk)
        for name, fields in frames:
            print(
                "[JETSON -> SIM_MCU] %s %s"
                % (name, json.dumps(fields, ensure_ascii=False)),
                flush=True,
            )
        return frames

    def wait_for(self, expected, timeout_s):
        expected = {str(name).upper() for name in expected}
        deadline = time.monotonic() + float(timeout_s)
        next_notice = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            for name, fields in self.read(min(0.2, deadline - time.monotonic())):
                if name == "ERROR":
                    raise RuntimeError("coordinator error: %s" % fields)
                if name in expected:
                    return name, fields
            if time.monotonic() >= next_notice:
                remaining = max(0, int(deadline - time.monotonic()))
                print(
                    "[TEST] still waiting for %s (%d s remaining)"
                    % ("/".join(sorted(expected)), remaining),
                    flush=True,
                )
                next_notice = time.monotonic() + 5.0
        raise RuntimeError("timeout waiting for %s" % sorted(expected))


def pause(message, automatic):
    print("\n[ACTION] %s" % message, flush=True)
    if not automatic:
        input("准备好后按 Enter：")


def start_coordinator_session(mcu, process, timeout_s=30.0):
    """Retry idempotent START until the child has opened the pseudo-serial."""
    deadline = time.monotonic() + float(timeout_s)
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                "coordinator exited during startup with code %s" % process.returncode
            )
        mcu.send("START", "HW_FLOW")
        retry_deadline = min(deadline, time.monotonic() + 1.0)
        while time.monotonic() < retry_deadline:
            for name, fields in mcu.read(
                min(0.2, retry_deadline - time.monotonic())
            ):
                if name == "READY" and fields == ["HW_FLOW"]:
                    return
    raise RuntimeError("timeout waiting for coordinator READY")


def terminate(process):
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2.0)


def main():
    args = arguments_parser().parse_args()
    config = load_config(args.config)
    coord = config["coordinator"]
    host = args.host or coord.get("maix_tcp_host", "0.0.0.0")
    port = int(args.port or coord.get("maix_tcp_port", 5000))

    if not args.skip_network_config:
        interface = ensure_maix_network(required=False)
        if interface is None:
            print(
                "[NETWORK] Maix USB NIC is not present yet; connect/power the "
                "board before confirming the first ACTION prompt",
                flush=True,
            )

    master_fd, slave_fd = pty.openpty()
    tty.setraw(slave_fd)
    slave_name = os.ttyname(slave_fd)
    command = [
        sys.executable,
        "-m",
        "jetson_recognition.run",
        "--config",
        args.config,
        "coordinator",
        "--maix-transport",
        "tcp",
        "--maix-tcp-host",
        host,
        "--maix-tcp-port",
        str(port),
        "--mcu-serial",
        slave_name,
        "--gui",
    ]
    if args.camera:
        command.extend(["--camera", args.camera])

    print(
        "[TEST] launching coordinator; Maix TCP=%s:%d simulated MCU=%s"
        % (host, port, slave_name),
        flush=True,
    )
    print(
        "[TEST] GUI enabled: Detection + Mask; press q/Esc or Ctrl+C to stop",
        flush=True,
    )
    process = subprocess.Popen(command, cwd=str(PROJECT_ROOT))
    mcu = SimulatedMCU(master_fd)
    try:
        # A START written before pyserial opens the PTY can be discarded during
        # serial initialization, so retry this idempotent startup handshake.
        start_coordinator_session(mcu, process)

        pause(
            "在 MaixVision 运行 maixcam_pro/main.py，并展示任务二维码",
            args.auto,
        )
        if not args.skip_network_config:
            ensure_maix_network(required=True)
        print(
            "[TEST] waiting for MAIX_QR/TASK_PLAN (%.0f s); Maix should show "
            "TCP_CONNECTED and QR_SENT" % args.timeout,
            flush=True,
        )
        _name, task_fields = mcu.wait_for(("TASK_PLAN",), args.timeout)
        task_code = task_fields[0]
        task = decode_task_code(task_code)
        first_batch = task["batches"][0]
        second_batch = task["batches"][1]
        storage_ring_by_color = {
            item["color_id"]: item["ring_id"] for item in first_batch
        }
        print(
            "[TEST] TASK_CODE_LOCKED=%s" % task_code,
            flush=True,
        )
        for batch_number, batch in enumerate((first_batch, second_batch), 1):
            summary = "  ".join(
                "%s(%s)->环%s"
                % (COLOR_NAMES[item["color_id"]], item["color_id"], item["ring_id"])
                for item in batch
            )
            print("[PLAN] 第%d批 %s" % (batch_number, summary), flush=True)

        completed = []

        def perform_pick(seq, item, scene, use_task_queue, stage):
            color_id = item["color_id"]
            pause(
                "%s：让颜色 %s(%s) 出现在 %s 识别区"
                % (stage, COLOR_NAMES[color_id], color_id, scene),
                args.auto,
            )
            fields = [seq, "PICK", scene]
            if not use_task_queue:
                fields.append(color_id)
            mcu.send("REQ", *fields)
            mcu.wait_for(("ACCEPTED",), 5.0)
            _name, grasp = mcu.wait_for(("GRASP_READY",), args.timeout)
            if grasp[1] != color_id:
                raise RuntimeError("wrong color result: %s" % grasp)
            # Reply immediately while the short-lived visual grant is valid.
            mcu.send("EXEC", seq)
            mcu.wait_for(("EXEC_ACK",), 5.0)
            mcu.send("DONE", seq, "OK")
            mcu.wait_for(("DONE_ACK",), 5.0)
            completed.append((seq, "PICK", scene, color_id))

        def perform_place(seq, item, scene, stage):
            ring_id = item["ring_id"]
            pause(
                "%s：让数字 %s 圆环出现在 %s 识别区"
                % (stage, ring_id, scene),
                args.auto,
            )
            mcu.send("REQ", seq, "PLACE", scene, ring_id)
            mcu.wait_for(("ACCEPTED",), 5.0)
            _name, alignment = mcu.wait_for(("ALIGN_READY",), args.timeout)
            if alignment[1] != "PLACE" or alignment[2] != ring_id:
                raise RuntimeError("wrong ring result: %s" % alignment)
            mcu.send("DONE", seq, "OK")
            mcu.wait_for(("DONE_ACK",), 5.0)
            completed.append((seq, "PLACE", scene, ring_id))

        def perform_stack(seq, item, storage_ring_id, stage):
            color_id = item["color_id"]
            pause(
                "%s：在 STORAGE 大ROI内放置第一批同色物料 %s(%s)；任务环号=%s"
                % (
                    stage,
                    COLOR_NAMES[color_id],
                    color_id,
                    storage_ring_id,
                ),
                args.auto,
            )
            mcu.send(
                "REQ", seq, "STACK", "STORAGE", color_id, storage_ring_id
            )
            mcu.wait_for(("ACCEPTED",), 5.0)
            _name, alignment = mcu.wait_for(("ALIGN_READY",), args.timeout)
            if alignment[1] != "STACK" or alignment[2] != color_id:
                raise RuntimeError("wrong stack result: %s" % alignment)
            mcu.send("DONE", seq, "OK")
            mcu.wait_for(("DONE_ACK",), 5.0)
            completed.append(
                (seq, "STACK", "STORAGE", color_id, storage_ring_id)
            )

        # 第一批：原料区抓三件，再依次放粗加工区、从粗加工区取回、
        # 最后平放到暂存区。
        for index, item in enumerate(first_batch, 1):
            perform_pick(
                "B1_TP_PICK_%d" % index,
                item,
                args.color_scene.upper(),
                True,
                "第一批%d/3 原料区抓取" % index,
            )
        for index, item in enumerate(first_batch, 1):
            perform_place(
                "B1_ROUGH_PLACE_%d" % index,
                item,
                args.ring_scene.upper(),
                "第一批%d/3 粗加工区放置" % index,
            )
        for index, item in enumerate(first_batch, 1):
            perform_pick(
                "B1_ROUGH_PICK_%d" % index,
                item,
                args.ring_scene.upper(),
                False,
                "第一批%d/3 粗加工区取回" % index,
            )
        for index, item in enumerate(first_batch, 1):
            perform_place(
                "B1_STORAGE_PLACE_%d" % index,
                item,
                "STORAGE",
                "第一批%d/3 暂存区平放" % index,
            )

        # 第二批：同样经过原料区和粗加工区，最后按第一批的同色物料
        # 中心进行码垛，不按第二批粗加工环号寻找暂存目标。
        for index, item in enumerate(second_batch, 1):
            perform_pick(
                "B2_TP_PICK_%d" % index,
                item,
                args.color_scene.upper(),
                True,
                "第二批%d/3 原料区抓取" % index,
            )
        for index, item in enumerate(second_batch, 1):
            perform_place(
                "B2_ROUGH_PLACE_%d" % index,
                item,
                args.ring_scene.upper(),
                "第二批%d/3 粗加工区放置" % index,
            )
        for index, item in enumerate(second_batch, 1):
            perform_pick(
                "B2_ROUGH_PICK_%d" % index,
                item,
                args.ring_scene.upper(),
                False,
                "第二批%d/3 粗加工区取回" % index,
            )
        for index, item in enumerate(second_batch, 1):
            perform_stack(
                "B2_STORAGE_STACK_%d" % index,
                item,
                storage_ring_by_color[item["color_id"]],
                "第二批%d/3 暂存区同色码垛" % index,
            )

        print(
            json.dumps(
                {
                    "state": "FULL_HARDWARE_FLOW_OK",
                    "task_code": task_code,
                    "materials": 6,
                    "vision_actions": len(completed),
                    "stack_actions": sum(
                        action[1] == "STACK" for action in completed
                    ),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        return 0
    except KeyboardInterrupt:
        print("\n[TEST] stopped by user", flush=True)
        return 130
    except (OSError, RuntimeError, ValueError) as error:
        print(
            json.dumps(
                {"state": "HARDWARE_FLOW_FAILED", "error": str(error)},
                ensure_ascii=False,
            ),
            flush=True,
        )
        return 1
    finally:
        terminate(process)
        os.close(slave_fd)
        os.close(master_fd)


if __name__ == "__main__":
    raise SystemExit(main())
