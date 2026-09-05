"""Read-only MaixCAM USB/TCP connection diagnosis for the Jetson."""

import argparse
import socket
import subprocess
from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from tools.configure_maix_network import find_maix_interface


DEFAULT_JETSON_IP = "10.33.117.105"
DEFAULT_MAIX_IP = "10.33.117.1"
DEFAULT_PORT = 5000


def run(command):
    return subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )


def local_address(interface):
    result = run(["ip", "-4", "-o", "address", "show", "dev", interface])
    return result.stdout.strip() if result.returncode == 0 else ""


def established_sessions(port):
    result = run(
        [
            "ss",
            "-H",
            "-nt",
            "state",
            "established",
            "sport",
            "=",
            ":%d" % port,
        ]
    )
    if result.returncode != 0:
        return []
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def local_listener(host, port):
    probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    probe.settimeout(0.5)
    try:
        return probe.connect_ex((host, port)) == 0
    finally:
        probe.close()


def main():
    parser = argparse.ArgumentParser(
        description="只读检查 MaixCAM USB 虚拟网卡和 Jetson TCP 服务"
    )
    parser.add_argument("--jetson-ip", default=DEFAULT_JETSON_IP)
    parser.add_argument("--maix-ip", default=DEFAULT_MAIX_IP)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    arguments = parser.parse_args()

    interface = find_maix_interface()
    if interface is None:
        print("[FAIL] 未发现 MaixCAM RNDIS 虚拟网卡")
        print("[NEXT] 检查 Maix 供电和 USB 数据线，然后重新运行本命令")
        return 1
    print("[OK] MaixCAM 虚拟网卡: %s" % interface)

    address = local_address(interface)
    expected = arguments.jetson_ip + "/"
    if expected not in address:
        print("[FAIL] %s 没有地址 %s/24" % (interface, arguments.jetson_ip))
        print("[NEXT] python3 tools/configure_maix_network.py")
        return 2
    print("[OK] Jetson USB 地址: %s/24" % arguments.jetson_ip)

    ping = run(["ping", "-c", "1", "-W", "1", arguments.maix_ip])
    if ping.returncode != 0:
        print("[FAIL] 无法 ping 通 MaixCAM: %s" % arguments.maix_ip)
        print("[NEXT] 确认 Maix 已启动；必要时重新插拔数据线")
        return 3
    print("[OK] MaixCAM 网络可达: %s" % arguments.maix_ip)

    sessions = established_sessions(arguments.port)
    if sessions:
        print("[OK] Maix TCP 已连接到 Jetson :%d" % arguments.port)
        for session in sessions:
            print("       %s" % session)
        return 0

    if not local_listener(arguments.jetson_ip, arguments.port):
        print("[FAIL] Jetson 没有监听 TCP %s:%d" % (arguments.jetson_ip, arguments.port))
        print(
            "[NEXT] 仅测通信: python3 -m jetson_recognition.run maix-net "
            "--host 0.0.0.0 --port %d" % arguments.port
        )
        print("[NEXT] 完整模拟: python3 tools/run_hardware_flow_test.py")
        return 4

    print("[WARN] Jetson 正在监听 :%d，但 Maix 尚未建立 TCP 会话" % arguments.port)
    print("[NEXT] 在 MaixVision 重新运行 maixcam_pro/main.py")
    print("[NEXT] Maix 端应显示 TCP_CONNECTED；Jetson 端应显示 MAIX_TCP_CONNECTED")
    return 5


if __name__ == "__main__":
    raise SystemExit(main())
