"""Bind the MaixCAM USB RNDIS interface to Jetson's static IPv4 profile."""

import argparse
from pathlib import Path
import subprocess


MAIX_USB_VENDOR = "359f"
MAIX_USB_PRODUCT = "2120"
DEFAULT_PROFILE = "maixcam-usb-static"
DEFAULT_ADDRESS = "10.33.117.105/24"


def _usb_ids(device_path):
    path = device_path.resolve()
    for parent in (path,) + tuple(path.parents):
        vendor_file = parent / "idVendor"
        product_file = parent / "idProduct"
        if vendor_file.is_file() and product_file.is_file():
            return (
                vendor_file.read_text().strip().lower(),
                product_file.read_text().strip().lower(),
            )
    return None, None


def find_maix_interface(sys_class_net=Path("/sys/class/net")):
    """Return Sipeed's RNDIS interface (the 10.33.117.x network)."""
    for interface in sorted(sys_class_net.iterdir()):
        device_path = interface / "device"
        if not device_path.exists():
            continue
        vendor, product = _usb_ids(device_path)
        driver_path = device_path / "driver"
        driver = driver_path.resolve().name if driver_path.exists() else ""
        if (
            vendor == MAIX_USB_VENDOR
            and product == MAIX_USB_PRODUCT
            and driver == "rndis_host"
        ):
            return interface.name
    return None


def _run(command):
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
    )
    if result.returncode:
        raise RuntimeError(
            "%s failed: %s" % (" ".join(command), result.stdout.strip())
        )
    return result.stdout.strip()


def ensure_maix_network(
    profile=DEFAULT_PROFILE,
    address=DEFAULT_ADDRESS,
    required=True,
):
    """Create/update/activate the static profile on the current Maix NIC."""
    interface = find_maix_interface()
    if interface is None:
        if required:
            raise RuntimeError(
                "MaixCAM USB network interface not found; check board power and data cable"
            )
        return None

    # The coordinator service normally runs without root privileges. When the
    # persistent profile is already correct, do not attempt a privileged write.
    active_profile = _run(
        ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", interface]
    )
    active_addresses = _run(
        ["nmcli", "-g", "IP4.ADDRESS", "device", "show", interface]
    ).splitlines()
    if active_profile == profile and address in active_addresses:
        print(
            "[NETWORK] %s already uses %s via %s"
            % (interface, address, profile),
            flush=True,
        )
        return interface

    profile_exists = subprocess.run(
        ["nmcli", "connection", "show", profile],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0
    if not profile_exists:
        _run(
            [
                "nmcli",
                "connection",
                "add",
                "type",
                "ethernet",
                "ifname",
                interface,
                "con-name",
                profile,
            ]
        )
    _run(
        [
            "nmcli",
            "connection",
            "modify",
            profile,
            "connection.interface-name",
            interface,
            "connection.autoconnect",
            "yes",
            "connection.autoconnect-priority",
            "100",
            "ipv4.method",
            "manual",
            "ipv4.addresses",
            address,
            "ipv4.gateway",
            "",
            "ipv4.dns",
            "",
            "ipv4.never-default",
            "yes",
        ]
    )
    _run(["nmcli", "connection", "up", profile, "ifname", interface])
    print(
        "[NETWORK] %s uses %s via %s" % (interface, address, profile),
        flush=True,
    )
    return interface


def main():
    parser = argparse.ArgumentParser(
        description="assign the fixed Jetson address to the detected MaixCAM USB NIC"
    )
    parser.add_argument("--profile", default=DEFAULT_PROFILE)
    parser.add_argument("--address", default=DEFAULT_ADDRESS)
    parser.add_argument(
        "--optional",
        action="store_true",
        help=(
            "exit 0 when the MaixCAM is unplugged; use this from systemd "
            "ExecStartPre so a missing MaixCAM cannot loop the vision service"
        ),
    )
    arguments = parser.parse_args()
    interface = ensure_maix_network(
        arguments.profile,
        arguments.address,
        required=not arguments.optional,
    )
    if interface is None:
        print(
            "[NETWORK] MaixCAM USB interface absent; skipping network setup "
            "(the vision service keeps the TCP port open and waits)",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
