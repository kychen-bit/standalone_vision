import tempfile
import unittest
from pathlib import Path

from tools.configure_maix_network import find_maix_interface


def fake_usb_interface(root, name, driver, vendor="359f", product="2120"):
    class_net = root / "sys" / "class" / "net"
    interface = class_net / name
    usb_device = root / "devices" / (name + "-device")
    usb_interface = usb_device / (name + ":1.0")
    driver_path = root / "drivers" / driver
    interface.mkdir(parents=True)
    usb_interface.mkdir(parents=True)
    driver_path.mkdir(parents=True, exist_ok=True)
    (usb_device / "idVendor").write_text(vendor)
    (usb_device / "idProduct").write_text(product)
    (interface / "device").symlink_to(usb_interface)
    (usb_interface / "driver").symlink_to(driver_path)
    return class_net


class MaixNetworkTests(unittest.TestCase):
    def test_rndis_interface_is_selected_over_second_maix_usb_interface(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            class_net = fake_usb_interface(root, "enx001", "cdc_ether")
            fake_usb_interface(root, "usb0", "rndis_host")
            self.assertEqual(find_maix_interface(class_net), "usb0")

    def test_non_maix_rndis_interface_is_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            class_net = fake_usb_interface(
                root, "usb0", "rndis_host", vendor="1234", product="5678"
            )
            self.assertIsNone(find_maix_interface(class_net))


if __name__ == "__main__":
    unittest.main()
