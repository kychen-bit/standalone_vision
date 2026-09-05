import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest import mock


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def load_maix_main():
    fake_maix = types.ModuleType("maix")
    for name in ("app", "camera", "display", "image", "pinmap", "uart"):
        setattr(fake_maix, name, object())
    fake_config = types.ModuleType("config")
    path = PROJECT_ROOT / "maixcam_pro" / "main.py"
    spec = importlib.util.spec_from_file_location("maix_main_for_test", path)
    module = importlib.util.module_from_spec(spec)
    with mock.patch.dict(sys.modules, {"maix": fake_maix, "config": fake_config}):
        spec.loader.exec_module(module)
    return module


class FakeSocket:
    def __init__(self):
        self.fail_send = False
        self.sent = []
        self.closed = False

    def settimeout(self, _timeout):
        pass

    def connect(self, address):
        self.address = address

    def sendall(self, data):
        if self.fail_send:
            raise OSError("stale socket")
        self.sent.append(data)

    def close(self):
        self.closed = True


class MaixTcpReconnectTests(unittest.TestCase):
    def test_idle_heartbeat_detects_stale_socket_and_reconnects(self):
        module = load_maix_main()
        sockets = [FakeSocket(), FakeSocket()]
        clock = [0.0]

        with mock.patch.object(module.socket, "socket", side_effect=sockets), mock.patch.object(
            module.time, "monotonic", side_effect=lambda: clock[0]
        ):
            sender = module.TcpFrameSender(
                "10.33.117.105",
                5000,
                retry_interval_s=1.0,
                heartbeat_interval_s=2.0,
            )
            self.assertTrue(sender._connect())
            self.assertEqual(sender.connection_generation, 1)

            sockets[0].fail_send = True
            clock[0] = 2.1
            self.assertFalse(sender.maintain())
            self.assertIsNone(sender.socket)

            clock[0] = 3.2
            self.assertTrue(sender.maintain())
            self.assertIs(sender.socket, sockets[1])
            self.assertEqual(sender.connection_generation, 2)


if __name__ == "__main__":
    unittest.main()
