import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tools import start_competition as launcher


class CompetitionLauncherTests(unittest.TestCase):
    def config_file(self, directory):
        path = Path(directory) / 'test config.json'
        path.write_text(json.dumps({
            'camera': {'device': '/dev/video0', 'v4l2_control_profile': 'locked',
                       'v4l2_control_profiles': {'locked': {}}},
            'coordinator': {'mcu_serial': '/dev/serial/by-id/REPLACE_WITH_MCU_UART',
                            'maix_transport': 'tcp', 'baud': 57600}}))
        return path

    def test_cli_paths_override_config_and_no_simulated_mcu(self):
        with tempfile.TemporaryDirectory() as work:
            path = self.config_file(work)
            args = launcher.parser().parse_args(['--config', str(path), '--mcu-serial',
                                                '/dev/ttyUSB2', '--camera', '/dev/video2', '--headless'])
            config, command, devices = launcher.launch_settings(args)
            self.assertEqual(devices, {'mcu_serial': '/dev/ttyUSB2', 'camera': '/dev/video2'})
            self.assertIn('coordinator', command)
            self.assertNotIn('--gui', command)
            self.assertEqual(command[command.index('--config') + 1], str(path))
            self.assertEqual(config['coordinator']['baud'], 57600)

    def test_dry_run_does_not_touch_devices_or_network_or_exec(self):
        with tempfile.TemporaryDirectory() as work:
            path = self.config_file(work)
            with patch.object(launcher.sys, 'argv', ['launcher', '--config', str(path), '--dry-run']), \
                    patch.object(launcher, 'preflight') as preflight, \
                    patch.object(launcher.os, 'execv') as execute:
                self.assertEqual(launcher.main(), 0)
                preflight.assert_not_called()
                execute.assert_not_called()

    def test_unconfigured_mcu_blocks_launch(self):
        with tempfile.TemporaryDirectory() as work:
            path = self.config_file(work)
            args = launcher.parser().parse_args(['--config', str(path), '--headless'])
            config, _, devices = launcher.launch_settings(args)
            errors = launcher.preflight(args, config, devices)
            self.assertTrue(any('mcu_serial is not configured' in error for error in errors))

    def test_check_only_never_executes_coordinator(self):
        with tempfile.TemporaryDirectory() as work:
            path = self.config_file(work)
            with patch.object(launcher.sys, 'argv', ['launcher', '--config', str(path), '--check-only']), \
                    patch.object(launcher, 'preflight', return_value=[]), \
                    patch.object(launcher.os, 'execv') as execute:
                self.assertEqual(launcher.main(), 0)
                execute.assert_not_called()
