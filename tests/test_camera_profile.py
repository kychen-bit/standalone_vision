import unittest

from tools.capture_camera_profile import locked_profile, parse_controls


class CameraProfileTests(unittest.TestCase):
    def test_parse_and_lock_profile(self):
        output = """
white_balance_temperature: 4875
exposure_time_absolute: 100
auto_exposure: 1 (Manual Mode)
focus_absolute: 392
saturation: 60
gain: 3
power_line_frequency: 1
"""
        values = parse_controls(output)
        profile = locked_profile(values)
        self.assertEqual(profile["white_balance_temperature"], 4875)
        self.assertEqual(profile["exposure_time_absolute"], 100)
        self.assertEqual(values["auto_exposure"], 1)
        self.assertEqual(profile["focus_absolute"], 392)
        self.assertEqual(profile["white_balance_automatic"], 0)
        self.assertEqual(profile["auto_exposure"], 1)
        self.assertEqual(profile["focus_automatic_continuous"], 0)
        self.assertTrue(profile["_replace_base"])


class ConvergenceTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        from tools.capture_camera_profile import ALL_CONTROLS, AUTO_CONTROLS
        self.values = {name: 0 for name in ALL_CONTROLS}
        self.values.update(AUTO_CONTROLS)
        self.values.update(white_balance_temperature=4200, exposure_time_absolute=80,
                           focus_absolute=400, saturation=60)
        self.features = {'tiles': np.full((4, 4, 3), 100.0), 'sharpness': 100.0}

    def test_requires_full_window_and_rejects_drifting_controls(self):
        from tools.capture_camera_profile import ConvergenceWindow
        window = ConvergenceWindow(3)
        for index in range(6):
            stable, _ = window.add(index * 0.5, self.values, self.features)
            self.assertFalse(stable)
        self.assertTrue(window.add(3, self.values, self.features)[0])
        changed = dict(self.values, exposure_time_absolute=120)
        self.assertFalse(window.add(3.5, changed, self.features)[0])
        for index in range(8, 14):
            stable, _ = window.add(index * 0.5, changed, self.features)
        self.assertTrue(stable)

    def test_constant_controls_do_not_hide_image_changes_or_missing_frames(self):
        from tools.capture_camera_profile import ConvergenceWindow
        window = ConvergenceWindow(3)
        for index in range(7):
            features = dict(self.features, tiles=self.features['tiles'] + index * 3)
            stable, report = window.add(index * 0.5, self.values, features)
        self.assertFalse(stable)
        self.assertIn('image_changing', report['reasons'])
        stable, report = window.add(6, self.values, self.features)
        self.assertFalse(stable)
        self.assertIn('window_incomplete', report['reasons'])

    def test_focus_hunting_rejected_even_when_brightness_is_constant(self):
        from tools.capture_camera_profile import ConvergenceWindow
        window = ConvergenceWindow(3)
        for index in range(7):
            features = dict(self.features, sharpness=100 + (index % 2) * 60)
            stable, report = window.add(index * 0.5, self.values, features)
        self.assertFalse(stable)
        self.assertIn('focus_or_texture_changing', report['reasons'])

    def test_auto_profile_cannot_inherit_locked_modes_and_does_not_mutate_config(self):
        import copy
        from tools.capture_camera_profile import automatic_config, AUTO_CONTROLS
        config = {'v4l2_control_profile': 'locked', 'v4l2_control_profiles': {
            'camera_default_auto': {'auto_exposure': 1, 'white_balance_temperature': 4200,
                                    'focus_absolute': 750, '_replace_base': False}}}
        original = copy.deepcopy(config)
        result = automatic_config(config)
        auto = result['v4l2_control_profiles']['camera_default_auto']
        for name, value in AUTO_CONTROLS.items():
            self.assertEqual(auto[name], value)
        self.assertNotIn('focus_absolute', auto)
        self.assertNotIn('white_balance_temperature', auto)
        self.assertTrue(auto['_replace_base'])
        self.assertEqual(config, original)

    def test_observation_rejects_disabled_auto_and_times_out_without_frames(self):
        from unittest.mock import MagicMock, patch
        from tools import capture_camera_profile as capture
        args = capture.parser().parse_args(['--headless', '--timeout-seconds', '12'])
        camera = MagicMock()
        camera.read.return_value = object()
        with patch.object(capture, 'read_controls', return_value=dict(self.values, auto_exposure=1)):
            with self.assertRaisesRegex(RuntimeError, 'automatic controls'):
                capture.wait_for_stability(camera, '/dev/video0', args, True)
        camera.read.return_value = None
        with patch.object(capture.time, 'monotonic', side_effect=[0, 0, 1, 5, 6, 13]):
            with self.assertRaisesRegex(RuntimeError, 'timeout'):
                capture.wait_for_stability(camera, '/dev/video0', args, True)

    def test_failure_restores_hardware_and_does_not_save(self):
        import json
        from pathlib import Path
        import tempfile
        from unittest.mock import patch, MagicMock
        from tools import capture_camera_profile as capture
        for failure in ('timeout', 'lock_image_jump'):
            with self.subTest(failure=failure), tempfile.TemporaryDirectory() as work:
                path = Path(work) / 'config.json'
                original = json.dumps({'camera': {'device': '/dev/video0'}})
                path.write_text(original)
                locked = dict(self.values, white_balance_automatic=0, auto_exposure=1,
                              focus_automatic_continuous=0)
                outcomes = RuntimeError('timeout') if failure == 'timeout' else [
                    (self.values, self.features, {}),
                    (locked, dict(self.features, tiles=self.features['tiles'] + 80), {})]
                with patch.object(capture.sys, 'argv', ['capture', '--config', str(path), '--headless']), \
                        patch.object(capture, 'read_controls', return_value=locked), \
                        patch.object(capture, 'UVCCamera', return_value=MagicMock()), \
                        patch.object(capture, 'apply_profile'), \
                        patch.object(capture, 'verify_profile'), \
                        patch.object(capture, 'wait_for_stability', side_effect=outcomes), \
                        patch.object(capture, 'restore_controls') as restore:
                    with self.assertRaises(RuntimeError):
                        capture.main()
                    restore.assert_called_once_with('/dev/video0', locked)
                self.assertEqual(path.read_text(), original)
                self.assertFalse(list(Path(work).glob('*.bak')))

    def test_success_saves_only_after_lock_image_verification(self):
        import json
        from pathlib import Path
        import tempfile
        from unittest.mock import patch, MagicMock
        from tools import capture_camera_profile as capture
        with tempfile.TemporaryDirectory() as work:
            path = Path(work) / 'config.json'
            original = json.dumps({'camera': {'device': '/dev/video0'}})
            path.write_text(original)
            locked = dict(self.values, white_balance_automatic=0, auto_exposure=1,
                          focus_automatic_continuous=0)
            outcomes = [(self.values, self.features, {}), (locked, self.features, {})]
            with patch.object(capture.sys, 'argv', ['capture', '--config', str(path), '--headless',
                                                   '--activate', '--profile', 'test_locked']), \
                    patch.object(capture, 'read_controls', return_value=self.values), \
                    patch.object(capture, 'UVCCamera', return_value=MagicMock()), \
                    patch.object(capture, 'apply_profile'), \
                    patch.object(capture, 'verify_profile'), \
                    patch.object(capture, 'wait_for_stability', side_effect=outcomes), \
                    patch.object(capture, 'restore_controls') as restore:
                self.assertEqual(capture.main(), 0)
                restore.assert_not_called()
            result = json.loads(path.read_text())['camera']
            self.assertEqual(result['v4l2_control_profile'], 'test_locked')
            self.assertEqual(result['v4l2_control_profiles']['test_locked']['auto_exposure'], 1)
            self.assertEqual(next(Path(work).glob('*.bak')).read_text(), original)


if __name__ == "__main__":
    unittest.main()
