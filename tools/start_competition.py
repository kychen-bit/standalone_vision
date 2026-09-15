"""Preflight and launch the real MCU coordinator (no simulated protocol traffic)."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import shlex
import shutil
import stat
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# The 2026 incident: /lib/udev/uvcdynctrl appended ~2.3 KB per video4linux add
# event to this file until it reached 196 GB and filled the system disk.
UVCDYNCTRL_LOG = Path('/var/log/uvcdynctrl-udev.log')


def parser():
    root = argparse.ArgumentParser(description=__doc__)
    root.add_argument('--config', default='config/jetson.json')
    root.add_argument('--mcu-serial', help='real MCU /dev/serial/by-id/...; overrides config')
    root.add_argument('--camera', help='camera device path; overrides config')
    root.add_argument('--headless', action='store_true', help='disable Detection/Mask windows')
    root.add_argument('--skip-network-config', action='store_true',
                      help='use an already configured Maix network')
    mode = root.add_mutually_exclusive_group()
    mode.add_argument('--dry-run', action='store_true',
                      help='show resolved settings and command without hardware access')
    mode.add_argument('--check-only', action='store_true',
                      help='check dependencies/devices without opening them or changing network')
    root.add_argument('--disk-warn-percent', type=float, default=85.0,
                      help='warn above this filesystem usage (default 85)')
    root.add_argument('--disk-fail-percent', type=float, default=95.0,
                      help='refuse to launch above this filesystem usage (default 95)')
    root.add_argument('--uvcdynctrl-log-max-mb', type=float, default=64.0,
                      help='refuse to launch when the UVC udev log exceeds this size')
    return root


def launch_settings(args):
    path = Path(args.config).expanduser()
    if not path.is_absolute():
        path = PROJECT_ROOT / path
    path = path.resolve()
    config = json.loads(path.read_text(encoding='utf-8'))
    coord = config['coordinator']
    camera = args.camera or config['camera']['device']
    if isinstance(camera, int) or (isinstance(camera, str) and camera.isdigit()):
        camera = '/dev/video%s' % camera
    mcu = args.mcu_serial or coord.get('mcu_serial', '')
    transport = coord.get('maix_transport', 'tcp')
    if transport not in ('tcp', 'serial'):
        raise ValueError('coordinator.maix_transport must be tcp or serial')
    command = [sys.executable, '-u', '-m', 'jetson_recognition.run', '--config', str(path),
               'coordinator', '--mcu-serial', mcu, '--camera', camera,
               '--maix-transport', transport]
    devices = {'camera': camera, 'mcu_serial': mcu}
    if transport == 'tcp':
        command.extend(['--maix-tcp-host', coord.get('maix_tcp_host', '0.0.0.0'),
                        '--maix-tcp-port', str(coord.get('maix_tcp_port', 5000))])
    else:
        devices['maix_serial'] = coord.get('maix_serial', '')
        command.extend(['--maix-serial', devices['maix_serial']])
    if not args.headless:
        command.append('--gui')
    return config, command, devices


def disk_findings(arguments, log_path=None):
    """Refuse to start on a disk that is about to fill up again.

    A full system disk breaks the GUI, the NVIDIA DRM and file copies long
    before it breaks recognition, so this is checked before hardware is opened.
    """
    warnings = []
    errors = []
    log_path = Path(log_path) if log_path else UVCDYNCTRL_LOG
    try:
        usage = shutil.disk_usage(str(PROJECT_ROOT))
        percent = usage.used / usage.total * 100.0 if usage.total else 0.0
    except OSError as error:
        warnings.append('cannot read filesystem usage: %s' % error)
        percent = 0.0
    if percent >= arguments.disk_fail_percent:
        errors.append(
            'filesystem is %.1f%% full; free space before launching '
            '(see docs/Jetson磁盘写满与USB枚举风暴.md)' % percent
        )
    elif percent >= arguments.disk_warn_percent:
        warnings.append('filesystem is %.1f%% full' % percent)
    try:
        log_size = log_path.stat().st_size
    except OSError:
        log_size = 0
    if log_size > arguments.uvcdynctrl_log_max_mb * 1024 * 1024:
        errors.append(
            '%s is %.1f MB; the UVC udev helper is logging a camera '
            're-enumeration storm. Truncate it and run '
            'tools/install_log_guard.sh' % (
                log_path, log_size / 1024.0 / 1024.0)
        )
    return warnings, errors


def preflight(args, config, devices):
    errors = []
    for name, value in devices.items():
        if not value or 'REPLACE_' in value:
            errors.append('%s is not configured; set coordinator.mcu_serial or use --mcu-serial for MCU' % name)
            continue
        path = Path(value)
        try:
            if not path.is_absolute() or not stat.S_ISCHR(path.stat().st_mode):
                errors.append('%s is not a character device: %s' % (name, path))
            elif not os.access(str(path), os.R_OK | os.W_OK):
                errors.append('no read/write permission for %s: %s' % (name, path))
        except OSError:
            errors.append('%s device is missing: %s' % (name, path))
    if devices.get('mcu_serial') and devices.get('maix_serial'):
        if Path(devices['mcu_serial']).resolve() == Path(devices['maix_serial']).resolve():
            errors.append('MCU and Maix must use different serial devices')
    for module in ('cv2', 'numpy', 'serial'):
        if importlib.util.find_spec(module) is None:
            errors.append('missing Python dependency: %s' % module)
    if shutil.which('v4l2-ctl') is None:
        errors.append('v4l2-ctl is missing (install v4l-utils)')
    if not args.headless and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
        errors.append('no graphical display; use --headless over SSH')
    if config['coordinator'].get('maix_transport', 'tcp') == 'tcp' and not args.skip_network_config:
        if shutil.which('nmcli') is None:
            errors.append('nmcli is missing; configure Maix network then use --skip-network-config')
    profile = config['camera'].get('v4l2_control_profile')
    profiles = config['camera'].get('v4l2_control_profiles', {})
    if profile and profile not in profiles:
        errors.append('unknown camera profile: %s' % profile)
    disk_warnings, disk_errors = disk_findings(args)
    for warning in disk_warnings:
        print('[PREFLIGHT] %s' % warning, file=sys.stderr)
    errors.extend(disk_errors)
    return errors


def main():
    args = parser().parse_args()
    config, command, devices = launch_settings(args)
    print(json.dumps({'state': 'COMPETITION_LAUNCH', 'devices': devices,
                      'camera_profile': config['camera'].get('v4l2_control_profile'),
                      'mcu_baud': config['coordinator'].get('baud', 115200),
                      'gui': not args.headless,
                      'command': ' '.join(shlex.quote(part) for part in command)},
                     ensure_ascii=False), flush=True)
    if args.dry_run:
        print('DRY_RUN: no devices opened, no network changes, coordinator not started.', flush=True)
        return 0
    if (not args.check_only
            and config['coordinator'].get('maix_transport', 'tcp') == 'tcp'
            and not args.skip_network_config):
        from tools.configure_maix_network import ensure_maix_network
        if ensure_maix_network(required=False) is None:
            print('[NETWORK] Maix is absent; the service will retry after other required devices appear.', flush=True)
    errors = preflight(args, config, devices)
    if errors:
        for error in errors:
            print('[PREFLIGHT] ' + error, file=sys.stderr)
        return 1
    if args.check_only:
        print('PREFLIGHT_OK: paths/dependencies only; camera streaming, port availability and MCU protocol remain untested.', flush=True)
        return 0
    print('[RUN] Wait for COORDINATOR_READY, then MCU START -> READY, then show QR to Maix. Ctrl+C stops the service.', flush=True)
    os.chdir(str(PROJECT_ROOT))
    os.execv(sys.executable, command)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print('[LAUNCH_FAILED] %s' % error, file=sys.stderr)
        raise SystemExit(1)
