"""Watch the two things that filled the Jetson system disk in the past.

The 2026 disk-full incident was not caused by the vision code itself: the UVC
udev helper ``/lib/udev/uvcdynctrl`` appends ~2.3 KB (a full udev environment
dump plus the ``uvcdynctrl --addctrl`` output) to
``/var/log/uvcdynctrl-udev.log`` for **every** ``video4linux`` "add" event, and
nothing rotates or caps that file. A USB camera that re-enumerates in a loop
(the DECXIN camera does) therefore turns into an unbounded log.

This tool is the safety net: it samples the filesystem and that log on a timer,
reports the growth rate so an enumeration storm is visible early, and truncates
the runaway log before the disk fills up again.

Exit codes: 0 = healthy, 1 = warning, 2 = critical.
"""

import argparse
import json
from pathlib import Path
import shutil
import time

DEFAULT_STATE = Path("/tmp/standalone-vision-disk-guard.json")
UVCDYNCTRL_LOG = Path("/var/log/uvcdynctrl-udev.log")

# A vision run opens the camera once per process and re-plugging produces a few
# KB. Anything above this is a storm, not normal hot-plug traffic.
STORM_FIRINGS_PER_HOUR = 120.0


def _read_state(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def _write_state(path, state):
    try:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(state), encoding="utf-8")
    except OSError:
        pass


def _firing_count(path, max_bytes=512 * 1024 * 1024):
    """Count ``Triggered at`` markers, the helper's per-event header."""
    try:
        if path.stat().st_size > max_bytes:
            # Already far past any sane size; do not spend time scanning it.
            return None
        total = 0
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            for line in handle:
                if line.startswith("Triggered at"):
                    total += 1
        return total
    except OSError:
        return None


def sample(path=UVCDYNCTRL_LOG, root="/", state_path=DEFAULT_STATE,
           truncate_bytes=64 * 1024 * 1024, warn_percent=85.0,
           critical_percent=92.0):
    """Return a JSON-serialisable report plus the triggered findings."""
    now_s = time.time()
    previous = _read_state(state_path)

    usage = shutil.disk_usage(root)
    percent = (usage.used / usage.total * 100.0) if usage.total else 0.0

    log_size = None
    log_firings = None
    firings_per_hour = None
    growth_bytes_per_hour = None
    if path.exists():
        stat = path.stat()
        log_size = stat.st_size
        log_firings = _firing_count(path)
        elapsed_s = now_s - float(previous.get("timestamp_s", now_s))
        if elapsed_s > 60.0:
            if previous.get("log_size") is not None:
                growth_bytes_per_hour = (
                    (log_size - float(previous["log_size"])) / elapsed_s * 3600.0
                )
            if previous.get("log_firings") is not None:
                firings_per_hour = (
                    (log_firings - float(previous["log_firings"])) / elapsed_s * 3600.0
                )

    findings = []
    severity = 0
    if percent >= critical_percent:
        severity = 2
        findings.append("filesystem %s is %.1f%% full" % (root, percent))
    elif percent >= warn_percent:
        severity = max(severity, 1)
        findings.append("filesystem %s is %.1f%% full" % (root, percent))

    if log_size is not None and log_size >= truncate_bytes:
        severity = 2
        findings.append(
            "%s grew to %.1f MiB (limit %.1f MiB)"
            % (path, log_size / 1024.0 / 1024.0, truncate_bytes / 1024.0 / 1024.0)
        )
    if firings_per_hour is not None and firings_per_hour > STORM_FIRINGS_PER_HOUR:
        severity = max(severity, 1)
        findings.append(
            "UVC udev helper fires %.0f times/hour; the camera is re-enumerating "
            "in a loop" % firings_per_hour
        )

    report = {
        "state": "DISK_GUARD",
        "severity": ["ok", "warning", "critical"][severity],
        "filesystem": {"root": root, "percent": round(percent, 1),
                       "used_bytes": usage.used, "free_bytes": usage.free},
        "uvcdynctrl_log": {
            "path": str(path),
            "size_bytes": log_size,
            "firings": log_firings,
            "growth_bytes_per_hour": (
                None if growth_bytes_per_hour is None else round(growth_bytes_per_hour)
            ),
            "firings_per_hour": (
                None if firings_per_hour is None else round(firings_per_hour, 1)
            ),
        },
        "findings": findings,
    }

    _write_state(state_path, {"timestamp_s": now_s, "log_size": log_size,
                              "log_firings": log_firings})
    return report, severity


def truncate_log(path=UVCDYNCTRL_LOG, keep_bytes=64 * 1024):
    """Keep the newest ``keep_bytes`` of the runaway log and drop the rest."""
    try:
        size = path.stat().st_size
    except OSError:
        return None
    if size <= keep_bytes:
        return None
    with open(path, "rb+") as handle:
        handle.seek(size - keep_bytes)
        tail = handle.read()
        handle.seek(0)
        handle.write(tail)
        handle.truncate()
    return size


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--log", default=str(UVCDYNCTRL_LOG))
    parser.add_argument("--root", default="/")
    parser.add_argument("--state", default=str(DEFAULT_STATE))
    parser.add_argument("--truncate-mb", type=float, default=64.0,
                        help="truncate the UVC udev log above this size")
    parser.add_argument("--keep-mb", type=float, default=0.064,
                        help="bytes of the newest log content to keep")
    parser.add_argument("--warn-percent", type=float, default=85.0)
    parser.add_argument("--critical-percent", type=float, default=92.0)
    parser.add_argument("--no-truncate", action="store_true",
                        help="report only, never modify the log")
    arguments = parser.parse_args()

    report, severity = sample(
        path=Path(arguments.log),
        root=arguments.root,
        state_path=Path(arguments.state),
        truncate_bytes=int(arguments.truncate_mb * 1024 * 1024),
        warn_percent=arguments.warn_percent,
        critical_percent=arguments.critical_percent,
    )

    if severity == 2 and not arguments.no_truncate:
        freed = truncate_log(Path(arguments.log),
                             int(arguments.keep_mb * 1024 * 1024))
        if freed is not None:
            report["truncated"] = {
                "path": arguments.log,
                "freed_bytes": freed,
                "note": "root cause is the uvcdynctrl udev helper; run "
                        "tools/install_log_guard.sh to stop it writing at all",
            }

    print(json.dumps(report, ensure_ascii=False), flush=True)
    return severity


if __name__ == "__main__":
    raise SystemExit(main())
