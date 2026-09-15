#!/usr/bin/env bash
# Stop the UVC udev helper from being able to fill the system disk again.
#
# Background: /lib/udev/uvcdynctrl runs on every video4linux "add" event with
# debug=1 and appends a full udev environment dump plus the uvcdynctrl output
# to /var/log/uvcdynctrl-udev.log. A camera that re-enumerates in a loop (the
# DECXIN USB camera does, see docs/Jetson磁盘写满与USB枚举风暴.md) therefore
# grows that file without limit; 196 GB filled the whole 233 GB system disk.
#
# This script is idempotent. Run it with sudo:
#   sudo bash tools/install_log_guard.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UVCDYNCTRL_HELPER="/lib/udev/uvcdynctrl"
LOGROTATE_TARGET="/etc/logrotate.d/uvcdynctrl-udev"
JOURNAL_DROPIN_DIR="/etc/systemd/journald.conf.d"
JOURNAL_DROPIN="${JOURNAL_DROPIN_DIR}/99-standalone-vision-limits.conf"

if [[ "${EUID}" -ne 0 ]]; then
    echo "Please run: sudo bash tools/install_log_guard.sh" >&2
    exit 1
fi

echo "[1/5] Bind the uvcdynctrl helper to /dev/null (debug=0)"
if [[ -f "${UVCDYNCTRL_HELPER}" ]]; then
    if grep -q '^debug=1' "${UVCDYNCTRL_HELPER}"; then
        [[ -f "${UVCDYNCTRL_HELPER}.orig" ]] || \
            cp -a "${UVCDYNCTRL_HELPER}" "${UVCDYNCTRL_HELPER}.orig"
        sed -i 's/^debug=1/debug=0/' "${UVCDYNCTRL_HELPER}"
        echo "    patched: ${UVCDYNCTRL_HELPER} (backup ${UVCDYNCTRL_HELPER}.orig)"
    elif grep -q '^debug=0' "${UVCDYNCTRL_HELPER}"; then
        echo "    already patched"
    else
        echo "    WARNING: unexpected helper format, inspect ${UVCDYNCTRL_HELPER}" >&2
    fi
else
    echo "    uvcdynctrl helper absent; nothing to patch"
fi
udevadm control --reload-rules

echo "[2/5] Install logrotate fallback for /var/log/uvcdynctrl-udev.log"
install -o root -g root -m 0644 \
    "${PROJECT_ROOT}/deploy/logrotate/uvcdynctrl-udev" "${LOGROTATE_TARGET}"
logrotate --debug "${LOGROTATE_TARGET}" >/dev/null

echo "[3/5] Install the disk guard timer"
install -o root -g root -m 0644 \
    "${PROJECT_ROOT}/deploy/disk-guard.service" /etc/systemd/system/disk-guard.service
install -o root -g root -m 0644 \
    "${PROJECT_ROOT}/deploy/disk-guard.timer" /etc/systemd/system/disk-guard.timer

echo "[4/5] Cap the journal so it cannot fill the disk either"
install -d -o root -g root -m 0755 "${JOURNAL_DROPIN_DIR}"
cat > "${JOURNAL_DROPIN}" <<'EOF'
# Keep systemd journals from competing with the vision workspace for disk.
[Journal]
SystemMaxUse=1G
SystemKeepFree=8G
MaxRetentionSec=2week
EOF

systemctl daemon-reload
systemctl restart systemd-journald
systemctl enable --now disk-guard.timer

echo "[5/5] Current state"
df -h / | tail -1
ls -lh /var/log/uvcdynctrl-udev.log 2>/dev/null || echo "    (log not present)"
journalctl --disk-usage

cat <<'EOF'

Done. The UVC udev helper no longer writes a log, logrotate and the five
minute disk-guard timer are active as safety nets.

Check the guard at any time:
    systemctl list-timers disk-guard.timer --no-pager
    journalctl -u disk-guard.service -n 20 --no-pager
    python3 tools/disk_guard.py --no-truncate

Reminder: the log only grew because the camera re-enumerated in a loop. If
`firings_per_hour` in the guard report is not near zero, fix the USB side too
(powered hub, shorter cable, MaixCAM on the other USB bus, only one process
using the camera).
EOF
