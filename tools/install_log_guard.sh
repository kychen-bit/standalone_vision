#!/usr/bin/env bash
# Stop the UVC udev helper from being able to fill the system disk again.
#
# Background: /lib/udev/uvcdynctrl runs on every video4linux "add" event with
# debug=1 and appends a full udev environment dump plus the uvcdynctrl output
# to /var/log/uvcdynctrl-udev.log. A camera that re-enumerates in a loop (the
# DECXIN USB camera does, see docs/Jetson磁盘写满与USB枚举风暴.md) therefore
# grows that file without limit; 196 GB filled the whole 233 GB system disk.
#
# This script is idempotent and never aborts half way. Run it with sudo:
#   sudo bash tools/install_log_guard.sh
#   sudo bash tools/install_log_guard.sh --with-logrotate   # apt install logrotate too
#
# A failing optional step must never abort the rest of the installation. The
# first run of this script died in step 2 on a machine without the logrotate
# package (`logrotate --debug` -> 127) because of `set -e`, so the disk-guard
# timer and the journal limits were silently never applied. Steps now record
# failures and the installer always reaches the self-check at the end.
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
UVCDYNCTRL_HELPER="/lib/udev/uvcdynctrl"
UVCDYNCTRL_RULE_MASK="/etc/udev/rules.d/80-uvcdynctrl.rules"
LOGROTATE_TARGET="/etc/logrotate.d/uvcdynctrl-udev"
JOURNAL_DROPIN_DIR="/etc/systemd/journald.conf.d"
JOURNAL_DROPIN="${JOURNAL_DROPIN_DIR}/99-standalone-vision-limits.conf"

WITH_LOGROTATE=0
for argument in "$@"; do
    case "${argument}" in
        --with-logrotate) WITH_LOGROTATE=1 ;;
        -h|--help)
            sed -n '2,14p' "${BASH_SOURCE[0]}"
            exit 0
            ;;
        *)
            echo "unknown option: ${argument} (try --help)" >&2
            exit 2
            ;;
    esac
done

failures=()
note() { echo "    $*"; }
warn() { echo "    WARNING: $*" >&2; }
fail() {
    echo "    FAILED: $*" >&2
    failures+=("$*")
}

if [[ "${EUID}" -ne 0 ]]; then
    echo "Please run: sudo bash tools/install_log_guard.sh" >&2
    exit 1
fi

echo "[1/5] Disable the legacy uvcdynctrl udev helper"
if [[ -f "${UVCDYNCTRL_HELPER}" ]]; then
    if grep -q '^debug=1' "${UVCDYNCTRL_HELPER}"; then
        [[ -f "${UVCDYNCTRL_HELPER}.orig" ]] || \
            cp -a "${UVCDYNCTRL_HELPER}" "${UVCDYNCTRL_HELPER}.orig"
        sed -i 's/^debug=1/debug=0/' "${UVCDYNCTRL_HELPER}"
    fi
    if grep -q '^debug=0' "${UVCDYNCTRL_HELPER}"; then
        note "debug=0 confirmed in ${UVCDYNCTRL_HELPER}"
    else
        fail "cannot set debug=0 in ${UVCDYNCTRL_HELPER}"
    fi
else
    note "uvcdynctrl helper absent; nothing to patch"
fi
ln -sfn /dev/null "${UVCDYNCTRL_RULE_MASK}" \
    || fail "mask ${UVCDYNCTRL_RULE_MASK}"
if [[ "$(readlink "${UVCDYNCTRL_RULE_MASK}" 2>/dev/null)" == "/dev/null" ]]; then
    note "udev rule masked: ${UVCDYNCTRL_RULE_MASK} -> /dev/null"
else
    fail "uvcdynctrl udev rule is still active"
fi
udevadm control --reload-rules || fail "udevadm control --reload-rules"

echo "[2/5] Rotation fallback for /var/log/uvcdynctrl-udev.log"
if ! command -v logrotate >/dev/null 2>&1 && (( WITH_LOGROTATE )); then
    note "installing the logrotate package"
    apt-get install -y logrotate || fail "apt-get install logrotate"
fi
if command -v logrotate >/dev/null 2>&1; then
    install -o root -g root -m 0644 \
        "${PROJECT_ROOT}/deploy/logrotate/uvcdynctrl-udev" "${LOGROTATE_TARGET}" \
        || fail "install ${LOGROTATE_TARGET}"
    # Debug mode exits non-zero for reasons unrelated to the installed file, so
    # it must only warn (this is what aborted the first run).
    logrotate --debug "${LOGROTATE_TARGET}" >/dev/null 2>&1 \
        || warn "logrotate --debug returned non-zero; the size limit still applies"
    note "rotation config installed at ${LOGROTATE_TARGET}"
else
    note "logrotate is NOT installed: no rotation config is active"
    note "the disk-guard timer below still truncates the log every 5 minutes"
    note "to add rotation: sudo apt-get install -y logrotate (or rerun with --with-logrotate)"
fi

echo "[3/5] Install the disk guard timer"
install -o root -g root -m 0644 \
    "${PROJECT_ROOT}/deploy/disk-guard.service" /etc/systemd/system/disk-guard.service \
    || fail "install disk-guard.service"
install -o root -g root -m 0644 \
    "${PROJECT_ROOT}/deploy/disk-guard.timer" /etc/systemd/system/disk-guard.timer \
    || fail "install disk-guard.timer"

echo "[4/5] Cap the journal so it cannot fill the disk either"
install -d -o root -g root -m 0755 "${JOURNAL_DROPIN_DIR}" || fail "create ${JOURNAL_DROPIN_DIR}"
cat > "${JOURNAL_DROPIN}" <<'EOF'
# Keep systemd journals from competing with the vision workspace for disk.
[Journal]
SystemMaxUse=1G
SystemKeepFree=8G
MaxRetentionSec=2week
EOF
[[ -s "${JOURNAL_DROPIN}" ]] || fail "write ${JOURNAL_DROPIN}"

systemctl daemon-reload || fail "systemctl daemon-reload"
systemctl restart systemd-journald || fail "restart systemd-journald"
systemctl enable --now disk-guard.timer || fail "enable disk-guard.timer"

echo "[5/5] Self-check"
helper_debug="$(grep -m1 '^debug=' "${UVCDYNCTRL_HELPER}" 2>/dev/null || echo 'debug=<unknown>')"
note "udev helper   : ${helper_debug}   (debug=0 means the log is /dev/null)"
note "udev rule     : $(readlink "${UVCDYNCTRL_RULE_MASK}" 2>/dev/null || echo active)   (/dev/null means disabled)"
note "logrotate     : $(command -v logrotate >/dev/null 2>&1 && echo "installed, $( [[ -f ${LOGROTATE_TARGET} ]] && echo 'config present' || echo 'no config')" || echo 'not installed (optional)')"
note "guard timer   : $(systemctl is-enabled disk-guard.timer 2>&1) / $(systemctl is-active disk-guard.timer 2>&1)"
note "journal cap   : $( [[ -f ${JOURNAL_DROPIN} ]] && echo present || echo missing)"
note "log size      : $(ls -lh /var/log/uvcdynctrl-udev.log 2>/dev/null | awk '{print $5", mtime "$6" "$7" "$8}' || echo 'not present')"
df -h / | tail -1
journalctl --disk-usage

if (( ${#failures[@]} )); then
    echo
    echo "Finished with ${#failures[@]} failed step(s):" >&2
    for item in "${failures[@]}"; do
        echo "  - ${item}" >&2
    done
    exit 1
fi

cat <<'EOF'

Done. The legacy uvcdynctrl udev helper is disabled and no longer queries each
new video node or writes a log; the five minute disk-guard timer is the safety
net (logrotate is optional extra).

Verify it any time without sudo:
    python3 tools/disk_guard.py --no-truncate     # guard state, sizes, growth rate
    systemctl list-timers disk-guard.timer --no-pager
    journalctl -u disk-guard.service -n 20 --no-pager

End-to-end proof that the log can no longer grow: re-plug the camera once, then
check that the size of /var/log/uvcdynctrl-udev.log did not change.

Reminder: the log only grew because the camera re-enumerated in a loop. If
`firings_per_hour` in the guard report is not near zero, fix the USB side too
(powered hub, shorter cable, MaixCAM on the other USB bus, only one process
using the camera).
EOF
