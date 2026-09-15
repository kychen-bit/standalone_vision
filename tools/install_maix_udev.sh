#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_SOURCE="${PROJECT_ROOT}/config/70-maixcam-rndis-net.rules"
RULE_TARGET="/etc/udev/rules.d/70-maixcam-rndis-net.rules"
UNIT_SOURCE="${PROJECT_ROOT}/deploy/maixcam-network@.service"
UNIT_TARGET="/etc/systemd/system/maixcam-network@.service"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run: sudo bash tools/install_maix_udev.sh" >&2
  exit 1
fi

install -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
install -m 0644 "${UNIT_SOURCE}" "${UNIT_TARGET}"
udevadm control --reload-rules
systemctl daemon-reload

echo "Installed ${RULE_TARGET}"
echo "Installed ${UNIT_TARGET}"
echo "Now unplug and reconnect the MaixCAM USB cable once; network setup is automatic."
echo "Expected interface: maixcam0  10.33.117.105/24"
