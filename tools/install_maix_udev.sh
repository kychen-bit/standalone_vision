#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULE_SOURCE="${PROJECT_ROOT}/config/70-maixcam-rndis-net.rules"
RULE_TARGET="/etc/udev/rules.d/70-maixcam-rndis-net.rules"

if [[ "${EUID}" -ne 0 ]]; then
  echo "Please run: sudo bash tools/install_maix_udev.sh" >&2
  exit 1
fi

install -m 0644 "${RULE_SOURCE}" "${RULE_TARGET}"
udevadm control --reload-rules

echo "Installed ${RULE_TARGET}"
echo "Now unplug and reconnect the MaixCAM USB cable once."
echo "Then run: python3 tools/configure_maix_network.py"
echo "Expected interface: maixcam0  10.33.117.105/24"
