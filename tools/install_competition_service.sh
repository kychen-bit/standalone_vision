#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit_source="${project_root}/deploy/standalone-vision.service"
unit_target="/etc/systemd/system/standalone-vision.service"
network_unit_source="${project_root}/deploy/maixcam-network@.service"
network_unit_target="/etc/systemd/system/maixcam-network@.service"
rule_source="${project_root}/config/70-maixcam-rndis-net.rules"
rule_target="/etc/udev/rules.d/70-maixcam-rndis-net.rules"

if [[ ! -f "${unit_source}" ]]; then
    echo "missing service unit: ${unit_source}" >&2
    exit 1
fi

sudo install -o root -g root -m 0644 "${unit_source}" "${unit_target}"
sudo install -o root -g root -m 0644 "${network_unit_source}" "${network_unit_target}"
sudo install -o root -g root -m 0644 "${rule_source}" "${rule_target}"
sudo udevadm control --reload-rules
sudo systemctl daemon-reload
sudo systemctl enable --now standalone-vision.service
sudo systemctl --no-pager --full status standalone-vision.service

echo
echo "Installed the competition service and MaixCAM hot-plug network service."
echo "Unplug and reconnect MaixCAM once; the host will configure maixcam0 automatically."
