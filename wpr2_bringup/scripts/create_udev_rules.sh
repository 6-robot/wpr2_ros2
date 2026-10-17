#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for rule in ftdi.rules rplidar.rules 99-obsensor-libusb.rules; do
  sudo install -m 644 "$script_dir/$rule" "/etc/udev/rules.d/$rule"
done
sudo udevadm control --reload-rules
sudo udevadm trigger
printf 'Device rules installed. Reconnect the devices.\n'
