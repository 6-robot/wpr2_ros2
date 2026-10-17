#!/usr/bin/env bash
set -euo pipefail
for rule in ftdi.rules rplidar.rules 99-obsensor-libusb.rules; do
  sudo rm -f -- "/etc/udev/rules.d/$rule"
done
sudo udevadm control --reload-rules
printf 'WPR2 device rules removed.\n'
