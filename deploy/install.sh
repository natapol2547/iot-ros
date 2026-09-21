#!/usr/bin/env bash
# Install the robot's system configuration on the Raspberry Pi.
#
#   sudo deploy/install.sh              # install and enable start at boot
#   sudo deploy/install.sh --uninstall  # remove everything this script installed
#
# Installs:
#   /etc/udev/rules.d/99-iot-robot-stm32.rules      /dev/stm32 symlink for the Nucleo
#   /etc/systemd/network/80-iot-robot-can0.network  can0 at 1 Mbit/s via systemd-networkd
#   /etc/systemd/system/iot-robot.service           robot.launch.py at boot
#   /etc/default/iot-robot                          launch arguments (kept if present)
# and adds the user to the dialout (serial) and video (camera) groups.
#
# Build the robot environment first (`pixi install -e robot --locked` and
# `pixi run -e robot build`); see docs/hardware.md.
set -euo pipefail

deploy_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(dirname "$deploy_dir")"
user="${SUDO_USER:-}"
pixi=""
use_apt=1
enable=1
uninstall=0

# DESTDIR only exists to test the script against a scratch directory
udev_rule="${DESTDIR:-}/etc/udev/rules.d/99-iot-robot-stm32.rules"
network_file="${DESTDIR:-}/etc/systemd/network/80-iot-robot-can0.network"
service_file="${DESTDIR:-}/etc/systemd/system/iot-robot.service"
env_file="${DESTDIR:-}/etc/default/iot-robot"

usage() {
  cat <<EOF
Usage: sudo $0 [--user NAME] [--pixi PATH] [--no-apt] [--no-enable] [--uninstall]

  --user NAME   Account that owns the checkout and runs the robot (default: \$SUDO_USER)
  --pixi PATH   pixi executable (default: ~NAME/.pixi/bin/pixi, then NAME's PATH)
  --no-apt      Do not apt-get install can-utils
  --no-enable   Install the service but do not enable it at boot
  --uninstall   Remove the files above and disable the service
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --user) user="$2"; shift 2 ;;
    --pixi) pixi="$2"; shift 2 ;;
    --no-apt) use_apt=0; shift ;;
    --no-enable) enable=0; shift ;;
    --uninstall) uninstall=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

if [ "$uninstall" -eq 1 ]; then
  systemctl disable --now iot-robot.service 2>/dev/null || true
  rm -f "$service_file" "$udev_rule" "$network_file"
  systemctl daemon-reload
  udevadm control --reload-rules
  networkctl reload 2>/dev/null || true
  echo "Removed the service, udev rule and CAN network file."
  echo "Left in place: $env_file, group memberships, can-utils and systemd-networkd."
  exit 0
fi

if [ -z "$user" ] || [ "$user" = "root" ]; then
  echo "Cannot tell which account runs the robot. Pass --user NAME." >&2
  exit 1
fi
if ! home="$(getent passwd "$user" | cut -d: -f6)" || [ -z "$home" ]; then
  echo "No such user: $user" >&2
  exit 1
fi

if [ -z "$pixi" ]; then
  if [ -x "$home/.pixi/bin/pixi" ]; then
    pixi="$home/.pixi/bin/pixi"
  else
    pixi="$(sudo -u "$user" -i sh -c 'command -v pixi' || true)"
  fi
fi
if [ -z "$pixi" ] || [ ! -x "$pixi" ]; then
  echo "pixi not found for $user. Install it first (curl -fsSL https://pixi.sh/install.sh | sh)" >&2
  echo "or pass --pixi PATH." >&2
  exit 1
fi

step() {
  echo "==> $*"
}

if [ "$use_apt" -eq 1 ]; then
  # candump/cansend for debugging the bus; slcand for slcan adapters
  step "Installing can-utils"
  apt-get install -y can-utils
fi

step "Installing udev rule for /dev/stm32"
install -D -m 0644 "$deploy_dir/udev/99-iot-robot-stm32.rules" "$udev_rule"
udevadm control --reload-rules
udevadm trigger --subsystem-match=tty --action=add

step "Configuring can0 at 1 Mbit/s with systemd-networkd"
install -D -m 0644 "$deploy_dir/network/80-iot-robot-can0.network" "$network_file"
systemctl enable systemd-networkd.service
if systemctl is-active --quiet NetworkManager; then
  # Enabling networkd also enables its wait-online unit. With NetworkManager in charge
  # of the real network, networkd manages only can0, which is never "online", so the
  # unit would stall boot for two minutes. NetworkManager-wait-online covers the rest.
  systemctl disable systemd-networkd-wait-online.service 2>/dev/null || true
fi
systemctl restart systemd-networkd.service

step "Adding $user to the dialout and video groups"
usermod -aG dialout,video "$user"

step "Installing iot-robot.service (user $user, checkout $repo)"
mkdir -p "$(dirname "$service_file")"
sed -e "s|@USER@|$user|g" -e "s|@REPO@|$repo|g" -e "s|@PIXI@|$pixi|g" \
  "$deploy_dir/systemd/iot-robot.service" > "$service_file"
chmod 0644 "$service_file"
if [ ! -e "$env_file" ]; then
  install -D -m 0644 "$deploy_dir/systemd/iot-robot.env" "$env_file"
else
  echo "Keeping existing $env_file"
fi
systemctl daemon-reload
if [ "$enable" -eq 1 ]; then
  systemctl enable iot-robot.service
fi

cat <<EOF

Done.
  CAN link:      ip -details link show can0    (appears once the adapter is plugged in)
  STM32:         ls -l /dev/stm32
  Start now:     sudo systemctl start iot-robot
  Logs:          journalctl -u iot-robot -f
  Settings:      $env_file
Log out and back in for the new group memberships to apply to your own shell.
EOF
