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
# enables the I2C bus of a Raspberry Pi for the LSM9DS1 IMU (raspi-config), and adds the
# user to the dialout (serial), video (camera) and i2c (IMU) groups. Running it again is
# safe; on a machine that is not a Raspberry Pi the I2C step is skipped.
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
  --no-apt      Do not apt-get install can-utils and i2c-tools
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
  echo "Left in place: $env_file, group memberships, the I2C setting, can-utils,"
  echo "i2c-tools and systemd-networkd."
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
  # candump/cansend for debugging the bus; slcand for slcan adapters. i2cdetect for
  # finding the IMU; the Debian package also creates the i2c group and its udev rule
  step "Installing can-utils and i2c-tools"
  apt-get install -y can-utils i2c-tools
fi

# The LSM9DS1 IMU is on I2C bus 1 (header pins 3 and 5), which is off by default
i2c_note=""
if ! grep -qas "Raspberry Pi" /proc/device-tree/model; then
  step "Not a Raspberry Pi: skipping the I2C setup"
elif [ -n "${DESTDIR:-}" ]; then
  step "DESTDIR is set: skipping the I2C setup, which changes the boot configuration"
elif command -v raspi-config >/dev/null 2>&1; then
  # get_i2c prints 0 when dtparam=i2c_arm=on is in config.txt. That line alone does not
  # create /dev/i2c-1 (the i2c-dev module does), so a missing device node also triggers
  # do_i2c, which is safe to repeat
  if [ "$(raspi-config nonint get_i2c)" = "0" ] && [ -e /dev/i2c-1 ]; then
    step "I2C is already enabled"
  else
    # Sets dtparam=i2c_arm=on in config.txt, adds i2c-dev to /etc/modules and applies
    # both now
    step "Enabling I2C with raspi-config"
    raspi-config nonint do_i2c 0
    udevadm settle --timeout=5 || true
  fi
  if [ ! -e /dev/i2c-1 ]; then
    i2c_note="Reboot to finish enabling I2C: /dev/i2c-1 does not exist yet."
  fi
elif [ -e /dev/i2c-1 ]; then
  step "raspi-config not found, but /dev/i2c-1 exists: I2C is already enabled"
else
  step "raspi-config not found: enable I2C by hand"
  cat <<EOF
    Add the line "dtparam=i2c_arm=on" to /boot/firmware/config.txt (/boot/config.txt on
    older images), add "i2c-dev" to /etc/modules, and reboot.
EOF
  i2c_note="Enable I2C as described above, then reboot."
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

# /dev/i2c-* belongs to the i2c group on Raspberry Pi OS and with Debian's i2c-tools
groups="dialout,video"
if getent group i2c >/dev/null; then
  groups="$groups,i2c"
fi
step "Adding $user to the groups ${groups//,/, }"
usermod -aG "$groups" "$user"

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
  IMU:           i2cdetect -y 1                (expect 1e and 6b in the table)
  Start now:     sudo systemctl start iot-robot
  Logs:          journalctl -u iot-robot -f
  Settings:      $env_file
Log out and back in for the new group memberships to apply to your own shell.
EOF
if [ -n "$i2c_note" ]; then
  echo "$i2c_note"
fi
