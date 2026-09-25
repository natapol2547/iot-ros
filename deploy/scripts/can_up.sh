#!/usr/bin/env bash
# Bring up the CAN link to the wheel motors by hand.
#
#   sudo deploy/scripts/can_up.sh                        # can0 at 1 Mbit/s (SocketCAN adapter)
#   sudo deploy/scripts/can_up.sh --slcan /dev/ttyACM1   # slcan adapter, creates can0 via slcand
#
# At boot the systemd-networkd file installed by deploy/install.sh does the same for
# SocketCAN adapters. This script is for bench work, a link that went bus-off, and slcan
# adapters, which networkd cannot configure. See deploy/README.md for adapter types.
set -euo pipefail

original_args=("$@")
interface=can0
bitrate=1000000
slcan_device=""
serial_baud=""

usage() {
  cat <<EOF
Usage: sudo $0 [--interface NAME] [--bitrate BPS] [--slcan TTY [--serial-baud BAUD]]

  --interface NAME     CAN interface to configure (default: can0)
  --bitrate BPS        CAN bitrate in bit/s (default: 1000000, what the AK45-10 uses)
  --slcan TTY          Attach an slcan (Lawicel protocol) adapter on TTY with slcand
                       and create NAME from it
  --serial-baud BAUD   UART baud rate for slcan adapters behind a USB-UART bridge
                       (CH340, CP210x). Not needed for USB CDC adapters (/dev/ttyACM*)
EOF
}

while [ $# -gt 0 ]; do
  case "$1" in
    --interface) interface="$2"; shift 2 ;;
    --bitrate) bitrate="$2"; shift 2 ;;
    --slcan) slcan_device="$2"; shift 2 ;;
    --serial-baud) serial_baud="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [ "$(id -u)" -ne 0 ]; then
  echo "Run as root: sudo $0 ${original_args[*]}" >&2
  exit 1
fi

link_exists() {
  ip link show dev "$interface" > /dev/null 2>&1
}

if [ -n "$slcan_device" ]; then
  # slcand takes the bitrate as a code (-sN) and configures the adapter itself
  case "$bitrate" in
    10000) code=0 ;; 20000) code=1 ;; 50000) code=2 ;; 100000) code=3 ;;
    125000) code=4 ;; 250000) code=5 ;; 500000) code=6 ;; 800000) code=7 ;;
    1000000) code=8 ;;
    *) echo "slcan supports 10k, 20k, 50k, 100k, 125k, 250k, 500k, 800k and 1M bit/s" >&2
       exit 2 ;;
  esac
  if ! command -v slcand > /dev/null; then
    echo "slcand not found. Install it with: sudo apt install can-utils" >&2
    exit 1
  fi
  if [ ! -c "$slcan_device" ]; then
    echo "$slcan_device is not a serial device. Adapters found:" >&2
    ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null >&2 || echo "  none" >&2
    exit 1
  fi
  if link_exists; then
    echo "$interface already exists. If an old slcand owns it, stop it first:" >&2
    echo "  sudo pkill slcand" >&2
    exit 1
  fi
  # -o opens the channel, -c closes it when slcand exits
  args=(-o -c "-s$code")
  if [ -n "$serial_baud" ]; then
    args+=(-S "$serial_baud")
  fi
  slcand "${args[@]}" "$slcan_device" "$interface"
  for _ in $(seq 1 30); do
    link_exists && break
    sleep 0.1
  done
  if ! link_exists; then
    echo "slcand did not create $interface. Is $slcan_device really an slcan adapter?" >&2
    exit 1
  fi
  ip link set dev "$interface" up
else
  if ! link_exists; then
    echo "CAN interface $interface does not exist." >&2
    present=$(ip -brief link show type can 2>/dev/null | awk '{print $1}' | tr '\n' ' ')
    echo "CAN interfaces present: ${present:-none}" >&2
    if ls /dev/ttyACM* /dev/ttyUSB* > /dev/null 2>&1; then
      echo "Serial devices present: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')" >&2
      echo "If the adapter is an slcan device, run: sudo $0 --slcan <tty>" >&2
    fi
    echo "See deploy/README.md, 'CAN adapters'." >&2
    exit 1
  fi
  # Bitrate can only be changed while the link is down
  ip link set dev "$interface" down
  # Automatic bus-off recovery where the driver supports it. gs_usb (candleLight)
  # rejects restart-ms ("Operation not supported"); after a bus-off such a link
  # needs this script again.
  if ! ip link set dev "$interface" type can bitrate "$bitrate" restart-ms 100 2> /dev/null; then
    echo "$interface: driver does not support automatic bus-off restart; setting the bitrate only." >&2
    ip link set dev "$interface" type can bitrate "$bitrate"
  fi
  ip link set dev "$interface" up
fi

# Shows the controller state (ERROR-ACTIVE is healthy) and the bitrate
ip -details link show dev "$interface"
