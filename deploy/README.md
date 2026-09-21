# deploy

System configuration for the Raspberry Pi on the robot. Everything here is installed
by one script; the ROS side lives in `src/iot_robot_bringup`. Pi setup from a blank SD
card, motor configuration and the first power-on checklist are in
[docs/hardware.md](../docs/hardware.md).

| File | Installed as | Purpose |
| --- | --- | --- |
| `udev/99-iot-robot-stm32.rules` | `/etc/udev/rules.d/` | `/dev/stm32` symlink for the Nucleo's ST-LINK serial port (USB ID `0483:374b`) |
| `network/80-iot-robot-can0.network` | `/etc/systemd/network/` | `can0` up at 1 Mbit/s with bus-off auto-restart, whenever the adapter appears |
| `systemd/iot-robot.service` | `/etc/systemd/system/` | Runs `pixi run -e robot robot` at boot and restarts it if it exits |
| `systemd/iot-robot.env` | `/etc/default/iot-robot` | `ROS_DOMAIN_ID`, extra launch arguments, and the CAN interface and motor IDs the service stops |
| `scripts/can_up.sh` | (run from the checkout) | Manual CAN bring-up, including slcan adapters |

## Install

Build the robot environment first, then run the installer as root from the checkout:

```bash
pixi install -e robot --locked
pixi run -e robot build
sudo deploy/install.sh
sudo systemctl start iot-robot      # or reboot
journalctl -u iot-robot -f
```

`install.sh` also installs `can-utils` (`candump`, `cansend`, `slcand`), enables
`systemd-networkd` and adds the user to the `dialout` and `video` groups. Options:
`--user NAME` (default: the user who ran sudo), `--pixi PATH`, `--no-apt`, `--no-enable`.
`sudo deploy/install.sh --uninstall` removes the udev rule, network file and service.

The service runs as that user from this checkout, so updating the robot is:

```bash
git pull --recurse-submodules
pixi run -e robot build
sudo systemctl restart iot-robot
```

Launch arguments for the service go in `/etc/default/iot-robot`, for example
`IOT_ROBOT_ARGS=gizmo_mode:=servo right_direction:=1`. Restart the service after editing it.

### Behaviour at boot

- The service does not wait for the CAN adapter. If `can0` or a motor is missing, the
  launch's CAN pre-flight check fails with an explanation, the service exits, and systemd
  retries every 5 s until the motors answer. `journalctl -u iot-robot` shows why.
- The service uses `pixi run --frozen`, which installs exactly what `pixi.lock` pins and
  never re-solves. A boot never waits on the network or changes package versions.
- Stopping the service sends SIGINT, the same as Ctrl-C, so the controllers shut down
  cleanly and the motor plugin stops the wheels. `KillMode=mixed` sends it to `ros2
  launch` only, which forwards it to each node once; with the default `KillMode` every
  node would get it twice and some would die mid-shutdown. Whatever still runs after
  15 s gets SIGKILL.
- After every stop, crash or kill, `ExecStopPost` runs `cubemars_tool stop`: zero speed
  for 0.3 s, then release, on `IOT_CAN_INTERFACE` (default `can0`) to `IOT_MOTOR_IDS`
  (default `1 2`). It covers the case where the launch itself dies before it can stop
  the motors, and does nothing when the CAN link is not up. If `IOT_ROBOT_ARGS` changes
  `can_interface`, `left_can_id` or `right_can_id`, set these two in
  `/etc/default/iot-robot` to match.
- `LimitRTPRIO=99` lets `controller_manager` use real-time (SCHED_FIFO) scheduling.
- When NetworkManager is active (Raspberry Pi OS), `install.sh` disables
  `systemd-networkd-wait-online`. networkd then manages only `can0`, which never counts
  as online, so the wait-online unit would otherwise stall boot for two minutes.

## CAN adapters

`cubemars_hardware` and the pre-flight check talk to the motors through **SocketCAN**:
the kernel exposes the CAN bus as a network interface (`can0`) and programs open it with
a socket. Whether a USB-CAN adapter works depends on how Linux sees it, not on what the
box says. There are three kinds.

| Kind | Linux shows | Examples | Works here |
| --- | --- | --- | --- |
| Native SocketCAN | `can0` as soon as it is plugged in | candleLight firmware (`gs_usb`, USB ID `1d50:606f`): CANable 2.0, Innomaker USB2CAN, MKS CANable. PEAK PCAN-USB (`peak_usb`, `0c72:000c`). 8devices USB2CAN (`usb_8dev`). SPI HATs with MCP2515 or MCP2518FD | Yes. Recommended |
| slcan (serial-line CAN) | a serial port, `/dev/ttyACM*` or `/dev/ttyUSB*` | CANable with the slcan firmware (`16d0:117e`), USBtin, Lawicel CANUSB | Yes, through `slcand` |
| Proprietary serial or USB protocol | a serial port (`1a86:7523`, CH340) or nothing | Waveshare USB-CAN-A, most "USB-CAN analyzer" boxes sold with Windows software, CANalyst-II (`04d8:0053`) | No |

Identify an adapter by plugging it in and running:

```bash
lsusb                              # USB ID, compare with the table
ip -brief link show type can       # a native adapter shows up here immediately
ls /dev/ttyACM* /dev/ttyUSB*       # a new tty instead means a serial adapter
sudo dmesg | tail                  # driver name: gs_usb, peak_usb, cdc_acm, ch341
```

**Native SocketCAN.** Nothing else to do: the networkd file sets the bitrate every time the
interface appears. By hand: `sudo deploy/scripts/can_up.sh` (or `pixi run -e robot can-up`).
CANable boards and their clones ship with either firmware and can be reflashed from slcan
to candleLight over USB DFU. That is worth doing: it turns a serial adapter into a native
one.

**SPI HATs** need a device-tree overlay in `/boot/firmware/config.txt` whose oscillator
frequency matches the crystal on the board, e.g. for a Waveshare RS485 CAN HAT with a
12 MHz crystal: `dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25`. The MCP2515
has only two receive buffers, so at 1 Mbit/s with two motors reporting at 200 Hz it can
drop frames on a busy Pi; an MCP2518FD board does not have that limit.

**slcan.** The adapter speaks an ASCII protocol over a serial port, and `slcand` (from
`can-utils`) turns that into `can0`. The bitrate is set by `slcand`, so networkd cannot
configure these adapters. Bring one up with

```bash
sudo deploy/scripts/can_up.sh --slcan /dev/serial/by-id/usb-<adapter>
```

using the stable `/dev/serial/by-id/` name; the STM32 board is also a `/dev/ttyACM*`
device, and the numbering depends on plug order. Adapters behind a USB-UART chip (CH340,
CP210x) also need `--serial-baud` set to the rate their firmware uses. To do this at boot,
add a oneshot unit that runs the same command before `iot-robot.service`:

```ini
# /etc/systemd/system/iot-robot-slcan.service
[Unit]
Description=slcan adapter as can0
Before=iot-robot.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/home/<user>/iot-ros/deploy/scripts/can_up.sh --slcan /dev/serial/by-id/usb-<adapter>

[Install]
WantedBy=multi-user.target
```

**Proprietary adapters** do not create a SocketCAN interface at all. The Waveshare
USB-CAN-A, for example, is a CH340 USB-serial chip running Waveshare's own binary
framing; `slcand` cannot talk to it. Using one would need a user-space bridge from its
protocol to a `vcan` interface, which this project does not include. Replace it with a
candleLight adapter or a CAN HAT.

### Checking the bus

```bash
ip -details link show can0           # "can state ERROR-ACTIVE", "bitrate 1000000"
candump can0                         # status frames 00002901 and 00002902 from the motors
pixi run -e robot can-check          # interface up and both motors reporting
```

`ERROR-PASSIVE` or `BUS-OFF` means frames are not being acknowledged: motors unpowered,
CANH and CANL swapped, a bitrate mismatch, or missing termination. With everything
powered off, the resistance between CANH and CANL should be about 60 ohm (two 120 ohm
terminators in parallel).
