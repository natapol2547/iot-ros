# deploy

System configuration for the Raspberry Pi on the robot. Everything here is installed
by one script; the ROS side lives in `src/iot_robot_bringup`. Pi setup from a blank SD
card and motor configuration are in [docs/hardware.md](../docs/hardware.md), the
one-time setup in order in [docs/todo.md](../docs/todo.md), and the routine for every
session in [docs/checklist.md](../docs/checklist.md).

| File | Installed as | Purpose |
| --- | --- | --- |
| `udev/99-iot-robot-stm32.rules` | `/etc/udev/rules.d/` | `/dev/stm32` symlink for the Nucleo's ST-LINK serial port (USB ID `0483:374b`) |
| `network/80-iot-robot-can0.network` | `/etc/systemd/network/` | `can0` up at 1 Mbit/s whenever the adapter appears (no bus-off auto-restart: gs_usb does not support it; recover with `can_up.sh`) |
| `systemd/iot-robot.service` | `/etc/systemd/system/` | Runs `pixi run -e robot robot` at boot and restarts it if it exits |
| `systemd/iot-robot.env` | `/etc/default/iot-robot` | `ROS_DOMAIN_ID`, extra launch arguments, and the CAN interface the service stops the motors on |
| `scripts/can_up.sh` | (run from the checkout) | Manual CAN bring-up, including slcan adapters |

## Install

Build the robot environment first, then run the installer as root from the checkout:

```bash
pixi install -e robot --locked
pixi run -e robot build
sudo deploy/install.sh
sudo reboot                         # finishes enabling I2C; the service starts at boot
journalctl -u iot-robot -f
```

The service zeroes the gizmo as it starts, so put the gizmo at its zero pose before it
does ([docs/checklist.md](../docs/checklist.md)). On a new robot, install with
`--no-enable` until the first power-on checks in [docs/todo.md](../docs/todo.md) have
passed, then `sudo systemctl enable --now iot-robot`.

Besides the files above, `install.sh`:

- installs `can-utils` (`candump`, `cansend`, `slcand`) and `i2c-tools` (`i2cdetect`),
- enables `systemd-networkd`,
- enables the Pi's I2C bus 1 for the LSM9DS1 IMU with `raspi-config` (it sets
  `dtparam=i2c_arm=on` in `/boot/firmware/config.txt` and loads `i2c-dev`). If
  `/dev/i2c-1` does not exist yet it says `Reboot to finish enabling I2C`. Without
  `raspi-config` it prints the manual steps instead; on a machine that is not a
  Raspberry Pi it skips this step,
- adds the user to the `dialout` (serial), `video` (camera) and `i2c` (IMU) groups. Log
  out and back in for them to apply to your own shell.

Options: `--user NAME` (default: the user who ran sudo), `--pixi PATH`, `--no-apt` (skip
`can-utils` and `i2c-tools`), `--no-enable` (install the service but do not start it at
boot). Running it again is safe. `sudo deploy/install.sh --uninstall` removes the udev
rule, network file and service; it leaves `/etc/default/iot-robot`, the group
memberships, the I2C setting, `can-utils`, `i2c-tools` and systemd-networkd in place.

The service runs as that user from this checkout, so updating the robot is:

```bash
git pull --recurse-submodules
pixi run -e robot build
sudo systemctl restart iot-robot    # gizmo at its zero pose first
```

### Settings

Launch arguments for the service go in `IOT_ROBOT_ARGS` in `/etc/default/iot-robot`,
separated by spaces. Restart the service after editing it. For example:

```bash
IOT_ROBOT_ARGS=gizmo_mode:=fixed                      # gizmo motors not driven (default: can)
IOT_ROBOT_ARGS=camera:=false web:=false
IOT_ROBOT_ARGS=motors:=/home/pi/motors.yaml           # another motor file, absolute path
```

`pixi run -e robot ros2 launch iot_robot_bringup robot.launch.py --show-args` lists the
arguments. `ros2 launch` silently ignores arguments it does not know, so a misspelt one
has no effect and no error.

**Motor CAN IDs and directions are not service settings.** They live in
`src/iot_robot_bringup/config/motors.yaml` in this checkout
([docs/hardware.md](../docs/hardware.md#motorsyaml)): 10 gizmo yaw, 11 gizmo pitch,
12 left wheel, 13 right wheel, confirmed against the hardware on 2026-09-24. Edit that
file (or run `pixi run -e robot can-identify --write` to re-check the mapping after a
motor is replaced or given a new ID); the change takes effect at the next
`sudo systemctl restart iot-robot`, without a rebuild, because the build installs the
file as a link. `IOT_MOTOR_IDS` and the launch arguments `left_can_id`, `right_can_id`,
`left_direction` and `right_direction` no longer exist. `install.sh` keeps an existing
`/etc/default/iot-robot`, so on a Pi installed before this change, remove them from that
file by hand: nothing reads them any more.

`IOT_CAN_INTERFACE` (default `can0`) is the interface the stop step below uses. If
`IOT_ROBOT_ARGS` sets `can_interface`, set it to the same value. The stop step always
reads the package's `motors.yaml`, even when `motors:=` points elsewhere, but it also
stops every other motor it hears, so a different file is still covered.

### Starting and stopping

```bash
sudo systemctl start iot-robot       # gizmo at its zero pose first
sudo systemctl restart iot-robot     # after an E-stop or a motor fault, or to apply settings
sudo systemctl stop iot-robot        # motors get zero speed, then are released
sudo systemctl disable iot-robot     # no longer start at boot (enable to undo)
journalctl -u iot-robot -e           # the latest log lines
```

After `stop` the wheels roll freely and the gizmo goes limp. Stop the service before
using `cubemars_tool jog` or `can-identify`, which refuse to run while the robot is
commanding the motors, or before opening `/dev/stm32` or the camera from another
program.

### Behaviour at boot

- The service does not wait for the CAN adapter. If `can0` or a motor is missing, for
  example because the E-stop is pressed, the launch's CAN pre-flight check fails with an
  explanation, the service exits, and systemd retries every 5 s until the motors answer.
  `journalctl -u iot-robot` shows why. The start after that zeroes the gizmo wherever it
  is, which is why the gizmo goes to its zero pose before the E-stop is released
  ([docs/checklist.md](../docs/checklist.md)).
- Each automatic restart (`Restart=always`) zeroes the gizmo again. A start that fails
  while the motors are activated, for example a motor that does not answer or cannot be
  zeroed, ends the launch, and systemd retries after 5 s. A motor fault while running
  does not: the launch keeps running with the motors stopped until someone restarts it.
- The service uses `pixi run --frozen`, which installs exactly what `pixi.lock` pins and
  never re-solves. A boot never waits on the network or changes package versions.
- Stopping the service sends SIGINT, the same as Ctrl-C, so the controllers shut down
  cleanly and the motor plugin stops the motors. `KillMode=mixed` sends it to `ros2
  launch` only, which forwards it to each node once; with the default `KillMode` every
  node would get it twice and some would die mid-shutdown. Whatever still runs after
  15 s gets SIGKILL.
- After every stop, crash or kill, `ExecStopPost` runs
  `cubemars_tool --interface ${IOT_CAN_INTERFACE} stop`: zero speed for 0.3 s, then
  release, to every CAN ID in `motors.yaml` plus any other motor it hears on the bus. It
  covers the case where the launch itself dies before it can stop the motors, and does
  nothing when the CAN link is not up. The journal shows
  `Motors 12, 13, 10, 11 on can0: zero speed for 0.3 s, then released` (the tool lists
  the IDs in `motors.yaml` order, wheels first, not in numerical order). This step
  ignores `gizmo_mode`: it stops all four motors even when the gizmo was not driven.
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
has only two receive buffers, so at 1 Mbit/s with four motors reporting at 200 Hz it can
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
candump can0                         # status frames 0000290A to 0000290D from the motors
pixi run -e robot can-check          # interface up and every motor in motors.yaml reporting
pixi run -e robot can-watch          # live values of every motor, labelled with its joint
```

A status frame's ID is `0x2900` plus the motor's CAN ID, so CAN ID 10 (gizmo yaw) sends
`0000290A` and CAN ID 12 (left wheel) sends `0000290C`.

`ERROR-PASSIVE` or `BUS-OFF` means frames are not being acknowledged: motors unpowered,
CANH and CANL swapped, a bitrate mismatch, or missing termination. With everything
powered off, the resistance between CANH and CANL should be about 60 ohm (two 120 ohm
terminators in parallel).
