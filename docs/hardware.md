# Real hardware

How the real robot's software and hardware fit together: Raspberry Pi setup, motor
configuration, the gizmo, the IMU, how the motors are stopped, and troubleshooting.

- [todo.md](todo.md): the one-time setup, in order, as a checklist.
- [checklist.md](checklist.md): what to do in every session, including the gizmo zero
  pose.
- [wiring.md](wiring.md): wiring diagrams, parts list, fuses and CAN termination.
- [web.md](web.md): the browser controller.
- [deploy/README.md](../deploy/README.md): the system files installed on the Pi.

| Part | Connection | ROS side |
| --- | --- | --- |
| Raspberry Pi 4 Model B | 5.1 V from a buck converter | Everything runs in the `robot` pixi environment |
| 2 × CubeMars AK45-10 (V2.0, servo mode) for the wheels, CAN IDs 12 (left) and 13 (right) | USB-CAN adapter with native SocketCAN (`gs_usb`), 1 Mbit/s, `can0` | `cubemars_hardware_safe` in `ros2_control_node`: `WheelSystem`, `diff_drive_controller` |
| 2 × CubeMars AK45-10 for the gizmo (pan/tilt), CAN IDs 10 (yaw) and 11 (pitch) | Same CAN bus | `GizmoSystem`, `gizmo_controller` |
| Raspberry Pi Camera v2.1 (IMX219) on the gizmo | 15-pin ribbon to the Pi's CAMERA connector | `camera_ros` → `/camera/image_raw`, `/camera/camera_info` |
| Nucleo-F401RE with 2 × HC-SR04P | ST-LINK USB, `/dev/stm32`, 115200 8N1 | `stm32_bridge` → `/ultrasonic/left`, `/ultrasonic/right`, `/battery_state` |
| LSM9DS1 IMU | Pi I2C bus 1 (header pins 1, 3, 5, 9) | `lsm9ds1_node` → `/imu/data_raw`, `/imu/mag`; `imu_filter_madgwick` → `/imu/data` |
| 6S LiPo with XT60, main switch SW1, E-stop S1 | S1 cuts all four motors; the Pi stays on ([wiring.md](wiring.md#1-power-distribution)) | |

The Raspberry Pi controls all four motors over CAN, through `ros2_control`. The Nucleo
only reads sensors (the two ultrasonic sensors and, optionally, the battery voltage) and
drives no motors.

The CAN IDs come from `src/iot_robot_bringup/config/motors.yaml`
([motors.yaml](#motorsyaml)). Which ID sits on which joint was confirmed against the
hardware on 2026-09-24; `can-identify` re-checks it after a motor is replaced or given a
new ID.

`robot.launch.py` in `iot_robot_bringup` starts all of it, plus `robot_state_publisher`,
`twist_mux` and the web controller:

```bash
pixi run -e robot robot                                   # real hardware
pixi run -e robot robot-mock                              # no motors or camera needed
pixi run -e robot ros2 launch iot_robot_bringup robot.launch.py --show-args
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `mock` | `false` | Replace the motors with `mock_components`; no CAN needed |
| `camera` | `true` | Start `camera_ros` |
| `web` | `true` | Start the browser controller on port 8080 |
| `imu` | `true` | Start the LSM9DS1 driver and the orientation filter (`imu.launch.py`). A missing sensor never stops the robot; see [IMU](#imu) |
| `start_estopped` | `true` | Start with the web E-stop engaged, so a restart never drives by itself |
| `check_can` | `true` | Check the CAN link and the status frames of every motor in use before starting |
| `can_interface` | `can0` | SocketCAN interface |
| `motors` | the package's `config/motors.yaml` | Motor CAN IDs, directions and `zero_on_start`, per joint. Pass an absolute path to use another file |
| `stm32_port` | `/dev/stm32` | Serial port of the Nucleo |
| `gizmo_mode` | `can` | `can`: the gizmo motors are driven over CAN (`gizmo_controller`). `fixed`: the gizmo is not driven and `stm32_bridge` reports its joints at 0. No other value is accepted |

`ros2 launch` accepts arguments that a launch file does not declare, without a warning.
A misspelt argument, or one that no longer exists such as `left_can_id` or
`left_direction`, is silently ignored; `--show-args` lists the real ones.

## Raspberry Pi setup

### Operating system

Use **Raspberry Pi OS Lite (64-bit)**, Bookworm or later. The robot environment is built
for 64-bit ARM (`linux-aarch64`) and glibc 2.36 or newer; Bookworm has 2.36 and Trixie
2.41. Lite has no desktop, which leaves more memory and CPU for the perception models.
Raspberry Pi OS rather than Ubuntu because its kernel and camera stack are the ones the
Raspberry Pi camera documentation and tools assume.

In Raspberry Pi Imager, set the hostname (e.g. `iot-robot`, which gives
`iot-robot.local` on the network), the user, Wi-Fi and SSH before writing the card. A
32 GB card or larger is recommended: the robot environment is about 760 MB to download and
several GB installed.

### Power

A Pi 4 needs 5.1 V and a supply good for 3 A. It has no USB Power Delivery and no
`PSU_MAX_CURRENT` setting; its USB ports share 1.2 A, far more than the CAN adapter and
the Nucleo draw. Below about 4.63 V it logs undervoltage and slows down. Set the buck to
5.1 V and check it with a meter before connecting the Pi; the ways to feed it are in
[wiring.md](wiring.md#feeding-51-v-into-the-pi). Check under load with
`vcgencmd get_throttled`: `throttled=0x0` means no undervoltage since boot.

### ROS environment

Everything runs from the pixi environment named `robot`; no system ROS install is needed.
On the Pi every pixi command needs `-e robot`, because the default environment (MuJoCo,
RViz) only exists for the laptop.

```bash
curl -fsSL https://pixi.sh/install.sh | sh          # then log out and back in
git clone --recurse-submodules <repository URL> ~/iot-ros
cd ~/iot-ros
pixi install -e robot --locked
pixi run -e robot build
```

`--recurse-submodules` fetches `src/external/cubemars_hardware`, the upstream motor
driver; `src/external/cubemars_hardware_safe` in this repository builds on it (see
[Stopping the motors](#stopping-the-motors)). In an existing clone, run
`git submodule update --init` instead. `--locked` installs exactly the
versions in `pixi.lock` and fails instead of re-solving if the lock is out of date.

The build uses `--symlink-install`: `install/` holds links to the configuration, launch
and URDF files in `src/`. Edits to existing files, such as `motors.yaml`,
`controllers.yaml`, `imu.yaml` or the URDFs, take effect at the next start of the robot
without a rebuild. New files need `pixi run -e robot build` once, for example after a
`git pull` that adds one. `scripts/activate.sh` only sources `install/` when it was built
with the active pixi environment; otherwise it says so and asks for
`pixi run -e robot clean && pixi run -e robot build`.

The person-following models are not in git (`models/` is ignored; see the README). Copy
them from the laptop if the robot should follow people:

```bash
rsync -av models/ iot-robot.local:iot-ros/models/
```

### Camera

1. Connect the camera with the Pi powered off. The camera's own 15-pin ribbon fits the
   Pi 4's CAMERA connector; [wiring.md](wiring.md#4-camera-ribbon) shows which way round.
2. Raspberry Pi OS detects the camera itself (`camera_auto_detect=1` in
   `/boot/firmware/config.txt`). Check it:

   ```bash
   rpicam-hello --list-cameras     # sudo apt install rpicam-apps-lite if missing
   ```

   It should list `imx219` with a `1640x1232` mode.
3. Check it in ROS without the motors:

   ```bash
   pixi run -e robot ros2 launch iot_robot_bringup robot.launch.py mock:=true web:=false
   pixi run -e robot ros2 topic hz /camera/image_raw        # second terminal
   ```

Only one program can use the camera at a time. Stop the robot (or the service) before
running `rpicam-hello`, and the other way round.

The image settings are in `src/iot_robot_bringup/config/camera.yaml`: sensor mode
1640:1232 (the full field of view, 2 × 2 binned) scaled to 640 × 480. The smaller sensor
modes crop the image, which would change the field of view the perception nodes assume.
The camera uses the Raspberry Pi fork of libcamera from conda-forge, pinned in
`pixi.toml`. It is independent of the host's libcamera, which `rpicam-hello` uses (0.7
on Raspberry Pi OS Trixie). The conda build needs `LIBCAMERA_IPA_PROXY_PATH` and
`LIBCAMERA_IPA_CONFIG_PATH`, which `scripts/activate.sh` sets for every `pixi run`; the
comment there explains why. Because the conda build's IPA module signatures do not
verify, libcamera runs the IPA in a separate process, which the upstream
`ros-jazzy-camera-ros` 0.7.0 cannot start (it aborts in `Camera::start()` with
`A list of V4L2 controls requires a ControlInfoMap`). The workspace therefore builds a
patched copy in `src/external/camera_ros`, which overlays the pixi package; its
`ParameterHandler.hpp` and `CMakeLists.txt` mark the two changes with `iot-ros:`
comments. To test the camera through pixi's libcamera, with the robot
stopped: `pixi run -e robot cam -l`, then
`pixi run -e robot cam -c1 --capture=30 -s width=640,height=480`.

### STM32 sensor board

The Nucleo only reads sensors; it drives no motors. Flash firmware 1.2.0 from the
[iot-stm32](https://github.com/natapol2547/iot-stm32) repository (its README, section
"Flashing"). Plug the Nucleo's ST-LINK USB port into the Pi. After `deploy/install.sh` it
appears as `/dev/stm32`; before that it is `/dev/ttyACM<n>` and needs the `dialout`
group. The firmware sends one line per measurement at about 15 Hz:

```text
D1:<left cm>,D2:<right cm>[,V:<battery volts>]\r\n       # -1.0 or 0.0 means no echo
# comment lines start with '#': after every reset the banner "# iot-stm32 1.2.0"
# and the compiled-in features, "# battery=off" by default
# warning: <left|right> <fault>     after about 1 s of failed pings, repeated every ~10 s
# info: <left|right> sensor recovered
```

The `V:` field only appears in a firmware built with battery sensing
(`APP_ENABLE_BATTERY=1`). The link is one-way: the Pi sends nothing to the board.
`stm32_bridge` publishes the distances as `sensor_msgs/Range` in metres (no echo is
`+inf`) and the voltage as `sensor_msgs/BatteryState`, and reconnects on its own when
the board is unplugged or reset. With `gizmo_mode:=fixed` it also publishes the gizmo
joint states; with `can` the joint state broadcaster does.

A failed ping is sent as `-1.0`, the same as open space, so an unplugged sensor would
look permanently clear. The firmware therefore reports a lasting failure with a
`# warning:` line. From that line until `# info: <side> sensor recovered` or the next
boot banner, the bridge publishes `NaN` (REP 117's invalid reading) on that side instead
of `+inf`, and logs `STM32: warning: <side> <fault>. Publishing NaN on /ultrasonic/<side>
until the sensor recovers`. The web page shows that side as grey "No data" and refuses
forward motion until it recovers (see `require_ultrasonic` in [web.md](web.md)). The faults and
what to check for each are listed in the firmware repository's README (`iot-stm32`,
section "Testing over the serial port"). A sensor reported as `echo stuck high` usually
needs a power cycle: unplug the Nucleo's USB for a few seconds (the RESET button does not
power the sensors down).

```bash
pixi run -e robot bridge                                    # with the board
pixi run -e robot ros2 topic echo /ultrasonic/left
```

Without the board, `fake_stm32` emulates it on a pseudo-terminal:

```bash
pixi run -e robot fake-stm32 --battery                      # creates /tmp/fake_stm32
pixi run -e robot bridge --ros-args -p port:=/tmp/fake_stm32
```

### IMU

The LSM9DS1 sits on the Pi's I2C bus 1 ([wiring.md](wiring.md#5-imu-lsm9ds1)).
`deploy/install.sh` enables I2C with `raspi-config`, installs `i2c-tools` and adds the
user to the `i2c` group; reboot afterwards. Check it:

```bash
ls -l /dev/i2c-1
i2cdetect -y 1                  # 1e (magnetometer) and 6b (accelerometer/gyroscope)
pixi run -e robot ros2 launch iot_robot_bringup imu.launch.py     # on its own
pixi run -e robot ros2 topic hz /imu/data_raw                     # about 100 Hz
```

`robot.launch.py` starts the same two nodes (`imu:=true`, the default; `imu:=false` leaves
them out); `imu.launch.py` runs them on their own, without the `base_link` to `imu_link`
transform. Parameters are in `src/iot_robot_bringup/config/imu.yaml`. In an environment
without the IMU packages (the laptop's default environment, e.g. `pixi run robot-mock`),
`robot.launch.py` logs `Not starting the IMU` and starts everything else.

| Topic | Type | Content |
| --- | --- | --- |
| `/imu/data_raw` | `sensor_msgs/Imu` | Angular velocity (rad/s, gyroscope bias removed) and linear acceleration (m/s², including gravity); no orientation (`orientation_covariance[0]` is -1) |
| `/imu/mag` | `sensor_msgs/MagneticField` | Magnetic field in tesla |
| `/imu/data` | `sensor_msgs/Imu` | The same plus an orientation from `imu_filter_madgwick` (node `imu_filter`) |

- All three are in the frame `imu_link`, at about 100 Hz (`rate`). The chip runs at
  119 Hz (`odr`), ±4 g, 500 °/s and ±4 gauss.
- At start the node measures the gyroscope bias from 100 samples
  (`gyro_bias_samples`) and publishes nothing until the robot has stood still for one
  window (every sample within 0.05 rad/s, `gyro_bias_tolerance`). Keep the robot still
  for about 1 s after the software starts. If it moves, the log says
  `The robot moved while the gyroscope bias was measured` and the node tries again.
- The filter runs without the magnetometer (`use_mag: false`): the motors and the battery
  current disturb it, and it needs a hard- and soft-iron calibration first. Yaw in
  `/imu/data` therefore starts at 0 and drifts; roll and pitch follow gravity. The filter
  publishes no TF (`publish_tf: false`).
- A missing or failing chip is not fatal. The node logs a warning, retries every second
  (`reconnect_period`) and reinitialises the chip when no new sample arrives for 0.5 s
  (`stale_timeout`). A parameter error ends it with a `FATAL` message, and the launch
  restarts it after 2 s.
- The pose of `imu_link` on `base_link` is a placeholder (`imu_joint` in
  `src/iot_robot_description/urdf/robot.urdf.xacro`, `xyz="0 0 0.05" rpy="0 0 0"`).
  Measure and set it once the IMU is mounted ([todo.md](todo.md#5-imu)).

## CAN adapter

The motor driver needs a **SocketCAN** interface, `can0`, at 1 Mbit/s. The adapter on
this robot is a native SocketCAN (`gs_usb`, candleLight) USB adapter, confirmed working.
`deploy/install.sh` configures `can0` with systemd-networkd whenever the adapter
appears. Other adapters, and why some cheap USB-CAN adapters cannot be used at all, are
described in [deploy/README.md, "CAN adapters"](../deploy/README.md#can-adapters).

- **Termination:** a CAN bus needs a 120 ohm resistor at each end: here the adapter and
  the far end of the chain. With everything powered off, measure between CANH and CANL:
  about 60 ohm is correct, about 120 ohm means one terminator is missing, about 40 ohm
  means there are three. See [wiring.md](wiring.md#termination-check).
- **Wiring:** one chain, CANH to CANH, CANL to CANL, and a common ground between the
  adapter and the motors ([wiring.md](wiring.md#2-can-bus)). Plug and unplug CAN wires
  only with the motor supply off.

Bring the link up by hand with `pixi run -e robot can-up` (it runs
`sudo deploy/scripts/can_up.sh`), or install `deploy/` so it comes up at boot.

## Motor configuration

The motor plugin drives the motors in **servo mode**: speed commands for the wheels,
position commands for the gizmo, and it reads their periodic status frames. Three things
have to agree: the settings stored in each motor, `motors.yaml`, and the wiring.

### R-Link settings

Configure each motor once, on its own, with CubeMars' R-Link adapter and the "Upper
Computer" program (Windows; downloads on the
[CubeMars support page](https://www.cubemars.com/article.php?id=261)). Connect the
R-Link to the motor's 3-pin UART port and open its COM port at **921600 baud**. That is
the serial link between the PC and the R-Link, used only for configuration; it is not the
CAN bitrate. The CAN bus runs at 1 Mbit/s, and nothing on the Pi uses 921600.

| Setting | Value | Why |
| --- | --- | --- |
| Mode | Servo mode | MIT (force control) mode uses a different CAN protocol |
| CAN ID | 10 gizmo yaw, 11 gizmo pitch, 12 left wheel, 13 right wheel | Must match `motors.yaml`. Any unique ID from 1 to 254 works; new motors ship with ID 1 |
| CAN bitrate | 1 Mbit/s | The default; must match `can0` |
| Send status over CAN (`send_can_status`) | Enabled | Without it there is no position or velocity feedback, and the pre-flight check fails |
| Status upload frequency | 100 to 200 Hz | At least the controller manager's 50 Hz, or the driver warns about missing frames every cycle; `can-check` warns below 50 Hz. The plugin stops a component when one of its motors sends nothing for 100 ms (`status_timeout_ms`) |
| CAN communication timeout (`timeout_msec`) | 200 to 300 ms. **Required** | A motor keeps its last speed command until it gets another one. This timeout is the only thing that stops a wheel when the Pi, its power or the CAN link fails |
| Brake current after the timeout (`timeout_brake_current`) | 0 (freewheel) or a small braking current such as 1 A | What the motor does when the timeout fires. 0 lets the robot roll out; a braking current stops it sooner but also holds against pushing |

The names in brackets are the fields of the motor's application settings, as they
appear in a settings file exported from the Upper Computer (`.AppParams`). An export
published for AK70-10 motors in servo mode has `timeout_msec` 1000 and
`timeout_brake_current` 0, so do not assume the factory value is short enough: set it.
If your firmware shows the setting under another name, or not at all, the CAN timeout
test in [todo.md](todo.md#stops) decides whether the robot may be driven.

Record the firmware version shown by the program: if the motors ever disagree with
the driver, it is the first thing to compare.

### motors.yaml

`src/iot_robot_bringup/config/motors.yaml` is the only place the CAN IDs and directions
are set. `robot.launch.py` (the URDF and the CAN pre-flight check), `cubemars_tool` and
the service's stop step all read it.

```yaml
motors:
  wheel_joint_left:
    can_id: 12
    direction: 1
  wheel_joint_right:
    can_id: 13
    direction: -1
  gizmo_yaw_joint:
    can_id: 10
    direction: 1
    zero_on_start: true
  gizmo_pitch_joint:
    can_id: 11
    direction: 1
    zero_on_start: true
```

- `can_id`: the ID set on the motor with the R-Link, 1 to 254, unique on the bus. The
  four values above were confirmed against the hardware on 2026-09-24; re-check them
  with `can-identify` after a motor is replaced or given a new ID.
- `direction`: `1` or `-1`, so that a positive command moves the joint in its positive
  URDF direction: the wheels drive the robot forward, yaw turns the camera left
  (counter-clockwise seen from above), pitch tilts it down. The plugin applies it to
  every command and state. Unlike `can_id`, these four values are still assumptions:
  check each one with `cubemars_tool jog` ([todo.md](todo.md#units-and-direction)).
- `zero_on_start`: gizmo joints only. The pose at software start becomes the zero
  ([The gizmo](#the-gizmo)).

Edits take effect at the next start of the robot (Ctrl-C and start again, or
`sudo systemctl restart iot-robot`), without a rebuild, because the build installs the
file as a link. The launch checks the file first and stops with a list of every problem,
such as a missing joint, an ID outside 1 to 254 or the same ID on two joints. With
`gizmo_mode:=fixed` the gizmo entries may be missing. To use another file,
pass `motors:=/absolute/path/motors.yaml`.

### Motor tools

`cubemars_tool` (in `iot_robot_drivers`) talks to the motors directly, with the robot
software stopped (`check`, `watch` and `stop` also work while it runs). It reads the IDs
and joint names from `motors.yaml`. Options before the command: `--interface` (default
`can0`), `--motors PATH`.

Two orders are in use, so a list of IDs is not always ascending. `check` and `stop`, and
the launch's own stop line, follow the joints — wheels first, then the gizmo — so their
IDs read 12, 13, 10, 11. `watch` and `identify` go by CAN ID instead: 10, 11, 12, 13.

| Command | pixi task | What it does |
| --- | --- | --- |
| `check` | `can-check` | Checks the link and lists every motor heard: state (`ok`, `FAULT`, `MISSING`, `UNEXPECTED` for an ID not in `motors.yaml`), values and status rate. Ends with `OK: can0 is up and motors 12, 13, 10, 11 respond`, or `FAIL:` with the IDs it did hear. `--gizmo-mode fixed` checks the wheels only, and ends with `OK: can0 is up and motors 12, 13 respond` |
| `watch` | `can-watch` | Live table of every motor: speed, position, current, temperature, fault. Only listens, so it also works while the robot runs. Values are as the motors report them, before `direction` |
| `identify` | `can-identify` | Moves each motor about 5° out and back, then asks which joint moved and prints the mapping. `--write` saves the `can_id` values to `motors.yaml`, keeping everything else. Refuses while another program commands the motors |
| `jog --joint NAME` (or `--id N`) | | Turns one motor at `--velocity` (default 1.0 rad/s at the output) for `--duration` (default 2 s), then says which `direction` to set. Gizmo travel is capped at 15° (`--max-gizmo-travel`) |
| `stop` | | Zero speed for 0.3 s, then release, to every ID in `motors.yaml` plus any other motor heard. Does nothing when the link is down |

```bash
pixi run -e robot can-check
pixi run -e robot can-identify --write
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool jog --joint gizmo_pitch_joint
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool stop
```

The step-by-step procedure for a new robot (IDs, directions, speed scale, CAN timeout
test) is in [todo.md, section 6](todo.md#6-first-power-on-wheels-off-the-ground).

### Units

The driver converts rad/s to electrical RPM with `pole_pairs` 14 and `gear_ratio` 10
(`src/iot_robot_bringup/urdf/iot_robot.urdf.xacro`). A wrong pole count makes every
speed wrong by the same factor, which the speed-scale jog in [todo.md](todo.md#units-and-direction)
catches. Positions are taken as output-shaft degrees, as the upstream driver does; the
quarter-turn check with `can-watch` in the same section confirms it on the real motors.
Effort is the motor current × `kt` 0.127 Nm/A × `gear_ratio`.

## The gizmo

The camera sits on a pan/tilt head (the "gizmo") driven by two AK45-10 motors in
position mode (`gizmo_mode:=can`, the default). `gizmo_controller`
(`position_controllers/JointGroupPositionController`) takes
`std_msgs/Float64MultiArray` `[yaw, pitch]` in radians on `/gizmo_controller/commands`,
from `target_follower` or the web page's camera aim.

| Joint | 0 means | Limits | Positive |
| --- | --- | --- | --- |
| `gizmo_yaw_joint` | Camera straight ahead | ±0.785 rad (±45°) | Turns left |
| `gizmo_pitch_joint` | Camera level | -0.785 to 0 rad (up to 45° up) | Tilts down; 0 is the lowest the camera points |

- **Zero on start.** The AK45-10 has a single-turn encoder and forgets its output
  position at power-off. When `GizmoSystem` activates, the plugin sets a temporary
  origin on both gizmo motors (CubeMars servo mode 5, "set origin", not saved in the
  motor), so the pose at that moment becomes 0. It checks that each motor then reads
  within 1° of 0 within 500 ms and logs `<joint> (CAN ID n): temporary origin set`.
  This happens at every start and restart of the robot software, which is why the gizmo
  must be at its zero pose then ([checklist.md](checklist.md)).
- **Hold.** Between the zeroing and `gizmo_controller` taking over (the spawner logs
  `Configured and activated gizmo_controller`) the motors are released, so the pitch can
  sag for a moment. From then on the gizmo holds its position.
- **Clamp and slew.** The plugin clamps every position command to the URDF limits and
  moves the target at no more than the URDF velocity limit (1 rad/s), starting from the
  measured position. At start it logs, for example,
  `gizmo_pitch_joint: position commands clamped to [-0.785, 0.000] rad, at most 1.000 rad/s`.
  In real mode the controller manager's own joint limiter is switched off for the gizmo
  joints (`<limits enable="false"/>` in `iot_robot.urdf.xacro`): its position check would
  deactivate `gizmo_controller` whenever the pitch, zeroed at its upper limit, overshot 0
  by half a degree. To slow the gizmo down, lower `velocity` on the gizmo joints in
  `src/iot_robot_description/urdf/robot.urdf.xacro`; that applies to the real robot, the
  mock and the simulation.
- **Stall guard.** When a gizmo motor pushes with more than `stall_effort` (1.0 Nm) for
  `stall_time_ms` (500 ms), the plugin stops the gizmo: it is blocked, or pressing
  against a hard stop because its zero was set in the wrong pose. Both values are in
  `src/iot_robot_bringup/urdf/iot_robot.urdf.xacro` and are starting values; tune them
  on the robot ([todo.md](todo.md#tuning)). The wheels have no stall guard.
- **`gizmo_mode:=fixed`** leaves the gizmo motors (CAN IDs 10 and 11) out of
  `ros2_control`: nothing commands, zeroes or checks them, and `stm32_bridge` publishes
  both gizmo joints at 0. The CAN pre-flight check and the launch's own stop then cover
  the wheels only, CAN IDs 12 and 13. `cubemars_tool stop`, which the systemd service
  runs after every stop, still covers all four plus anything else it hears.

## Stopping the motors

A motor in servo mode keeps its last command until it receives another one. The motors
belong to two `ros2_control` hardware components that stop independently:
`WheelSystem` (both wheels) and `GizmoSystem` (yaw and pitch). A stop always means zero
speed for 300 ms (`brake_time_ms`), then zero current (release): a stopped wheel rolls
freely and a stopped gizmo goes limp.

| Situation | What stops |
| --- | --- |
| No velocity command for 0.5 s (teleop closed, web page closed) | `diff_drive_controller` commands zero speed (`cmd_vel_timeout`). Wheels only |
| Web E-stop | `twist_mux` blocks every velocity input; the wheels stop within 0.5 s. The gizmo is not stopped: a running follower keeps aiming it, and the page refuses camera aim until the E-stop is released |
| Ctrl-C or `systemctl stop` | The plugin stops both components. `robot.launch.py` sends the same stop to every motor in use once `ros2_control_node` has exited |
| A wheel motor reports a fault or sends no status for 100 ms | `WheelSystem` stops both wheels and logs a `FATAL` message naming the motor. `diff_drive_controller` and `joint_state_broadcaster` go inactive, so odometry and `/joint_states` stop. The gizmo keeps working |
| A gizmo motor reports a fault, sends no status for 100 ms, or stalls | `GizmoSystem` stops both gizmo motors. `gizmo_controller` and `joint_state_broadcaster` go inactive. The wheels keep driving and odometry continues, after a pause of about 300 ms while the gizmo stop runs |
| Hardware E-stop S1 | All four drives lose power; both components stop with `No status frames from the motor on ...` |
| A controller is deactivated while its component stays active | That controller's motors get the stop (`No controller commands ... any more`) |
| A motor cannot be activated at start (silent, fault, zero failed) | `ros2_control_node` exits, `robot.launch.py` stops all motors and exits; the service retries after 5 s |
| `ros2_control_node` crashes or is killed | `robot.launch.py` sends the stop itself, then ends the launch |
| The launch itself is killed | `iot-robot.service` runs `cubemars_tool stop` after every stop (`ExecStopPost`) |
| The Pi hangs or loses power, or the CAN link breaks | Only the motors' own `timeout_msec` ([R-Link settings](#r-link-settings)) |

The plugin is `cubemars_hardware_safe`, a subclass of the upstream `cubemars_hardware`
that adds the stops, the fault check, the status watchdog, the per-joint direction, the
gizmo clamp, zeroing and stall guard. It sends no command frames unless its component is
active. Its parameters are in `src/iot_robot_bringup/urdf/iot_robot.urdf.xacro`:
`status_timeout_ms` (100) and `brake_time_ms` (300) for both components, and
`stall_effort`, `stall_time_ms` for the gizmo joints. Keep `status_timeout_ms` above two
status periods of the motors.

**After a fault, a silent motor, a stall or the hardware E-stop the stop is latched.**
The component logs `The motors of <component> were stopped after an error. They stay
stopped until the robot is restarted.` The launch keeps running, so the service does not
restart by itself. Fix the cause, put the gizmo at its zero pose, then restart: Ctrl-C
and start again, or `sudo systemctl restart iot-robot`. The restarted robot starts with
the web E-stop engaged (`start_estopped:=true`), so it only moves once someone releases
the stop on the page. The full order after an S1 press is in
[wiring.md](wiring.md#1-power-distribution) and [checklist.md](checklist.md#during-use).

The service restarts the robot by itself only when the launch exits, for example after
an activation failure. Each start zeroes the gizmo wherever it is at that moment, so
check the zero after an unexpected restart.

To stop the motors by hand, for example after an interrupted bench test:

```bash
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool stop                  # motors.yaml IDs and any heard
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool --interface can0 stop --ids 12 13   # the two wheels only
```

## Running at boot

```bash
sudo deploy/install.sh
sudo systemctl start iot-robot
journalctl -u iot-robot -f
```

This installs the `/dev/stm32` udev rule, brings `can0` up at 1 Mbit/s with
systemd-networkd whenever the adapter appears, enables I2C for the IMU, and runs
`pixi run -e robot robot` as your user at boot, restarting it if it exits. If the motors
are not powered yet (E-stop pressed), the CAN pre-flight check fails and the service
retries every 5 s; the robot starts, and zeroes the gizmo, within seconds of the E-stop
being released. After every stop, crash or kill of the launch, the service sends zero
speed and release to the motors in `motors.yaml` and any other motor it hears
(`cubemars_tool stop`). Launch arguments go in `IOT_ROBOT_ARGS` in `/etc/default/iot-robot`;
if they change `can_interface`, set `IOT_CAN_INTERFACE` there to match. Details, updating
and uninstalling: [deploy/README.md](../deploy/README.md).

## Camera calibration

The robot starts with a nominal calibration computed from the datasheet field of view
(fx = fy = 529.1 px, principal point at the image centre, no distortion), the same as
the simulated camera. Distance estimates from the camera are only as good as this, so
calibrate the real camera once it is mounted.

On first start, the launch file copies the nominal calibration to
`~/.ros/camera_info/iot_robot_camera.yaml` on the Pi and points `camera_ros` at it.
`camera_calibration`'s "commit" overwrites that file, so the result survives rebuilds.
Delete the file to go back to the nominal values.

1. Print a checkerboard and measure a square. The example assumes 8 × 6 inner corners
   and 25 mm squares.
2. Start the robot (or the service) on the Pi.
3. On the laptop, with the same `ROS_DOMAIN_ID`, calibrate from the compressed stream;
   the raw 640 × 480 stream is about 27 MB/s at 30 fps, too much for most Wi-Fi links:

   ```bash
   pixi run ros2 run image_transport republish --ros-args \
     -p in_transport:=compressed -p out_transport:=raw \
     -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_calib
   pixi run ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
     --ros-args -r image:=/camera/image_calib -p camera:=/camera
   ```

4. Move the board around the whole image, tilted and at different distances, until
   **CALIBRATE** is enabled. Click it, check the result, then click **COMMIT**. That
   sends the calibration to the camera node, which writes it to the file above.

`camera_ros` names the camera from the sensor model, its ID and the resolution. A log
message that this name does not match `iot_robot_camera` in the file is harmless; the
calibration is loaded anyway, and a commit writes the camera's own name.

## Testing without hardware

These run on the laptop as well as the Pi:

```bash
pixi run robot-mock                     # robot.launch.py with mock motors, no camera
pixi run fake-stm32 --battery           # then: pixi run bridge --ros-args -p port:=/tmp/fake_stm32

# Unit tests for the drivers (protocol, bridge against the fake board, CAN codec,
# motors.yaml, IMU driver); the CAN bus tests are skipped
pixi run bash -c 'cd src/iot_robot_drivers && python -m pytest test'
```

The CAN tests, including the motor plugin end to end, run against `fake_cubemars`, an
emulator of the AK45-10 servo protocol, on a virtual CAN bus in a throwaway network
namespace (no root). They need the robot environment, because the laptop environment
does not build the plugin:

```bash
pixi run -e robot build
pixi run -e robot bash -c 'cd src/iot_robot_drivers && unshare -rn sh -c "ip link set lo up && \
  ip link add dev vcan0 type vcan && ip link set vcan0 up && \
  IOT_TEST_CAN_INTERFACE=vcan0 python -m pytest test"'
```

`setup.cfg` in `iot_robot_drivers` passes `-p no:launch_testing -p no:launch_ros` to
pytest. Those ROS launch plugins use a hook signature that the installed pytest no longer
accepts (`PluginValidationError`), and these tests do not need them. The vcan
tests are skipped unless `IOT_TEST_CAN_INTERFACE` is set; they need the `vcan` kernel
module, which most desktop kernels load on demand.

`fake_cubemars` also runs the whole robot without motors (`sudo` for the vcan
interface; it refuses to run on a real CAN interface):

```bash
sudo ip link add vcan0 type vcan && sudo ip link set vcan0 up
pixi run -e robot ros2 run iot_robot_drivers fake_cubemars --interface vcan0 --position 10=20
pixi run -e robot robot can_interface:=vcan0 camera:=false        # second terminal
```

It answers for CAN IDs 10 to 13 by default. `--fault ID=CODE[@S]`, `--silent ID[@S]`,
`--load-current ID=A`, `--stuck ID` and `--ignore-origin ID` fake a fault, a silent
motor, a stall and a failed zero; `--help` lists them all.

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| Launch exits: `Invalid motor configuration ...` with a list | `motors.yaml` has a missing joint, a bad or duplicate `can_id`, or a bad `direction` | Fix the listed lines ([motors.yaml](#motorsyaml)) |
| Launch exits: `CAN interface 'can0' does not exist` | Adapter unplugged, or not a SocketCAN adapter | [CAN adapter](#can-adapter) |
| Launch exits: `CAN interface 'can0' is down` | Bitrate never configured | `pixi run -e robot can-up`, or `sudo deploy/install.sh` |
| Launch exits: `No status frames from gizmo_pitch_joint (CAN ID 11) ...` followed by `Status frames were heard from ... CAN ID n (not in motors.yaml)` | That motor has another ID than `motors.yaml` says | `pixi run -e robot can-identify --write`, or set its `can_id` in `motors.yaml` |
| Launch exits: `No status frames from ...` and no other IDs heard | Motors unpowered (E-stop pressed, SW1 off), status feedback disabled, wiring, termination | Release S1; `pixi run -e robot can-check`; [R-Link settings](#r-link-settings). The service retries by itself every 5 s |
| `can0` is `ERROR-PASSIVE` or `BUS-OFF` | Nothing acknowledges frames: motors off, CANH/CANL swapped, wrong bitrate, termination | Check wiring and the 60 ohm reading, then `pixi run -e robot can-up` |
| `can0` is `DOWN` after boot; `networkctl status can0` shows `off (failed)` and the journal `Device doesn't support restart from Bus Off` | An older `80-iot-robot-can0.network` with `RestartSec=`, which gs_usb rejects | Rerun `sudo deploy/install.sh`; it installs the current file and restarts systemd-networkd |
| `No CAN message received from CAN ID` warnings every cycle, or `can-check` warns `sends status at N Hz` | Status upload slower than 50 Hz | Raise it to 100 to 200 Hz in the Upper Computer |
| Launch exits: `Motor fault: ...` | The motor reports a fault (voltage, temperature, stall) | Fix the cause, power-cycle the motor |
| `Cannot activate GizmoSystem: no status frames from the motor on ...` or `... reports fault ...` | A motor went silent or faulted between the pre-flight check and the start | As for the two rows above. The service retries after 5 s and zeroes the gizmo again: put it at its zero pose |
| `Could not zero gizmo_... (CAN ID n): it reads ... deg ... after the set-origin command` | The motor ignored the set-origin command: not servo-mode firmware, or another program commanding it | Check the mode in the Upper Computer; stop other programs (`cubemars_tool` jogs) |
| `... has pushed with ... Nm (stall_effort ...)` and the gizmo goes limp | The gizmo is blocked, was forced by hand, or was zeroed in the wrong pose and is pressing against a hard stop | Free it, put it at its zero pose, restart. If it trips during normal moves, raise `stall_effort` ([The gizmo](#the-gizmo)) |
| The camera does not look straight ahead and level after **Centre** | The gizmo was not at its zero pose when the software started | Put it there and `sudo systemctl restart iot-robot` ([checklist.md](checklist.md)) |
| `Motor on ... (CAN ID n) reports fault ...: ... Stopping the motors of WheelSystem.` (or `GizmoSystem`) | A motor reported a fault while running (voltage sag, over-temperature) | Fix the cause, then restart the robot ([Stopping the motors](#stopping-the-motors)) |
| `No status frames from the motor on ... for ... ms` | The hardware E-stop was pressed, or a motor lost power or its CAN cable | Recover as in [checklist.md](checklist.md#during-use) |
| `The motors of ... were stopped after an error` and the robot no longer drives, or the camera aim does nothing | One of the messages above; the stop is latched on purpose | Restart: Ctrl-C and start again, or `sudo systemctl restart iot-robot`, with the gizmo at its zero pose |
| `/joint_states` stopped, the web page's "Base" is off | A component stopped after an error (see above) | Restart the robot |
| `Could not send stop frames` or `Could not send every stop frame` | The CAN link went away while stopping | The motors stop on their own `timeout_msec`; check the adapter |
| A wheel or gizmo joint moves the wrong way | `direction` for that joint | Flip it in `motors.yaml`; check with `cubemars_tool jog --joint <name>` |
| Odometry off by a constant factor | `wheel_radius`, or `pole_pairs` if it is 2× | The speed-scale and 1 m checks in [todo.md](todo.md#6-first-power-on-wheels-off-the-ground) |
| Launch exits: `The motor plugin package cubemars_hardware_safe is not installed` | Started from the laptop (default) environment, which does not build the motor plugin | `mock:=true` on the laptop; on the robot `pixi run -e robot build` and `pixi run -e robot robot` |
| Launch exits: `ros2_control_node exited with code ...` or `was killed by signal ...` | The controller manager failed to start or crashed, for example a motor that could not be activated. With `check_can:=false` this is usually a missing or down CAN interface | The `[FATAL]` lines just above it name the cause |
| `Another program is sending commands to CAN ID(s) ...` from `cubemars_tool` | The robot is running | `sudo systemctl stop iot-robot` (or Ctrl-C) first |
| `/dev/i2c-1 does not exist` from `lsm9ds1` | I2C not enabled | `sudo deploy/install.sh` (or `sudo raspi-config nonint do_i2c 0`) and reboot |
| `No permission to open /dev/i2c-1` | User not in the `i2c` group | `sudo deploy/install.sh`, then log in again |
| `No LSM9DS1 ... answers at I2C address 0x6B ...`, repeated every second | IMU unpowered or miswired, or another address | `i2cdetect -y 1`; set `ag_address` / `mag_address` in `imu.yaml` to what it shows |
| No `/imu/data_raw` although the IMU was found | The robot moved while the gyroscope bias was measured | Keep the robot still for about 1 s |
| `Cannot open /dev/stm32` | Board unplugged, udev rule not installed, user not in `dialout` | `ls -l /dev/stm32 /dev/ttyACM*`; `sudo deploy/install.sh`; log in again |
| `No data from STM32 ... for 1 s` | Firmware not running or wrong baud rate | Reset the Nucleo; check the firmware uses 115200 8N1 |
| `Ignoring malformed line from STM32` | Firmware prints another format | Compare with the protocol in [STM32 sensor board](#stm32-sensor-board) |
| `STM32: warning: <side> sensor not responding` (or `echo stuck high`, `echo pulses too short`) | The firmware sees no valid echo from that sensor: 5 V, GND, TRIG or ECHO wiring, or a locked-up sensor | Check the sensor's wiring ([wiring.md](wiring.md#3-nucleo-f401re-sensor-board)); for `echo stuck high` unplug the Nucleo's USB for a few seconds. `/ultrasonic/<side>` is `NaN` until `STM32: info: <side> sensor recovered` |
| `camera_ros` finds no camera | Ribbon reversed or loose, another program has the camera | `rpicam-hello --list-cameras` with the robot stopped |
| `camera_node` dies with `Call timeout!`, `Failed to call init: -110` and `no cameras available`, while `rpicam-hello` works | `LIBCAMERA_IPA_PROXY_PATH` is not set, e.g. the node was started outside `pixi run` | Start it through `pixi run -e robot`, which sources `scripts/activate.sh` |
| `camera_node` logs `FATAL Serializer ... A list of V4L2 controls requires a ControlInfoMap`, then `Failed to call start: -110`, and publishes no images | The unpatched `camera_ros` from pixi is running: the workspace was not built, or `install/` is not sourced | `pixi run -e robot build`; `ros2 pkg prefix camera_ros` must print `<repo>/install/camera_ros` |
| The laptop sees no topics from the Pi | Different `ROS_DOMAIN_ID`, or the Wi-Fi blocks multicast between clients | Match the domain; try a phone hotspot or wired link to rule out the network |
| `pixi` says the environment does not support `linux-aarch64` | Missing `-e robot` | Add `-e robot` to every command on the Pi |
| `activate.sh: install/ was built with another pixi environment` | `install/` comes from the other environment | `pixi run -e robot clean && pixi run -e robot build` |
| `can_frame has no member named len` when building | `cubemars_hardware` built in the laptop environment, whose kernel headers are too old | Build with `-e robot`; the laptop's `build` task skips it and `cubemars_hardware_safe` |
| `Could not enable FIFO RT scheduling policy` | No real-time permission when run by hand | Harmless; the service grants it (`LimitRTPRIO=99`) |
| `Velocity command timed out. Braking.` every second | No one is sending velocity commands | Normal while idle |
| The service restarts every few seconds | Launch fails, usually the CAN pre-flight while the E-stop is pressed | `journalctl -u iot-robot -e` shows the reason |
