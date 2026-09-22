# One-time setup

Everything that has to be done once before the robot is used, in the order to do it.
Work from top to bottom and tick each box. Items marked **Decision** need your choice;
the recommended option comes first. The routine for every session afterwards is in
[checklist.md](checklist.md).

Commands run on the Raspberry Pi in the checkout (`~/iot-ros`) unless a step says
otherwise. ROS commands need the `pixi run -e robot` prefix, because there is no system
ROS install on the Pi ([hardware.md](hardware.md#ros-environment)).

## 1. Build and wire

- [ ] Get the parts: [wiring.md, Bill of materials](wiring.md#bill-of-materials).
- [ ] **Decision:** the buck converter model and how the Pi gets 5.1 V, option A1 or A2
      ([wiring.md, Feeding 5.1 V into the Pi](wiring.md#feeding-51-v-into-the-pi)).
      The buck needs 5.1 V, 3 A or more and an input rating of 30 V or more.
- [ ] **Decision (optional):** ultrasonic sensors on 5 V with the ECHO dividers (as
      drawn) or on 3.3 V without them
      ([wiring.md, Alternative: HC-SR04P on 3.3 V](wiring.md#alternative-hc-sr04p-on-33-v)).
- [ ] Before building the CAN harness, measure a bare motor and the bare adapter for
      built-in terminators ([wiring.md, Termination check](wiring.md#termination-check)).
- [ ] Power harness, fuses and E-stop:
      [wiring.md, section 1](wiring.md#1-power-distribution).
- [ ] CAN harness: [wiring.md, section 2](wiring.md#2-can-bus) and
      [Rules for this bus](wiring.md#rules-for-this-bus).
- [ ] Nucleo and ultrasonic sensors:
      [wiring.md, section 3](wiring.md#3-nucleo-f401re-sensor-board).
- [ ] Camera ribbon: [wiring.md, section 4](wiring.md#4-camera-ribbon).
- [ ] IMU on the Pi header, mounted on the chassis 15-20 cm from the motors and power
      leads: [wiring.md, section 5](wiring.md#5-imu-lsm9ds1).
- [ ] **CAN termination check:** everything off and plugged together, CAN_H to CAN_L
      reads about 60 Ω ([table](wiring.md#termination-check)).
- [ ] The "Wiring checks (no power)" and "First power, Pi disconnected" parts of
      [wiring.md, Safety checklist](wiring.md#safety-checklist-before-first-power-on),
      including the buck output at 5.1 V before the Pi is connected.

## 2. Configure each motor (R-Link)

Configure each motor alone, before it joins the bus: new motors ship with CAN ID 1, and
two motors with the same ID cannot be told apart. Connect the CubeMars R-Link to the
motor's 3-pin UART port, open the CubeMars Upper Computer (Windows) and connect to the
R-Link's COM port at **921600 baud**. Details:
[hardware.md, Motor configuration](hardware.md#motor-configuration).

> **921600 is not the CAN bitrate.** It is the serial speed between the PC and the
> R-Link, used only while configuring a motor. The CAN bus between the motors and the Pi
> runs at **1 Mbit/s** (1000000 bit/s), and nothing on the Pi uses 921600.

| Setting in the Upper Computer | Value |
| --- | --- |
| Mode | Servo mode |
| CAN ID | 10 left wheel, 11 right wheel, 12 gizmo yaw, 13 gizmo pitch |
| CAN bitrate | 1 Mbit/s |
| Send status over CAN (`send_can_status`) | Enabled |
| Status upload frequency | 100 to 200 Hz (more also works) |
| CAN communication timeout (`timeout_msec`) | 200 to 300 ms. **Required** |
| Brake current after the timeout (`timeout_brake_current`) | 0 (freewheel) or about 1 A |

- [ ] Left wheel motor: settings above, CAN ID 10. Label the motor "10".
- [ ] Right wheel motor: CAN ID 11, label "11".
- [ ] Gizmo yaw motor: CAN ID 12, label "12".
- [ ] Gizmo pitch motor: CAN ID 13, label "13".
- [ ] Write down the firmware version the program shows for each motor.
- [ ] **Decision:** brake current after the CAN timeout. 0 (recommended) lets the robot
      roll out when the Pi or the link dies; a small braking current stops it sooner but
      also holds against pushing.

The IDs only have to be unique (1 to 254). If a motor ends up with another ID, that is
fixed in software in step 6, not here.

## 3. Raspberry Pi 4

- [ ] Raspberry Pi OS Lite (64-bit), with hostname (e.g. `iot-robot`), user, Wi-Fi and
      SSH set in Raspberry Pi Imager ([hardware.md, Operating system](hardware.md#operating-system)).
- [ ] pixi, the checkout and the build
      ([hardware.md, ROS environment](hardware.md#ros-environment)):

  ```bash
  curl -fsSL https://pixi.sh/install.sh | sh          # then log out and back in
  git clone --recurse-submodules <repository URL> ~/iot-ros
  cd ~/iot-ros
  pixi install -e robot --locked
  pixi run -e robot build
  ```

- [ ] System files, without starting the robot at boot yet:
      `sudo deploy/install.sh --no-enable`. It installs the `/dev/stm32` rule, `can0`
      at 1 Mbit/s, the service, `can-utils` and `i2c-tools`, enables I2C and adds you
      to the `dialout`, `video` and `i2c` groups ([deploy/README.md](../deploy/README.md)).
      `--no-enable` keeps the robot from starting (and zeroing the gizmo) at every boot
      until the checks below have passed.
- [ ] `sudo reboot`, which finishes the I2C setup and the group changes.
- [ ] Camera: `rpicam-hello --list-cameras` lists `imx219`
      ([hardware.md, Camera](hardware.md#camera)).
- [ ] Only for person following: copy `models/` from the laptop
      (`rsync -av models/ iot-robot.local:iot-ros/models/`, run on the laptop).

## 4. Flash the Nucleo

The Nucleo only reads the ultrasonic sensors (and, optionally, the battery voltage); the
Pi controls all four motors over CAN.

- [ ] Build and flash firmware 1.2.0 from the `iot-stm32` repository: its README,
      section [Flashing](https://github.com/natapol2547/iot-stm32#flashing)
      (`~/iot-stm32/README.md` on the PC).
- [ ] Plug the Nucleo's ST-LINK USB into the Pi: `ls -l /dev/stm32` shows a link to
      `ttyACM0` (or another number).
- [ ] Banner: `pixi run -e robot python -m serial.tools.miniterm /dev/stm32 115200`,
      press the black RESET button. Expect `# iot-stm32 1.2.0`, `# battery=off`,
      then about 15 lines per second like `D1:57.3,D2:-1.0`. Exit with Ctrl+].
- [ ] Sensors: `pixi run -e robot bridge` and, in a second terminal,
      `pixi run -e robot ros2 topic echo /ultrasonic/left --field range`. A hand 30 cm in
      front of the left sensor reads about 0.3. Repeat with `/ultrasonic/right`.

## 5. IMU

- [ ] `ls -l /dev/i2c-1` exists and `i2cdetect -y 1` shows `1e` and `6b`. Other
      addresses (`1c`, `6a`): set `mag_address` / `ag_address` in
      `src/iot_robot_bringup/config/imu.yaml`.
- [ ] `pixi run -e robot ros2 launch iot_robot_bringup imu.launch.py`, robot standing
      still. The log says `Estimating the gyroscope bias`, then `Gyroscope bias [...]`.
- [ ] In a second terminal: `pixi run -e robot ros2 topic hz /imu/data_raw` (about
      100 Hz) and `pixi run -e robot ros2 topic echo /imu/data_raw --once`.
- [ ] Axis check, robot level: the `imu_link` axis pointing up reads about +9.8 in
      `linear_acceleration`; turning the robot left gives a positive `angular_velocity`
      about the same axis ([wiring.md, section 5](wiring.md#5-imu-lsm9ds1)).
- [ ] Measure the IMU pose and enter it as the `imu_joint` origin in
      `src/iot_robot_description/urdf/robot.urdf.xacro` (now a placeholder,
      `xyz="0 0 0.05" rpy="0 0 0"`):
  - `xyz`: the chip's position in metres from `base_link`, which lies on the robot's
    centre line between the two wheel axles, 36 mm above the floor (x forward, y left,
    z up).
  - `rpy`: chip flat and facing up, `rpy="0 0 <yaw>"` with the angle from the robot's
    forward direction to `imu_link` x, counter-clockwise seen from above: x forward
    `0`, x to the left `1.5708`, x backward `3.1416`, x to the right `-1.5708`.
    Mounted upside down, add a roll of `3.1416` and repeat the axis check.
  - It takes effect at the next start of the robot; no rebuild
    ([why](hardware.md#ros-environment)).
- [ ] Optional: record `/imu/data_raw` with the robot still and replace the estimated
      `linear_acceleration_stddev` and `angular_velocity_stddev` in `imu.yaml`.

## 6. First power-on, wheels off the ground

Robot on blocks, gizmo free to move, E-stop (S1) and SW1 within reach. The robot
software must not be running (`systemctl is-active iot-robot` says `inactive`).

### Power and bus

- [ ] SW1 on with S1 **pressed**: the Pi boots, and `vcgencmd get_throttled` says
      `throttled=0x0` (no undervoltage).
- [ ] Release S1: all four drive LEDs light blue.
- [ ] `ip -details link show can0`: `can state ERROR-ACTIVE`, `bitrate 1000000`.
- [ ] `pixi run -e robot can-check`: one row per motor, state `ok`, 100-200 frames/s,
      ending in `OK: can0 is up and motors 10, 11, 12, 13 respond`. If a motor is
      missing, the output also lists the IDs it did hear; continue with the next step.

### Which motor is which ID

The mapping from CAN ID to joint lives in one file,
`src/iot_robot_bringup/config/motors.yaml`. The robot launch, the URDF, `cubemars_tool`
and the service's stop step all read it. The build installs it as a link to the source
file, so an edit takes effect at the next start of the robot, without a rebuild.

- [ ] `pixi run -e robot can-identify`. Each motor in turn moves about 5° out and back
      twice, then goes limp; answer which joint moved (its number or name; `r` repeats,
      `s` skips, `q` quits). The tool refuses to run while the robot software is
      commanding the motors.
- [ ] If the result differs from `motors.yaml`: `pixi run -e robot can-identify --write`
      saves the new `can_id` values and keeps the rest of the file. Or edit `can_id` in
      `motors.yaml` by hand.
- [ ] Label each motor with its joint as well as its ID.

`can-identify` and `can-watch` run `cubemars_tool identify` and `cubemars_tool watch`
(`pixi run -e robot ros2 run iot_robot_drivers cubemars_tool <command> --help`). To give
a motor a different ID on the motor itself, use the R-Link (step 2) and then update
`motors.yaml`.

### Units and direction

- [ ] Units: `pixi run -e robot can-watch`, then turn one wheel by hand a quarter turn.
      Its position changes by about 90 deg. About 900 means the motor reports rotor
      degrees, not output degrees: stop here and report it, because the gizmo limits
      and zeroing assume output degrees. Ctrl-C ends `can-watch`.
- [ ] Wheel directions, one at a time:

  ```bash
  pixi run -e robot ros2 run iot_robot_drivers cubemars_tool jog --joint wheel_joint_left
  pixi run -e robot ros2 run iot_robot_drivers cubemars_tool jog --joint wheel_joint_right
  ```

  The wheel turns for 2 s at 1 rad/s. If it turns the way that drives the robot forward,
  set `direction: 1` for that joint in `motors.yaml`, otherwise `-1`. The tool prints
  the rule at the end. The commands below shorten the same prefix to `...`.
- [ ] Gizmo yaw: put the camera roughly straight ahead, then
      `... cubemars_tool jog --joint gizmo_yaw_joint`. Correct is a turn to the left
      (counter-clockwise seen from above). Travel is limited to 15°.
- [ ] Gizmo pitch: tilt the camera up about 20° first, then
      `... cubemars_tool jog --joint gizmo_pitch_joint`. Correct is a tilt **down**.
- [ ] Fix any wrong `direction` in `motors.yaml`, and jog that joint again.
- [ ] Speed scale: `... cubemars_tool jog --joint wheel_joint_left --duration 6.283`
      turns the wheel about one revolution (slightly less, because of the start). Two
      revolutions or half of one means `pole_pairs` is wrong.

### Stops

- [ ] **CAN timeout (pass/fail):** start
      `... cubemars_tool jog --joint wheel_joint_left --duration 20`, then pull the
      CAN adapter's USB plug out of the Pi (not the CAN wires). The wheel stops within
      about 0.5 s. Repeat for the right wheel. If a wheel keeps turning, set
      `timeout_msec` (step 2) and repeat. **Do not drive the robot until both wheels
      pass.** Plug the adapter back in and run `pixi run -e robot can-check`.
- [ ] **Hardware E-stop:** start the same 20 s jog and press S1 while the wheel turns.
      The wheel coasts to a stop, the gizmo goes limp and the Pi stays up. Keep S1
      pressed until the jog has exited, then release it. Nothing moves.

### Gizmo zero pose

The motors forget their position at power-off, so the robot software makes the gizmo's
pose at the moment it starts the zero of both gizmo joints (`zero_on_start` in
`motors.yaml`). From the URDF joint axes and limits:

| Joint | Zero (0) means | Range |
| --- | --- | --- |
| `gizmo_yaw_joint` | Camera looks straight ahead, along the robot's forward direction | ±45°; positive turns left |
| `gizmo_pitch_joint` | Camera looks level (horizontal), robot on level ground | -45° to 0°; negative tilts up. 0 is the lowest the camera ever points |

A zero set in the wrong pose shifts every angle: a camera started while tilted down can
never look up the full 45°, and one started while tilted up is driven past its real
range, where the stall guard stops it.

- [ ] **Decision:** how to find the zero pose each time. Alignment marks across the yaw
      and pitch joints (recommended: quick to add, nothing to design), or a mechanical
      stop at pitch level (repeatable by feel, but a zero set with the head pressed
      against it leaves no margin, and the stall guard trips if it pushes).
- [ ] Add the marks or the stop.
- [ ] Measure the camera lens height above the floor and put a mark on a wall at that
      height, straight ahead of the robot. With a correct zero it sits at the image
      centre at yaw 0, pitch 0.

### First full start, by hand

- [ ] Put the gizmo at its zero pose and hold it there. Start the robot:
      `pixi run -e robot robot`. Keep holding the gizmo until it stiffens (the log says
      `Configured and activated gizmo_controller`), and keep the robot still for 1 s
      more for the IMU.
- [ ] The log shows `CAN pre-flight OK`, a `temporary origin set` line for
      `gizmo_yaw_joint` and `gizmo_pitch_joint`, and a `position commands clamped to`
      line for each gizmo joint.
- [ ] In a second terminal:
      `pixi run -e robot ros2 control list_controllers` shows `joint_state_broadcaster`,
      `diff_drive_controller` and `gizmo_controller` `active`;
      `pixi run -e robot ros2 control list_hardware_components` shows `WheelSystem` and
      `GizmoSystem` `active`.
- [ ] Open `http://iot-robot.local:8080` (your hostname). Hold STOP for 1 s to release
      the E-stop, press **Centre** under Camera aim: the wall mark is at the image
      centre. Drag the camera aim dot: left turns left, up tilts up.
- [ ] Drive with the joystick: forward turns both wheels forward; a left turn drives the
      right wheel forward and the left wheel backward.
- [ ] Web E-stop: press STOP; the wheels stop within 0.5 s and ignore the joystick.
- [ ] Ctrl-C while a wheel turns: the wheels stop at once. The log shows
      `Stopping the motors of WheelSystem` and `... GizmoSystem`, then
      `Sent zero speed and release to motors [10, 11, 12, 13] on can0`.
- [ ] Motor watchdog: start again (gizmo at its zero pose), release the web E-stop,
      drive, and press S1. Both components log `No status frames from the motor on ...`
      and stop. Then recover as in [checklist.md](checklist.md#during-use).

### Tuning

- [ ] **Stall guard:** with the robot running, aim the camera around its whole range
      from the web page while watching
      `pixi run -e robot ros2 topic echo /joint_states --field effort`. Set
      `stall_effort` in `src/iot_robot_bringup/urdf/iot_robot.urdf.xacro` (now 1.0 Nm,
      held for `stall_time_ms` 500) well above the largest gizmo effort seen. Takes
      effect at the next start.
- [ ] On the ground, slowly: drive 1 m straight, measured with a tape, and compare with
      `pixi run -e robot ros2 topic echo /diff_drive_controller/odom --field pose.pose.position`.
      A constant error: adjust `wheel_radius`; a curve: `wheel_separation` or a tyre
      (`src/iot_robot_bringup/config/controllers.yaml`).

## 7. Start at boot

- [ ] Gizmo at its zero pose (the service zeroes it as it starts), then
      `sudo systemctl enable --now iot-robot` and `journalctl -u iot-robot -f`.
- [ ] `sudo systemctl stop iot-robot`: the journal shows the stop lines from the
      Ctrl-C test and `Motors 10, 11, 12, 13 on can0: zero speed for 0.3 s, then released`.
- [ ] Reboot with S1 pressed and follow [checklist.md](checklist.md) once from the top.
- [ ] Calibrate the camera ([hardware.md, Camera calibration](hardware.md#camera-calibration)).
- [ ] Measure the person-detector rate on the Pi 4 (`pixi run -e robot ros2 topic hz
      /person/position` while following a person). The README's estimate is for a Pi 5.

## 8. Still open

| Item | State | What to do |
| --- | --- | --- |
| Battery capacity | Unknown (6S LiPo, XT60) | **Decision.** Only the run time depends on it. The pack's continuous rating (capacity × C rating) must cover at least 15 A. |
| Battery voltage on the web page | Off: firmware built with `battery=off`, divider not fitted | **Decision.** Until then, watch the pack with a LiPo checker on the balance lead. To add it: R1/R2 divider ([wiring.md](wiring.md#battery-sense-optional)) and firmware built with `-DAPP_ENABLE_BATTERY=1`. |
| Hardware E-stop feedback to ROS | Not built; the motor plugin stops when the drives go silent | **Decision.** A second contact on S1 read by the Nucleo could engage `/e_stop` automatically (wiring.md, [E-stop warning](wiring.md#1-power-distribution)). Needs firmware and driver work. |
| Gizmo pitch sag at start | The gizmo motors are released between the zeroing and `gizmo_controller` taking over (a second or two) | Hold the gizmo until it stiffens (in [checklist.md](checklist.md)). |
| Magnetometer | Not used (`use_mag: false` in `imu.yaml`); yaw in `/imu/data` drifts | **Decision.** Needs a hard- and soft-iron calibration on the finished robot first. |
| Ultrasonic crosstalk | Not seen yet | If phantom close readings appear, build the firmware with `-DAPP_US_SLOT_MS=40` (12.5 readings/s). |
| Old settings on a Pi installed before this version | `/etc/default/iot-robot` is kept by `install.sh` | Remove `IOT_MOTOR_IDS` and any `left_can_id`, `right_can_id`, `left_direction`, `right_direction`: they are ignored without a warning. |
