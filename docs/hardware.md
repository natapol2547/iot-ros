# Real hardware

How to set up the Raspberry Pi, configure the motors, bring the robot up for the first
time and keep it running. Wiring diagrams are in [wiring.md](wiring.md); the browser
controller is described in [web.md](web.md); the system files installed on the Pi are
described in [deploy/README.md](../deploy/README.md).

| Part | Connection | ROS side |
| --- | --- | --- |
| 2 × CubeMars AK45-10 (servo mode, CAN IDs 1 and 2) | USB-CAN adapter or CAN HAT, 1 Mbit/s, `can0` | `cubemars_hardware_safe` plugin (wraps `cubemars_hardware`) in `ros2_control_node`, `diff_drive_controller` |
| Raspberry Pi Camera v2.1 (IMX219) | CSI ribbon cable | `camera_ros` → `/camera/image_raw`, `/camera/camera_info` |
| Nucleo-F401RE with 2 × HC-SR04 | ST-LINK USB, `/dev/stm32`, 115200 8N1 | `stm32_bridge` → `/ultrasonic/left`, `/ultrasonic/right`, `/battery_state` |
| 6S LiPo, 5 V buck converter for the Pi | | |

`robot.launch.py` in `iot_robot_bringup` starts all of it, plus `twist_mux` and the web
controller:

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
| `check_can` | `true` | Check the CAN link and both motors before starting anything |
| `can_interface` | `can0` | SocketCAN interface |
| `left_can_id`, `right_can_id` | `1`, `2` | Motor CAN IDs |
| `left_direction`, `right_direction` | `1`, `-1` | `1` if a positive motor speed drives the robot forward, else `-1` |
| `stm32_port` | `/dev/stm32` | Serial port of the Nucleo |
| `gizmo_mode` | `fixed` | `fixed`: the pan/tilt joints are reported at 0; `servo`: drive hobby servos through the Nucleo |

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

A Pi 5 wants 5 V at 5 A. It only learns what the supply can deliver through USB-C Power
Delivery, which a buck converter does not speak, so by default it limits all USB ports
together to 600 mA and warns at boot. The CAN adapter and the Nucleo draw far less than
that, so the default is fine. If the buck is rated for 5 A and more USB devices are
added, raise the limit with `PSU_MAX_CURRENT=5000` in `sudo rpi-eeprom-config --edit`.
Set the buck to 5.1 V and check it with a meter before connecting the Pi.

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
[Stopping the wheels](#stopping-the-wheels)). In an existing clone, run
`git submodule update --init` instead. `--locked` installs exactly the
versions in `pixi.lock` and fails instead of re-solving if the lock is out of date.

The person-following models are not in git (`models/` is ignored; see the README). Copy
them from the laptop if the robot should follow people:

```bash
rsync -av models/ iot-robot.local:iot-ros/models/
```

### Camera

1. Connect the camera with the Pi powered off. A Pi 5 has the smaller 22-pin camera
   connectors and needs a 22-pin to 15-pin cable; a Pi 4 takes the camera's own 15-pin
   cable. The Raspberry Pi camera documentation shows which way round it goes.
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
`pixi.toml`, so it behaves like `rpicam-hello` on the host.

### STM32 sensor board

Plug the Nucleo's ST-LINK USB port into the Pi. After `deploy/install.sh` it appears as
`/dev/stm32`; before that it is `/dev/ttyACM<n>` and needs the `dialout` group. The
firmware sends one line per measurement at about 15 Hz:

```text
D1:<left cm>,D2:<right cm>[,V:<battery volts>]\r\n       # -1.0 or 0.0 means no echo
# comment lines start with '#', e.g. the banner "# iot-stm32 1.0.0"
# warning: <left|right> <fault>     after about 1 s of failed pings, repeated every ~10 s
# info: <left|right> sensor recovered
```

and accepts `G:<yaw deg>,<pitch deg>\n` for the gizmo servos when `gizmo_mode:=servo`.
`stm32_bridge` publishes the distances as `sensor_msgs/Range` in metres (no echo is
`+inf`) and the voltage as `sensor_msgs/BatteryState`. It reconnects on its own when the
board is unplugged or reset.

A failed ping is sent as `-1.0`, the same as open space, so an unplugged sensor would
look permanently clear. The firmware therefore reports a lasting failure with a
`# warning:` line. From that line until `# info: <side> sensor recovered` or the next
boot banner, the bridge publishes `NaN` (REP 117's invalid reading) on that side instead
of `+inf`, and logs `STM32: warning: <side> <fault>. Publishing NaN on /ultrasonic/<side>
until the sensor recovers`. The web page shows that side as grey "No data" and refuses
forward motion until it recovers (see `require_ultrasonic` in [web.md](web.md)). The faults and
what to check for each are listed in the firmware repository's README (`iot-stm32`,
section "Testing over the serial port").

```bash
pixi run -e robot bridge                                    # with the board
pixi run -e robot ros2 topic echo /ultrasonic/left
```

Without the board, `fake_stm32` emulates it on a pseudo-terminal:

```bash
pixi run -e robot fake-stm32 --battery                      # creates /tmp/fake_stm32
pixi run -e robot bridge --ros-args -p port:=/tmp/fake_stm32 -p gizmo_mode:=servo
```

## CAN adapter

The motor driver needs a **SocketCAN** interface, `can0`, at 1 Mbit/s. What that takes
depends on the adapter, and some cheap USB-CAN adapters cannot be used at all. See
[deploy/README.md, "CAN adapters"](../deploy/README.md#can-adapters) for how to identify
yours and how each kind is set up.

- **Termination:** a CAN bus needs a 120 ohm resistor at each end, typically one in the
  adapter (often a jumper or switch) and one at the far motor. With everything powered
  off, measure between CANH and CANL: about 60 ohm is correct, about 120 ohm means one
  terminator is missing, about 40 ohm means there are three.
- **Wiring:** CANH to CANH, CANL to CANL, and a common ground between the adapter and the
  motors. See [wiring.md](wiring.md).

Bring the link up by hand with `pixi run -e robot can-up` (it runs
`sudo deploy/scripts/can_up.sh`), or install `deploy/` so it comes up at boot.

## Motor configuration

The motor plugin drives the motors in **servo mode** with speed commands and reads
their periodic status frames. Configure each motor once, on its own, with CubeMars'
R-Link adapter and the "Upper Computer" program (Windows; downloads on the
[CubeMars support page](https://www.cubemars.com/article.php?id=261)):

| Setting | Value | Why |
| --- | --- | --- |
| Mode | Servo mode | MIT (force control) mode uses a different CAN protocol |
| CAN ID | 1 for the left wheel, 2 for the right | Must match `left_can_id` / `right_can_id` |
| CAN bitrate | 1 Mbit/s | The default; must match `can0` |
| Send status over CAN (`send_can_status`) | Enabled | Without it there is no position or velocity feedback, and the pre-flight check fails |
| Status upload frequency | 100 to 200 Hz | At least the controller manager's 50 Hz, or the driver warns about missing frames every cycle. The plugin also stops both wheels when a motor sends nothing for 100 ms (`status_timeout_ms`) |
| CAN communication timeout (`timeout_msec`) | 200 to 300 ms. **Required** | The motor keeps its last speed command until it gets another one. This timeout is the only thing that stops a wheel when the Pi, its power or the CAN link fails |
| Brake current after the timeout (`timeout_brake_current`) | 0 (freewheel) or a small braking current such as 1 A | What the motor does when the timeout fires. 0 lets the robot roll out; a braking current stops it sooner but also holds against pushing |

The names in brackets are the fields of the motor's application settings, as they
appear in a settings file exported from the Upper Computer (`.AppParams`). An export
published for AK70-10 motors in servo mode has `timeout_msec` 1000 and
`timeout_brake_current` 0, so do not assume the factory value is short enough: set it.
If your firmware shows the setting under another name, or not at all, checklist step 7
decides whether the robot may be driven.

Record the firmware version shown by the program: if the motors ever disagree with
the driver, it is the first thing to compare.

The driver converts rad/s to electrical RPM with `pole_pairs` 14 and `gear_ratio` 10
(`src/iot_robot_bringup/urdf/iot_robot.urdf.xacro`). A wrong pole count makes every
speed wrong by the same factor, which the first power-on checklist catches.

## Stopping the wheels

A motor in servo mode keeps turning at the last speed it was sent until it receives
another command. Each way the software can stop is covered by a layer that sends a stop:

| Situation | What stops the wheels |
| --- | --- |
| No velocity command for 0.5 s (teleop closed, web page closed) | `diff_drive_controller` commands zero speed (`cmd_vel_timeout`) |
| E-stop pressed | `twist_mux` blocks every velocity input; 0.5 s later the controller commands zero speed |
| Ctrl-C, `systemctl stop`, controllers deactivated | The plugin sends zero speed for 300 ms, then zero current (release) |
| A motor reports a fault, or sends no status for 100 ms | The plugin stops both wheels and logs a `FATAL` message naming the motor. The robot stays stopped until it is restarted |
| `ros2_control_node` crashes or is killed | `robot.launch.py` sends the same zero speed and release itself, then ends the launch |
| The launch itself is killed | `iot-robot.service` runs `cubemars_tool stop` after every stop (`ExecStopPost`) |
| The Pi hangs or loses power, or the CAN link breaks | Only the motors' own `timeout_msec` ([Motor configuration](#motor-configuration)) |

The plugin is `cubemars_hardware_safe`, a subclass of the upstream `cubemars_hardware`
that adds the stop, the fault check and the status watchdog. Its parameters are in
`src/iot_robot_bringup/urdf/iot_robot.urdf.xacro`: `status_timeout_ms` (100) and
`brake_time_ms` (300). Keep `status_timeout_ms` above two status periods of the motors.
After the stop the motors are released and turn freely, so the robot can be pushed by
hand and nothing holds current while no program supervises the motors.

After a fault or a silent motor the hardware component and the controllers are
inactive, so velocity commands no longer reach the motors. Fix the cause, then restart: Ctrl-C and start again, or
`sudo systemctl restart iot-robot`. The service does not restart by itself in this case,
because the launch keeps running.

The hardware E-stop (S1 in [wiring.md](wiring.md)) cuts the drives' power, so their
status frames stop and the plugin latches the stop as for a silent motor. Releasing S1
therefore does not restart motion: the drives boot and receive no commands until the
robot is restarted. The restarted robot starts with the web E-stop engaged
(`start_estopped:=true`), so it only moves once someone releases the stop on the page.

To stop the motors by hand, for example after an interrupted bench test:

```bash
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool stop            # IDs 1 and 2 on can0
pixi run -e robot ros2 run iot_robot_drivers cubemars_tool --interface can0 stop --ids 1 2
```

## First power-on checklist

Do this once with a new robot, and again after changing motors, wiring or firmware.
Keep the battery disconnect switch in reach throughout.

1. **Wheels off the ground.** Put the robot on a stand so the wheels spin freely.
2. **Bus resistance.** With everything off, measure about 60 ohm between CANH and CANL.
3. **Buck output.** Battery connected, Pi disconnected: the buck gives 5.1 V.
4. **Power up.** Pi first, then the motors. Check the link:

   ```bash
   ip -details link show can0        # "can state ERROR-ACTIVE", "bitrate 1000000"
   candump can0                      # 00002901 and 00002902 at 100-200 Hz each
   pixi run -e robot can-check       # "OK: can0 is up and motors 1, 2 respond"
   ```

5. **Jog each motor alone** and watch which way the wheel turns:

   ```bash
   pixi run -e robot ros2 run iot_robot_drivers cubemars_tool jog --id 1 --velocity 1.0
   pixi run -e robot ros2 run iot_robot_drivers cubemars_tool jog --id 2 --velocity 1.0
   ```

   For each wheel: if it turns the way that would drive the robot forward, its direction
   is `1`, otherwise `-1`. The defaults (`left_direction:=1 right_direction:=-1`) assume
   mirror-mounted motors. Put any change in the service settings (`IOT_ROBOT_ARGS` in
   `/etc/default/iot-robot`) or pass it on the command line.
6. **Speed scale.** `jog --id 1 --velocity 1.0 --duration 6.283` should turn the wheel
   about one revolution (a little less, because of the acceleration at the start). Two
   revolutions, or half of one, means `pole_pairs` is wrong.
7. **Loss of commands (pass/fail).** Start a long jog (`--duration 20`) and unplug the
   CAN adapter while the wheel turns. The wheel must stop within about 0.5 s. Do the
   same for the other motor. If a wheel keeps turning, set `timeout_msec` (see
   [Motor configuration](#motor-configuration)) and repeat. **Do not continue until
   both wheels pass:** without the timeout, a crash of the Pi leaves the robot driving
   until the battery switch is turned off. Plug the adapter back in afterwards and run
   `pixi run -e robot can-check`.
8. **Full launch, still on the stand.** `pixi run -e robot robot`, then from the laptop
   on the same network and `ROS_DOMAIN_ID`, `pixi run teleop`, or use the web page. The
   robot starts with the web E-stop engaged: hold the release button on the page first
   (or add `start_estopped:=false`).
   Forward turns both wheels forward; a left turn drives the right wheel forward and the
   left wheel backward. Check `ros2 control list_controllers` shows both controllers
   `active`.
9. **Stops.** Press the E-stop in the web page, or
   `pixi run -e robot ros2 topic pub -t 3 /e_stop std_msgs/msg/Bool "{data: true}"`: the
   wheels stop and ignore teleop until it is released (on the web page, or the same
   command with `data: false`). `-t 3` sends the message three times, once a second:
   `--once` goes out as soon as one subscriber is found, so it can reach twist_mux but
   not the web node (or the reverse), and the page then shows the wrong E-stop state.
   Stop teleop: the wheels stop about
   0.5 s after the last command (`cmd_vel_timeout` of the diff drive controller).
10. **Software stop.** Drive with teleop held down, then press Ctrl-C in the terminal
    running the robot. The wheels must stop at once, and the log shows
    `Stopping the wheel motors (deactivate)` followed by
    `Sent zero speed and release to motors [1, 2] on can0`.
11. **Motor watchdog.** Start the robot again and drive with teleop held down. Unplug
    the CAN cable of one motor (not the adapter). Both wheels must stop within about
    0.5 s, with a `FATAL` message `No status frames from the motor on ...`. Plug the
    cable back in and restart the robot.
12. **On the ground, slowly.** Drive 1 m straight, measured with a tape, and compare with
    `ros2 topic echo /diff_drive_controller/odom --field pose.pose.position`. A
    consistent error means `wheel_radius` in `config/controllers.yaml` needs adjusting;
    a curve when driving straight means `wheel_separation` or one tyre differs.
13. **Install the service** (next section) once all of this passes, then repeat step 10
    with `sudo systemctl stop iot-robot` instead of Ctrl-C: `journalctl -u iot-robot`
    shows the same two lines and `Motors 1, 2 on can0: zero speed for 0.3 s, then
    released`.

## Running at boot

```bash
sudo deploy/install.sh
sudo systemctl start iot-robot
journalctl -u iot-robot -f
```

This installs the `/dev/stm32` udev rule, brings `can0` up at 1 Mbit/s with
systemd-networkd whenever the adapter appears, and runs `pixi run -e robot robot` as
your user at boot, restarting it if it exits. If the motors are not powered yet, the CAN
pre-flight check fails and the service retries every 5 s. After every stop, crash or
kill of the launch, the service sends zero speed and release to both motors
(`cubemars_tool stop`). Settings are in `/etc/default/iot-robot`; if `IOT_ROBOT_ARGS`
changes `can_interface` or the motor IDs, set `IOT_CAN_INTERFACE` and `IOT_MOTOR_IDS`
there to match. Details, updating and uninstalling: [deploy/README.md](../deploy/README.md).

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

# Unit tests for the drivers (protocol, bridge against the fake board, CAN codec, stop)
pixi run colcon test --packages-select iot_robot_drivers && pixi run colcon test-result --verbose
pixi run bash -c 'cd src/iot_robot_drivers && python -m pytest test'     # same, directly

# The CAN tests against a virtual CAN bus, in a throwaway network namespace (no root)
pixi run bash -c 'cd src/iot_robot_drivers && unshare -rn sh -c "ip link set lo up && \
  ip link add dev vcan0 type vcan && ip link set vcan0 up && \
  IOT_TEST_CAN_INTERFACE=vcan0 python -m pytest test/test_cubemars.py"'
```

`setup.cfg` in `iot_robot_drivers` passes `-p no:launch_testing -p no:launch_ros` to
pytest. Those ROS launch plugins use a hook signature that the installed pytest no longer
accepts (`PluginValidationError`), and these tests do not need them. The vcan
tests are skipped unless `IOT_TEST_CAN_INTERFACE` is set; they need the `vcan` kernel
module, which most desktop kernels load on demand.

## Troubleshooting

| Symptom | Likely cause | Fix |
| --- | --- | --- |
| Launch exits: `CAN interface 'can0' does not exist` | Adapter unplugged, or not a SocketCAN adapter | [CAN adapter](#can-adapter) |
| Launch exits: `CAN interface 'can0' is down` | Bitrate never configured | `pixi run -e robot can-up`, or `sudo deploy/install.sh` |
| Launch exits: `No status frames from motor CAN ID(s) ...` | Motors unpowered, IDs differ, status feedback disabled, wiring | `candump can0`; [Motor configuration](#motor-configuration) |
| `can0` is `ERROR-PASSIVE` or `BUS-OFF` | Nothing acknowledges frames: motors off, CANH/CANL swapped, wrong bitrate, termination | Check wiring and the 60 ohm reading, then `pixi run -e robot can-up` |
| `No CAN message received from CAN ID` warnings every cycle | Status upload slower than 50 Hz | Raise it to 100 to 200 Hz in the Upper Computer |
| Launch exits: `Motor fault: ...` | The motor reports a fault (voltage, temperature, stall) | Fix the cause, power-cycle the motor |
| Launch exits: `The motor plugin package cubemars_hardware_safe is not installed` | Started from the laptop (default) environment, which does not build the motor plugin | `mock:=true` on the laptop; on the robot `pixi run -e robot build` and `pixi run -e robot robot` |
| Launch exits: `ros2_control_node exited with code ...` or `was killed by signal ...` | The controller manager failed to start or crashed. With `check_can:=false` this is usually a missing or down CAN interface | The `[FATAL]` lines just above it name the cause |
| `Motor on wheel_joint_... (CAN ID n) reports fault ...: ... Stopping both wheels.` | A motor reported a fault while driving (voltage sag, over-temperature, stall) | Fix the cause, then restart the robot ([Stopping the wheels](#stopping-the-wheels)) |
| `No status frames from the motor on ... for ... ms` | A motor lost power or its CAN cable, or its status rate is below 20 Hz | Check the cable and power, then restart the robot |
| `Wheel motors stopped after an error` and the robot no longer drives | One of the two messages above; the stop is latched on purpose | Restart: Ctrl-C and start again, or `sudo systemctl restart iot-robot` |
| `Could not send stop frames` or `Could not send every stop frame` | The CAN link went away while stopping | The motors stop on their own `timeout_msec`; check the adapter |
| A wheel turns the wrong way | Direction for that motor | Flip `left_direction` or `right_direction` |
| Odometry off by a constant factor | `wheel_radius`, or `pole_pairs` if it is 2× | Checklist steps 6 and 12 |
| `Cannot open /dev/stm32` | Board unplugged, udev rule not installed, user not in `dialout` | `ls -l /dev/stm32 /dev/ttyACM*`; `sudo deploy/install.sh`; log in again |
| `No data from STM32 ... for 1 s` | Firmware not running or wrong baud rate | Reset the Nucleo; check the firmware uses 115200 8N1 |
| `Ignoring malformed line from STM32` | Firmware prints another format | Compare with the protocol in [STM32 sensor board](#stm32-sensor-board) |
| `STM32: warning: <side> sensor not responding` (or `echo stuck high`, `echo pulses too short`) | The firmware sees no valid echo from that sensor: 5 V, GND, TRIG or ECHO wiring, or a locked-up sensor | Check the sensor's wiring ([wiring.md](wiring.md#3-nucleo-f401re-sensor-board)); power-cycle a sensor stuck high. `/ultrasonic/<side>` is `NaN` until `STM32: info: <side> sensor recovered` |
| `camera_ros` finds no camera | Ribbon reversed or loose, wrong cable on a Pi 5, another program has the camera | `rpicam-hello --list-cameras` with the robot stopped |
| The laptop sees no topics from the Pi | Different `ROS_DOMAIN_ID`, or the Wi-Fi blocks multicast between clients | Match the domain; try a phone hotspot or wired link to rule out the network |
| `pixi` says the environment does not support `linux-aarch64` | Missing `-e robot` | Add `-e robot` to every command on the Pi |
| `can_frame has no member named len` when building | `cubemars_hardware` built in the laptop environment, whose kernel headers are too old | Build with `-e robot`; the laptop's `build` task skips it and `cubemars_hardware_safe` |
| `Could not enable FIFO RT scheduling policy` | No real-time permission when run by hand | Harmless; the service grants it (`LimitRTPRIO=99`) |
| `Velocity command timed out. Braking.` every second | No one is sending velocity commands | Normal while idle |
| The service restarts every few seconds | Launch fails, usually the CAN pre-flight | `journalctl -u iot-robot -e` shows the reason |
