# Every-session checklist

For every use of the robot, after the one-time setup in [todo.md](todo.md).

**Why the gizmo zero pose matters:** the gizmo motors cannot remember their position
when they lose power. Every time the robot software starts, it takes the gizmo's pose at
that moment as yaw 0 and pitch 0, so the gizmo has to be at its zero pose then: **camera
straight ahead and level**.

## Before power-on

- [ ] Battery charged (above 21 V, 3.5 V per cell) and undamaged: no swelling, dents or
      warm cells.
- [ ] SW1 off. E-stop S1 **pressed**.
- [ ] Cables tidy; the gizmo turns through its whole range without pulling a cable.
- [ ] Wheels on blocks, or the floor around the robot clear.

## Power-on

- [ ] Plug in the XT60, then SW1 on, with S1 still pressed. The Pi boots; the motors
      stay off. The robot software tries to start every 5 s and waits for the motors
      (the journal shows `CAN pre-flight check failed`, which is expected here).
- [ ] The Nucleo's green LED (LD2) blinks once a second.
- [ ] Put the gizmo at its **zero pose** by hand (camera straight ahead and level, on
      the marks) and **hold it there**.
- [ ] Release S1. The four drive LEDs light blue. Keep holding the gizmo until it
      stiffens, usually within 10 to 20 s.
- [ ] Keep the robot still for 1 s more, while the IMU measures its gyroscope bias.

What the software does as it starts: it checks `motors.yaml` and that all four motors
answer on the CAN bus, sets the gizmo's current pose as its zero, then holds the gizmo
there. The web page starts with its E-stop engaged, so nothing drives until someone
releases it.

## Starting to drive

- [ ] Open `http://iot-robot.local:8080` (your Pi's hostname).
- [ ] The video runs, "Base" in the top bar is on (odometry arrives), and both
      ultrasonic lights turn red when a hand is 30 cm in front of that sensor.
- [ ] Release the E-stop: press and hold STOP for 1 s.
- [ ] **Confirm the zero:** press **Centre** under Camera aim. The camera looks straight
      ahead and level; the wall mark at lens height sits at the image centre.
- [ ] If it does not: put the gizmo at its zero pose, run
      `sudo systemctl restart iot-robot`, hold the gizmo until it stiffens, and check
      again.
- [ ] Start slowly (speed slider low).
- [ ] Remember the two stops:
  - **STOP on the page** stops the wheels only. A running follower still aims the
    camera.
  - **S1** cuts all four motors. Afterwards the software must be restarted
    (next section).

From a terminal on the Pi, `journalctl -u iot-robot -b | grep "temporary origin"` shows
one line per gizmo joint for each start, and right after a start
`pixi run -e robot ros2 topic echo /joint_states --once` or `pixi run -e robot can-watch`
shows both gizmo joints near 0. These only prove that a zero was set; whether it was set
in the right pose shows in the camera image.

## During use

- [ ] **Never move the gizmo by hand while the robot software runs.** It holds its
      position; forcing it trips the stall guard, and the gizmo stays limp until a
      restart.
- [ ] **After S1, a motor fault, or motors that stopped responding** (the wheels no
      longer drive, the camera aim does nothing, or the gizmo went limp):
  1. Stop everything that commands motion: teleop, the follower, the joystick.
  2. Release S1 if it was pressed; wait until all four drive LEDs are blue, then 1-2 s.
  3. Put the gizmo at its zero pose and hold it.
  4. `sudo systemctl restart iot-robot`, holding the gizmo until it stiffens.
  5. Confirm the zero (Centre), then release the web E-stop.

  Details: [wiring.md, E-stop warning](wiring.md#1-power-distribution).
- [ ] **If the page shows "Connection lost" and comes back with the E-stop engaged**,
      the software restarted by itself and zeroed the gizmo wherever it was. Confirm
      the zero before driving on.
- [ ] Battery: stop at 21 V (3.5 V per cell).

## Shutting down

- [ ] Press STOP on the page, then press S1. The gizmo goes limp: support the camera.
- [ ] Shut the Pi down: `sudo shutdown -h now` in an SSH session on the Pi. Wait until
      its green LED stops flashing. The robot software stops cleanly on the way down.
- [ ] SW1 off, then unplug the XT60.
- [ ] LiPo: charge with a balance charger, attended, in a LiPo bag. Not used for more
      than a few days: storage charge, about 3.8 V per cell (22.8 V).
