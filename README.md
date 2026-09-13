# iot_robot

A differential-drive robot with a pan/tilt camera, built on ROS 2 Jazzy and simulated in MuJoCo.
The goal is vision-based following: first a yellow ball, then a specific person recognised by
appearance, with distance estimated from a single camera.

The whole ROS environment is managed by [pixi](https://pixi.sh) using
[RoboStack](https://robostack.github.io) packages, so no system ROS install is needed.

## Hardware

| Part | Details |
| --- | --- |
| Computer | Raspberry Pi with a CAN interface |
| Drive | 2 × CubeMars AK45-10 (10:1, 7 N·m peak, 180 rpm output) over CAN |
| Camera | Raspberry Pi Camera v2.1 (IMX219, 62.2° × 48.8° FOV) on a pan/tilt "gizmo" |
| Range | 2 × ultrasonic sensors angled ±45° forward, read by an STM32 that talks to the Pi over I2C |
| IMU | LSM9DS1 on the Pi's I2C bus |

There is no lidar, so Nav2 is not used. Following is done with visual servoing.

## Quick start

```bash
pixi install          # first time only
pixi run sim          # regenerates the MJCF, builds, launches MuJoCo + controllers
pixi run teleop       # (second terminal) drive with the keyboard
pixi run follow       # (second terminal) detect and follow the yellow ball
```

In the MuJoCo viewer, double-click a body to select it and Ctrl + right-drag to push it around.
That is the easiest way to move the ball.

Image topics can be viewed with:

```bash
pixi run ros2 run rqt_image_view rqt_image_view /ball/debug_image
```

(`pixi run rqt_image_view` alone does not work.)

## Pixi tasks

| Task | What it does |
| --- | --- |
| `build` | `colcon build --symlink-install` in Release mode |
| `clean` | Deletes `build/`, `install/` and `log/` |
| `display` | Shows the URDF in RViz with joint sliders |
| `mjcf` | Converts the URDF into the MuJoCo model in `src/iot_robot_mujoco/mjcf/` |
| `sim` | Runs `mjcf` and `build`, then launches the simulation |
| `teleop` | Keyboard teleop, publishing to `/cmd_vel/teleop` |
| `ball` | Runs the ball detector on its own |
| `follow` | Ball detector + target follower |
| `person` | Laptop webcam + person detector (Step 6) |

`scripts/activate.sh` sources `install/setup.bash` on every `pixi run`, so built packages are
always on the path.

## Packages

```
src/
├── iot_robot_description   URDF/xacro, meshes, RViz display (ament_cmake)
├── iot_robot_mujoco        MuJoCo model inputs, ros2_control config, sim launch, twist_mux (ament_cmake)
├── iot_robot_perception    ball_detector, person_detector (ament_python)
└── iot_robot_behavior      target_follower, ball_follow launch (ament_python)
```

`iot_robot_mujoco/urdf/iot_robot_sim.urdf.xacro` includes the description and adds the MuJoCo
`ros2_control` hardware block. The real robot will get its own wrapper with the CubeMars hardware
plugin, so the description package stays shared between sim and hardware.

## Topics and data flow

```
MuJoCo camera ──► /camera/image_raw, /camera/camera_info, /camera/depth
                        │
                ball_detector ──► /ball/position (PointStamped, camera_optical_frame)
                        │         /ball/debug_image
                        ▼
                target_follower ──► /gizmo_controller/commands (Float64MultiArray [yaw, pitch])
                        │
                        └──► /cmd_vel/follower ──┐
teleop_twist_keyboard ──► /cmd_vel/teleop ───────┤
                                                 ▼
                                   twist_mux ──► /diff_drive_controller/cmd_vel
```

twist_mux gives teleop priority 100 (timeout 1.0 s) over the follower's priority 10
(timeout 0.5 s). Pressing a teleop key overrides the follower for the wheels, and control
returns to the follower about a second after the last key press. The teleop timeout is long
because `teleop_twist_keyboard` only publishes while a key is pressed.

Other useful topics:

- `/joint_states`: wheel and gizmo joint positions
- `/diff_drive_controller/odom` and the `odom → base_footprint` TF
- `/simulator/floating_base_state`: ground-truth robot pose from MuJoCo (sim only)

## Key numbers

| Value | Setting | Where |
| --- | --- | --- |
| Wheel separation | 0.349 m (measured at the contact centres) | `iot_robot_mujoco/config/controllers.yaml` |
| Wheel radius | 0.0754 m | same |
| Gizmo yaw limits | −0.785 … 0.785 rad | URDF + `mujoco_inputs.xml` |
| Gizmo pitch limits | −0.785 … 0 rad (negative tilts **up**) | URDF + `mujoco_inputs.xml` |
| Camera | 640 × 480, vertical FOV 48.8°, fy ≈ 529 px, 15 Hz in sim | `mujoco_inputs.xml`, sim xacro |
| Ball | 10 cm diameter, yellow | `mjcf/scene.xml` |

## Progress

- [x] Robot description (URDF from Blender meshes) and RViz display
- [x] MuJoCo simulation with ros2_control, diff drive and keyboard teleop
- [x] Step 4: ball detection with monocular distance
- [x] Step 5: ball following with gizmo aiming, twist_mux teleop override, gizmo speed limit
- [ ] Step 6: person detection with YOLO26n-pose on a laptop webcam (in progress)
- [ ] Step 6b: follow a person; follower keeps the target in the `odom` frame between detections
- [ ] Step 7: re-identification, so the robot remembers one specific person
- [ ] Step 8: real hardware (CubeMars motors, Pi camera, IMU, STM32 ultrasonics)

## Perception

### Ball detector (Step 4)

HSV threshold → morphological open → largest contour → `minEnclosingCircle`.

Distance from the known diameter D and pixel radius r:

```
Z = fx · D / (2 · r · cosθ)
cosθ = 1 / sqrt(1 + ((u − cx)/fx)² + ((v − cy)/fy)²)
```

The `cosθ` term corrects for a sphere's outline stretching when it is away from the image
centre. Without it the error reaches about 6 % at 20° off-axis. In sim, the node checks its
estimate against the depth camera. The error is below 1.5 % from 0.95 m to 3.7 m.

### Person detector (Step 6)

- **Model:** YOLO26n-pose (Ultralytics), exported to ONNX at 320 × 320 and run with
  ONNX Runtime. The node does its own letterboxing (pad value 114, RGB, /255, CHW) and decodes
  the NMS-free output `[1, 300, 57]`: `x1, y1, x2, y2, score, class`, then 17 × `(x, y, visibility)`
  COCO keypoints, all in letterboxed input pixels.
- **Why not the `ultralytics` Python package:** it pulls in PyTorch (about 4 GB with CUDA) and a
  pip OpenCV that conflicts with `cv_bridge`. `onnxruntime` from conda-forge is 16 MB and works
  the same way on the Pi.
- **Distance:** shoulder-midpoint to hip-midpoint length, `L = person_height · torso_ratio`
  (1.70 m · 0.29). With both points in normalised image coordinates, `Z = L / length`.
  A torso stays in view up close, when the legs are cut off, and its length barely changes
  when the person turns sideways.
- **Target choice:** the largest person whose shoulders and hips are both visible. This is
  replaced by re-identification in Step 7.
- **Licence:** YOLO26 weights are AGPL-3.0. That is fine for this project, but it matters if the
  robot or its software is ever distributed commercially.
- **Speed:** about 6–8 Hz on the development laptop while the simulation is running. The
  Ultralytics Pi 5 benchmark suggests roughly 10 Hz at 320 px. This still needs to be measured.

#### Exporting the model

Run this in a normal terminal, **not** through `pixi run`. Pixi's `PYTHONPATH` would leak ROS
Python packages into uv's interpreter. It needs about 1.5 GB of free disk space temporarily.

```bash
mkdir -p models && cd models
uvx --torch-backend cpu --from ultralytics --with onnx --with onnxslim --with onnxruntime \
  yolo export model=yolo26n-pose.pt format=onnx imgsz=320 nms=False
uv cache clean torch torchvision ultralytics   # reclaims ~2 GB
```

- `--torch-backend cpu` stops uv from downloading the CUDA build of torch. Passing
  `--index-url` does not work for this, because an extra index takes precedence.
- `nms=False` produces the end-to-end output described above. NCNN export does not support
  end-to-end output, so switching to NCNN on the Pi would need an NMS step.

`models/` is gitignored. The launch file finds the model through `$PIXI_PROJECT_ROOT/models/`.

#### Camera calibration

An uncalibrated `usb_cam` publishes a camera matrix of zeros. In that case the detector waits
instead of publishing wrong distances. To calibrate, run this with `pixi run person` active and
a checkerboard in view. `--size` counts inner corners; `--square` is in metres.

```bash
pixi run ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image_raw \
  -r camera/set_camera_info:=/camera/usb_cam/set_camera_info
```

`usb_cam` offers the service privately, as `/camera/usb_cam/set_camera_info`. The calibrator
looks for `camera/set_camera_info` and ignores `-p camera:=...`, so the service name has to be
remapped.

`--size` is **squares minus one** in each direction; a board with 9 × 7 squares is `8x6`. If the
size is wrong the board is never detected. In that case no corners are drawn and the X/Y/Size/Skew
bars never appear, because they only show up after the first detection.

1. Move the board around until all bars are green.
2. Press **CALIBRATE**, then **COMMIT**.

`usb_cam` saves the result to `~/.ros/camera_info/webcam.yaml` and loads it on every start.
A checkerboard shown on a phone screen works; measure one square with a ruler.

The development laptop's webcam calibrated to fx ≈ 511, fy ≈ 510, cx ≈ 317, cy ≈ 271, with
small distortion. That is about a 64° horizontal field of view.

To check the distance, stand at measured distances and watch
`ros2 topic echo /person/position --field point.z`.

- **Constant ratio error:** adjust `torso_ratio`.
- **Error that changes with distance:** redo the calibration.

## Behaviour: target_follower

- Transforms the target into `base_link`.
- Aims the gizmo:
  - yaw = bearing to the target (the yaw axis passes through the `base_link` origin);
  - pitch = −atan2(height, horizontal distance), measured from the `gizmo_pitch_link` origin.
- Drives at 20 Hz:
  - angular velocity is proportional to bearing;
  - linear velocity is proportional to `distance − follow_distance`, scaled by `cos(bearing)` so it
    only drives forward when roughly facing the target.
- If there has been no detection for `lost_timeout`, it recentres the gizmo and spins towards
  where the target was last seen.

Result in sim: the robot stops 0.60 m from the ball with the camera centred to within 0.3°.

## Lessons learned

These cost real debugging time. Check here first when something similar breaks.

### Environment and build

- **setuptools ≥ 80 breaks `colcon build --symlink-install` for ament_python packages**
  ("option --editable not recognized"). `setuptools = "<80"` is pinned in `pixi.toml`.
- **New files need a rebuild,** even with `--symlink-install`. Examples are a new node, launch
  file or config. Only edits to existing files are picked up automatically.
- ament_python packages install launch/config files through `data_files` in `setup.py`. That is
  their equivalent of CMake's `install(DIRECTORY ...)`.
- **RoboStack's `camera_calibration` is missing its `semver` dependency.** `cameracalibrator`
  fails with `ModuleNotFoundError: No module named 'semver'`; fix with `pixi add semver`.
- **Do not `pixi add --pypi ultralytics`.** It downloads CUDA PyTorch and can fill the disk.
- conda-forge `mujoco-python` does not solve with RoboStack Jazzy. The MJCF converter uses PyPI
  `mujoco==3.10.*`, matching the libmujoco used by `mujoco_ros2_control`.

### Simulation (mujoco_ros2_control 0.0.3)

- Use the `ros2_control_node` shipped by `mujoco_ros2_control`. The stock one does not work.
- The converter drops URDF `<collision>` tags and uses the visual meshes for collision.
  The `visual` and `collision` default classes must exist in `mujoco_inputs.xml`.
- The right wheel axis had to be flipped (`<axis xyz="0 0 -1"/>`). **The real right motor will
  need the same sign flip.**
- The casters originally carried the robot's weight and killed traction. They are lifted 2 mm
  (`--no-fuse` plus `modify_element`) so the wheels carry the load.
- The depth image has the same timestamp as the RGB image but arrives just after it. Match
  them by stamp in the depth callback.
- URDF joint velocity limits are only enforced when `enforce_command_limits: true` is set on
  `controller_manager`. To slow the gizmo down, lower `velocity` on the gizmo joints in the URDF.

### ROS

- Jazzy's diff_drive_controller expects `TwistStamped`. Teleop needs `-p stamped:=true`, and
  twist_mux needs `use_stamped: true`.
- **Looking up a TF at the image timestamp fails,** because the image is newer than the latest
  transform (robot_state_publisher publishes TF at 20 Hz). Use `rclpy.time.Time()` to get
  the latest transform instead.
- Catch `(KeyboardInterrupt, ExternalShutdownException)` in `main()`. Otherwise every node
  prints a traceback when a launch file is stopped.

### Performance

- **Call `cv2.setNumThreads(1)` in every OpenCV node.** The conda OpenCV build uses OpenMP,
  and its thread pool fought the other nodes for CPU. With it, the ball detector went from 82 ms
  to 12 ms per frame. Before the fix, the gizmo jumped around and the robot lost the ball.

### Testing

- `ros2 launch` started in the background from a non-interactive shell ignores Ctrl-C/SIGINT.
  Leftover simulations on the same `ROS_DOMAIN_ID` then fight over `/clock`. Start test runs
  with `setsid` and kill the whole process group, or use a separate `ROS_DOMAIN_ID`.

## Notes for the real hardware (Step 8)

- **Motors:** planned driver is
  [cubemars_hardware](https://github.com/OpenFieldAutomation-OFA/cubemars_hardware), a
  ros2_control SystemInterface.
  - `pole_pairs` and `gear_ratio` need calibrating.
  - `send_can_status` must be enabled through R-Link.
  - Servo-mode speed is in electrical RPM.
- **Camera:** `camera_ros` (libcamera). The conda-forge libcamera `rpi_fork` build supports the
  Pi pipelines inside pixi. Use sensor mode 1640:1232 (full field of view, binned) scaled to
  640 × 480. The sensor's native 640 × 480 mode is cropped. Publish on `/camera/image_raw` and
  `/camera/camera_info` so the perception nodes work unchanged.
- **Meshes:** the wheel STL meshes are 5.34 mm off-centre in local X. This is corrected with
  the URDF visual origin.
- **Model format on the Pi:** start with ONNX at 320. Try NCNN if it is too slow; Ultralytics
  benchmarks it about 2× faster on a Pi 5, but it needs a separate NMS step.
