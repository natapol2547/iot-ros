"""Bring up the real iot_robot: CubeMars motors over CAN, camera, STM32 sensor board, IMU, web UI.

    ros2 launch iot_robot_bringup robot.launch.py                     # real hardware
    ros2 launch iot_robot_bringup robot.launch.py gizmo_mode:=fixed   # wheels only
    ros2 launch iot_robot_bringup robot.launch.py mock:=true camera:=false

The motor CAN IDs and directions come from motors.yaml (config/motors.yaml, or
motors:=PATH), which is checked before anything starts. Unless mock:=true, the CAN link
and every motor in use (the wheels, plus the gizmo motors in gizmo_mode can) are checked
too. If one is missing the launch exits with instructions, instead of starting a
controller that never moves it.

The motors cannot remember their position over a power cycle. In gizmo_mode can, the
pose the gizmo is in when the robot starts therefore becomes its zero (zero_on_start in
motors.yaml): put the gizmo at its zero pose before starting (docs/checklist.md).

The LSM9DS1 IMU (imu.launch.py, imu:=true) never holds up or stops the robot: its node
retries by itself when the sensor is missing, and environments without the IMU packages
(the laptop) skip it with a message.

Stopping the motors does not depend on a clean shutdown alone: the motor plugin
(cubemars_hardware_safe) stops them when it is deactivated or sees a motor fault, this
file sends the same stop frames to every motor in use whenever ros2_control_node exits
for any reason (including a crash or SIGKILL), and the motors' own CAN timeout covers
the rest.
"""

import importlib.util
import os
import shlex
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from iot_robot_drivers import motor_config
from iot_robot_drivers.cubemars import CanBus, CanCheckError, describe, preflight, stop_motors
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
    RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# camera_calibration's "commit" writes here, outside the install space
CAMERA_INFO = os.path.expanduser("~/.ros/camera_info/iot_robot_camera.yaml")
MOTOR_PLUGIN_PACKAGE = "cubemars_hardware_safe"
# What imu.launch.py needs besides iot_robot_drivers; only the robot environment has them
IMU_FILTER_PACKAGE = "imu_filter_madgwick"
IMU_I2C_MODULE = "smbus2"
# Seconds a spawner waits for the controller manager before giving up, so a controller
# manager that never comes up ends the launch instead of hanging it
CONTROLLER_MANAGER_TIMEOUT = 30


def is_true(value):
    return value.lower() in ("true", "1", "yes")


def check_motor_plugin():
    try:
        get_package_share_directory(MOTOR_PLUGIN_PACKAGE)
    except PackageNotFoundError:
        raise RuntimeError(
            f"The motor plugin package {MOTOR_PLUGIN_PACKAGE} is not installed in this "
            "environment. The laptop (default) environment does not build it; there, use "
            "mock:=true. On the robot, build and launch with the robot environment: "
            "`pixi run -e robot build`, `pixi run -e robot robot`."
        ) from None


def load_motors(path, gizmo_mode):
    """Validate motors.yaml before xacro reads it, for a clear message instead of a trace."""
    try:
        return motor_config.load(path, require_gizmo=gizmo_mode == "can")
    except motor_config.MotorConfigError as err:
        raise RuntimeError(
            f"{err}\nFix the file (docs/todo.md), or pass another one with motors:=PATH."
        ) from None


def check_can(interface, can_ids, names):
    try:
        heard = preflight(interface, can_ids, timeout=2.0, names=names)
    except CanCheckError as err:
        raise RuntimeError(
            f"CAN pre-flight check failed, not starting the robot.\n{err}\n"
            "To run without motors use mock:=true; to skip this check, check_can:=false."
        ) from None
    except OSError as err:
        raise RuntimeError(f"Cannot open CAN interface '{interface}': {err}") from None
    return LogInfo(msg="CAN pre-flight OK: " + ", ".join(
        f"{describe(can_id, names)} {status.temperature_c} C"
        for can_id, status in heard.items()))


def missing_imu_packages():
    missing = []
    try:
        get_package_share_directory(IMU_FILTER_PACKAGE)
    except PackageNotFoundError:
        missing.append(IMU_FILTER_PACKAGE)
    if importlib.util.find_spec(IMU_I2C_MODULE) is None:
        missing.append(IMU_I2C_MODULE)
    return missing


def imu_actions():
    """Include imu.launch.py, or explain why the IMU is left out.

    The IMU is optional: a missing sensor only makes lsm9ds1_node log and retry, and an
    environment without its packages (the laptop's default environment) skips it,
    rather than failing the whole launch.
    """
    missing = missing_imu_packages()
    if missing:
        return [LogInfo(msg=(
            f"Not starting the IMU: this environment lacks {' and '.join(missing)} (only "
            "the robot environment has them). Pass imu:=false to hide this message."))]
    share = get_package_share_directory("iot_robot_bringup")
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", "imu.launch.py")),
        # Passed explicitly: an include sees the parent's launch configurations, so an
        # unrelated params_file given to this launch would otherwise reach the IMU
        launch_arguments={"params_file": os.path.join(share, "config", "imu.yaml")}.items(),
    )]


def camera_info_url():
    if not os.path.exists(CAMERA_INFO):
        os.makedirs(os.path.dirname(CAMERA_INFO), exist_ok=True)
        shutil.copy(os.path.join(get_package_share_directory("iot_robot_bringup"),
                                 "config", "camera_info", "imx219_640x480.yaml"), CAMERA_INFO)
    return "file://" + CAMERA_INFO


def shutdown_if_failed(action, what):
    """Stop the whole launch when a one-shot process such as a spawner fails."""
    def on_exit(event, context):
        # Processes killed while the launch shuts down exit non-zero; that is expected
        if event.returncode != 0 and not context.is_shutdown:
            # Raising makes ros2 launch log the reason, shut down and exit with code 1
            raise RuntimeError(
                f"{what} failed with exit code {event.returncode}; the messages from "
                "controller_manager above say why.")
        return []
    return RegisterEventHandler(OnProcessExit(target_action=action, on_exit=on_exit))


def send_motor_stop(interface, can_ids):
    """Zero speed, then release, from the launch process itself.

    Runs after ros2_control_node has exited, so no controller can override it, and it
    also covers a crash or SIGKILL of ros2_control_node, where the plugin gets no chance
    to stop the motors.
    """
    logger = get_logger("robot.launch.py")
    try:
        with CanBus(interface, status_only=True) as bus:
            stop_motors(bus, can_ids)
        logger.info(f"Sent zero speed and release to motors {can_ids} on {interface}")
    except OSError as err:
        logger.error(
            f"Could not send stop frames to motors {can_ids} on {interface}: "
            f"{err.strerror}. If the motors were turning, only their own CAN timeout "
            "stops them (timeout_msec, docs/hardware.md).")


def on_control_node_exit(mock, interface, can_ids):
    def on_exit(event, context):
        if not mock:
            send_motor_stop(interface, can_ids)
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            how = (f"was killed by signal {-event.returncode}" if event.returncode < 0
                   else f"exited with code {event.returncode}")
            raise RuntimeError(
                f"ros2_control_node {how}, stopping the robot. Its last messages above "
                "say why (CAN interface, motor plugin, controller configuration).")
        return [Shutdown(reason="ros2_control_node exited")]
    return on_exit


def launch_setup(context):
    arg = {name: context.launch_configurations[name] for name in (
        "mock", "camera", "web", "imu", "check_can", "can_interface", "motors",
        "stm32_port", "gizmo_mode", "start_estopped")}
    mock = is_true(arg["mock"])
    gizmo_mode = arg["gizmo_mode"]
    gizmo_can = gizmo_mode == "can"
    # xacro needs an absolute path; a relative one is taken from the launch's directory
    motors_path = os.path.abspath(os.path.expanduser(arg["motors"]))
    motors = load_motors(motors_path, gizmo_mode)
    # The motors robot.launch.py commands, checks and stops. In gizmo_mode fixed the
    # gizmo motors may still be powered on the bus, but nothing drives them
    in_use = motors.in_use(gizmo_mode)
    can_ids = [motor.can_id for motor in in_use]

    actions = [LogInfo(msg=f"Motors from {motors_path}: " + ", ".join(
        describe(motor.can_id, motors.names()) for motor in in_use))]
    if not mock:
        check_motor_plugin()
        if is_true(arg["check_can"]):
            actions.append(check_can(arg["can_interface"], can_ids, motors.names()))
        zeroed = [motor.joint for motor in in_use if motor.zero_on_start]
        if zeroed:
            actions.append(LogInfo(msg=(
                f"{', '.join(zeroed)}: the pose at start-up becomes the zero "
                "(zero_on_start in motors.yaml). If the gizmo was not at its zero pose "
                "when the robot started, put it there and restart (docs/checklist.md).")))

    pkg = FindPackageShare("iot_robot_bringup")
    urdf = PathJoinSubstitution([pkg, "urdf", "iot_robot.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    twist_mux = PathJoinSubstitution(
        [FindPackageShare("iot_robot_behavior"), "config", "twist_mux.yaml"])
    # The URDF reads the CAN IDs and directions from motors.yaml itself
    robot_description = ParameterValue(Command([
        "xacro ", urdf,
        " mock:=", arg["mock"],
        " can_interface:=", arg["can_interface"],
        " motors:=", shlex.quote(motors_path),
        " gizmo_mode:=", gizmo_mode,
    ]), value_type=str)

    def spawner(name):
        node = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", controllers,
                       "--controller-manager-timeout", str(CONTROLLER_MANAGER_TIMEOUT)],
        )
        return [node, shutdown_if_failed(node, f"Starting {name}")]

    # Reads the URDF from /robot_description, published by robot_state_publisher
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controllers, {"use_sim_time": False}],
    )

    actions += [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description, "use_sim_time": False}],
        ),
        control_node,
        RegisterEventHandler(OnProcessExit(
            target_action=control_node,
            on_exit=on_control_node_exit(mock, arg["can_interface"], can_ids))),
        *spawner("joint_state_broadcaster"),
        *spawner("diff_drive_controller"),
        # Pan/tilt position commands, e.g. from target_follower
        *(spawner("gizmo_controller") if gizmo_can else []),
        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux, {"use_sim_time": False}],
            remappings=[("cmd_vel_out", "/diff_drive_controller/cmd_vel")],
        ),
        # Ultrasonics and battery voltage, plus the gizmo joint states in gizmo_mode
        # fixed. The Nucleo only reads sensors; every motor is driven from here over
        # CAN. The bridge reconnects by itself, so it runs even when the board is
        # unplugged
        Node(
            package="iot_robot_drivers",
            executable="stm32_bridge",
            parameters=[{"port": arg["stm32_port"], "gizmo_mode": gizmo_mode}],
            respawn=True,
            respawn_delay=2.0,
        ),
    ]

    if is_true(arg["imu"]):
        actions += imu_actions()

    if is_true(arg["camera"]):
        actions.append(Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            parameters=[PathJoinSubstitution([pkg, "config", "camera.yaml"]),
                        {"camera_info_url": camera_info_url()}],
            respawn=True,
            respawn_delay=2.0,
        ))

    if is_true(arg["web"]):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("iot_robot_web"), "launch", "web.launch.py"])),
            launch_arguments={"use_sim_time": "false",
                              "start_estopped": arg["start_estopped"]}.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mock", default_value="false",
            description="Replace the CAN motors with mock_components (no hardware needed)"),
        DeclareLaunchArgument(
            "camera", default_value="true", description="Start the Pi camera (camera_ros)"),
        DeclareLaunchArgument(
            "web", default_value="true", description="Start the browser controller"),
        DeclareLaunchArgument(
            "imu", default_value="true",
            description="Start the LSM9DS1 IMU and its orientation filter (imu.launch.py, "
                        "config/imu.yaml). A missing sensor does not stop the robot"),
        DeclareLaunchArgument(
            "start_estopped", default_value="true",
            description="Start with the web E-stop engaged, so a restart (the recovery "
                        "after a motor fault or the hardware E-stop) never drives by itself"),
        DeclareLaunchArgument(
            "check_can", default_value="true",
            description="Check the CAN link and the status frames of every motor in use "
                        "before starting"),
        DeclareLaunchArgument(
            "can_interface", default_value="can0", description="SocketCAN interface"),
        DeclareLaunchArgument(
            "motors",
            default_value=os.path.join(
                get_package_share_directory("iot_robot_bringup"), "config", "motors.yaml"),
            description="Motor configuration: the CAN ID and direction of each joint's "
                        "motor (docs/todo.md)"),
        DeclareLaunchArgument(
            "stm32_port", default_value="/dev/stm32",
            description="Serial port of the Nucleo sensor board (udev symlink)"),
        DeclareLaunchArgument(
            "gizmo_mode", default_value="can", choices=["can", "fixed"],
            description="can: the gizmo motors are driven over CAN (gizmo_controller); "
                        "fixed: gizmo not driven, constant joint states from stm32_bridge"),
        OpaqueFunction(function=launch_setup),
    ])
