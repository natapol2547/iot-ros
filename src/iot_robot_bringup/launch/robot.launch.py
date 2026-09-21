"""Bring up the real iot_robot: wheel motors over CAN, camera, STM32 sensor board, web UI.

    ros2 launch iot_robot_bringup robot.launch.py                    # real hardware
    ros2 launch iot_robot_bringup robot.launch.py mock:=true camera:=false

Unless mock:=true, the CAN link and both motors are checked before anything starts. If
either is missing the launch exits with instructions, instead of starting a controller
that never moves the wheels.

Stopping the wheels does not depend on a clean shutdown alone: the motor plugin
(cubemars_hardware_safe) stops them when it is deactivated or sees a motor fault, this
file sends the same stop frames whenever ros2_control_node exits for any reason
(including a crash or SIGKILL), and the motors' own CAN timeout covers the rest.
"""

import os
import shutil

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
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


def check_can(interface, can_ids):
    # Imported here so mock:=true works without the drivers package on the path
    from iot_robot_drivers.cubemars import CanCheckError, preflight
    try:
        heard = preflight(interface, can_ids, timeout=2.0)
    except CanCheckError as err:
        raise RuntimeError(
            f"CAN pre-flight check failed, not starting the robot.\n{err}\n"
            "To run without motors use mock:=true; to skip this check, check_can:=false."
        ) from None
    except OSError as err:
        raise RuntimeError(f"Cannot open CAN interface '{interface}': {err}") from None
    return LogInfo(msg="CAN pre-flight OK: " + ", ".join(
        f"motor {can_id} {status.temperature_c} C" for can_id, status in sorted(heard.items())))


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
    from iot_robot_drivers.cubemars import CanBus, stop_motors
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
        "mock", "camera", "web", "check_can", "can_interface", "left_can_id",
        "right_can_id", "left_direction", "right_direction", "stm32_port", "gizmo_mode",
        "start_estopped")}
    mock = is_true(arg["mock"])
    can_ids = [int(arg["left_can_id"]), int(arg["right_can_id"])]

    actions = []
    if not mock:
        check_motor_plugin()
        if is_true(arg["check_can"]):
            actions.append(check_can(arg["can_interface"], can_ids))

    pkg = FindPackageShare("iot_robot_bringup")
    urdf = PathJoinSubstitution([pkg, "urdf", "iot_robot.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    twist_mux = PathJoinSubstitution(
        [FindPackageShare("iot_robot_behavior"), "config", "twist_mux.yaml"])
    robot_description = ParameterValue(Command([
        "xacro ", urdf,
        " mock:=", arg["mock"],
        " can_interface:=", arg["can_interface"],
        " left_can_id:=", arg["left_can_id"],
        " right_can_id:=", arg["right_can_id"],
        " left_direction:=", arg["left_direction"],
        " right_direction:=", arg["right_direction"],
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
        Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux, {"use_sim_time": False}],
            remappings=[("cmd_vel_out", "/diff_drive_controller/cmd_vel")],
        ),
        # Ultrasonics, battery voltage and the gizmo joint states. The bridge reconnects
        # by itself, so it runs even when the board is unplugged
        Node(
            package="iot_robot_drivers",
            executable="stm32_bridge",
            parameters=[{"port": arg["stm32_port"], "gizmo_mode": arg["gizmo_mode"]}],
            respawn=True,
            respawn_delay=2.0,
        ),
    ]

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
            "start_estopped", default_value="true",
            description="Start with the web E-stop engaged, so a restart (the recovery "
                        "after a motor fault or the hardware E-stop) never drives by itself"),
        DeclareLaunchArgument(
            "check_can", default_value="true",
            description="Check the CAN link and motor status frames before starting"),
        DeclareLaunchArgument(
            "can_interface", default_value="can0", description="SocketCAN interface"),
        DeclareLaunchArgument(
            "left_can_id", default_value="1", description="CAN ID of the left wheel motor"),
        DeclareLaunchArgument(
            "right_can_id", default_value="2", description="CAN ID of the right wheel motor"),
        DeclareLaunchArgument(
            "left_direction", default_value="1",
            description="1 if a positive left motor speed drives the robot forward, else -1"),
        DeclareLaunchArgument(
            "right_direction", default_value="-1",
            description="1 if a positive right motor speed drives the robot forward, else -1"),
        DeclareLaunchArgument(
            "stm32_port", default_value="/dev/stm32",
            description="Serial port of the Nucleo sensor board (udev symlink)"),
        DeclareLaunchArgument(
            "gizmo_mode", default_value="fixed", choices=["fixed", "servo"],
            description="fixed: publish constant gizmo joint states; "
                        "servo: drive hobby servos on the Nucleo"),
        OpaqueFunction(function=launch_setup),
    ])
