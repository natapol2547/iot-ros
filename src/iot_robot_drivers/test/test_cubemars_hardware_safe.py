"""cubemars_hardware_safe end to end, against fake_cubemars on a virtual CAN bus.

Runs ros2_control_node with the real motor plugin, the bringup URDF
(iot_robot_bringup/urdf/iot_robot.urdf.xacro, reading a motors.yaml written by the test)
and the robot's controllers.yaml, spawns the robot's controllers and commands them. The
emulated motors log every command frame, which is what the tests check.

Needs a vcan interface in IOT_TEST_CAN_INTERFACE (test_cubemars.py shows how to get one
without root) and the workspace built and sourced. The ROS domain is
IOT_TEST_ROS_DOMAIN_ID (default 91), with discovery limited to localhost.
"""

import math
import os
import signal
import subprocess
import time

import pytest

CAN_INTERFACE = os.environ.get("IOT_TEST_CAN_INTERFACE")
if not CAN_INTERFACE:
    pytest.skip("set IOT_TEST_CAN_INTERFACE to a vcan interface to run the ros2_control "
                "tests", allow_module_level=True)

rclpy = pytest.importorskip("rclpy")
xacro = pytest.importorskip("xacro")
from ament_index_python.packages import (  # noqa: E402
    PackageNotFoundError, get_package_prefix, get_package_share_directory)
from controller_manager_msgs.srv import (  # noqa: E402
    ListControllers, ListHardwareComponents, SetHardwareComponentState, SwitchController)
from geometry_msgs.msg import TwistStamped  # noqa: E402
from lifecycle_msgs.msg import State  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from std_msgs.msg import Float64MultiArray, String  # noqa: E402

from iot_robot_drivers.fake_cubemars import (  # noqa: E402
    MODE_CURRENT, MODE_POSITION, MODE_SET_ORIGIN, MODE_SPEED, FakeCubeMars)

try:
    for package in ("cubemars_hardware_safe", "diff_drive_controller",
                    "position_controllers", "joint_state_broadcaster"):
        get_package_prefix(package)
    BRINGUP = get_package_share_directory("iot_robot_bringup")
    CONTROLLER_MANAGER = os.path.join(get_package_prefix("controller_manager"), "lib",
                                      "controller_manager")
except PackageNotFoundError as err:
    pytest.skip(f"package {err} is not built or not sourced", allow_module_level=True)

URDF = os.path.join(BRINGUP, "urdf", "iot_robot.urdf.xacro")
CONTROLLERS = os.path.join(BRINGUP, "config", "controllers.yaml")
DOMAIN_ID = int(os.environ.get("IOT_TEST_ROS_DOMAIN_ID", "91"))

LEFT, RIGHT, YAW, PITCH = 10, 11, 12, 13
MOTOR_IDS = (LEFT, RIGHT, YAW, PITCH)
ERPM_PER_RAD_S = 14 * 10 * 60.0 / (2.0 * math.pi)
WHEEL_RADIUS = 0.0754
WHEEL_SEPARATION = 0.349
# URDF limits of the gizmo joints
YAW_LIMIT = 0.785398
GIZMO_VELOCITY = 1.0

MOTORS_YAML = """\
motors:
  wheel_joint_left: {{can_id: 10, direction: 1}}
  wheel_joint_right: {{can_id: 11, direction: -1}}
  gizmo_yaw_joint: {{can_id: 12, direction: {yaw_direction}, zero_on_start: true}}
  gizmo_pitch_joint: {{can_id: 13, direction: 1, zero_on_start: true}}
"""
ALL_CONTROLLERS = ("joint_state_broadcaster", "diff_drive_controller", "gizmo_controller")


class Robot:
    """Emulated motors, ros2_control_node and a test node that talks to it."""

    def __init__(self, tmp_path, yaw_direction=1, gizmo_mode="can", prepare=None):
        self.tmp_path = tmp_path
        self.fake = FakeCubeMars(CAN_INTERFACE, ids=MOTOR_IDS)
        if prepare:
            with self.fake.lock:
                prepare(self.fake)
        self.control = None
        self.context = rclpy.context.Context()
        rclpy.init(context=self.context, domain_id=DOMAIN_ID)
        self.node = rclpy.create_node("hardware_test", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.joints = {}
        self.node.create_subscription(JointState, "/joint_states", self.on_joints, 10)
        self.cmd_vel = self.node.create_publisher(
            TwistStamped, "/diff_drive_controller/cmd_vel", 10)
        self.gizmo_commands = self.node.create_publisher(
            Float64MultiArray, "/gizmo_controller/commands", 10)
        self.list_controllers = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers")
        self.list_hardware = self.node.create_client(
            ListHardwareComponents, "/controller_manager/list_hardware_components")
        self.set_hardware = self.node.create_client(
            SetHardwareComponentState, "/controller_manager/set_hardware_component_state")
        self.switch = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller")

        motors = tmp_path / "motors.yaml"
        motors.write_text(MOTORS_YAML.format(yaw_direction=yaw_direction))
        urdf = xacro.process_file(URDF, mappings={
            "can_interface": CAN_INTERFACE, "motors": str(motors),
            "gizmo_mode": gizmo_mode}).toxml()
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.description = self.node.create_publisher(String, "/robot_description", latched)
        self.description.publish(String(data=urdf))

        self.env = dict(os.environ, ROS_DOMAIN_ID=str(DOMAIN_ID),
                        ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
        arguments = ["--ros-args", "--params-file", CONTROLLERS]
        self.log_path = tmp_path / "ros2_control_node.log"
        self.log_file = open(self.log_path, "w")
        self.control = subprocess.Popen(
            [os.path.join(CONTROLLER_MANAGER, "ros2_control_node"), *arguments],
            stdout=self.log_file, stderr=subprocess.STDOUT, env=self.env,
            start_new_session=True)

    # ROS helpers -----------------------------------------------------------------

    def on_joints(self, msg):
        for index, name in enumerate(msg.name):
            self.joints[name] = (
                msg.position[index] if index < len(msg.position) else math.nan,
                msg.velocity[index] if index < len(msg.velocity) else math.nan)

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)

    def wait(self, condition, timeout, message, every=None):
        deadline = time.monotonic() + timeout
        while not condition():
            if time.monotonic() > deadline:
                pytest.fail(f"{message}\n--- ros2_control_node log ---\n{self.log()}")
            if every:
                every()
            self.spin(0.02)

    def call(self, client, request, timeout=10.0):
        self.wait(lambda: client.service_is_ready(), timeout,
                  f"{client.srv_name} is not available")
        future = client.call_async(request)
        self.wait(future.done, timeout, f"{client.srv_name} did not answer")
        return future.result()

    def log(self):
        self.log_file.flush()
        return self.log_path.read_text()

    def hardware_states(self):
        response = self.call(self.list_hardware, ListHardwareComponents.Request())
        return {component.name: component.state.label for component in response.component}

    def controller_states(self):
        response = self.call(self.list_controllers, ListControllers.Request())
        return {controller.name: controller.state for controller in response.controller}

    def set_hardware_state(self, name, state_id, label):
        request = SetHardwareComponentState.Request(
            name=name, target_state=State(id=state_id, label=label))
        return self.call(self.set_hardware, request).ok

    def switch_controllers(self, activate=(), deactivate=()):
        request = SwitchController.Request(
            activate_controllers=list(activate), deactivate_controllers=list(deactivate),
            strictness=SwitchController.Request.STRICT)
        request.timeout.sec = 5
        return self.call(self.switch, request).ok

    def wait_for_hardware(self, expected, timeout=10.0):
        states = {}

        def reached():
            states.update(self.hardware_states())
            return all(states.get(name) == label for name, label in expected.items())
        self.wait(reached, timeout, f"hardware did not reach {expected}: {states}")

    def spawn(self, *controllers, timeout=40.0):
        spawner = subprocess.run(
            [os.path.join(CONTROLLER_MANAGER, "spawner"), *controllers,
             "--param-file", CONTROLLERS, "--controller-manager-timeout", "20"],
            env=self.env, capture_output=True, text=True, timeout=timeout)
        assert spawner.returncode == 0, (
            f"spawner failed:\n{spawner.stdout}{spawner.stderr}\n--- log ---\n{self.log()}")

    def drive(self, linear=0.0, angular=0.0, seconds=0.0):
        """Publish cmd_vel at 20 Hz for seconds (once if 0) while spinning."""
        deadline = time.monotonic() + seconds
        while True:
            msg = TwistStamped()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.twist.linear.x = float(linear)
            msg.twist.angular.z = float(angular)
            self.cmd_vel.publish(msg)
            if time.monotonic() >= deadline:
                return
            self.spin(0.05)

    def point_gizmo(self, yaw, pitch):
        self.gizmo_commands.publish(Float64MultiArray(data=[float(yaw), float(pitch)]))

    def joint(self, name):
        return self.joints.get(name, (math.nan, math.nan))

    def motor_position(self, can_id):
        with self.fake.lock:
            return self.fake.motors[can_id].position_deg

    def interrupt(self, timeout=15.0):
        """SIGINT ros2_control_node, as ros2 launch does on Ctrl+C; returns its exit code."""
        os.killpg(self.control.pid, signal.SIGINT)
        return self.control.wait(timeout=timeout)

    def close(self):
        if self.control is not None and self.control.poll() is None:
            try:
                self.interrupt()
            except subprocess.TimeoutExpired:
                os.killpg(self.control.pid, signal.SIGKILL)
                self.control.wait()
        self.log_file.close()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.try_shutdown(context=self.context)
        self.fake.close()


@pytest.fixture
def start_robot(tmp_path):
    robots = []

    def start(**kwargs):
        robot = Robot(tmp_path, **kwargs)
        robots.append(robot)
        return robot
    yield start
    for robot in robots:
        robot.close()


def ends_with_stop(commands, since=None, brake_time=0.3):
    """True if commands after since are zero speed for about brake_time, then a release."""
    commands = [command for command in commands if since is None or command.time >= since]
    if not commands or (commands[-1].mode, commands[-1].value) != (MODE_CURRENT, 0.0):
        return False
    braking = []
    for command in reversed(commands[:-1]):
        if (command.mode, command.value) != (MODE_SPEED, 0.0):
            break
        braking.append(command)
    return len(braking) >= 5 and braking[0].time - braking[-1].time >= brake_time - 0.05


def assert_stopped(robot, can_ids, since):
    for can_id in can_ids:
        assert robot.fake.wait_for(
            lambda: ends_with_stop(robot.fake.commands(can_id), since), 3.0), (
            f"CAN ID {can_id} was not stopped: "
            f"{[str(c) for c in robot.fake.commands(can_id, since=since)][-8:]}")


def test_gizmo_is_zeroed_then_clamped_and_slewed_by_the_plugin(start_robot):
    def prepare(fake):
        fake.motors[YAW].set_position(30.0)
        fake.motors[PITCH].set_position(-20.0)
    robot = start_robot(yaw_direction=-1, prepare=prepare)
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    # The bringup URDF switches the controller manager's joint limiter off for the gizmo
    # joints, so only the plugin limits their commands
    for joint in ("gizmo_yaw_joint", "gizmo_pitch_joint"):
        assert f"Joint limits are disabled for joint '{joint}'" in robot.log()

    for can_id in (YAW, PITCH):
        commands = robot.fake.commands(can_id)
        modes = [command.mode for command in commands]
        # Released, then a temporary origin (1 byte, 0), before any position command
        assert modes[:2] == [MODE_CURRENT, MODE_SET_ORIGIN], [str(c) for c in commands]
        assert commands[1].data == b"\x00"
        assert MODE_POSITION not in modes
        assert abs(robot.motor_position(can_id)) < 0.05
    assert "Could not zero" not in robot.log()

    robot.spawn(*ALL_CONTROLLERS)
    robot.wait(lambda: abs(robot.joint("gizmo_yaw_joint")[0]) < 0.01
               and abs(robot.joint("gizmo_pitch_joint")[0]) < 0.01, 5.0,
               "the zeroed gizmo does not read 0 in /joint_states")
    # Holding position 0 before the first command
    robot.spin(0.3)
    assert robot.fake.commands(YAW, MODE_POSITION)
    assert abs(robot.motor_position(YAW)) < 0.05

    # Both beyond their limits: yaw to its upper limit, pitch stays at its upper limit 0
    start = time.monotonic()
    robot.point_gizmo(2.0, 0.5)
    robot.wait(lambda: abs(robot.joint("gizmo_yaw_joint")[0] - YAW_LIMIT) < 0.005, 5.0,
               "yaw did not reach its upper limit")
    elapsed = time.monotonic() - start
    assert elapsed > 0.9 * YAW_LIMIT / GIZMO_VELOCITY
    robot.spin(0.2)
    # direction -1: the motor turns the other way
    assert robot.motor_position(YAW) == pytest.approx(-45.0, abs=0.05)

    targets = [command.value for command in robot.fake.commands(YAW, MODE_POSITION, start)]
    steps = [b - a for a, b in zip(targets, targets[1:])]
    assert min(targets) == pytest.approx(-45.0, abs=1e-3)
    assert all(step <= 1e-6 for step in steps), "the yaw target moved back"
    # 1 rad/s over one 20 ms cycle is 1.15 deg; a late cycle may slew up to 50 ms
    assert max(abs(step) for step in steps) < 2.9
    assert sum(abs(step) > 0.5 for step in steps) >= 30
    pitch_targets = [command.value for command in robot.fake.commands(PITCH, MODE_POSITION)]
    assert max(pitch_targets) <= 1e-3, "pitch was commanded above its upper limit"

    robot.point_gizmo(0.0, -0.3)
    robot.wait(lambda: abs(robot.joint("gizmo_pitch_joint")[0] + 0.3) < 0.005, 5.0,
               "pitch did not reach -0.3 rad")
    robot.wait(lambda: abs(robot.joint("gizmo_yaw_joint")[0]) < 0.005, 5.0,
               "yaw did not return to 0")
    assert robot.motor_position(PITCH) == pytest.approx(math.degrees(-0.3), abs=0.1)
    # The jumps above would each make the controller manager's limiter log an error
    assert "out of limits" not in robot.log()


def test_wheels_turn_the_robot_the_right_way(start_robot):
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn("joint_state_broadcaster", "diff_drive_controller")

    start = time.monotonic()
    robot.drive(linear=0.2, seconds=1.0)
    wheel = 0.2 / WHEEL_RADIUS
    left = robot.fake.commands(LEFT, MODE_SPEED, start)[-1].value
    right = robot.fake.commands(RIGHT, MODE_SPEED, start)[-1].value
    # Left motor direction 1, right motor direction -1 (motors.yaml)
    assert left == pytest.approx(wheel * ERPM_PER_RAD_S, abs=2.0)
    assert right == pytest.approx(-wheel * ERPM_PER_RAD_S, abs=2.0)
    for name in ("wheel_joint_left", "wheel_joint_right"):
        position, velocity = robot.joint(name)
        assert velocity == pytest.approx(wheel, rel=0.02), name
        assert position > 1.0, name

    # Turning left in place: the left wheel backwards, the right wheel forwards, which
    # is the same motor direction for both mirror-mounted motors
    start = time.monotonic()
    robot.drive(angular=1.0, seconds=1.0)
    wheel = 1.0 * WHEEL_SEPARATION / 2.0 / WHEEL_RADIUS
    assert robot.fake.commands(LEFT, MODE_SPEED, start)[-1].value == pytest.approx(
        -wheel * ERPM_PER_RAD_S, abs=2.0)
    assert robot.fake.commands(RIGHT, MODE_SPEED, start)[-1].value == pytest.approx(
        -wheel * ERPM_PER_RAD_S, abs=2.0)
    assert robot.joint("wheel_joint_left")[1] == pytest.approx(-wheel, rel=0.02)
    assert robot.joint("wheel_joint_right")[1] == pytest.approx(wheel, rel=0.02)


def test_deactivating_a_component_stops_only_its_motors(start_robot):
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn(*ALL_CONTROLLERS)
    robot.drive(linear=0.2, seconds=0.5)

    since = time.monotonic()
    assert robot.set_hardware_state("WheelSystem", State.PRIMARY_STATE_INACTIVE, "inactive")
    assert_stopped(robot, (LEFT, RIGHT), since)
    robot.drive(linear=0.2, seconds=0.5)
    # No wheel commands once inactive; the gizmo keeps holding its position
    assert robot.fake.commands(LEFT, MODE_SPEED, time.monotonic() - 0.3) == []
    assert robot.fake.commands(YAW, MODE_POSITION, time.monotonic() - 0.3)
    assert robot.hardware_states()["GizmoSystem"] == "active"

    since = time.monotonic()
    assert robot.set_hardware_state("GizmoSystem", State.PRIMARY_STATE_INACTIVE, "inactive")
    assert_stopped(robot, (YAW, PITCH), since)
    robot.spin(0.3)
    assert robot.fake.commands(YAW, MODE_POSITION, time.monotonic() - 0.2) == []


def test_deactivating_a_controller_stops_the_motors_it_drove(start_robot):
    # The upstream plugin sends nothing for a joint that no controller commands, so
    # without the plugin's own stop the motor would keep its last command until its CAN
    # timeout (if one is set at all)
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn(*ALL_CONTROLLERS)
    robot.point_gizmo(0.3, -0.2)
    robot.drive(linear=0.2, seconds=1.0)

    since = time.monotonic()
    assert robot.switch_controllers(deactivate=["diff_drive_controller"])
    assert_stopped(robot, (LEFT, RIGHT), since)
    assert ("No controller commands wheel_joint_left (CAN ID 10) any more: zero speed for "
            "300 ms, then release" in robot.log())
    robot.spin(0.3)
    # Nothing more for the wheels; the gizmo keeps holding its position
    assert robot.fake.commands(LEFT, since=time.monotonic() - 0.2) == []
    assert robot.fake.commands(YAW, MODE_POSITION, time.monotonic() - 0.2)
    assert robot.fake.commands(YAW, MODE_SPEED, since) == []
    assert robot.hardware_states() == {"WheelSystem": "active", "GizmoSystem": "active"}

    # Reactivated, the wheels are driven again
    assert robot.switch_controllers(activate=["diff_drive_controller"])
    start = time.monotonic()
    robot.drive(linear=0.2, seconds=0.5)
    recent = robot.fake.commands(LEFT, MODE_SPEED, start)
    assert recent and recent[-1].value > 1000.0

    since = time.monotonic()
    assert robot.switch_controllers(deactivate=["gizmo_controller"])
    robot.drive(linear=0.2, seconds=0.5)
    assert_stopped(robot, (YAW, PITCH), since)
    robot.drive(linear=0.2, seconds=0.3)
    assert robot.fake.commands(YAW, since=time.monotonic() - 0.2) == []
    recent = robot.fake.commands(LEFT, MODE_SPEED, time.monotonic() - 0.2)
    assert recent and recent[-1].value > 1000.0
    assert robot.controller_states()["joint_state_broadcaster"] == "active"


def test_a_gizmo_fault_stops_both_gizmo_motors_and_leaves_the_wheels(start_robot):
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn(*ALL_CONTROLLERS)
    robot.drive(linear=0.2, seconds=0.5)

    since = time.monotonic()
    robot.fake.inject_fault(PITCH, 7)
    robot.drive(linear=0.2, seconds=1.5)
    assert_stopped(robot, (YAW, PITCH), since)
    assert "gizmo_pitch_joint (CAN ID 13) reports fault 7: motor stall" in robot.log()
    assert robot.hardware_states() == {"WheelSystem": "active", "GizmoSystem": "unconfigured"}
    controllers = robot.controller_states()
    assert controllers["diff_drive_controller"] == "active"
    assert controllers["gizmo_controller"] == "inactive"
    # The broadcaster reads the gizmo state interfaces too, so it is stopped with them
    assert controllers["joint_state_broadcaster"] == "inactive"
    # The wheels are still driven
    recent = robot.fake.commands(LEFT, MODE_SPEED, time.monotonic() - 0.3)
    assert recent and recent[-1].value > 1000.0
    assert robot.control.poll() is None


def test_a_silent_wheel_motor_stops_both_wheels_and_leaves_the_gizmo(start_robot):
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn(*ALL_CONTROLLERS)
    robot.drive(linear=0.2, seconds=0.5)

    since = time.monotonic()
    robot.fake.silence(RIGHT)
    robot.drive(linear=0.2, seconds=1.0)
    assert_stopped(robot, (LEFT, RIGHT), since)
    assert "No status frames from the motor on wheel_joint_right (CAN ID 11)" in robot.log()
    assert robot.hardware_states() == {"WheelSystem": "unconfigured", "GizmoSystem": "active"}
    controllers = robot.controller_states()
    assert controllers["diff_drive_controller"] == "inactive"
    assert controllers["joint_state_broadcaster"] == "inactive"
    assert controllers["gizmo_controller"] == "active"
    assert robot.fake.commands(YAW, MODE_POSITION, time.monotonic() - 0.3)
    assert robot.control.poll() is None


def test_fixed_gizmo_mode_leaves_the_gizmo_motors_alone(start_robot):
    robot = start_robot(gizmo_mode="fixed")
    robot.wait_for_hardware({"WheelSystem": "active"})
    assert robot.hardware_states() == {"WheelSystem": "active"}
    robot.spawn("joint_state_broadcaster", "diff_drive_controller")
    robot.drive(linear=0.2, seconds=0.5)
    assert robot.fake.commands(LEFT, MODE_SPEED)
    assert robot.fake.commands(YAW) == [] and robot.fake.commands(PITCH) == []


def test_a_failed_zero_names_the_joint_and_stops_the_gizmo(start_robot):
    def prepare(fake):
        fake.motors[YAW].set_position(30.0)
        fake.motors[YAW].ignore_origin = True
    robot = start_robot(prepare=prepare)
    # controller_manager treats a component that fails to activate at start-up as fatal
    robot.wait(lambda: robot.control.poll() is not None, 15.0,
               "ros2_control_node kept running after a failed activation")
    assert robot.control.returncode != 0
    assert ("Could not zero gizmo_yaw_joint (CAN ID 12): it reads 30.0 deg"
            in robot.log())
    for can_id in (YAW, PITCH):
        assert ends_with_stop(robot.fake.commands(can_id))
        assert robot.fake.commands(can_id, MODE_POSITION) == []
    for can_id in (LEFT, RIGHT):
        assert all(command.value == 0.0 for command in robot.fake.commands(can_id))


def test_the_stall_guard_stops_a_blocked_gizmo(start_robot):
    def prepare(fake):
        fake.motors[YAW].stuck = True
        # 1.0 A * 0.127 Nm/A * 10 = 1.27 Nm, above stall_effort 1.0 Nm
        fake.motors[YAW].load_current_a = 1.0
    robot = start_robot(prepare=prepare)
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn(*ALL_CONTROLLERS)
    robot.point_gizmo(0.5, 0.0)
    robot.wait(lambda: "has pushed with" in robot.log(), 5.0, "the stall guard did not trip")
    assert "gizmo_yaw_joint (CAN ID 12) has pushed with 1.27 Nm" in robot.log()
    first_position = robot.fake.commands(YAW, MODE_POSITION)[0].time
    assert_stopped(robot, (YAW, PITCH), first_position)
    stop = robot.fake.commands(YAW, MODE_SPEED)[0].time
    assert stop - first_position >= 0.45
    assert robot.hardware_states()["WheelSystem"] == "active"


def test_a_joint_pushed_past_its_limit_keeps_its_controller(start_robot):
    # With the controller manager's joint limiter on the gizmo joints (switched off in
    # the bringup URDF), the first command here would raise an error and the controller
    # manager would deactivate gizmo_controller
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    # Pushed 5 deg above the pitch upper limit (0) and held there
    with robot.fake.lock:
        robot.fake.motors[PITCH].set_position(5.0)
        robot.fake.motors[PITCH].stuck = True
    robot.spin(0.2)
    robot.spawn(*ALL_CONTROLLERS)
    start = time.monotonic()
    robot.point_gizmo(0.0, 0.0)
    robot.spin(1.0)
    assert robot.control.poll() is None, robot.log()
    assert robot.controller_states()["gizmo_controller"] == "active"
    targets = [command.value for command in robot.fake.commands(PITCH, MODE_POSITION)]
    # The target starts where the joint is, then slews down to the limit
    assert targets[0] == pytest.approx(5.0, abs=0.2)
    later = [command.value for command in robot.fake.commands(PITCH, MODE_POSITION, start)]
    assert later[-1] == pytest.approx(0.0, abs=1e-3)
    assert max(abs(b - a) for a, b in zip(later, later[1:])) < 2.9
    assert "out of limits" not in robot.log()


def test_stopping_the_controller_manager_stops_every_motor(start_robot):
    robot = start_robot()
    robot.wait_for_hardware({"WheelSystem": "active", "GizmoSystem": "active"})
    robot.spawn(*ALL_CONTROLLERS)
    robot.drive(linear=0.2, seconds=0.5)
    since = time.monotonic()
    robot.interrupt()
    assert_stopped(robot, MOTOR_IDS, since)
