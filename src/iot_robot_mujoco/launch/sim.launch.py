from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("iot_robot_mujoco")
    urdf = PathJoinSubstitution([pkg, "urdf", "iot_robot_sim.urdf.xacro"])
    controllers = PathJoinSubstitution([pkg, "config", "controllers.yaml"])
    headless = LaunchConfiguration("headless")

    robot_description = ParameterValue(
        Command(["xacro ", urdf, " headless:=", headless]), value_type=str
    )

    def spawner(name):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--param-file", controllers],
        )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": True}],
        ),
        # mujoco_ros2_control ships its own patched ros2_control_node; the stock one won't work
        Node(
            package="mujoco_ros2_control",
            executable="ros2_control_node",
            output="both",
            parameters=[{"use_sim_time": True}, controllers],
            on_exit=Shutdown(),
        ),
        spawner("joint_state_broadcaster"),
        spawner("diff_drive_controller"),
        spawner("gizmo_controller"),
    ])
