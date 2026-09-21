from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("iot_robot_web"), "config", "web_controller.yaml"])

    return LaunchDescription([
        # Also passed on to the follow launches the controller starts
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("port", default_value="8080"),
        DeclareLaunchArgument(
            "start_estopped", default_value="false",
            description="Engage the E-stop at every start (robot.launch.py sets true)"),
        Node(
            package="iot_robot_web",
            executable="web_controller",
            output="screen",
            parameters=[config, {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool),
                "port": ParameterValue(LaunchConfiguration("port"), value_type=int),
                "start_estopped": ParameterValue(
                    LaunchConfiguration("start_estopped"), value_type=bool),
            }],
        ),
    ])
