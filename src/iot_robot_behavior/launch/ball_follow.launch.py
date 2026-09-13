from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = ParameterValue(
        LaunchConfiguration("use_sim_time"), value_type=bool)
    config = PathJoinSubstitution(
        [FindPackageShare("iot_robot_behavior"), "config", "ball_follow.yaml"])

    ball_detector = Node(
        package="iot_robot_perception",
        executable="ball_detector",
        parameters=[config, {"use_sim_time": use_sim_time}],
    )

    target_follower = Node(
        package="iot_robot_behavior",
        executable="target_follower",
        parameters=[config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("target", "/ball/position"),
            ("cmd_vel", "/cmd_vel/follower"),
            ("gizmo_commands", "/gizmo_controller/commands"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        ball_detector,
        target_follower,
    ])
