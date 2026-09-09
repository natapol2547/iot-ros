from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf = PathJoinSubstitution([
        FindPackageShare("iot_robot_description"), "urdf", "robot.urdf.xacro"
    ])

    # Run xacro at launch time and hand the expanded URDF to the publisher.
    # value_type=str is essential: without it the XML gets parsed as YAML and
    # you get a cryptic type error instead of a robot.
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    return LaunchDescription([
        # Reads the URDF + /joint_states, publishes the TF tree.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        # Slider GUI that fakes /joint_states. Replaced by real controllers later.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(package="rviz2", executable="rviz2", output="screen"),
    ])
