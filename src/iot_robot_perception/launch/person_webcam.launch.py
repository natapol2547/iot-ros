from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("iot_robot_perception"), "config", "person_webcam.yaml"])
    default_model = PathJoinSubstitution(
        [EnvironmentVariable("PIXI_PROJECT_ROOT"), "models", "yolo26n-pose.onnx"])

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value=default_model),
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            namespace="camera",
            parameters=[config],
        ),
        Node(
            package="iot_robot_perception",
            executable="person_detector",
            parameters=[
                config, {"model_path": LaunchConfiguration("model_path")}],
        ),
    ])
