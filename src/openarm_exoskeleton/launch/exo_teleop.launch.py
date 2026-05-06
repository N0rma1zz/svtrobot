from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("left_port", default_value="/dev/ttyUSB1", description="Left exoskeleton serial port"),
        DeclareLaunchArgument("right_port", default_value="/dev/ttyUSB0", description="Right exoskeleton serial port"),
        DeclareLaunchArgument("calibration_file", default_value="~/.openarm_exo_calibration.json", description="Calibration file path"),
        DeclareLaunchArgument("publish_rate", default_value="50.0", description="Publish rate in Hz"),
        DeclareLaunchArgument("recalibrate_on_start", default_value="false", description="Recalibrate zero pose on startup"),
        DeclareLaunchArgument("calibration_only", default_value="false", description="Only calibrate and exit"),
        Node(
            package="openarm_exoskeleton",
            executable="servo_zero_node",
            name="dual_arm_servo_teleop",
            output="screen",
            parameters=[{
                "left_port": LaunchConfiguration("left_port"),
                "right_port": LaunchConfiguration("right_port"),
                "calibration_file": LaunchConfiguration("calibration_file"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "recalibrate_on_start": LaunchConfiguration("recalibrate_on_start"),
                "calibration_only": LaunchConfiguration("calibration_only"),
            }],
        ),
    ])