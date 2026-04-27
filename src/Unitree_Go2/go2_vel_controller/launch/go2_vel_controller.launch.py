from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    publish_rate = LaunchConfiguration("publish_rate")
    max_linear = LaunchConfiguration("max_linear")
    max_angular = LaunchConfiguration("max_angular")

    return LaunchDescription(
        [
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("max_linear", default_value="1.0"),
            DeclareLaunchArgument("max_angular", default_value="2.0"),
            Node(
                package="go2_vel_controller",
                executable="go2_vel_controller.py",
                name="go2_vel_controller",
                output="screen",
                parameters=[
                    {
                        "cmd_vel_topic": cmd_vel_topic,
                        "publish_rate": ParameterValue(publish_rate, value_type=float),
                        "max_linear": ParameterValue(max_linear, value_type=float),
                        "max_angular": ParameterValue(max_angular, value_type=float),
                    }
                ],
            ),
        ]
    )
