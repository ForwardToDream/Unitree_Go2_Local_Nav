from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    intent_topic = LaunchConfiguration("intent_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    nav_feedback_topic = LaunchConfiguration("nav_feedback_topic")
    nav_decision_topic = LaunchConfiguration("nav_decision_topic")
    publish_rate = LaunchConfiguration("publish_rate")
    default_speed = LaunchConfiguration("default_speed")

    return LaunchDescription(
        [
            DeclareLaunchArgument("intent_topic", default_value="/mdog/owner_intent"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("nav_feedback_topic", default_value="/mdog/nav_feedback"),
            DeclareLaunchArgument("nav_decision_topic", default_value="/mdog/nav_decision"),
            DeclareLaunchArgument("publish_rate", default_value="10.0"),
            DeclareLaunchArgument("default_speed", default_value="0.20"),
            Node(
                package="mdog_owner_intent_ui",
                executable="mdog_owner_intent_ui.py",
                name="mdog_owner_intent_ui",
                output="screen",
                parameters=[
                    {
                        "intent_topic": intent_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "nav_feedback_topic": nav_feedback_topic,
                        "nav_decision_topic": nav_decision_topic,
                        "publish_rate": ParameterValue(publish_rate, value_type=float),
                        "default_speed": ParameterValue(default_speed, value_type=float),
                    }
                ],
            ),
        ]
    )
