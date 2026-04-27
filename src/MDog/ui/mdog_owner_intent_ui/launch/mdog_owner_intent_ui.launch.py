from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    intent_topic = LaunchConfiguration("intent_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    nav_feedback_topic = LaunchConfiguration("nav_feedback_topic")
    nav_decision_topic = LaunchConfiguration("nav_decision_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("intent_topic", default_value="/mdog/owner_intent"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("nav_feedback_topic", default_value="/mdog/nav_feedback"),
            DeclareLaunchArgument("nav_decision_topic", default_value="/mdog/nav_decision"),
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
                    }
                ],
            ),
        ]
    )
