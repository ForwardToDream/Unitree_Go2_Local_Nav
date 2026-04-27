from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_map_to_odom_tf = LaunchConfiguration("use_map_to_odom_tf")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", map_frame, odom_frame],
        condition=IfCondition(use_map_to_odom_tf),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_map_to_odom_tf",
                default_value="true",
                description="Publish a static map->odom transform as a localization placeholder.",
            ),
            DeclareLaunchArgument(
                "map_frame",
                default_value="map",
                description="Global map frame id.",
            ),
            DeclareLaunchArgument(
                "odom_frame",
                default_value="odom",
                description="Local odometry frame id.",
            ),
            map_to_odom_tf,
        ]
    )
