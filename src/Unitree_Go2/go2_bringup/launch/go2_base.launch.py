from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_xacro = LaunchConfiguration("use_xacro")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_map_to_odom_tf = LaunchConfiguration("use_map_to_odom_tf")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_gazebo_lidar_plugin = LaunchConfiguration("enable_gazebo_lidar_plugin")
    gazebo_lidar_cloud_topic = LaunchConfiguration("gazebo_lidar_cloud_topic")
    gazebo_lidar_update_rate = LaunchConfiguration("gazebo_lidar_update_rate")
    gazebo_lidar_cloud_horizontal_samples = LaunchConfiguration(
        "gazebo_lidar_cloud_horizontal_samples"
    )
    gazebo_lidar_cloud_vertical_samples = LaunchConfiguration(
        "gazebo_lidar_cloud_vertical_samples"
    )
    gazebo_lidar_scan_samples = LaunchConfiguration("gazebo_lidar_scan_samples")
    gazebo_lidar_sensor_type = LaunchConfiguration("gazebo_lidar_sensor_type")
    gazebo_lidar_min_angle = LaunchConfiguration("gazebo_lidar_min_angle")
    gazebo_lidar_max_angle = LaunchConfiguration("gazebo_lidar_max_angle")
    gazebo_lidar_vertical_min_angle = LaunchConfiguration(
        "gazebo_lidar_vertical_min_angle"
    )
    gazebo_lidar_vertical_max_angle = LaunchConfiguration(
        "gazebo_lidar_vertical_max_angle"
    )
    gazebo_lidar_min_range = LaunchConfiguration("gazebo_lidar_min_range")
    gazebo_lidar_max_range = LaunchConfiguration("gazebo_lidar_max_range")
    enable_depth_camera = LaunchConfiguration("enable_depth_camera")
    enable_gazebo_depth_plugin = LaunchConfiguration("enable_gazebo_depth_plugin")
    enable_gazebo_planar_move = LaunchConfiguration("enable_gazebo_planar_move")
    freeze_gazebo_model = LaunchConfiguration("freeze_gazebo_model")
    rmw_implementation = LaunchConfiguration("rmw_implementation")
    cyclonedds_uri = LaunchConfiguration("cyclonedds_uri")

    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    sport_request_topic = LaunchConfiguration("sport_request_topic")
    sport_state_topic = LaunchConfiguration("sport_state_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    odom_source = LaunchConfiguration("odom_source")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    scan_topic = LaunchConfiguration("scan_topic")

    xacro_file = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "urdf", "go2_description.urdf.xacro"]
    )
    urdf_file = PathJoinSubstitution(
        [FindPackageShare("go2_description"), "urdf", "go2_description.urdf"]
    )

    robot_description_xacro = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " enable_lidar:=",
                enable_lidar,
                " enable_gazebo_lidar_plugin:=",
                enable_gazebo_lidar_plugin,
                " gazebo_scan_topic:=",
                scan_topic,
                " gazebo_lidar_cloud_topic:=",
                gazebo_lidar_cloud_topic,
                " gazebo_lidar_update_rate:=",
                gazebo_lidar_update_rate,
                " gazebo_lidar_cloud_horizontal_samples:=",
                gazebo_lidar_cloud_horizontal_samples,
                " gazebo_lidar_cloud_vertical_samples:=",
                gazebo_lidar_cloud_vertical_samples,
                " gazebo_lidar_scan_samples:=",
                gazebo_lidar_scan_samples,
                " gazebo_lidar_sensor_type:=",
                gazebo_lidar_sensor_type,
                " gazebo_lidar_min_angle:=",
                gazebo_lidar_min_angle,
                " gazebo_lidar_max_angle:=",
                gazebo_lidar_max_angle,
                " gazebo_lidar_vertical_min_angle:=",
                gazebo_lidar_vertical_min_angle,
                " gazebo_lidar_vertical_max_angle:=",
                gazebo_lidar_vertical_max_angle,
                " gazebo_lidar_min_range:=",
                gazebo_lidar_min_range,
                " gazebo_lidar_max_range:=",
                gazebo_lidar_max_range,
                " enable_depth_camera:=",
                enable_depth_camera,
                " enable_gazebo_depth_plugin:=",
                enable_gazebo_depth_plugin,
                " enable_gazebo_planar_move:=",
                enable_gazebo_planar_move,
                " freeze_gazebo_model:=",
                freeze_gazebo_model,
                " gazebo_cmd_vel_topic:=",
                cmd_vel_topic,
            ]
        ),
        value_type=str,
    )
    robot_description_urdf = ParameterValue(
        Command(["cat ", urdf_file]),
        value_type=str,
    )

    robot_state_publisher_xacro = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_xacro,
                "publish_frequency": 30.0,
                "ignore_timestamp": True,
            }
        ],
        condition=IfCondition(use_xacro),
    )

    robot_state_publisher_urdf = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_urdf,
                "publish_frequency": 30.0,
                "ignore_timestamp": True,
            }
        ],
        condition=UnlessCondition(use_xacro),
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        condition=IfCondition(use_joint_state_publisher),
    )

    cmd_bridge = Node(
        package="go2_cmd_bridge",
        executable="go2_cmd_vel_bridge",
        name="go2_cmd_vel_bridge",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": cmd_vel_topic,
                "sport_request_topic": sport_request_topic,
            }
        ],
    )

    state_estimator = Node(
        package="go2_state_estimation",
        executable="go2_state_estimator",
        name="go2_state_estimator",
        output="screen",
        parameters=[
            {
                "odom_source": odom_source,
                "cmd_vel_topic": cmd_vel_topic,
                "sport_state_topic": sport_state_topic,
                "odom_topic": odom_topic,
                "odom_frame": odom_frame,
                "base_frame": base_frame,
                "publish_tf": publish_odom_tf,
            }
        ],
    )

    localization_stub = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("go2_bringup"),
                    "launch",
                    "localization_stub.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_map_to_odom_tf": use_map_to_odom_tf,
            "map_frame": map_frame,
            "odom_frame": odom_frame,
        }.items(),
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
            DeclareLaunchArgument(
                "use_xacro",
                default_value="true",
                description="If true, parse xacro at launch; if false, use pre-generated URDF.",
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_map_to_odom_tf", default_value="true"),
            DeclareLaunchArgument(
                "rmw_implementation",
                default_value="rmw_cyclonedds_cpp",
                description="ROS 2 middleware implementation.",
            ),
            DeclareLaunchArgument(
                "cyclonedds_uri",
                default_value='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="false" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>',
                description="CycloneDDS configuration XML passed via CYCLONEDDS_URI.",
            ),
            DeclareLaunchArgument("enable_lidar", default_value="true"),
            DeclareLaunchArgument(
                "enable_gazebo_lidar_plugin", default_value="false"
            ),
            DeclareLaunchArgument(
                "gazebo_lidar_cloud_topic", default_value="/unilidar/cloud"
            ),
            DeclareLaunchArgument("gazebo_lidar_update_rate", default_value="5.55"),
            DeclareLaunchArgument(
                "gazebo_lidar_cloud_horizontal_samples", default_value="592"
            ),
            DeclareLaunchArgument(
                "gazebo_lidar_cloud_vertical_samples", default_value="39"
            ),
            DeclareLaunchArgument("gazebo_lidar_scan_samples", default_value="592"),
            DeclareLaunchArgument(
                "gazebo_lidar_sensor_type",
                default_value="gpu_ray",
                description="Gazebo lidar sensor type: ray or gpu_ray",
            ),
            DeclareLaunchArgument(
                "gazebo_lidar_min_angle", default_value="-3.14159265359"
            ),
            DeclareLaunchArgument(
                "gazebo_lidar_max_angle", default_value="3.14159265359"
            ),
            DeclareLaunchArgument(
                "gazebo_lidar_vertical_min_angle", default_value="-0.83775804096"
            ),
            DeclareLaunchArgument(
                "gazebo_lidar_vertical_max_angle", default_value="0.83775804096"
            ),
            DeclareLaunchArgument("gazebo_lidar_min_range", default_value="0.05"),
            DeclareLaunchArgument("gazebo_lidar_max_range", default_value="30.0"),
            DeclareLaunchArgument("enable_depth_camera", default_value="true"),
            DeclareLaunchArgument("enable_gazebo_depth_plugin", default_value="false"),
            DeclareLaunchArgument("enable_gazebo_planar_move", default_value="false"),
            DeclareLaunchArgument("freeze_gazebo_model", default_value="false"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument(
                "sport_request_topic", default_value="/api/sport/request"
            ),
            DeclareLaunchArgument(
                "sport_state_topic", default_value="lf/sportmodestate"
            ),
            DeclareLaunchArgument("odom_topic", default_value="/go2/odom"),
            DeclareLaunchArgument(
                "odom_source",
                default_value="cmd_vel",
                description="cmd_vel or sport_mode_state",
            ),
            DeclareLaunchArgument(
                "publish_odom_tf",
                default_value="true",
                description="Whether go2_state_estimator publishes odom->base_link TF.",
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("scan_topic", default_value="/unilidar/laserscan"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("go2_bringup"), "rviz", "go2_base.rviz"]
                ),
            ),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", rmw_implementation),
            SetEnvironmentVariable("CYCLONEDDS_URI", cyclonedds_uri),
            localization_stub,
            joint_state_publisher,
            robot_state_publisher_xacro,
            robot_state_publisher_urdf,
            cmd_bridge,
            state_estimator,
            rviz2,
        ]
    )
