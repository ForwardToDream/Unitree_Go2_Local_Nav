from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    use_gazebo_gui = LaunchConfiguration("use_gazebo_gui")
    use_vel_controller = LaunchConfiguration("use_vel_controller")
    freeze_gazebo_model = LaunchConfiguration("freeze_gazebo_model")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    spawn_x = LaunchConfiguration("x")
    spawn_y = LaunchConfiguration("y")
    spawn_z = LaunchConfiguration("z")
    spawn_roll = LaunchConfiguration("R")
    spawn_pitch = LaunchConfiguration("P")
    spawn_yaw = LaunchConfiguration("Y")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gzserver.launch.py"]
            )
        ),
        launch_arguments={
            "world": world,
            "verbose": "true",
            "factory": "true",
            "init": "true",
            "force_system": "true",
        }.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gzclient.launch.py"]
            )
        ),
        launch_arguments={
            "verbose": "true",
        }.items(),
        condition=IfCondition(use_gazebo_gui),
    )

    base_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("go2_bringup"), "launch", "go2_base.launch.py"]
            )
        ),
        launch_arguments={
            "use_joint_state_publisher": use_joint_state_publisher,
            "use_xacro": "true",
            "use_rviz": use_rviz,
            "use_map_to_odom_tf": "true",
            "enable_lidar": "true",
            "enable_gazebo_lidar_plugin": "true",
            "gazebo_lidar_sensor_type": "gpu_ray",
            "enable_depth_camera": "true",
            "enable_gazebo_depth_plugin": "true",
            "enable_gazebo_planar_move": "true",
            "freeze_gazebo_model": freeze_gazebo_model,
            "cmd_vel_topic": cmd_vel_topic,
            "publish_odom_tf": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-topic",
            "robot_description",
            "-package_to_model",
            "-entity",
            entity_name,
            "-timeout",
            "60.0",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-R",
            spawn_roll,
            "-P",
            spawn_pitch,
            "-Y",
            spawn_yaw,
        ],
    )

    delayed_spawn_entity = TimerAction(
        period=2.0,
        actions=[spawn_entity],
    )

    vel_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("go2_vel_controller"),
                    "launch",
                    "go2_vel_controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "cmd_vel_topic": cmd_vel_topic,
        }.items(),
        condition=IfCondition(use_vel_controller),
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description="Keep non-actuated joints visible in RViz/TF.",
            ),
            DeclareLaunchArgument(
                "use_gazebo_gui",
                default_value="false",
                description="Start Gazebo Classic GUI client (gzclient).",
            ),
            DeclareLaunchArgument(
                "use_vel_controller",
                default_value="true",
                description="Start the Go2 Tk velocity controller GUI.",
            ),
            DeclareLaunchArgument(
                "freeze_gazebo_model",
                default_value="true",
                description="Lock Go2 leg joints in a standing pose while keeping base motion enabled.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "worlds", "empty.world"]
                ),
            ),
            DeclareLaunchArgument("entity_name", default_value="go2"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.35"),
            DeclareLaunchArgument("R", default_value="0.0"),
            DeclareLaunchArgument("P", default_value="0.0"),
            DeclareLaunchArgument("Y", default_value="0.0"),
            gzserver,
            gzclient,
            base_stack,
            delayed_spawn_entity,
            vel_controller,
        ]
    )
