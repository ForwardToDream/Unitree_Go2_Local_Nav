from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_frame = LaunchConfiguration("target_frame")
    lidar_cloud_topic = LaunchConfiguration("lidar_cloud_topic")
    depth_cloud_topic = LaunchConfiguration("depth_cloud_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    use_intent_ui = LaunchConfiguration("use_intent_ui")
    use_dog_model = LaunchConfiguration("use_dog_model")
    use_owner_model = LaunchConfiguration("use_owner_model")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    intent_topic = LaunchConfiguration("intent_topic")
    fused_points_topic = LaunchConfiguration("fused_points_topic")
    local_grid_topic = LaunchConfiguration("local_grid_topic")
    traversability_topic = LaunchConfiguration("traversability_topic")

    resolution = LaunchConfiguration("resolution")
    roi_front_m = LaunchConfiguration("roi_front_m")
    roi_back_m = LaunchConfiguration("roi_back_m")
    roi_side_m = LaunchConfiguration("roi_side_m")
    human_width_m = LaunchConfiguration("human_width_m")
    human_height_m = LaunchConfiguration("human_height_m")
    safety_margin_m = LaunchConfiguration("safety_margin_m")
    ground_z_in_base_m = LaunchConfiguration("ground_z_in_base_m")
    stale_timeout_sec = LaunchConfiguration("stale_timeout_sec")

    self_filter_front_m = LaunchConfiguration("self_filter_front_m")
    self_filter_back_m = LaunchConfiguration("self_filter_back_m")
    self_filter_side_m = LaunchConfiguration("self_filter_side_m")
    self_filter_min_z = LaunchConfiguration("self_filter_min_z")
    self_filter_max_z = LaunchConfiguration("self_filter_max_z")

    owner_dog_rear_x_m = LaunchConfiguration("owner_dog_rear_x_m")
    owner_follow_gap_m = LaunchConfiguration("owner_follow_gap_m")
    owner_depth_m = LaunchConfiguration("owner_depth_m")

    return LaunchDescription(
        [
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp"),
            SetEnvironmentVariable(
                "CYCLONEDDS_URI",
                '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" priority="default" multicast="false" /></Interfaces></General><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>',
            ),
            DeclareLaunchArgument("target_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_cloud_topic", default_value="/unilidar/cloud"),
            DeclareLaunchArgument("depth_cloud_topic", default_value="/go2/camera/depth/points"),
            DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("use_intent_ui", default_value="true"),
            DeclareLaunchArgument("use_dog_model", default_value="true"),
            DeclareLaunchArgument("use_owner_model", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("mdog_bringup"), "rviz", "mdog_local_nav.rviz"]
                ),
            ),
            DeclareLaunchArgument("intent_topic", default_value="/mdog/owner_intent"),
            DeclareLaunchArgument("fused_points_topic", default_value="/mdog/fused_points"),
            DeclareLaunchArgument("local_grid_topic", default_value="/mdog/local_grid"),
            DeclareLaunchArgument("traversability_topic", default_value="/mdog/traversability"),
            DeclareLaunchArgument("resolution", default_value="0.10"),
            DeclareLaunchArgument("roi_front_m", default_value="4.0"),
            DeclareLaunchArgument("roi_back_m", default_value="0.8"),
            DeclareLaunchArgument("roi_side_m", default_value="2.0"),
            DeclareLaunchArgument("human_width_m", default_value="0.8"),
            DeclareLaunchArgument("human_height_m", default_value="1.8"),
            DeclareLaunchArgument("safety_margin_m", default_value="0.15"),
            DeclareLaunchArgument("ground_z_in_base_m", default_value="-0.35"),
            DeclareLaunchArgument("stale_timeout_sec", default_value="0.5"),
            DeclareLaunchArgument("self_filter_front_m", default_value="0.75"),
            DeclareLaunchArgument("self_filter_back_m", default_value="0.65"),
            DeclareLaunchArgument("self_filter_side_m", default_value="0.45"),
            DeclareLaunchArgument("self_filter_min_z", default_value="-0.60"),
            DeclareLaunchArgument("self_filter_max_z", default_value="0.80"),
            DeclareLaunchArgument("owner_dog_rear_x_m", default_value="-0.65"),
            DeclareLaunchArgument("owner_follow_gap_m", default_value="0.30"),
            DeclareLaunchArgument("owner_depth_m", default_value="0.35"),
            Node(
                package="mdog_dog_model",
                executable="mdog_dog_model_node",
                name="mdog_dog_model",
                output="screen",
                condition=IfCondition(use_dog_model),
                parameters=[
                    {
                        "frame_id": target_frame,
                        "body_topic": "/mdog/dog_body",
                        "front_m": ParameterValue(self_filter_front_m, value_type=float),
                        "back_m": ParameterValue(self_filter_back_m, value_type=float),
                        "side_m": ParameterValue(self_filter_side_m, value_type=float),
                        "min_z": ParameterValue(self_filter_min_z, value_type=float),
                        "max_z": ParameterValue(self_filter_max_z, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_owner_model",
                executable="mdog_owner_model_node",
                name="mdog_owner_model",
                output="screen",
                condition=IfCondition(use_owner_model),
                parameters=[
                    {
                        "frame_id": target_frame,
                        "body_topic": "/mdog/owner_body",
                        "dog_rear_x_m": ParameterValue(owner_dog_rear_x_m, value_type=float),
                        "follow_gap_m": ParameterValue(owner_follow_gap_m, value_type=float),
                        "owner_depth_m": ParameterValue(owner_depth_m, value_type=float),
                        "owner_width_m": ParameterValue(human_width_m, value_type=float),
                        "owner_height_m": ParameterValue(human_height_m, value_type=float),
                        "owner_ground_z_m": ParameterValue(ground_z_in_base_m, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_cloud_transform",
                executable="mdog_cloud_transform_node",
                name="mdog_cloud_transform",
                output="screen",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "lidar_input_topic": lidar_cloud_topic,
                        "depth_input_topic": depth_cloud_topic,
                        "lidar_output_topic": "/mdog/lidar_points_base",
                        "depth_output_topic": "/mdog/depth_points_base",
                        "roi_front_m": ParameterValue(roi_front_m, value_type=float),
                        "roi_back_m": ParameterValue(roi_back_m, value_type=float),
                        "roi_side_m": ParameterValue(roi_side_m, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_cloud_self_filter",
                executable="mdog_cloud_self_filter_node",
                name="mdog_cloud_self_filter",
                output="screen",
                parameters=[
                    {
                        "dog_body_topic": "/mdog/dog_body",
                        "lidar_input_topic": "/mdog/lidar_points_base",
                        "depth_input_topic": "/mdog/depth_points_base",
                        "lidar_output_topic": "/mdog/lidar_points_filtered",
                        "depth_output_topic": "/mdog/depth_points_filtered",
                    }
                ],
            ),
            Node(
                package="mdog_cloud_fusion",
                executable="mdog_cloud_fusion_node",
                name="mdog_cloud_fusion",
                output="screen",
                parameters=[
                    {
                        "target_frame": target_frame,
                        "lidar_input_topic": "/mdog/lidar_points_filtered",
                        "depth_input_topic": "/mdog/depth_points_filtered",
                        "output_topic": fused_points_topic,
                        "stale_timeout_sec": ParameterValue(0.75, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_height_grid_builder",
                executable="mdog_height_grid_builder_node",
                name="mdog_height_grid_builder",
                output="screen",
                parameters=[
                    {
                        "input_topic": fused_points_topic,
                        "output_topic": "/mdog/height_grid",
                        "frame_id": target_frame,
                        "resolution": ParameterValue(resolution, value_type=float),
                        "roi_front_m": ParameterValue(roi_front_m, value_type=float),
                        "roi_back_m": ParameterValue(roi_back_m, value_type=float),
                        "roi_side_m": ParameterValue(roi_side_m, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_semantic_labeler",
                executable="mdog_semantic_labeler_node",
                name="mdog_semantic_labeler",
                output="screen",
                parameters=[
                    {
                        "input_topic": "/mdog/height_grid",
                        "output_topic": local_grid_topic,
                        "human_height_m": ParameterValue(human_height_m, value_type=float),
                        "ground_z_in_base_m": ParameterValue(ground_z_in_base_m, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_occupancy_projection",
                executable="mdog_occupancy_projection_node",
                name="mdog_occupancy_projection",
                output="screen",
                parameters=[{"input_topic": local_grid_topic, "output_topic": "/mdog/local_occupancy"}],
            ),
            Node(
                package="mdog_semantic_marker_viz",
                executable="mdog_semantic_marker_viz_node",
                name="mdog_semantic_marker_viz",
                output="screen",
                parameters=[{"input_topic": local_grid_topic, "output_topic": "/mdog/semantic_markers"}],
            ),
            Node(
                package="mdog_body_marker_viz",
                executable="mdog_body_marker_viz_node",
                name="mdog_body_marker_viz",
                output="screen",
                parameters=[
                    {
                        "dog_body_topic": "/mdog/dog_body",
                        "owner_body_topic": "/mdog/owner_body",
                        "output_topic": "/mdog/body_envelopes",
                    }
                ],
            ),
            Node(
                package="mdog_traversability_evaluator",
                executable="mdog_traversability_evaluator_node",
                name="mdog_traversability_evaluator",
                output="screen",
                parameters=[
                    {
                        "local_grid_topic": local_grid_topic,
                        "output_topic": traversability_topic,
                        "human_width_m": ParameterValue(human_width_m, value_type=float),
                        "safety_margin_m": ParameterValue(safety_margin_m, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_intent_arbitrator",
                executable="mdog_intent_arbitrator_node",
                name="mdog_intent_arbitrator",
                output="screen",
                parameters=[
                    {
                        "intent_topic": intent_topic,
                        "traversability_topic": traversability_topic,
                        "decision_topic": "/mdog/nav_decision",
                        "feedback_topic": "/mdog/nav_feedback",
                        "stale_timeout_sec": ParameterValue(stale_timeout_sec, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_cmd_vel_output",
                executable="mdog_cmd_vel_output_node",
                name="mdog_cmd_vel_output",
                output="screen",
                parameters=[
                    {
                        "decision_topic": "/mdog/nav_decision",
                        "cmd_vel_topic": cmd_vel_topic,
                        "stale_timeout_sec": ParameterValue(stale_timeout_sec, value_type=float),
                    }
                ],
            ),
            Node(
                package="mdog_owner_intent_ui",
                executable="mdog_owner_intent_ui.py",
                name="mdog_owner_intent_ui",
                output="screen",
                condition=IfCondition(use_intent_ui),
                parameters=[
                    {
                        "intent_topic": intent_topic,
                        "cmd_vel_topic": cmd_vel_topic,
                        "nav_feedback_topic": "/mdog/nav_feedback",
                        "nav_decision_topic": "/mdog/nav_decision",
                        "default_speed": ParameterValue(0.20, value_type=float),
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="mdog_rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
