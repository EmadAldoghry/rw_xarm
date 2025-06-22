# FILE: rw/launch/corrected_waypoint_navigation.launch.py
# FINAL and CORRECT Version

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_rw_dir = get_package_share_directory('rw')

    # ===================================================================================
    # === LAUNCH ARGUMENTS DEFINITION
    # ===================================================================================

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Logging level')
    waypoints_yaml_path_arg = DeclareLaunchArgument('waypoints_yaml_path', default_value=os.path.join(pkg_rw_dir, 'config', 'nav_waypoints.yaml'), description='Path to waypoints YAML file')
    correction_activation_distance_arg = DeclareLaunchArgument('correction_activation_distance', default_value='3.0', description='Distance to activate local correction')
    robot_base_frame_arg = DeclareLaunchArgument('robot_base_frame', default_value='base_footprint', description='Robot base frame for TF')
    global_frame_arg = DeclareLaunchArgument('global_frame', default_value='map', description='Global frame for navigation')
    local_target_arrival_threshold_arg = DeclareLaunchArgument('local_target_arrival_threshold', default_value='0.35', description='Threshold to consider local target reached')
    local_goal_update_threshold_arg = DeclareLaunchArgument('local_goal_update_threshold', default_value='0.2', description='Minimum distance change for updating local goal')
    correction_wait_timeout_arg = DeclareLaunchArgument('correction_wait_timeout', default_value='20.0', description='Timeout for waiting for local goal')
    img_w_arg = DeclareLaunchArgument('img_w', default_value='1920', description='Camera image width')
    img_h_arg = DeclareLaunchArgument('img_h', default_value='1200', description='Camera image height')
    hfov_arg = DeclareLaunchArgument('hfov', default_value='1.25', description='Camera horizontal FOV')
    camera_optical_frame_arg = DeclareLaunchArgument('camera_optical_frame', default_value='front_camera_link_optical')
    lidar_optical_frame_arg = DeclareLaunchArgument('lidar_optical_frame', default_value='front_lidar_link_optical')
    segmentation_node_name_arg = DeclareLaunchArgument('segmentation_node_name', default_value='goal_calculator_node')
    corrected_local_goal_topic_arg = DeclareLaunchArgument('corrected_local_goal_topic', default_value='/corrected_local_goal')
    input_image_topic_arg = DeclareLaunchArgument('input_image_topic', default_value='/camera/image')
    input_pc_topic_arg = DeclareLaunchArgument('input_pc_topic', default_value='/scan_02/points')

    # ===================================================================================
    # === NODE DEFINITIONS
    # ===================================================================================

    waypoint_server_node = Node(package='rw_py', executable='waypoint_server', name='waypoint_server_node', output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'waypoints_yaml_path': LaunchConfiguration('waypoints_yaml_path')}])
    proximity_monitor_node = Node(package='rw_py', executable='proximity_monitor', name='proximity_monitor_node', output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'robot_base_frame': LaunchConfiguration('robot_base_frame')}, {'global_frame': LaunchConfiguration('global_frame')}, {'correction_activation_distance': LaunchConfiguration('correction_activation_distance')}])
    waypoint_visualizer_node = Node(package='rw_py', executable='waypoint_visualizer', name='waypoint_visualizer_node', output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}])
    image_segmenter_node = Node(package='rw_py', executable='image_segmenter_node', name='image_segmenter_node', output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'input_image_topic': LaunchConfiguration('input_image_topic')}, {'output_mask_topic': '/target_mask'}, {'img_w': LaunchConfiguration('img_w')}, {'img_h': LaunchConfiguration('img_h')}])
    pointcloud_fuser_node = Node(package='rw_py', executable='pointcloud_fuser_node', name='pointcloud_fuser_node', output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'input_pc_topic': LaunchConfiguration('input_pc_topic')}, {'input_mask_topic': '/target_mask'}, {'output_pc_topic': '/target_points'}, {'img_w': LaunchConfiguration('img_w')}, {'img_h': LaunchConfiguration('img_h')}, {'hfov': LaunchConfiguration('hfov')}, {'camera_optical_frame': LaunchConfiguration('camera_optical_frame')}, {'lidar_optical_frame': LaunchConfiguration('lidar_optical_frame')}])
    goal_calculator_node = Node(package='rw_py', executable='goal_calculator_node', name=LaunchConfiguration('segmentation_node_name'), output='screen', parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'input_pc_topic': '/target_points'}, {'output_corrected_goal_topic': LaunchConfiguration('corrected_local_goal_topic')}, {'navigation_frame': LaunchConfiguration('global_frame')}])
    
    waypoint_follower_corrected_node = Node(
        package='rw_py', executable='follow_waypoints', name='waypoint_follower_corrected_node', output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'local_target_arrival_threshold': LaunchConfiguration('local_target_arrival_threshold')},
            {'local_goal_update_threshold': LaunchConfiguration('local_goal_update_threshold')},
            {'correction_wait_timeout': LaunchConfiguration('correction_wait_timeout')},
            {'segmentation_node_name': LaunchConfiguration('segmentation_node_name')},
        ]
    )
    
    fusion_visualizer_node = Node(
        package='rw_py', executable='fusion_visualizer_node', name='fusion_visualizer_node', output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'output_window': 'Fused View'},
            {'raw_image_topic': LaunchConfiguration('input_image_topic')},
            {'mask_topic': '/target_mask'},
            {'raw_pc_topic': LaunchConfiguration('input_pc_topic')},
            {'target_pc_topic': '/target_points'},
            {'img_w': LaunchConfiguration('img_w')},
            {'img_h': LaunchConfiguration('img_h')},
            {'hfov': LaunchConfiguration('hfov')},
            {'camera_optical_frame': LaunchConfiguration('camera_optical_frame')},
            {'lidar_optical_frame': LaunchConfiguration('lidar_optical_frame')},
        ]
    )

    crack_path_node = Node(
        package='rw_py', executable='crack_path_node', name='crack_path_node', output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_cloud_topic': '/camera2/points'},
            {'output_cloud_topic': '/projected_non_ground_points'},
            {'target_frame': 'base_link'},
        ]
    )

    fake_poses_node = Node(
        package='rw_py', executable='fake_poses', name='fake_poses_node', output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_topic': '/projected_non_ground_points'},
            {'output_topic': '/arm_path_poses'},
        ]
    )

    manipulation_server_node = Node(
        package='rw_py', executable='manipulation_action_server', name='manipulation_action_server', output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # ===================================================================================
    # === LAUNCH DESCRIPTION ASSEMBLY
    # ===================================================================================
    return LaunchDescription([
        use_sim_time_arg, log_level_arg, waypoints_yaml_path_arg, correction_activation_distance_arg,
        robot_base_frame_arg, global_frame_arg, local_target_arrival_threshold_arg,
        local_goal_update_threshold_arg, correction_wait_timeout_arg, img_w_arg, img_h_arg,
        hfov_arg, camera_optical_frame_arg, lidar_optical_frame_arg, segmentation_node_name_arg,
        corrected_local_goal_topic_arg, input_image_topic_arg, input_pc_topic_arg,

        LogInfo(msg="--- Launching INTEGRATED Navigation and Manipulation System ---"),

        waypoint_server_node,
        proximity_monitor_node,
        waypoint_visualizer_node,
        image_segmenter_node,
        pointcloud_fuser_node,
        goal_calculator_node,
        fusion_visualizer_node,
        crack_path_node,
        fake_poses_node,
        manipulation_server_node,
        waypoint_follower_corrected_node,
    ])