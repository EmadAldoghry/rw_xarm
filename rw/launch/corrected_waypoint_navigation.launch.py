import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Get Package Share Directories ---
    pkg_rw_dir = get_package_share_directory('rw')

    # ===================================================================================
    # === LAUNCH ARGUMENTS DEFINITION
    # ===================================================================================

    # --- Common Arguments ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='Logging level for all nodes (debug, info, warn, error, fatal)'
    )

    # --- Waypoint Server Arguments ---
    waypoints_yaml_path_arg = DeclareLaunchArgument(
        'waypoints_yaml_path',
        default_value=os.path.join(pkg_rw_dir, 'config', 'nav_waypoints.yaml'),
        description='Full path to the waypoints YAML file'
    )

    # --- Proximity Monitor Arguments ---
    correction_activation_distance_arg = DeclareLaunchArgument(
        'correction_activation_distance', default_value='3.0',
        description='Distance to global waypoint to activate local correction'
    )
    robot_base_frame_arg = DeclareLaunchArgument(
        'robot_base_frame', default_value='base_footprint',
        description='Robot base frame for TF lookups'
    )
    global_frame_arg = DeclareLaunchArgument(
        'global_frame', default_value='map',
        description='Global frame for navigation and TF'
    )

    # --- Orchestrator (follow_waypoints) Arguments ---
    local_target_arrival_threshold_arg = DeclareLaunchArgument(
        'local_target_arrival_threshold', default_value='0.35',
        description='Threshold to consider a local corrected target as reached'
    )
    local_goal_update_threshold_arg = DeclareLaunchArgument(
        'local_goal_update_threshold', default_value='0.2',
        description='Minimum distance change for updating local NavToPose goal'
    )
    correction_wait_timeout_arg = DeclareLaunchArgument(
        'correction_wait_timeout', default_value='20.0',
        description='Time to wait for a valid local goal before timing out.'
    )
    # [NEW] Argument to trigger manipulation
    min_arm_poses_arg = DeclareLaunchArgument(
        'min_arm_poses', default_value='6',
        description='Minimum number of arm poses detected to trigger the manipulation task'
    )

    # --- Sensor Fusion Pipeline Arguments (for local correction) ---
    img_w_arg = DeclareLaunchArgument('img_w', default_value='1920', description='Camera image width')
    img_h_arg = DeclareLaunchArgument('img_h', default_value='1200', description='Camera image height')
    hfov_arg = DeclareLaunchArgument('hfov', default_value='1.25', description='Camera horizontal field of view (radians)')
    camera_optical_frame_arg = DeclareLaunchArgument('camera_optical_frame', default_value='front_camera_link_optical', description='TF frame of the camera optical center')
    lidar_optical_frame_arg = DeclareLaunchArgument('lidar_optical_frame', default_value='front_lidar_link_optical', description='TF frame of the LiDAR sensor')
    input_image_topic_arg = DeclareLaunchArgument('input_image_topic', default_value='/camera/image', description='Topic for the raw input image')
    input_pc_topic_arg = DeclareLaunchArgument('input_pc_topic', default_value='/scan_02/points', description='Topic for the raw input point cloud')
    # ... other detailed perception args ...
    roi_x_start_arg = DeclareLaunchArgument('roi_x_start', default_value='0', description='ROI start pixel on x-axis')
    roi_y_start_arg = DeclareLaunchArgument('roi_y_start', default_value='228', description='ROI start pixel on y-axis')
    roi_x_end_arg = DeclareLaunchArgument('roi_x_end', default_value='1920', description='ROI end pixel on x-axis')
    roi_y_end_arg = DeclareLaunchArgument('roi_y_end', default_value='912', description='ROI end pixel on y-axis')
    black_h_min_arg = DeclareLaunchArgument('black_h_min', default_value='0', description='HSV H-min for segmentation')
    black_s_min_arg = DeclareLaunchArgument('black_s_min', default_value='0', description='HSV S-min for segmentation')
    black_v_min_arg = DeclareLaunchArgument('black_v_min', default_value='0', description='HSV V-min for segmentation')
    black_h_max_arg = DeclareLaunchArgument('black_h_max', default_value='180', description='HSV H-max for segmentation')
    black_s_max_arg = DeclareLaunchArgument('black_s_max', default_value='255', description='HSV S-max for segmentation')
    black_v_max_arg = DeclareLaunchArgument('black_v_max', default_value='0', description='HSV V-max for segmentation')
    min_contour_area_arg = DeclareLaunchArgument('min_contour_area', default_value='100', description='Min pixel area for contour')

    # ===================================================================================
    # === NODE DEFINITIONS
    # ===================================================================================

    # --- Core Waypoint and Mission Orchestration ---
    waypoint_server_node = Node(
        package='rw_py', executable='waypoint_server', name='waypoint_server_node', output='screen',
        parameters=[{'waypoints_yaml_path': LaunchConfiguration('waypoints_yaml_path')}]
    )
    proximity_monitor_node = Node(
        package='rw_py', executable='proximity_monitor', name='proximity_monitor_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_base_frame': robot_base_frame_arg.default_value},
            {'global_frame': global_frame_arg.default_value},
            {'correction_activation_distance': LaunchConfiguration('correction_activation_distance')}
        ]
    )
    waypoint_visualizer_node = Node(
        package='rw_py', executable='waypoint_visualizer', name='waypoint_visualizer_node', output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    follow_waypoints_node = Node(
        package='rw_py', executable='follow_waypoints', name='waypoint_follower_corrected_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'local_target_arrival_threshold': LaunchConfiguration('local_target_arrival_threshold')},
            {'correction_wait_timeout': LaunchConfiguration('correction_wait_timeout')},
            # [NEW] Pass the manipulation trigger parameter
            {'min_arm_poses': LaunchConfiguration('min_arm_poses')}
        ]
    )

    # --- Local Correction Perception Pipeline ---
    image_segmenter_node = Node(
        package='rw_py', executable='image_segmenter_node', name='image_segmenter_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_image_topic': LaunchConfiguration('input_image_topic')},
            {'img_w': LaunchConfiguration('img_w'), 'img_h': LaunchConfiguration('img_h')},
            {'roi_x_start': LaunchConfiguration('roi_x_start'), 'roi_y_start': LaunchConfiguration('roi_y_start'),
             'roi_x_end': LaunchConfiguration('roi_x_end'), 'roi_y_end': LaunchConfiguration('roi_y_end')},
            {'black_h_min': LaunchConfiguration('black_h_min'), 'black_s_min': LaunchConfiguration('black_s_min'), 'black_v_min': LaunchConfiguration('black_v_min'),
             'black_h_max': LaunchConfiguration('black_h_max'), 'black_s_max': LaunchConfiguration('black_s_max'), 'black_v_max': LaunchConfiguration('black_v_max')},
            {'min_contour_area': LaunchConfiguration('min_contour_area')},
        ]
    )
    pointcloud_fuser_node = Node(
        package='rw_py', executable='pointcloud_fuser_node', name='pointcloud_fuser_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'input_pc_topic': LaunchConfiguration('input_pc_topic')},
            {'img_w': LaunchConfiguration('img_w'), 'img_h': LaunchConfiguration('img_h'), 'hfov': LaunchConfiguration('hfov')},
            {'camera_optical_frame': LaunchConfiguration('camera_optical_frame'), 'lidar_optical_frame': LaunchConfiguration('lidar_optical_frame')},
        ]
    )
    goal_calculator_node = Node(
        package='rw_py', executable='goal_calculator_node', name='goal_calculator_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'navigation_frame': LaunchConfiguration('global_frame')},
        ]
    )
    fusion_visualizer_node = Node(
        package='rw_py', executable='fusion_visualizer_node', name='fusion_visualizer_node', output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'raw_image_topic': LaunchConfiguration('input_image_topic')},
            {'raw_pc_topic': LaunchConfiguration('input_pc_topic')},
            {'img_w': LaunchConfiguration('img_w'), 'img_h': LaunchConfiguration('img_h'), 'hfov': LaunchConfiguration('hfov')},
            {'camera_optical_frame': LaunchConfiguration('camera_optical_frame'), 'lidar_optical_frame': LaunchConfiguration('lidar_optical_frame')},
        ]
    )

    # --- Manipulation Perception & Execution Pipeline ---
    crack_path_node = Node(
        package='rw_py',
        executable='crack_path_node',
        name='crack_path_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    fake_poses_node = Node(
        package='rw_py',
        executable='fake_poses',
        name='fake_poses_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'z_offset': 0.15},
            {'min_distance_from_base': 0.25}
        ]
    )
    
    # [NEW] Launch the Manipulation Action Server
    manipulation_server_node = Node(
        package='rw_py',
        executable='manipulation_action_server',
        name='manipulation_action_server',
        output='screen'
    )

    # ===================================================================================
    # === LAUNCH DESCRIPTION ASSEMBLY
    # ===================================================================================
    return LaunchDescription([
        # --- Add all launch arguments ---
        use_sim_time_arg, log_level_arg,
        waypoints_yaml_path_arg,
        correction_activation_distance_arg, robot_base_frame_arg, global_frame_arg,
        local_target_arrival_threshold_arg, local_goal_update_threshold_arg,
        correction_wait_timeout_arg, min_arm_poses_arg,
        img_w_arg, img_h_arg, hfov_arg, camera_optical_frame_arg, lidar_optical_frame_arg,
        input_image_topic_arg, input_pc_topic_arg,
        roi_x_start_arg, roi_y_start_arg, roi_x_end_arg, roi_y_end_arg,
        black_h_min_arg, black_s_min_arg, black_v_min_arg,
        black_h_max_arg, black_s_max_arg, black_v_max_arg,
        min_contour_area_arg,

        LogInfo(msg="--- Launching INTEGRATED Navigation and Manipulation System ---"),

        # --- Add all the nodes to be launched ---
        waypoint_server_node,
        proximity_monitor_node,
        waypoint_visualizer_node,
        follow_waypoints_node,
        
        image_segmenter_node,
        pointcloud_fuser_node,
        goal_calculator_node,
        fusion_visualizer_node,
        
        crack_path_node,
        fake_poses_node,
        manipulation_server_node,
    ])