import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

# Define a function to be called by OpaqueFunction
def launch_setup(context, *args, **kwargs):
    # --- Get Package Share Directories ---
    pkg_rw_dir = get_package_share_directory('rw')

    # ===================================================================================
    # === LAUNCH ARGUMENTS RESOLUTION (from OpaqueFunction context)
    # ===================================================================================
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    waypoints_yaml_path = LaunchConfiguration('waypoints_yaml_path').perform(context)
    correction_activation_distance = float(LaunchConfiguration('correction_activation_distance').perform(context))
    robot_base_frame = LaunchConfiguration('robot_base_frame').perform(context)
    global_frame = LaunchConfiguration('global_frame').perform(context)
    local_target_arrival_threshold = float(LaunchConfiguration('local_target_arrival_threshold').perform(context))
    local_goal_update_threshold = float(LaunchConfiguration('local_goal_update_threshold').perform(context))
    correction_wait_timeout = float(LaunchConfiguration('correction_wait_timeout').perform(context))
    min_arm_poses = int(LaunchConfiguration('min_arm_poses').perform(context))
    img_w = int(LaunchConfiguration('img_w').perform(context))
    img_h = int(LaunchConfiguration('img_h').perform(context))
    hfov = float(LaunchConfiguration('hfov').perform(context))
    camera_optical_frame = LaunchConfiguration('camera_optical_frame').perform(context)
    lidar_optical_frame = LaunchConfiguration('lidar_optical_frame').perform(context)
    input_image_topic = LaunchConfiguration('input_image_topic').perform(context)
    input_pc_topic = LaunchConfiguration('input_pc_topic').perform(context)
    roi_x_start = int(LaunchConfiguration('roi_x_start').perform(context))
    roi_y_start = int(LaunchConfiguration('roi_y_start').perform(context))
    roi_x_end = int(LaunchConfiguration('roi_x_end').perform(context))
    roi_y_end = int(LaunchConfiguration('roi_y_end').perform(context))
    black_h_min = int(LaunchConfiguration('black_h_min').perform(context))
    black_s_min = int(LaunchConfiguration('black_s_min').perform(context))
    black_v_min = int(LaunchConfiguration('black_v_min').perform(context))
    black_h_max = int(LaunchConfiguration('black_h_max').perform(context))
    black_s_max = int(LaunchConfiguration('black_s_max').perform(context))
    black_v_max = int(LaunchConfiguration('black_v_max').perform(context))
    min_contour_area = int(LaunchConfiguration('min_contour_area').perform(context))

    # ===================================================================================
    # === MoveIt and Robot Description CONFIG
    # ===================================================================================
    moveit_config = (
        MoveItConfigsBuilder(
            context=context, dof=6, robot_type="xarm",
            ros2_control_plugin="uf_robot_hardware/UFRobotFakeSystemHardware",
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    # ===================================================================================
    # === NODE DEFINITIONS
    # ===================================================================================

    # --- Core Waypoint and Mission Orchestration ---
    waypoint_server_node = Node(
        package='rw_py', executable='waypoint_server', name='waypoint_server_node', output='screen',
        parameters=[{'waypoints_yaml_path': waypoints_yaml_path}]
    )
    proximity_monitor_node = Node(
        package='rw_py', executable='proximity_monitor', name='proximity_monitor_node', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_base_frame': robot_base_frame},
            {'global_frame': global_frame},
            {'correction_activation_distance': correction_activation_distance}
        ]
    )
    waypoint_visualizer_node = Node(
        package='rw_py', executable='waypoint_visualizer', name='waypoint_visualizer_node', output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    follow_waypoints_node = Node(
        package='rw_py', executable='follow_waypoints', name='waypoint_follower_corrected_node', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'local_target_arrival_threshold': local_target_arrival_threshold},
            {'correction_wait_timeout': correction_wait_timeout},
            {'min_arm_poses': min_arm_poses}
        ]
    )

    # --- Local Correction Perception Pipeline ---
    image_segmenter_node = Node(
        package='rw_py', executable='image_segmenter_node', name='image_segmenter_node', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_image_topic': input_image_topic},
            {'img_w': img_w, 'img_h': img_h},
            {'roi_x_start': roi_x_start, 'roi_y_start': roi_y_start,
             'roi_x_end': roi_x_end, 'roi_y_end': roi_y_end},
            {'black_h_min': black_h_min, 'black_s_min': black_s_min, 'black_v_min': black_v_min,
             'black_h_max': black_h_max, 'black_s_max': black_s_max, 'black_v_max': black_v_max},
            {'min_contour_area': min_contour_area},
        ]
    )
    pointcloud_fuser_node = Node(
        package='rw_py', executable='pointcloud_fuser_node', name='pointcloud_fuser_node', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_pc_topic': input_pc_topic},
            {'img_w': img_w, 'img_h': img_h, 'hfov': hfov},
            {'camera_optical_frame': camera_optical_frame, 'lidar_optical_frame': lidar_optical_frame},
        ]
    )
    goal_calculator_node = Node(
        package='rw_py', executable='goal_calculator_node', name='goal_calculator_node', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'navigation_frame': global_frame},
        ]
    )
    fusion_visualizer_node = Node(
        package='rw_py', executable='fusion_visualizer_node', name='fusion_visualizer_node', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'raw_image_topic': input_image_topic},
            {'raw_pc_topic': input_pc_topic},
            {'img_w': img_w, 'img_h': img_h, 'hfov': hfov},
            {'camera_optical_frame': camera_optical_frame, 'lidar_optical_frame': lidar_optical_frame},
        ]
    )

    # --- Manipulation Perception & Execution Pipeline ---
    crack_path_node = Node(
        package='rw_py',
        executable='crack_path_node',
        name='crack_path_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    fake_poses_node = Node(
        package='rw_py',
        executable='fake_poses',
        name='fake_poses_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'z_offset': 0.15},
            {'min_distance_from_base': 0.25}
        ]
    )
    
    manipulation_server_node = Node(
        package='rw_py',
        executable='manipulation_action_server',
        name='manipulation_action_server',
        output='screen'
    )
    
    # *** FIX: Pass correct dof and robot_type to the planner node ***
    xarm_planner_node = Node(
        name='xarm_planner_node',
        package='xarm_planner',
        executable='xarm_planner_node',
        output='screen',
        parameters=[
            moveit_params,
            {
                'robot_type': 'xarm',
                'dof': 6,
                'use_sim_time': use_sim_time
            }
        ],
    )

    return [
        LogInfo(msg="--- Launching INTEGRATED Navigation and Manipulation System ---"),
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
        xarm_planner_node,
    ]

def generate_launch_description():
    pkg_rw_dir = get_package_share_directory('rw')
    
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('log_level', default_value='info', description='Logging level for all nodes (debug, info, warn, error, fatal)'),
        DeclareLaunchArgument('waypoints_yaml_path', default_value=os.path.join(pkg_rw_dir, 'config', 'nav_waypoints.yaml'), description='Full path to the waypoints YAML file'),
        DeclareLaunchArgument('correction_activation_distance', default_value='3.0', description='Distance to global waypoint to activate local correction'),
        DeclareLaunchArgument('robot_base_frame', default_value='base_footprint', description='Robot base frame for TF lookups'),
        DeclareLaunchArgument('global_frame', default_value='map', description='Global frame for navigation and TF'),
        DeclareLaunchArgument('local_target_arrival_threshold', default_value='0.35', description='Threshold to consider a local corrected target as reached'),
        DeclareLaunchArgument('local_goal_update_threshold', default_value='0.2', description='Minimum distance change for updating local NavToPose goal'),
        DeclareLaunchArgument('correction_wait_timeout', default_value='20.0', description='Time to wait for a valid local goal before timing out.'),
        DeclareLaunchArgument('min_arm_poses', default_value='6', description='Minimum number of arm poses detected to trigger the manipulation task'),
        DeclareLaunchArgument('img_w', default_value='1920', description='Camera image width'),
        DeclareLaunchArgument('img_h', default_value='1200', description='Camera image height'),
        DeclareLaunchArgument('hfov', default_value='1.25', description='Camera horizontal field of view (radians)'),
        DeclareLaunchArgument('camera_optical_frame', default_value='front_camera_link_optical', description='TF frame of the camera optical center'),
        DeclareLaunchArgument('lidar_optical_frame', default_value='front_lidar_link_optical', description='TF frame of the LiDAR sensor'),
        DeclareLaunchArgument('input_image_topic', default_value='/camera/image', description='Topic for the raw input image'),
        DeclareLaunchArgument('input_pc_topic', default_value='/scan_02/points', description='Topic for the raw input point cloud'),
        DeclareLaunchArgument('roi_x_start', default_value='0', description='ROI start pixel on x-axis'),
        DeclareLaunchArgument('roi_y_start', default_value='228', description='ROI start pixel on y-axis'),
        DeclareLaunchArgument('roi_x_end', default_value='1920', description='ROI end pixel on x-axis'),
        DeclareLaunchArgument('roi_y_end', default_value='912', description='ROI end pixel on y-axis'),
        DeclareLaunchArgument('black_h_min', default_value='0', description='HSV H-min for segmentation'),
        DeclareLaunchArgument('black_s_min', default_value='0', description='HSV S-min for segmentation'),
        DeclareLaunchArgument('black_v_min', default_value='0', description='HSV V-min for segmentation'),
        DeclareLaunchArgument('black_h_max', default_value='180', description='HSV H-max for segmentation'),
        DeclareLaunchArgument('black_s_max', default_value='255', description='HSV S-max for segmentation'),
        DeclareLaunchArgument('black_v_max', default_value='0', description='HSV V-max for segmentation'),
        DeclareLaunchArgument('min_contour_area', default_value='100', description='Min pixel area for contour'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])