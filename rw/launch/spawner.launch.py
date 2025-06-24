import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    OpaqueFunction, 
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# uf_ros_lib utilities
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

# This function now sets up EVERYTHING.
def launch_setup(context, *args, **kwargs):
    
    # =================================================================
    # 1) Prepare MoveIt, ros2_control, and Robot Description
    # =================================================================
    dof = LaunchConfiguration('dof', default=6)
    robot_type = LaunchConfiguration('robot_type', default='xarm')
    prefix = LaunchConfiguration('prefix', default='')
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')
    limited = LaunchConfiguration('limited', default=True)
    attach_to = LaunchConfiguration('attach_to', default='base_link')
    attach_xyz = LaunchConfiguration('attach_xyz', default='"0.055 0.0 0.091"')
    attach_rpy = LaunchConfiguration('attach_rpy', default='"0 0 3.14"')
    add_gripper = LaunchConfiguration('add_gripper', default=False)
    add_vacuum_gripper = LaunchConfiguration('add_vacuum_gripper', default=False)
    add_bio_gripper = LaunchConfiguration('add_bio_gripper', default=False)
    ros_namespace = LaunchConfiguration('ros_namespace', default='').perform(context)
    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('rw'), 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        add_bio_gripper=add_bio_gripper.perform(context) in ('True', 'true'),
        ros_namespace=ros_namespace,
        update_rate=1000,
        use_sim_time=True,
        robot_type=robot_type.perform(context)
    )

    pkg_path = get_package_share_directory('rw')
    urdf_file = os.path.join(pkg_path, 'model', 'rwbot_with_xarm.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'srdf', 'rwbot_with_xarm.srdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    moveit_config = (
        MoveItConfigsBuilder(
            context=context, dof=dof, robot_type=robot_type, prefix=prefix, hw_ns=hw_ns, limited=limited,
            attach_to=attach_to, attach_xyz=attach_xyz, attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin, ros2_control_params=ros2_control_params,
            add_gripper=add_gripper, add_vacuum_gripper=add_vacuum_gripper, add_bio_gripper=add_bio_gripper,
        )
        .robot_description(file_path=urdf_file)
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path=kinematics_file)
        .joint_limits(file_path=joint_limits_file)
        .trajectory_execution(file_path=controllers_file)
        .planning_pipelines(config_folder=pipeline_filedir)
        .to_moveit_configs()
    )
    moveit_config_dump = yaml.dump(moveit_config.to_dict())

    # =================================================================
    # 2) Define Supporting Nodes (Bridges, EKF, etc.)
    #    (Moved here from generate_launch_description for correct scoping)
    # =================================================================
    pkg_rw = get_package_share_directory('rw')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_rw, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    gz_bridge_params_path = os.path.join(pkg_rw, 'config', 'gz_bridge.yaml')
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=['--ros-args', '-p', f'config_file:={gz_bridge_params_path}'],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image"],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    trajectory_node = Node(
        package='trajectory_server',
        executable='trajectory_server',
        name='trajectory_server',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    trajectory_odom_topic_node = Node(
        package='trajectory_server',
        executable='trajectory_server_topic_based',
        name='trajectory_server_odom_topic',
        parameters=[
            {'trajectory_topic': 'trajectory_raw'},
            {'odometry_topic': 'odom'},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # =================================================================
    # 3) Define Nav2 Launch Actions
    #    (Moved here from generate_launch_description for correct scoping)
    # =================================================================
    nav2_localization_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'localization_launch.py')
    nav2_navigation_launch_path = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    localization_params_path = os.path.join(pkg_rw, 'config', 'amcl_localization.yaml')
    navigation_params_path = os.path.join(pkg_rw, 'config', 'navigation.yaml')
    map_file_path = os.path.join(pkg_rw, 'maps', 'alpha_shape_nav2_map.yaml')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_localization_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': localization_params_path,
            'map': map_file_path,
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_navigation_launch_path),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': navigation_params_path,
        }.items()
    )

    # =================================================================
    # 4) Define Core Robot and MoveIt Launch Actions
    # =================================================================
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('rw'), 'launch', '_robot_on_rwbot_gz.launch.py'])),
        launch_arguments={
            'dof': dof, 'robot_type': robot_type, 'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            'show_rviz': 'true',
            'rviz_config': PathJoinSubstitution([FindPackageShare('rw'), 'rviz', 'moveit.rviz']),
        }.items(),
    )

    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])),
        launch_arguments={
            'prefix': prefix, 'attach_to': attach_to, 'attach_xyz': attach_xyz, 'attach_rpy': attach_rpy,
            'show_rviz': 'false', 'use_sim_time': 'true', 'moveit_config_dump': moveit_config_dump,
            'rviz_config': PathJoinSubstitution([FindPackageShare('rw'), 'rviz', 'moveit.rviz'])
        }.items(),
    )

    move_group_interface_params = {**moveit_config.robot_description, **moveit_config.robot_description_semantic, **moveit_config.robot_description_kinematics}
    xarm_planner_node = Node(
        name='xarm_planner_node', package='xarm_planner', executable='xarm_planner_node',
        output='screen',
        parameters=[move_group_interface_params, {'robot_type': robot_type, 'dof': dof, 'prefix': prefix}, {'use_sim_time': True}],
    )

    # =================================================================
    # 5) *** CORRECTED DELAYED LAUNCH ***
    #    This now triggers Nav2 to start only after the xarm_planner_node is successfully started.
    # =================================================================
    delayed_nav2_launch = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=xarm_planner_node,
            on_start=[
                localization_launch,
                navigation_launch
            ]
        )
    )

    # Delay MoveIt and Nav2 until controllers are up (e.g., 5 seconds after Gazebo)
    delayed_moveit_and_nav2 = TimerAction(
        period=5.0,
        actions=[
            robot_moveit_common_launch,
            xarm_planner_node,
            delayed_nav2_launch
        ]
    )

    # =================================================================
    # 6) Return the complete list of actions to be launched
    # =================================================================
    return [
        # Core robot simulation and MoveIt control
        robot_gazebo_launch,
        
        # Supporting nodes for sensing and state estimation
        ekf_node,
        gz_bridge_node,
        gz_image_bridge_node,
        relay_camera_info_node,
        
        # Custom trajectory servers
        trajectory_node,
        trajectory_odom_topic_node,

        # Delayed MoveIt and Nav2 launch
        delayed_moveit_and_nav2,
    ]


def generate_launch_description():
    # This function is now clean and only declares top-level arguments
    # before calling the main setup function.
    ld = LaunchDescription()

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )
    ld.add_action(sim_time_arg)

    # OpaqueFunction calls the launch_setup function at launch time,
    # allowing it to resolve LaunchConfigurations before creating nodes.
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld