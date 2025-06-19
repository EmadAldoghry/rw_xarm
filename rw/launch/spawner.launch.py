import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# Import MoveIt and UF Robot Libs
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file

def launch_setup(context, *args, **kwargs):
    # =================================================================
    # === 1. GATHER LAUNCH ARGUMENTS
    # =================================================================
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')
    rviz_config_file = LaunchConfiguration('rviz_config')
    show_rviz = LaunchConfiguration('rviz')

    # <<< FIX #1: Force dof=6 internally to prevent mismatch >>>
    # This ensures all MoveIt configs are generated for xarm6, matching your SRDF.
    dof = '6' 

    # Get other MoveIt arguments from launch
    robot_type = LaunchConfiguration('robot_type')
    prefix = LaunchConfiguration('prefix')
    hw_ns = LaunchConfiguration('hw_ns')
    limited = LaunchConfiguration('limited')
    attach_to = LaunchConfiguration('attach_to')
    attach_xyz = LaunchConfiguration('attach_xyz')
    attach_rpy = LaunchConfiguration('attach_rpy')
    add_gripper = LaunchConfiguration('add_gripper')

    # Get package paths
    pkg_rw = get_package_share_directory('rw')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # =================================================================
    # === 2. GENERATE MOVEIT & ROS2_CONTROL CONFIGS
    # =================================================================
    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem'

    ros2_control_params = generate_ros2_control_params_temp_file(
        os.path.join(pkg_rw, 'config', 'ros2_controllers.yaml'),
        prefix=prefix.perform(context),
        add_gripper=add_gripper.perform(context) in ('True', 'true'),
        robot_type=robot_type.perform(context)
    )

    # Note: Using the forced `dof` variable here
    moveit_config = (
        MoveItConfigsBuilder(
            context=context, dof=dof, robot_type=robot_type, prefix=prefix, hw_ns=hw_ns, limited=limited,
            attach_to=attach_to, attach_xyz=attach_xyz, attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin, ros2_control_params=ros2_control_params,
            add_gripper=add_gripper
        )
        .robot_description(file_path=os.path.join(pkg_rw, 'urdf', 'rwbot_with_xarm.urdf.xacro'))
        .robot_description_semantic(file_path=os.path.join(pkg_rw, 'srdf', 'rwbot_with_xarm.srdf.xacro'))
        .robot_description_kinematics(file_path=os.path.join(pkg_rw, 'config', 'kinematics.yaml'))
        .joint_limits(file_path=os.path.join(pkg_rw, 'config', 'joint_limits.yaml'))
        .trajectory_execution(file_path=os.path.join(pkg_rw, 'config', 'controllers.yaml'))
        .planning_pipelines(config_folder=os.path.join(pkg_rw, 'config'))
        .to_moveit_configs()
    )
    
    # =================================================================
    # === 3. DEFINE ALL NODES AND LAUNCH INCLUDES
    # =================================================================

    set_gazebo_path_action = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', "/home/aldoghry/gazebo_models")
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [PathJoinSubstitution([pkg_rw, 'worlds', world_name]), ' -r -v 3']}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        # <<< FIX #2 Part A: Remap the input to robot_state_publisher >>>
        # It will now listen to the clean, merged topic from joint_state_publisher.
        remappings=[('/joint_states', '/merged_joint_states')]
    )

    spawn_robot_node = Node(
        package="ros_gz_sim", executable="create",
        arguments=["-name", "rwbot_with_xarm", "-topic", "robot_description", "-x", "100.72", "-y", "5.53", "-z", "0.31", "-Y", "-2.90"],
        output="screen"
    )

    # <<< FIX #2 Part B: Configure joint_state_publisher to merge topics >>>
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time,
             # Listen to the raw joint_states topic where multiple nodes (gz plugins) publish
             'source_list': ['/joint_states']}
        ],
        # Publish the clean, merged topic
        remappings=[('/joint_states', '/merged_joint_states')]
    )

    gz_bridge_node = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        arguments=['--ros-args', '-p', f"config_file:={os.path.join(pkg_rw, 'config', 'gz_bridge.yaml')}"],
        output="screen"
    )

    ekf_node = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
        parameters=[os.path.join(pkg_rw, 'config', 'ekf.yaml'), {'use_sim_time': use_sim_time}]
    )
    
    move_group_node = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        # <<< FIX #2 Part C: Make MoveIt listen to the clean, merged topic >>>
        remappings=[('/joint_states', '/merged_joint_states')]
    )

    xarm_planner_node = Node(
        package='xarm_planner', executable='xarm_planner_node', name='xarm_planner_node', output='screen',
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}]
    )
    
    xarm_type_str = '{}{}'.format(robot_type.perform(context), dof)
    arm_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=[f'{xarm_type_str}_traj_controller', '-c', '/controller_manager'],
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_rw, 'rviz', rviz_config_file])],
        parameters=[moveit_config.to_dict(), {'use_sim_time': use_sim_time}],
        condition=IfCondition(show_rviz),
    )
    
    nav2_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_rw, 'config', 'amcl_localization.yaml'),
            'map': os.path.join(pkg_rw, 'maps', 'alpha_shape_nav2_map.yaml')
        }.items(),
    )
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(pkg_rw, 'config', 'navigation.yaml')
        }.items(),
    )

    delay_actions_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot_node,
            on_exit=[
                rviz_node,
                arm_controller_spawner,
                nav2_localization_launch,
                nav2_navigation_launch,
            ],
        )
    )

    return [
        set_gazebo_path_action,
        gazebo_launch,
        robot_state_publisher_node,
        joint_state_publisher_node, # No throttle needed, simplified logic
        spawn_robot_node,
        gz_bridge_node,
        ekf_node,
        move_group_node,
        xarm_planner_node,
        delay_actions_after_spawn,
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('world', default_value='pipeline_generated.world'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value='moveit.rviz'),
        DeclareLaunchArgument('robot_type', default_value='xarm'),
        # Note: We still declare dof=6 here for clarity, even though it's forced inside
        DeclareLaunchArgument('dof', default_value='6', description='Degrees of freedom of the arm: 5, 6, 7'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('hw_ns', default_value='xarm'),
        DeclareLaunchArgument('limited', default_value='true'),
        DeclareLaunchArgument('attach_to', default_value='base_link'),
        DeclareLaunchArgument('attach_xyz', default_value='"0.055 0.0 0.091"'),
        DeclareLaunchArgument('attach_rpy', default_value='"0 0 3.14"'),
        DeclareLaunchArgument('add_gripper', default_value='false'),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])