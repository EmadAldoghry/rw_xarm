import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription


# For the sub-launch includes
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

# uf_ros_lib utilities (as in your original code)
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_ros2_control_params_temp_file


def launch_setup(context, *args, **kwargs):
    # ----------------------------------------------------------------
    # 1) Prepare xArm MoveIt config, ros2_control params, etc.
    # ----------------------------------------------------------------
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
    ros2_control_plugin = 'gz_ros2_control/GazeboSimSystem'  # or 'gazebo_ros2_control/GazeboSystem'

    # Generate a temporary ros2_control params file
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

    # Paths to your URDF/SRDF and config YAMLs
    pkg_path = os.path.join(get_package_share_directory('rw'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'rwbot_with_xarm.urdf.xacro')
    srdf_file = os.path.join(pkg_path, 'srdf', 'rwbot_with_xarm.srdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'controllers.yaml')
    joint_limits_file = os.path.join(pkg_path, 'config', 'joint_limits.yaml')
    kinematics_file = os.path.join(pkg_path, 'config', 'kinematics.yaml')
    pipeline_filedir = os.path.join(pkg_path, 'config')

    # Build MoveIt configs
    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            dof=dof,
            robot_type=robot_type,
            prefix=prefix,
            hw_ns=hw_ns,
            limited=limited,
            attach_to=attach_to,
            attach_xyz=attach_xyz,
            attach_rpy=attach_rpy,
            ros2_control_plugin=ros2_control_plugin,
            ros2_control_params=ros2_control_params,
            add_gripper=add_gripper,
            add_vacuum_gripper=add_vacuum_gripper,
            add_bio_gripper=add_bio_gripper,
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

    # ----------------------------------------------------------------
    # 2) Include sub-launch: robot_on_rwbot_gz.launch.py
    #    This presumably starts Gazebo, spawns the robot, etc.
    # ----------------------------------------------------------------
    robot_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('rw'), 'launch', '_robot_on_rwbot_gz.launch.py'])
        ),
        launch_arguments={
            'dof': dof,
            'robot_type': robot_type,
            'prefix': prefix,
            'moveit_config_dump': moveit_config_dump,
            # If _robot_on_rwbot_gz.launch.py can handle show_rviz arg:
            'show_rviz': 'true',
            'rviz_config': PathJoinSubstitution([FindPackageShare('rw'), 'rviz', 'moveit.rviz']),
        }.items(),
    )

    # ----------------------------------------------------------------
    # 3) Include sub-launch: _robot_moveit_common2.launch.py
    #    This presumably starts MoveIt nodes (move_group, maybe RViz).
    # ----------------------------------------------------------------
    robot_moveit_common_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('xarm_moveit_config'), 'launch', '_robot_moveit_common2.launch.py'])
        ),
        launch_arguments={
            'prefix': prefix,
            'attach_to': attach_to,
            'attach_xyz': attach_xyz,
            'attach_rpy': attach_rpy,
            'show_rviz': 'false',   # or 'true' if you want that file to open RViz
            'use_sim_time': 'true',
            'moveit_config_dump': moveit_config_dump,
            'rviz_config': PathJoinSubstitution([FindPackageShare('rw'), 'rviz', 'moveit.rviz']) #moveit.rviz'
        }.items(),
    )

    # *** ADDED THE XARM PLANNER NODE ***
    # Prepare parameters for the planner node by extracting from moveit_config
    move_group_interface_params = {}
    move_group_interface_params.update(moveit_config.robot_description)
    move_group_interface_params.update(moveit_config.robot_description_semantic)
    move_group_interface_params.update(moveit_config.robot_description_kinematics)
    
    xarm_planner_node = Node(
        name='xarm_planner_node', # Standard name, can be namespaced if needed
        package='xarm_planner',
        executable='xarm_planner_node',
        output='screen',
        parameters=[
            move_group_interface_params, # Pass descriptions
            { 
                # Identify the target arm group within the mbot context
                'robot_type': robot_type, 
                'dof': dof, 
                'prefix': prefix
            },
            # Add any specific planner parameters here if needed, usually empty:
            {'use_sim_time': True}, 
        ],
    )

    # Return the sub-launches as actions
    return [
        robot_gazebo_launch,
        robot_moveit_common_launch,
        xarm_planner_node, # <-- Added the planner node
    ]


def generate_launch_description():
    # Example: we still want 'use_sim_time' as a top-level argument
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # You can keep bridging nodes, EKF, etc. here if they're NOT duplicated
    # in your sub-launch. If the sub-launch has them too, remove from here.
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

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
            "scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "imu@sensor_msgs/msg/Imu@gz.msgs.IMU",
            #"/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat",
            "/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
            "/camera2/depth_image@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera2/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked",
        ],
        output="screen",
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Relay node for camera_info if not done in sub-launch
    relay_camera_info_node = Node(
        package='topic_tools',
        executable='relay',
        name='relay_camera_info',
        output='screen',
        arguments=['camera/camera_info', 'camera/image/camera_info'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    # Put everything together
    ld = LaunchDescription()

    # Add any top-level DeclareLaunchArguments
    ld.add_action(sim_time_arg)

    # Add nodes that are NOT duplicated by the sub-launch
    ld.add_action(gz_bridge_node)
    #ld.add_action(relay_camera_info_node)
    ld.add_action(ekf_node)

    # OpaqueFunction that calls the function above to include the sub-launch files
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
