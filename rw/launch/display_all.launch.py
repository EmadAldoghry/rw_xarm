#!/usr/bin/env python3

"""
Launch file that:
1) Starts depthai_ros_driver with left, right, and color cameras plus an IMU.
2) (Optionally) rectifies all three cameras (left, right, color).
3) Publishes a colored point cloud using depth_image_proc.
4) Launches a simple EKF node to convert IMU data into an odometry transform (odom->base_link).
5) Launches RViz to display:
   - The three camera images (left, right, color),
   - The point cloud,
   - The camera moving in real time based on IMU data.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    """
    Opaque function that orchestrates:
      - The OAK-D camera(s) via depthai_ros_driver (with IMU enabled).
      - (Optional) rectification of left, right, and color images.
      - Depth + color to a colored point cloud.
      - A simple EKF to produce odom->base_link from IMU.
      - RViz visualization of images, point cloud, and camera motion.
    """

    depthai_prefix = get_package_share_directory("depthai_ros_driver")

    # Retrieve LaunchConfiguration parameters
    params_file   = LaunchConfiguration("params_file").perform(context)
    name          = LaunchConfiguration("name").perform(context)
    parent_frame  = LaunchConfiguration("parent_frame").perform(context)
    use_rviz      = LaunchConfiguration("use_rviz").perform(context)
    rviz_config   = LaunchConfiguration("rviz_config").perform(context)
    cam_pos_x     = LaunchConfiguration("cam_pos_x").perform(context)
    cam_pos_y     = LaunchConfiguration("cam_pos_y").perform(context)
    cam_pos_z     = LaunchConfiguration("cam_pos_z").perform(context)
    cam_roll      = LaunchConfiguration("cam_roll").perform(context)
    cam_pitch     = LaunchConfiguration("cam_pitch").perform(context)
    cam_yaw       = LaunchConfiguration("cam_yaw").perform(context)

    actions = []

    #
    # 1) Include depthai_ros_driver camera launch (with IMU support).
    #    Ensure your params_file sets 'pipeline_gen.i_enable_imu: true'
    #    and publishes the three image topics plus /<name>/imu.
    #
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": name,
                "params_file": params_file,
                "parent_frame": parent_frame,
                "cam_pos_x": cam_pos_x,
                "cam_pos_y": cam_pos_y,
                "cam_pos_z": cam_pos_z,
                "cam_roll": cam_roll,
                "cam_pitch": cam_pitch,
                "cam_yaw": cam_yaw,
                # We manage RViz ourselves, so set false here
                "use_rviz": "false",
            }.items(),
        )
    )

    #
    # 2) (Optional) Rectify the three camera images if needed (comment/uncomment as desired).
    #    If depthai_ros_driver is already outputting rectified streams,
    #    you can skip this block.
    #
    actions.append(
        LoadComposableNodes(
            target_container=f"{name}_container",
            composable_node_descriptions=[
                # Rectify LEFT
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_left",
                    remappings=[
                        ("image",       f"{name}/left/image_raw"),
                        ("camera_info", f"{name}/left/camera_info"),
                        ("image_rect",  f"{name}/left/image_rect"),
                    ],
                ),
                # Rectify RIGHT
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_right",
                    remappings=[
                        ("image",       f"{name}/right/image_raw"),
                        ("camera_info", f"{name}/right/camera_info"),
                        ("image_rect",  f"{name}/right/image_rect"),
                    ],
                ),
                # Rectify COLOR
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color",
                    remappings=[
                        ("image",       f"{name}/rgb/image_raw"),
                        ("camera_info", f"{name}/rgb/camera_info"),
                        ("image_rect",  f"{name}/rgb/image_rect"),
                    ],
                ),
            ],
        )
    )

    #
    # 3) Publish a colored point cloud using depth_image_proc.
    #    Adjust the input topics if you need rectified depth or color.
    #
    actions.append(
        LoadComposableNodes(
            target_container=f"{name}_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzrgbNode",
                    name="point_cloud_xyzrgb_node",
                    remappings=[
                        # Depth image (e.g. "stereo/image_raw" or "stereo/image_rect")
                        ("depth_registered/image_rect", f"{name}/stereo/image_raw"),
                        # Color image
                        ("rgb/image_rect_color",        f"{name}/rgb/image_raw"),
                        ("rgb/camera_info",             f"{name}/rgb/camera_info"),
                        # Output topic for the point cloud
                        ("points",                      f"{name}/points"),
                    ],
                ),
            ],
        )
    )

    #
    # 4) Minimal robot_localization EKF to fuse IMU data → odom->base_link
    #
    actions.append(
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[{
                'frequency': 30.0,
                'sensor_timeout': 0.5,
                'two_d_mode': True,  # Set to true if you only want 2D motion
                'publish_tf': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': parent_frame,
                'world_frame': 'odom',
                'imu0': '/oak/imu/data_fixed',
                'imu0_config': [
                                False, False, False,   # x, y, z position
                                True,  True,  False,   # roll, pitch, yaw orientation
                                False, False, False,   # x, y, z velocity
                                True,  True,  False,   # roll, pitch, yaw rates (disable problematic axis)
                                False, False, False    # x, y, z acceleration (disable if causing drift)
                                ],
                'imu0_remove_gravitational_acceleration': True,
                'imu0_differential': False,
                'imu0_relative': False,
                'print_diagnostics': True,

                # Add process noise covariance (tune these values)
                'process_noise_covariance': [
                    0.01, 0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0.01, 0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0.01, 0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0.01, 0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0.03, 0,    0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0.06, 0,     0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0.025, 0,     0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0.025, 0,    0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0.04, 0,    0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0.01, 0,    0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0.01, 0,    0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0.02, 0,    0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0.01, 0,    0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0.01, 0,
                    0,    0,    0,    0,    0,    0,    0,     0,     0,    0,    0,    0,    0,    0,    0.015
                ]
            }]
        )
    )

    actions.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        )
    )

    #
    # 5) Launch RViz to display:
    #    - The three camera images (/oak/left/image_raw, /oak/right/image_raw, /oak/rgb/image_raw)
    #    - The point cloud (/oak/points)
    #    - The odom->base_link TF from the EKF
    #
    if use_rviz.lower() == "true":
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    return actions


def generate_launch_description():
    """
    Main entry point for:
      - OAK-D cameras (with IMU),
      - Optional rectification,
      - Colored point cloud,
      - EKF-based odometry (odom->base_link),
      - RViz.
    """

    depthai_prefix = get_package_share_directory("rw")

    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak", description="Camera node name."),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="0.0"),
        DeclareLaunchArgument("cam_roll",  default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw",   default_value="0.0"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(depthai_prefix, "config", "camera.yaml"),
            description="Driver parameters (ensure i_enable_imu=true, etc.)."
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Enable RViz window."
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "rviz", "rgbd.rviz"),
            description="Path to an RViz config that displays left/right/color images + point cloud + TF."
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])