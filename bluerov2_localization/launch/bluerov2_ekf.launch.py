#!/usr/bin/env python3
import os
import math # Import math for PI
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace = 'bluerov2'

    # Find the package share directory for bluerov2_localization
    pkg_share = get_package_share_directory('bluerov2_localization')
    ekf_config = os.path.join(pkg_share, 'param', 'ekf.yaml')
    print(ekf_config)
    
    return LaunchDescription([

        # Static TF: ned -> robot_ned
        # Args: x y z yaw pitch roll parent_frame child_frame
        # 180 degrees = math.pi radians
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='ned_to_robot_ned_static_tf',
            namespace=namespace,
            arguments=['0.0', '0.0','0.0',str(3.14159265359*180/180), '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),

        # Depth to Odometry node
        Node(
            package='bluerov2_localization',
            executable='depth2odom',
            name='depth2odom',
            namespace=namespace,
            output='screen'
        ),
        Node(
            package='bluerov2_localization',
            executable='imu_correction',
            name='imu_correction',
            namespace=namespace,
            output='screen'
        ),
        # Dead Reckoning node
        Node(
            package='bluerov2_localization',
            executable='dead_reckoning',
            name='dead_reckoning',
            namespace=namespace,
            output='screen'
        ),

        # EKF node from robot_localization
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=namespace,
            output='screen',
            parameters=[ekf_config]
        )
    ])