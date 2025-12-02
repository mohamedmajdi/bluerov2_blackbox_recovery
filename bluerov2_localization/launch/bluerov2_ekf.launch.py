#!/usr/bin/env python3
import os
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

        # Depth to Odometry node
        Node(
            package='bluerov2_localization',
            executable='depth2odom',
            name='depth2odom',
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
        # imu node 
        # Node(
        #     package='bluerov2_localization',
        #     executable='imu_localization',
        #     name='imu_localization',
        #     namespace=namespace,
        #     output='screen'
        # ),

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
