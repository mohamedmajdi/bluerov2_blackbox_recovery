import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # ---------------------------------------------------------
    # CONFIGURATION
    # ---------------------------------------------------------
    # This transform defines where "imu_link" is relative to "base_link".
    # Since your IMU is facing backwards, we apply a 180 degree (3.14159 rad) Yaw rotation.
    
    # Arguments format: x y z yaw pitch roll parent_frame child_frame
    # TF_ARGS = ['0', '0', '0', '3.14159', '0', '0', 'base_link', 'imu_link']
    # ---------------------------------------------------------
    ns = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Robot namespace'
    )
    return LaunchDescription([
        namespace_arg,

        # # 1. STATIC TRANSFORM PUBLISHER
        # # This tells ROS: "imu_link is just base_link rotated by 180 degrees"
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='imu_tf_broadcaster',
        #     arguments=TF_ARGS
        # ),

        # 2. IMU REPUBLISHER NODE
        # This runs the script that renames the frame_id in the message
        Node(
            package='bluerov2_localization', # REPLACE with your actual package name
            namespace=ns,
            executable='imu_correction', 
            name='imu_correction',
            output='screen',
        )
    ])