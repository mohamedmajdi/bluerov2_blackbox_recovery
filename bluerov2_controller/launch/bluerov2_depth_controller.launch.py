from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    # Locate the config file dynamically
    namespace = LaunchConfiguration('namespace', default='bluerov2')

    config_path = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'param',
        'controller_param.yaml'
    ])

    config = LaunchConfiguration('config')
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=config_path,
        description='controller configuration file (.yaml)'
    )
    
    return LaunchDescription([
        config_arg,
        LifecycleNode(
            package='bluerov2_controller',
            executable='bluerov2_depth_hold',
            name='depth_controller',
            namespace=namespace,
            output='screen',
            parameters=[config],
        )
    ])
