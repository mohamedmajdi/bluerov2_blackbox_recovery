from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
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
        description='controller configuration file'
    )

    yaw_launch = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'launch',
        'bluerov2_yaw_controller.launch.py'
    ])
    depth_launch = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'launch',
        'bluerov2_depth_controller.launch.py'
    ])
    pitch_launch = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'launch',
        'bluerov2_pitch_controller.launch.py'
    ])
    roll_launch = PathJoinSubstitution([
        FindPackageShare('bluerov2_controller'),
        'launch',
        'bluerov2_roll_controller.launch.py'
    ])

    return LaunchDescription([
        config_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yaw_launch),
            launch_arguments={'namespace': namespace, 'param': config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depth_launch),
            launch_arguments={'namespace': namespace, 'param': config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pitch_launch),
            launch_arguments={'namespace': namespace, 'param': config}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(roll_launch),
            launch_arguments={'namespace': namespace, 'param': config}.items()
        ),
        Node(
            package='bluerov2_controller',
            executable='frame_transform',
            name='frame_transform',
            namespace=namespace,
            output='screen',
            parameters=[config],
        )
    ])
