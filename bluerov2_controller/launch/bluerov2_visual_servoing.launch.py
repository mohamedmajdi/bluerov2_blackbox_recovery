from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='bluerov2')

    # Use NPZ not YAML
    calib_path = PathJoinSubstitution([
        FindPackageShare('bluerov2_localization'),
        'param',
        'camera_calb_20_11.npz'
    ])

    calib_arg = DeclareLaunchArgument(
        'calib_file',
        default_value=calib_path,
        description='Camera calibration file (.npz)'
    )
    
    return LaunchDescription([
        calib_arg,
        LifecycleNode(
            package='bluerov2_controller',
            executable='visual_servoing',
            name='visual_servoing',
            namespace=namespace,
            output='screen',
            parameters=[{
                'calib_file': LaunchConfiguration('calib_file')
            }],
        )
    ])
