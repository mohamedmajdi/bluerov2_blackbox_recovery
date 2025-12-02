import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    ns = LaunchConfiguration('namespace')
    use_camera_tf = LaunchConfiguration('camera_tf')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Robot namespace'
    )
    camera_tf_arg = DeclareLaunchArgument(
        'camera_tf',
        default_value='true',
        description='Use defult tf between camera and base_link'
    )

    pkg_path = get_package_share_directory('bluerov2_localization')
    param_file_path = os.path.join(pkg_path, 'param', 'camera_calb_20_11.npz')

    calb = LaunchConfiguration('calb')
    calb_arg = DeclareLaunchArgument(
        'calb',
        default_value=param_file_path,
        description='camera calibration file (.npz)'
    )

    description_pkg = get_package_share_directory('bluerov2_description')
    description_launch = os.path.join(description_pkg,'launch','robot_descripition.launch.py')
    return LaunchDescription([
        namespace_arg,
        calb_arg,
        camera_tf_arg,
        Node(
            package='bluerov2_localization',
            executable='aruco_detector',
            name='aruco_detector',
            namespace=ns,
            output='screen',
            parameters=[{'calibration_file':calb}],
        ),

        Node(
            package='bluerov2_localization',
            executable='aruco_tf',
            name='aruco_tf',
            namespace=ns,
            output='screen',
        ),
        Node(
            package='bluerov2_localization',
            executable='aruco_localization',
            name='aruco_localization',
            namespace=ns,
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={'use_tank': 'true','use_camera_tf' : use_camera_tf}.items()
        ),
        
    ])