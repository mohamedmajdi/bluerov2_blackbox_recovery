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
    
    pkg_path = get_package_share_directory('bluerov2_bringup')
    teleop_pkg = get_package_share_directory('bluerov2_teleop')
    controller_pkg = get_package_share_directory('bluerov2_controller')
    localization_pkg = get_package_share_directory('bluerov2_localization')
    searching_pkg = get_package_share_directory('bluerov2_search')
    bringup_launch_dir = os.path.join(pkg_path, 'launch')
    param_file_path = os.path.join(pkg_path, 'param', 'startup_param.yaml')
    searching_param_file_path = os.path.join(searching_pkg, 'param', 'search.yaml')
    camera_param_file_path = os.path.join(localization_pkg, 'param', 'camera_calb_20_11.npz')
    with_camera_arg = DeclareLaunchArgument(
        'with_camera',
        default_value='true',
        description='Whether to launch the camera node'
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='bluerov2',
        description='Robot namespace'
    )
    init_arg = DeclareLaunchArgument(
        'initialize',
        default_value='false',
        description='Whether to run initialization test'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=param_file_path,
        description='Path to startup parameters yaml'
    )

    params_file = LaunchConfiguration('params_file')
    initialize = LaunchConfiguration('initialize')

    video_node = Node(
        package='bluerov2_bringup',
        executable='video',
        name='video',
        output='screen',
        namespace=ns,
    )

    startup_node = Node(
        package='bluerov2_bringup',
        executable='startup',
        name='startup',
        output='screen',
        namespace=ns,
        parameters=[
            ParameterFile(params_file),
            {'initialize': initialize}   
        ]
    )

    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch_dir, 'run_mavros.launch.py')
        ),
        launch_arguments={'namespace': ns}.items()
    )

    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg, 'launch', 'bluerov2_teleop.launch.py')
        ),
        launch_arguments={'namespace': ns}.items()
    )

    gimbal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'bluerov2_gimbal.launch.py')
        ),
        launch_arguments={'namespace': ns}.items()
    )

    controllers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'bluerov2_controller.launch.py')
        ),
        launch_arguments={'namespace': ns}.items()
    )
    vs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_pkg, 'launch', 'bluerov2_visual_servoing.launch.py')
        ),
        launch_arguments={'namespace': ns,'calib_file':camera_param_file_path}.items()
    )
    searching_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(searching_pkg, 'launch', 'bluerov2_search_pattern.launch.py')
        ),
        launch_arguments={'namespace': ns, 'config':searching_param_file_path}.items()
    )

    approaching_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(searching_pkg, 'launch', 'bluerov2_approaching.launch.py')
        ),
        launch_arguments={'namespace': ns,'config':searching_param_file_path}.items()
    )
    return LaunchDescription([
        namespace_arg,
        with_camera_arg,
        init_arg,
        params_file_arg,
        video_node,
        startup_node,
        controllers_launch,
        searching_launch,
        approaching_launch,
        vs_launch,
        # teleop_launch,
        # mavros_launch,
        # gimbal_launch
    ])
