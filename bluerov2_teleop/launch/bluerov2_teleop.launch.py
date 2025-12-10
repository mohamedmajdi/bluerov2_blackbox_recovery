import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    bringup_pkg_dir = get_package_share_directory('bluerov2_bringup')
    teleop_pkg_dir = get_package_share_directory('bluerov2_teleop')

    # Declare launch arguments
    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='bluerov2', description='Robot namespace'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_pkg_dir, 'param', 'startup_param.yaml'),
        description='Path to startup parameters yaml'
    )
    sim_arg = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='Use simulation mode (true for sim, false for real robot)'
    )

    # LaunchConfigurations
    ns = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    sim = LaunchConfiguration('sim')

    # Include gamepad launch file with sim argument forwarded
    gamepad_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(teleop_pkg_dir, 'launch', 'run_gamepad.launch.py')
        ),
        launch_arguments={
            'namespace': ns,
            'sim': sim
        }.items()
    )

    # Teleop node
    teleop_node = Node(
        package='bluerov2_teleop',
        executable='bluerov2_teleop',
        name='bluerov2_teleop',
        output='screen',
        namespace=ns,
        parameters=[params_file]
    )

    # Return LaunchDescription with all nodes and includes
    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        sim_arg,
        gamepad_launch,
        teleop_node
    ])
