from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    namespace = LaunchConfiguration('namespace', default='bluerov2')
    
    # Get path to parameter file
    package_share_dir = get_package_share_directory('bluerov2_controller')
    params_file = os.path.join(package_share_dir, 'param', 'visual_servoing_params.yaml')

    return LaunchDescription([
        # Launch the vision controller node with parameters
        Node(
            package='bluerov2_controller',
            executable='bluerov2_Vision_Controller_V2',
            name='bluerov2_Vision_Controller_V2',
            namespace=namespace,
            parameters=[params_file],
            output='screen' 
        ),
        
        # Launch rqt_reconfigure for dynamic parameter tuning
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'rqt_reconfigure', 'rqt_reconfigure', '/bluerov2/bluerov2_Vision_Controller_V2'
            ],
            output='screen'
        )
    ])