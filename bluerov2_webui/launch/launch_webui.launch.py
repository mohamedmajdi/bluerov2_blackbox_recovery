from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace('bluerov2'),

        # Start classic detection node
        Node(
            package='bluerov2_webui',
            executable='webui',
            name='webui_node',
            output='screen'
        ),
    ])