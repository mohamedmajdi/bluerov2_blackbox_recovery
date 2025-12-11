from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace('bluerov2'),

        # Start detection node
        Node(
            package='bluerov2_vision',
            executable='detection',
            name='detection_node',
            output='screen'
        ),
        # Start orientation node
        Node(
            package='bluerov2_vision',
            executable='orientation',
            name='orientation_node',
            output='screen'
        ),
        # Start camera info publisher node
        Node(
            package='bluerov2_vision',
            executable='camera_info_publisher',
            name='camera_info_publisher_node',
            output='screen'
        ),
    ])
