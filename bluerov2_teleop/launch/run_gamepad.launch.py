from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    joy_topic = LaunchConfiguration('joy_topic')
    sim = LaunchConfiguration('sim')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument('config_filepath', default_value=[
            '$(find-pkg-share bluerov2_teleop)/config/', joy_config, '.config.yaml'
        ]),
        DeclareLaunchArgument('joy_topic', default_value='joy'),
        DeclareLaunchArgument('sim', default_value='false', description='Use simulation limits if true'),

        # Group nodes under namespace
        GroupAction([
            PushRosNamespace(namespace),

            Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                output='screen',
                parameters=[{
                    'dev': joy_dev,
                    'deadzone': 0.2,
                    'autorepeat_rate': 0.0
                }]
            ),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                output='screen',
                parameters=[{
                    'require_enable_button': False,
                    'axis_linear.x': 1,
                    'axis_linear.y': 0,
                    'axis_linear.z': 4,
                    'axis_angular.yaw': 3,
                    'axis_angular.roll': 7,
                    'axis_angular.pitch': 6,
                    # Conditional scaling depending on sim flag
                    'scale_linear.x': PythonExpression(["0.5 if '", sim, "' == 'true' else 0.2"]),
                    'scale_linear.y': PythonExpression(["0.5 if '", sim, "' == 'true' else 0.2"]),
                    'scale_linear.z': PythonExpression(["1.0 if '", sim, "' == 'true' else 0.4"]),
                    'scale_angular.yaw': PythonExpression(["0.5 if '", sim, "' == 'true' else 0.2"]),
                    'scale_angular.roll': PythonExpression(["0.5 if '", sim, "' == 'true' else 0.2"]),
                    'scale_angular.pitch': PythonExpression(["0.5 if '", sim, "' == 'true' else 0.2"]),

                }]
            ),
        ])
    ])
