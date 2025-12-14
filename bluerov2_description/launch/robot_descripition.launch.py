import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    use_tank = LaunchConfiguration('use_tank')
    use_camera_tf = LaunchConfiguration('use_camera_tf')
    name_space = LaunchConfiguration('name_space')

    pkg_path = get_package_share_directory('bluerov2_description')
    rviz_config = os.path.join(pkg_path, 'rviz', 'bluerov_cirtesu.rviz')
    urdf_path = os.path.join(pkg_path,'urdf')
    # Definir las descripciones de los robots
    description_file_cirtesu = os.path.join(
        urdf_path, 
        'cirtesu', 
        'cirtesu.urdf.xacro'
    )
    robot_description_cirtesu = os.popen(f'xacro {description_file_cirtesu}').read().strip()
    description_file_bluerov2 = os.path.join(
        urdf_path, 
        'bluerov2_heavy', 
        'bluerov2.xacro'
    )
    robot_description_bluerov2 = os.popen(f'xacro {description_file_bluerov2}').read().strip()

    description_file_blackbox = os.path.join(
        urdf_path, 
        'blackbox', 
        'blackbox.urdf.xacro'
    )
    robot_description_blackbox = os.popen(f'xacro {description_file_blackbox}').read().strip()


    robot_state_publisher_node_bluerov2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        remappings=[('/robot_description', 'robot_description')],
        parameters=[
            {'robot_description': robot_description_bluerov2}
        ]
    )
    robot_state_publisher_node_blackbox = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher',
        remappings=[('/robot_description', '/blackbox/robot_description')],
        parameters=[
            {'robot_description': robot_description_blackbox}
        ]
    )
    robot_state_publisher_node_cirtesu = Node(
        condition=IfCondition(use_tank),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        name='robot_state_publisher_cirtesu',
        remappings=[('/robot_description', '/cirtesu/robot_description')],
        parameters=[
            {'robot_description': robot_description_cirtesu}
        ]
    )
    
    cirtesu_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "6.0",
                "--y", "4.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "3.1416",
                "--yaw", "3.14159",
                "--frame-id", "ned",
                "--child-frame-id", "cirtesu_tank"
            ]
        )
    world_ned_odom_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0.0",
                "--y", "0.0",
                "--z", "0.0",
                "--roll", "0.0",
                "--pitch", "0",
                "--yaw", "0.0",
                "--frame-id", "ned",
                "--child-frame-id", "odom"
            ]
        )
    
    # camera_robot_static_tf = Node(
    #     condition=IfCondition(use_camera_tf),
    #     package='tf2_ros',
    #         executable='static_transform_publisher',
    #         output='screen',
    #         arguments=[
    #             "--x", "0.0",
    #             "--y", "0.0.",
    #             "--z", "-0.2",
    #             "--roll", "-1.57",
    #             "--pitch", "-1.57",
    #             "--yaw", "0.0",
    #             "--frame-id", "camera",
    #             "--child-frame-id", "base_link"]
    # )

    camera_robot_static_tf = Node(
        condition=IfCondition(use_camera_tf),
        package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                "--x", "0.0",
                "--y", "0.1147",
                "--z", "-0.1638",
                "--roll", "-0.0",
                "--pitch", "-0.9599",
                "--yaw", "-1.5708",
                "--frame-id", "camera",
                "--child-frame-id", "base_link"]
    )

    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )

    return LaunchDescription([
        # Declarar el argumento 'use_depth' con valor por defecto 'true'
        DeclareLaunchArgument(
            'use_tank',
            default_value='true',
            description='Flag to indicate whether to publish the cirtesu tank or not'
        ),
        DeclareLaunchArgument(
            'use_camera_tf',
            default_value='true',
            description='Flag to indicate whether to publish static tf between the camera and the robot'
        ),
        robot_state_publisher_node_bluerov2,
        robot_state_publisher_node_cirtesu,
        cirtesu_static_tf,
        world_ned_odom_static_tf,
        camera_robot_static_tf,
        rviz_node,
        robot_state_publisher_node_blackbox
    ])

