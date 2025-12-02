from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'bluerov2_controller'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
            (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
            (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud',
    maintainer_email='todo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': ['bluerov2_gimbal = bluerov2_controller.bluerov2_gimbal:main',
                            'bluerov2_depth_hold = bluerov2_controller.bluerov2_depth_hold:main',
                            'bluerov2_yaw_hold = bluerov2_controller.bluerov2_yaw_hold:main',
                            'bluerov2_pitch_controller = bluerov2_controller.bluerov2_pitch_controller:main',
                            'bluerov2_Vision_Controller = bluerov2_controller.bluerov2_Vision_Controller:main',
                            'bluerov2_Vision_Controller_V2 = bluerov2_controller.bluerov2_Vision_Controller_V2:main',
                           ],
    },
)
