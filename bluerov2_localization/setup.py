from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluerov2_localization'

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
        (os.path.join('share', package_name, 'param'), glob('param/*.npz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud',
    maintainer_email='todo@todo.todo',
    description='BlueROV2 localization package',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'imu_localization = bluerov2_localization.imu_localization:main',
            'aruco_detector = bluerov2_localization.aruco_detector:main',
            'aruco_tf = bluerov2_localization.aruco_tf:main',
            'aruco_localization = bluerov2_localization.aruco_localization:main',
            'visual_odom = bluerov2_localization.visual_odom:main',
            'box_localization = bluerov2_localization.box_localization:main',
            'depth2odom = bluerov2_localization.depth2odom:main',
            'dead_reckoning = bluerov2_localization.dead_reckoning:main',
        ],
    },
)
