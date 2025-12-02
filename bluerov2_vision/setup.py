from setuptools import find_packages, setup
from glob import glob 
import os

package_name = 'bluerov2_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), ['config/detection_config.yaml']),
        (os.path.join('share', package_name, 'weights'), glob('weights/*.pt')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abin',
    maintainer_email='abinpramdya2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'detection = bluerov2_vision.detection_node:main',
            'webcam = bluerov2_vision.camera_node:main',
            'orientation = bluerov2_vision.orientation_node:main',
            'aruco = bluerov2_vision.aruco_node:main',
            'box_location = bluerov2_vision.box_location:main',
        ],
    },
)
