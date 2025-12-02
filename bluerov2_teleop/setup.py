from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'bluerov2_teleop'

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
        (os.path.join('share', package_name, 'config'), glob('config/*.config.yaml')),
        (os.path.join('share', package_name, 'lib'), glob('*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mahmoud',
    maintainer_email='mahmoud.tarek.aboelrayat@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bluerov2_teleop = bluerov2_teleop.gamepad:main',
        ],
    },
)
