from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'bluerov2_search'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
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
        'console_scripts': ['bluerov2_search_node = bluerov2_search.bluerov2_search_node:main',
                            'bluerov2_approaching_node = bluerov2_search.bluerov2_approaching_node:main'
                           ],
    },
)
