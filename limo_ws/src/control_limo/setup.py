import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_limo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yann',
    maintainer_email='yannkelvem40@gmail.com',
    description='Control nodes and launch files for LIMO parking and goal-to-pose navigation.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'go_to_pose = control_limo.go_to_pose:main',
            'parking_spot_navigator = control_limo.parking_spot_navigator_node:main',
            'parking_spot_indicator_node = control_limo.parking_spot_indicator_node:main',
            'parking_spot_randomizer_node = control_limo.parking_spot_randomizer_node:main',
        ],
    },
)
