import os
from glob import glob
from setuptools import setup

package_name = 'limo_apriltag_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        (os.path.join('lib', package_name), ['scripts/camera_info_publisher_node']),
        (os.path.join('lib', package_name), ['scripts/camera_info_relay_node']),
        (os.path.join('lib', package_name), ['scripts/udp_camera_receiver']),
        (os.path.join('lib', package_name), ['scripts/tag_overlay_node']),
        (os.path.join('lib', package_name), ['scripts/ippe_localizer']),
        (os.path.join('lib', package_name), ['scripts/static_map_publisher']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wireless Brains',
    maintainer_email='contact@wirelessbrains.dev',
    description='AprilTag tools for camera calibration relay, UDP camera input, and overlay visualization.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_info_publisher_node = limo_apriltag_tools.camera_info_publisher:main',
            'camera_info_relay_node = limo_apriltag_tools.camera_info_relay_node:main',
            'udp_camera_receiver_node = limo_apriltag_tools.udp_camera_receiver_node:main',
            'tag_overlay_node = limo_apriltag_tools.tag_overlay_node:main',
            'ippe_localizer = limo_apriltag_tools.ippe_localizer_node:main',
            'static_map_publisher = limo_apriltag_tools.static_map_publisher_node:main',
            'build_tag_map_offline = limo_apriltag_tools.build_tag_map_offline_entry:main',
            'analyze_distance_to_trajectory = limo_apriltag_tools.analyze_distance_angle_entry:main',
        ],
    },
)
