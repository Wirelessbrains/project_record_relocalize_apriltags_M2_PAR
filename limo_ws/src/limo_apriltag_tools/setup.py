import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'limo_apriltag_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ✅ marcador correto do ament index (evita warnings)
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # launch e configs
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),

        # ✅ instala um wrapper "libexec" onde o ros2 launch procura
        (os.path.join('lib', package_name), ['scripts/camera_info_publisher_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpdark',
    maintainer_email='jpdark@example.com',
    description='Pacote de calibração sincronizada e detecção AprilTag.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # continua existindo o entry point padrão (vai para bin/)
            'camera_info_publisher_node = limo_apriltag_tools.camera_info_publisher:main',
        ],
    },
)

