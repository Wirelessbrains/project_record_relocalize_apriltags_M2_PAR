# ~/limo_autonomy_project_M2_PAR/limo_ws/src/limo_apriltag_tools/setup.py
from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'limo_apriltag_tools'

setup(
    name=package_name,
    version='0.0.6',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='agilex',
    author_email='agilex@example.com',
    maintainer='agilex',
    maintainer_email='agilex@example.com',
    description='Conversor YUYV->MONO8 e integração AprilTag para o LIMO.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 👇 cria o executável ROS2 nativo
            'yuyv_to_mono = limo_apriltag_tools.yuyv_to_mono_node:main',
        ],
    },
)

