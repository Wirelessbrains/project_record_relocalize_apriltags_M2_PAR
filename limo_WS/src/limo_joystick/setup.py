import os # <-- ADD THIS IMPORT
from glob import glob # <-- ADD THIS IMPORT
from setuptools import find_packages, setup

package_name = 'limo_joystick'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 🟢 LINHA ESSENCIAL: Adicionada para instalar o conteúdo da pasta 'launch'
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yann',
    maintainer_email='yannkelvem40@gmail.com',
    description='Package for launching LIMO simulation with joystick teleoperation.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'mode_controller_node = limo_joystick.mode_controller_node:main',
        ],
    },
)
