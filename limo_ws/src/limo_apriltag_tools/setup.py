import os
from glob import glob
from setuptools import setup

package_name = 'limo_apriltag_tools'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name], # Mais limpo que find_packages()
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
        
        # --- AQUI ESTÁ A LÓGICA CORRETA ---
        # 1. A linha original que instala o wrapper BASH da câmera
        (os.path.join('lib', package_name), ['scripts/camera_info_publisher_node']),
        
        # 2. A nova linha que instala o wrapper BASH da tag
        (os.path.join('lib', package_name), ['scripts/tag_latcher']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jpdark',
    maintainer_email='jpdark@example.com',
    description='Pacote de calibração sincronizada e detecção AprilTag.',
    license='Apache-2.0',
    tests_require=['pytest'],
    
    # --- IMPORTANTE ---
    # Os entry_points SÃO necessários. 
    # Eles criam os executáveis em 'bin/' que os wrappers BASH chamam.
    entry_points={
        'console_scripts': [
            'camera_info_publisher_node = limo_apriltag_tools.camera_info_publisher:main',
            'tag_latcher = limo_apriltag_tools.tag_latcher_node:main',
        ],
    },
)
