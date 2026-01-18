import os
from glob import glob
from setuptools import setup

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # URDF files
        (os.path.join('share', package_name, 'urdf'), 
         glob(os.path.join('urdf', '*.urdf*'))),
        (os.path.join('share', package_name, 'urdf'), 
         glob(os.path.join('urdf', '*.xacro'))),
        # World files
        (os.path.join('share', package_name, 'worlds'), 
         glob(os.path.join('worlds', '*.sdf'))),
        # RViz config files
        (os.path.join('share', package_name, 'rviz'), 
         glob(os.path.join('rviz', '*.rviz'))),
        # Config files
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
        # Map files
        (os.path.join('share', package_name, 'maps'), 
         glob(os.path.join('maps', '*.*'))),
        # Scripts
        (os.path.join('share', package_name, 'scripts'), 
         glob(os.path.join('scripts', '*.sh'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='My differential drive robot with Nav2 and SLAM',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)