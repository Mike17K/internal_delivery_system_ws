import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'agv_fleet_adapter'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaipis',
    maintainer_email='kaipismike1@gmail.com',
    description='Open-RMF fleet adapter for the AGV fleet, bridging directly to Nav2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fleet_adapter = agv_fleet_adapter.fleet_adapter:main',
            'submit_patrol_task = agv_fleet_adapter.submit_patrol_task:main',
        ],
    },
)
