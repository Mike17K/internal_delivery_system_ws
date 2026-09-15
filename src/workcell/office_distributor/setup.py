import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'office_distributor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaipis',
    maintainer_email='kaipismike1@gmail.com',
    description='Sends each available robot to a random room from the generated office scene, one at a time',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'room_goal_distributor = office_distributor.room_goal_distributor:main',
        ],
    },
)
