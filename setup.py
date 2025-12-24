from setuptools import setup
from glob import glob
import os

package_name = 'diy_ackermann'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@example.com',
    description='DIY Ackermann Robot - Starter Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_test = diy_ackermann.lidar_test:main',
            'go_to_goal = diy_ackermann.go_to_goal:main',
            'circle_driver = diy_ackermann.circle_driver:main',
            'obstacle_avoidance = diy_ackermann.obstacle_avoidance:main',
        ],
    },
)
