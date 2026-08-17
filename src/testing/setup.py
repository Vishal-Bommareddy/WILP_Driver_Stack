from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'testing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wilp',
    maintainer_email='wilp@todo.todo',
    description='Standalone vehicle-control stack for calibration and testing.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_control_node = testing.vehicle_control_node:main',
            'keyboard_node = testing.keyboard_node:main',
        ],
    },
)
