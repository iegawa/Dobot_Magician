import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'magician_lite_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='inteli',
    maintainer_email='inteli@todo.todo',
    description='ROS 2 Jazzy driver node for Dobot Magician Lite providing real-time feedback',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'feedback_node = magician_lite_driver.feedback_node:main',
        ],
    },
)
