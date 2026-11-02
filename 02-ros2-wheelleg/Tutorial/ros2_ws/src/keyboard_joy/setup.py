from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'keyboard_joy'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
        install_requires=[
        'setuptools',
        'pynput',
    ],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='atarbabgei@gmail.com',
    description='A ROS 2 package to simulate joystick inputs with a keyboard.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_node = keyboard_joy.joy_node:main',
        ],
    },
)
