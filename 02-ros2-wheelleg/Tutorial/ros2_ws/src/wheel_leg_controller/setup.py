from setuptools import find_packages, setup

package_name = 'wheel_leg_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='RM Team',
    maintainer_email='rm@example.com',
    description='LQR and VMC controller for the wheel leg simulation',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'wheel_leg_controller = wheel_leg_controller.wheel_leg_controller:main',
        ],
    },
)
