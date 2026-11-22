"""Build and install settings for the voice_commands_urc ROS 2 Python package."""

from setuptools import find_packages, setup

# Name of the ROS 2 package (must match package.xml and the resource/ marker file)
package_name = 'voice_commands_urc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    # Installing the marker and package.xml so ROS 2 can find this package
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LUSI',
    maintainer_email='todo@todo.todo',
    description='Voice-command behavior state machine for the LUSI rover',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # Registering the node so `ros2 run voice_commands_urc behavior_controller` works
    entry_points={
        'console_scripts': [
            'behavior_controller = voice_commands_urc.behavior_controller:main',
        ],
    },
)
