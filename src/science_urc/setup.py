from setuptools import find_packages, setup

# Name of our Science Module ROS 2 package
package_name = 'science_urc'

setup(
    name=package_name,
    version='0.0.0',

    # Find and include the Python modules in this package
    packages=find_packages(exclude=['test']),

    # Install the files ROS 2 needs to recognize the package
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    # Basic Python package settings
    install_requires=['setuptools'],
    zip_safe=True,

    # Package information
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='ROS 2 integration for the LUSI Science Module',
    license='TODO: License declaration',

    # Dependencies used when running tests
    extras_require={
        'test': [
            'pytest',
        ],
    },

    # Register our ROS 2 nodes so they can be launched with ros2 run
    entry_points={
        'console_scripts': [
            'science_node = science_urc.science_node:main',
            'weight_sensor_node = science_urc.weight_sensor_node:main',
            'camera_node = science_urc.camera_node:main',
            'spectrometer_node = science_urc.spectrometer_node:main',
        ],
    },
)