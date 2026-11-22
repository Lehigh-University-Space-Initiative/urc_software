"""Build and install settings for the web_ui_urc ROS 2 Python package."""

from glob import glob

from setuptools import find_packages, setup

# Name of the ROS 2 package (must match package.xml and the resource/ marker file)
package_name = 'web_ui_urc'
share = 'share/' + package_name

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    # Installing the web page files and launch file next to package.xml
    # (web_server.py finds the page through the package's share directory)
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share, ['package.xml']),
        (share + '/launch', ['launch/web_ui_launch.py']),
        (share + '/web', glob('web/*.html') + glob('web/*.css') + glob('web/*.js')),
        (share + '/web/vendor', glob('web/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='LUSI',
    maintainer_email='todo@todo.todo',
    description='Browser interface for the LUSI rover',
    license='TODO: License declaration',
    tests_require=['pytest'],

    # Registering the nodes so `ros2 run web_ui_urc <name>` works
    entry_points={
        'console_scripts': [
            'web_server = web_ui_urc.web_server:main',
            'science_recorder = web_ui_urc.science_recorder:main',
        ],
    },
)
