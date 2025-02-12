from setuptools import find_packages, setup

package_name = 'sdp_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/hardware_interface', ['diff_drive_hardware.xml']),
        ('share/' + package_name, ['diff_drive_hardware.xml']),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/display_v2.launch.py']),
        ('share/' + package_name + '/launch', ['launch/display_v3.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ros2_control_v1.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rplidar_test.launch.py']),
        ('share/' + package_name + '/config', ['config/controller.yaml']),
        ('share/' + package_name + '/rviz', ['rviz/config.rviz']),
        ('share/' + package_name + '/rviz', ['rviz/config_v1.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/all_bot.urdf']),
    ],
    zip_safe=True,
    maintainer='sdp',
    maintainer_email='leoncioh@uci,edu',
    description='All in one package for the sdp robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = sdp_bot.control:main',
        ],
    },
)
