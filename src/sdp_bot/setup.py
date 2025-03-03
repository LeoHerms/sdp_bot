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
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/ekf.launch.py']),
        ('share/' + package_name + '/launch', ['launch/sdp.launch.py']),
        ('share/' + package_name + '/launch', ['launch/teleop.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/config.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/all_bot.urdf']),
        ('share/' + package_name + '/urdf', ['urdf/all_bot.sdf']),
        ('share/' + package_name + '/world', ['world/my_world.sdf']),
        ('share/' + package_name + '/config', ['config/ekf.yaml']),
        ('share/' + package_name + '/config', ['config/bridge_config.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
    ],
    zip_safe=True,
    maintainer='sdp',
    maintainer_email='leoncioh@uci.edu',
    description='All in one package for the sdp robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = sdp_bot.control:main',
            'bno055 = sdp_bot.bno055:main',
            'tele = sdp_bot.tele:main',
        ],
    },
)
