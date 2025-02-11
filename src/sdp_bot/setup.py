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
        ('share/' + package_name + '/urdf', ['urdf/sdp_bot.urdf.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/sdp_bot_core.xacro']),
        ('share/' + package_name + '/urdf', ['urdf/inertial_macros.xacro']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sdp',
    maintainer_email='sdp@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'control = sdp_bot.control:main',
        ],
    },
)
