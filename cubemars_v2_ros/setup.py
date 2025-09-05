from setuptools import find_packages, setup

package_name = 'cubemars_v2_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch files
        ('share/' + package_name + '/launch', [
            'launch/two_motors.launch.py',
            'launch/two_motors_joystick.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ciscor',
    maintainer_email='ciscor@todo.todo',
    description='Cubemars AK70/AK80 joystick control nodes (ROS 2)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # existing nodes
            'motor_node = cubemars_v2_ros.motor_node:main',
            'dual_joystick_position = cubemars_v2_ros.dual_joystick_position:main',
            # NEW: single combined node (left stick -> ak70, right -> ak80)
            'dual_joystick_turntable = cubemars_v2_ros.dual_joystick_turntable:main',
        ],
    },
)
