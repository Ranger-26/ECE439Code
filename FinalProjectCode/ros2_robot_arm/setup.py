from setuptools import setup

package_name = 'ros2_robot_arm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 package for stereo vision and robot arm control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_vision_position_detection = ros2_robot_arm.stereo_vision_position_detection:main',
            'arm_position_setter = ros2_robot_arm.arm_position_setter:main',
            'inverse_kinematics = ros2_robot_arm.inverse_kinematics:main',
            'servo_commands = ros2_robot_arm.servo_commands:main',
            'servo_gui = ros2_robot_arm.servo_gui:main',
        ],
    },
)
