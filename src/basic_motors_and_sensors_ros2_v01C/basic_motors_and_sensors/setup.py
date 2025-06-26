from setuptools import find_packages, setup

package_name = 'basic_motors_and_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_commander = basic_motors_and_sensors.motor_commander:main',
            'motor_executor = basic_motors_and_sensors.motor_executor:main',
            'sensors_node = basic_motors_and_sensors.sensors_node:main',
            'sensors_processor = basic_motors_and_sensors.sensors_processor:main',
            'encoders_node = basic_motors_and_sensors.encoders_node:main',
            'sensors_to_motor = basic_motors_and_sensors.sensors_to_motor:main'
	],
    },
)
