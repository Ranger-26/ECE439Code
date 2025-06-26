from setuptools import find_packages, setup

package_name = 'simple_motors'

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
    maintainer='Team4',
    maintainer_email='Team4@wisc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'motor_commander_right = simple_motors.motors_commander:main',
		'motor_executor_right = simple_motors.motors_executor:main',
		'motor_commander_left = simple_motors.motor_commander:main',
		'motor_executor_left = simple_motors.motor_executor:main'
        ],
    },
)
