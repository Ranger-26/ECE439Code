from setuptools import find_packages, setup

package_name = 'i2c_sensors'

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
    maintainer='peter adamczyk',
    maintainer_email='peter.adamczyk@wisc.edu',
    description='MPU 6050 IMU and other i2c sensors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_mpu6050_node=i2c_sensors.imu_mpu6050_node:main'
        ],
    },
)
