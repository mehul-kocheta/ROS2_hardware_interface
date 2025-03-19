import os
from glob import glob
from setuptools import setup

package_name = 'encoder_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'encoder_reader.encoder_serial_node',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mehul',
    maintainer_email='mehul@example.com',
    description='ROS2 node to read encoder data from Arduino',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/msg', ['msg/EncoderTicks.msg']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    entry_points={
        'console_scripts': [
            'encoder_serial_node = encoder_reader.encoder_serial_node:main',
            'encoder_odom = encoder_reader.encoder_odom:main',
            'motor_interface = encoder_reader.motor_interface:main',
        ],
    },
)
