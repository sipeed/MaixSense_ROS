from setuptools import setup

package_name = 'sipeed_tof'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','opencv-python'],
    zip_safe=True,
    maintainer='vmros',
    maintainer_email='vmros@todo.todo',
    description='ROS2 Package for SIPEED ToF Sensor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'publisher = sipeed_tof.publisher:main',
        ],
    },
)
