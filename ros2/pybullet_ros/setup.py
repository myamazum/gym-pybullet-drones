from setuptools import setup

package_name = 'pybullet_ros'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='yamazumi.mitsuhiro@gmail.com',
    description='setup PID Drone control in ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_pid = pybullet_ros.example_pid:main',
            'drone = pybullet_ros.simple_drone:main',
            ],
    },
)
