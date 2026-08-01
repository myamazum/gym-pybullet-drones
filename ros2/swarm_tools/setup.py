from setuptools import setup

package_name = 'swarm_tools'

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
    maintainer='Mitsuhiro Yamazumi',
    maintainer_email='yamazumi.mitsuhiro@gmail.com',
    description='Experimental swarm sensing alignment and control utilities',
    license='MIT',
    entry_points={
        'console_scripts': [
            'swarm_control = swarm_tools.swarm_test:main',
        ],
    },
)
