from setuptools import setup

package_name = 'motion_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/teleop.launch.py']),
        ('share/' + package_name, ['launch/teleop_dd.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arpa-e',
    maintainer_email='arpa-e@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = motion_planning.teleop_node:main',
            'teleop_node_dd = motion_planning.teleop_node_dd:main',
            'imu_extractor = motion_planning.imu_extractor:main',
        ],
    },
)
