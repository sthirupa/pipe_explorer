from setuptools import setup

package_name = 'flir_lepton_ros_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arpa-e',
    maintainer_email='tejasr@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flir_lepton_ros_driver_node = flir_lepton_ros_driver.thermal_cam_ros:main'
        ],
    },
)
