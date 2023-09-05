from setuptools import setup

package_name = 'data_compression'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/data_compression.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anvesh',
    maintainer_email='agummi@andrew.cmu.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'compress_realsense = data_compression.compress_realsense:main',
            'compress_thermal   = data_compression.compress_thermal:main',
            'compress_rgb       = data_compression.compress_rgb:main'
        ],
    },
)
