from setuptools import setup

package_name = 'lidar_box_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abcs',
    maintainer_email='your_email@example.com',
    description='RPLIDAR Box Filter',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'box_filter = lidar_box_filter.box_filter:main',
        ],
    },
)
