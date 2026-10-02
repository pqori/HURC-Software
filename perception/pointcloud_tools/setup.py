from setuptools import setup
import os
from glob import glob

package_name = 'pointcloud_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dorde Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='Lightweight utilities for point cloud processing',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_scan = pointcloud_tools.depth_to_scan:main',
            'grid_builder = pointcloud_tools.grid_builder:main',
        ],
    },
)

