from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resources/aruco_detector']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/scripts', [
            'scripts/cam_cal.py',
            'scripts/process.py',
            'scripts/generate_markers.py',
            'scripts/standalone_detector_3d.py',
            'scripts/run_detector.sh'
        ]),
    ],
    install_requires=[
        'setuptools',
        'opencv-python>=4.5',
        'numpy>=1.19',
        'scipy>=1.5',
    ],
    zip_safe=True,
    maintainer='Djordje Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='Production ArUco detector with ZED SDK integration',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector_node = aruco_detector.node:main',
            'generate_markers = scripts.generate_markers:main',
            'cam_cal = scripts.cam_cal:main',
            'process = scripts.process:main',
            'monitor_detections = scripts.monitor_detections:main',
        ],
    },
)
