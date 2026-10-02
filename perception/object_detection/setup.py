from setuptools import setup

package_name = 'object_detection'

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
    maintainer='rover',
    maintainer_email='rover@example.com',
    description='Object detection with enhanced selection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = object_detection.node:main',
            'zed_object_detector = object_detection.zed_detector:main',
            'zed_object_bridge = object_detection.zed_bridge:main',
            'select_object_service = object_detection.select_object_service:main',
        ],
    },
)
