from setuptools import setup

package_name = 'detection_pose_estimator'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/config',
            ['config/estimator_params.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/pose_estimator.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Djordje Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='Converts Detection2DArray + PointCloud2 → PoseStamped goals',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_estimator = detection_pose_estimator.node:main',
        ],
    },
)
