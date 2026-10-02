from setuptools import setup
import os
from glob import glob

package_name = 'slam_launch'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # Config files  
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dorde Ivanovic',
    maintainer_email='dorde_ivanovic@college.harvard.edu',
    description='URC SLAM and Complete Perception System Launch Package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Monitoring components
            'perception_health_monitor.py = slam_launch.monitoring.perception_health_monitor:main',
            'system_status.py = slam_launch.monitoring.system_status:main',
            'perception_guardian.py = slam_launch.monitoring.perception_guardian:main',
            'calibration_validator.py = slam_launch.monitoring.calibration_validator:main',
            
            # Navigation components
            'frame_coordinator = slam_launch.navigation.frame_coordinator:main',
            'topic_coordinator = slam_launch.navigation.topic_coordinator:main',
            'nav2_integration_bridge = slam_launch.navigation.nav2_integration_bridge:main',
            'sensor_costmap_publisher = slam_launch.navigation.sensor_costmap_publisher:main',
            'navigation_recovery = slam_launch.navigation.navigation_recovery:main',
            'localization_switcher = slam_launch.navigation.localization_switcher:main',
            'nav2_parameter_sync = slam_launch.navigation.nav2_parameter_sync:main',
            
            # Mapping components
            'mapping_coordinator = slam_launch.mapping.mapping_coordinator:main',
            
            # Integration components
            'semantic_mapper.py = slam_launch.integration.semantic_mapper:main',
            'sensor_fusion.py = slam_launch.integration.sensor_fusion:main',
        ],
    },
)
