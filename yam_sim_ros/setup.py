from glob import glob

from setuptools import find_packages, setup

package_name = "yam_sim_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rover Team",
    maintainer_email="rover@example.com",
    description="ROS 2 bridge for the I2RT YAM arm (MuJoCo sim or real arm)",
    license="BSD",
    entry_points={
        "console_scripts": [
            "bridge_node = yam_sim_ros.bridge_node:main",
        ],
    },
)
