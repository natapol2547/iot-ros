from glob import glob

from setuptools import find_packages, setup

package_name = "iot_robot_behavior"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tawan",
    maintainer_email="contact@findmy3d.com",
    description="Target following behaviours for the iot_robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "target_follower = iot_robot_behavior.target_follower:main",
        ],
    },
)
