from glob import glob

from setuptools import find_packages, setup

package_name = "iot_robot_perception"

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
    maintainer_email="6638050521@student.chula.ac.th",
    description="Camera-based perception for the iot_robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "ball_detector = iot_robot_perception.ball_detector:main",
            "person_detector = iot_robot_perception.person_detector:main",
        ],
    },
)
