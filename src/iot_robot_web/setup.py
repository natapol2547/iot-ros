from glob import glob

from setuptools import find_packages, setup

package_name = "iot_robot_web"

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
        # The page is served from the install space, found with get_package_share_directory
        ("share/" + package_name + "/static", glob("static/*")),
    ],
    install_requires=["setuptools"],
    # Tells `colcon test` to run pytest
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="tawan",
    maintainer_email="contact@findmy3d.com",
    description="Browser controller for the iot_robot, served on the local network.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "web_controller = iot_robot_web.web_controller:main",
        ],
    },
)
