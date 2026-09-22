from setuptools import find_packages, setup

package_name = "iot_robot_drivers"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    # Makes `colcon test` run the tests with pytest instead of unittest
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="tawan",
    maintainer_email="contact@findmy3d.com",
    description="Drivers for the real iot_robot: STM32 sensor bridge, CubeMars CAN tools "
                "and the LSM9DS1 IMU.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "stm32_bridge = iot_robot_drivers.stm32_bridge:main",
            "fake_stm32 = iot_robot_drivers.fake_stm32:main",
            "cubemars_tool = iot_robot_drivers.cubemars_tool:main",
            "fake_cubemars = iot_robot_drivers.fake_cubemars:main",
            "lsm9ds1_node = iot_robot_drivers.lsm9ds1_node:main",
        ],
    },
)
