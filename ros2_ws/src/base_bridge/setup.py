from setuptools import setup

package_name = "base_bridge"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dvbonn",
    maintainer_email="ndtrung2407@gmail.com",
    description="ROS2 <-> ESP32 mecanum base bridge: /cmd_vel<->SET_ROBOT_VELOCITY, ROBOT_STATE->/odom+tf. Transport TCP:2004 (CAN stub).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "base_node = base_bridge.base_node:main",
            "fake_odom = base_bridge.fake_odom:main",
        ],
    },
)
