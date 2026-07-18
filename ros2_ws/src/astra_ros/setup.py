from setuptools import setup

package_name = "astra_ros"

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
    description="Publish Astra depth as a ROS2 Image + CameraInfo for SLAM.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "astra_node = astra_ros.astra_node:main",
            "astra_color_node = astra_ros.astra_color_node:main",
            "camera_manager = astra_ros.camera_manager:main",
        ],
    },
)
