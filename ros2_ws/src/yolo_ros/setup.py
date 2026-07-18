from setuptools import setup

package_name = "yolo_ros"

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
    description="YOLOv8n as a ROS node: color image -> /yolo/image_annotated (bgr8) + /yolo/detections (JSON). Viewable in Foxglove. Reuses layer1_vision/vision_node.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "yolo_node = yolo_ros.yolo_node:main",
        ],
    },
)
