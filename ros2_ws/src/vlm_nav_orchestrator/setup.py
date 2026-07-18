from setuptools import setup

package_name = "vlm_nav_orchestrator"

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
    description="T10: detect (YOLO) -> drive to the object. Hybrid NAV2 map-goal + visual servoing. Math in geometry.py (unit-tested).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "orchestrator_node = vlm_nav_orchestrator.orchestrator_node:main",
        ],
    },
)
