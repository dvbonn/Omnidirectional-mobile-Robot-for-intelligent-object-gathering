"""
astra_scan - REAL Astra depth -> /scan, viewed in Foxglove (no Gazebo/base needed).
===================================================================================
Includes: robot_state_publisher (TF) + astra_node (depth) + depthimage_to_laserscan (/scan)
     + rosbridge (Foxglove :9090).

Run:  ros2 launch robot_bringup astra_scan.launch.py
Foxglove (laptop):  ws://192.168.1.250:9090 -> 3D panel: shows TF + the /scan LaserScan rays.

NOTE: Astra cap ~1m + FOV 58.6 deg (T0): range_max=0.9 (below the cap to avoid edge noise),
   scan_height=10 (merge rows around the center to reduce noise). Rays only cover ~58 deg ahead.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("robot_description")
    rosbridge_share = get_package_share_directory("rosbridge_server")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "description.launch.py")
        )
    )

    astra = Node(
        package="astra_ros",
        executable="astra_node",
        name="astra_node",
        output="screen",
    )

    depth2scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        parameters=[{
            "scan_height": 10,          # merge 10 rows around the center (default 1 is too noisy)
            "scan_time": 0.066,         # ~15 Hz
            "range_min": 0.20,          # m - below this is too close/noisy
            "range_max": 0.90,          # m - below the Astra cap ~1.02m (T0)
            "output_frame": "camera_depth_optical_frame",
        }],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
        ],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        )
    )

    return LaunchDescription([description, astra, depth2scan, rosbridge])
