"""
View via Foxglove Studio: robot_state_publisher (TF + URDF) + rosbridge WebSocket.
=================================================================================
Run on the Jetson:
    ros2 launch robot_bringup view_foxglove.launch.py

On a laptop/phone, open Foxglove Studio:
    Open connection -> choose "Rosbridge" (NOT "Foxglove WebSocket")
    Enter the address depending on how you connect:
      - Laptop on the same home router (WiFi/LAN):   ws://192.168.1.166:9090
      - Connected to the robot's hotspot (wlan0):    ws://192.168.5.1:9090
    Add a "3D" panel -> shows TF frames + the robot model (reads URDF from /robot_description).

    NOTE: rosbridge binds 0.0.0.0:9090 (all interfaces). If it still won't connect,
    check the current IP with `hostname -I` on the Jetson - DHCP may change the IP.

This is the FIRST visual checkpoint (no Gazebo needed): verify the Foxglove path
works before building the sim (T3).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)


def generate_launch_description():
    desc_share = get_package_share_directory("robot_description")
    rosbridge_share = get_package_share_directory("rosbridge_server")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "description.launch.py")
        )
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        )
    )

    return LaunchDescription([description, rosbridge])
