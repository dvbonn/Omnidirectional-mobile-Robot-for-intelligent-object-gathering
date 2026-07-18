"""
slam.launch.py - SLAM (slam_toolbox) with Astra depth -> /scan, sim-free or real base.
======================================================================================
Stack:  robot_state_publisher (TF) + astra_node (depth) + depthimage_to_laserscan (/scan)
        + odom source + slam_toolbox (online_async) + rosbridge (Foxglove).

SLAM only (teleop to map - W5). Full autonomy demo uses bringup.launch.py.
odom param (launch arg `odom`):
  odom:=fake  -> base_bridge/fake_odom (STATIONARY; smoke-test before the base exists)
  odom:=wifi  -> cpp_package/wifi_node (REAL BASE over WiFi, TCP server :2004)
  odom:=none  -> do not start odom (provide odom->base_link elsewhere)

Run smoke-test (no base yet):
  ros2 launch robot_bringup slam.launch.py odom:=fake
View in Foxglove: ws://192.168.1.250:9090 -> 3D panel: /map, /scan, TF, robot model.
Save map:  ros2 run nav2_map_server map_saver_cli -f maps/room

NOTE: Astra cap ~1m + FOV 58.6 deg -> drive SLOWLY, hug walls within 1m (see the T5 SOP).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("robot_description")
    bringup_share = get_package_share_directory("robot_bringup")
    rosbridge_share = get_package_share_directory("rosbridge_server")

    slam_params = os.path.join(bringup_share, "config", "slam_toolbox.yaml")

    odom = LaunchConfiguration("odom")

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "description.launch.py")
        )
    )

    astra = Node(
        package="astra_ros", executable="astra_node", name="astra_node", output="screen",
    )

    depth2scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        parameters=[{
            "scan_height": 10,
            "scan_time": 0.066,
            "range_min": 0.20,
            "range_max": 0.90,
            "output_frame": "camera_depth_optical_frame",
        }],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/depth/camera_info"),
        ],
    )

    # Odom source
    fake_odom = Node(
        package="base_bridge", executable="fake_odom", name="fake_odom", output="screen",
        condition=LaunchConfigurationEquals("odom", "fake"),
    )
    # REAL base: use cpp_package/wifi_node (TCP server :2004, ESP32 client connects over WiFi -
    # topology confirmed from the firmware). base_bridge/base_node (client) was REMOVED.
    wifi_node = Node(
        package="cpp_package", executable="wifi_node", name="socket_handler", output="screen",
        condition=LaunchConfigurationEquals("odom", "wifi"),
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom", default_value="wifi",
                              description="Odom source: fake (test) | wifi (real base wifi_node) | none"),
        description, astra, depth2scan, fake_odom, wifi_node, slam, rosbridge,
    ])
