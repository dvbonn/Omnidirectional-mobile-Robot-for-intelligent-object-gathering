"""
collect.launch.py - DEMO T10 end-to-end: detect (YOLO) -> drive to the object (Hybrid).
=======================================================================================
Assembles everything: camera_manager (USB2 hot-switch) + depthimage_to_laserscan + odom
+ slam_toolbox + NAV2 + yolo_node + vlm_nav_orchestrator + rosbridge.

Difference from bringup.launch.py: uses **camera_manager** (one camera, switching mode via
/camera_mode published by the orchestrator) INSTEAD of astra_node; + yolo_node + orchestrator.

NOTE: Astra USB2: camera_manager is in a single mode at a time -> /scan (depth) and /yolo (color)
   are NOT simultaneous; the orchestrator coordinates: DETECT(color) -> LOCATE(both) -> NAVIGATE(depth) -> APPROACH(both).
   SLAM/NAV2 only get /scan data in the NAVIGATE phase (depth) - the robot stops while detecting.

Run (real base):  ros2 launch robot_bringup collect.launch.py target_class:=bottle
Test no base:     ros2 launch robot_bringup collect.launch.py odom:=fake nav:=false
Foxglove: /yolo/image_annotated/compressed, /orchestrator/state, /map, TF.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource, PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("robot_description")
    bringup_share = get_package_share_directory("robot_bringup")
    rosbridge_share = get_package_share_directory("rosbridge_server")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")

    nav = LaunchConfiguration("nav")
    autonomous = LaunchConfiguration("autonomous")
    target_class = LaunchConfiguration("target_class")
    detect_scan_sec = LaunchConfiguration("detect_scan_sec")
    detect_look_sec = LaunchConfiguration("detect_look_sec")

    description = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(desc_share, "launch", "description.launch.py")))

    # Camera: one node that switches mode via /camera_mode (controlled by the orchestrator)
    camera = Node(package="astra_ros", executable="camera_manager", name="camera_manager",
                  output="screen", parameters=[{"mode": "depth"}])

    depth2scan = Node(
        package="depthimage_to_laserscan", executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan", output="screen",
        parameters=[{"scan_height": 10, "scan_time": 0.066, "range_min": 0.20,
                     "range_max": 0.90, "output_frame": "camera_depth_optical_frame"}],
        remappings=[("depth", "/camera/depth/image_raw"),
                    ("depth_camera_info", "/camera/depth/camera_info")])

    wifi_node = Node(package="cpp_package", executable="wifi_node", name="socket_handler",
                     output="screen", condition=LaunchConfigurationEquals("odom", "wifi"))
    explorer = Node(
        package="cpp_package", executable="explorer_node", name="frontier_based_exploration",
        output="screen", condition=IfCondition(autonomous),
    )

    slam = Node(package="slam_toolbox", executable="async_slam_toolbox_node",
                name="slam_toolbox", output="screen", parameters=[slam_params])

    nav2 = [
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[nav2_params], condition=IfCondition(nav)),
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             output="screen", parameters=[nav2_params], condition=IfCondition(nav)),
        Node(package="nav2_recoveries", executable="recoveries_server", name="recoveries_server",
             output="screen", parameters=[nav2_params], condition=IfCondition(nav)),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen", parameters=[nav2_params], condition=IfCondition(nav)),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_navigation", output="screen",
             parameters=[{"use_sim_time": False, "autostart": True,
                          "node_names": ["planner_server", "controller_server",
                                         "recoveries_server", "bt_navigator"]}],
             condition=IfCondition(nav)),
    ]

    yolo = Node(package="yolo_ros", executable="yolo_node", name="yolo_node",
                output="screen", parameters=[{"target_only": True}])

    orchestrator = Node(package="vlm_nav_orchestrator", executable="orchestrator_node",
                        name="vlm_nav_orchestrator", output="screen",
                        parameters=[{"target_class": target_class,
                                     "detect_scan_sec": detect_scan_sec,
                                     "detect_look_sec": detect_look_sec}])

    rosbridge = IncludeLaunchDescription(AnyLaunchDescriptionSource(
        os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")))

    return LaunchDescription([
        DeclareLaunchArgument("odom", default_value="wifi",
                              description="wifi (real base) | fake (test without a base)"),
        DeclareLaunchArgument("nav", default_value="true",
                              description="true: enable NAV2 | false: skip (test camera/detect)"),
        DeclareLaunchArgument("autonomous", default_value="true",
                              description="true: WFD explorer autonomy | false: manual nav"),
        DeclareLaunchArgument("target_class", default_value="",
                              description="target object class (e.g. bottle); '' = highest confidence"),
        DeclareLaunchArgument("detect_scan_sec", default_value="8.0",
                              description="DETECT: seconds in depth (explore) per duty-cycle"),
        DeclareLaunchArgument("detect_look_sec", default_value="2.5",
                              description="DETECT: seconds in color (look for object) per duty-cycle"),
        description, camera, depth2scan, wifi_node, slam,
        *nav2, yolo, orchestrator, rosbridge, explorer
    ])
