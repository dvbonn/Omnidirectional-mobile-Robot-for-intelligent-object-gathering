"""
yolo.launch.py - Test the YOLOv8n node in Foxglove (Astra color -> detect -> annotated image).
==============================================================================================
Stack:  astra_color_node (/camera/color/image_raw) + yolo_node + rosbridge.
NOTE: opens the camera in COLOR mode -> do NOT run alongside SLAM (astra_node depth) - USB2 single client.

Run:  ros2 launch robot_bringup yolo.launch.py
      ros2 launch robot_bringup yolo.launch.py target_only:=false   # detect ALL COCO classes

Foxglove (ws://<ip>:9090, Rosbridge type):
  - Image panel  **/yolo/image_annotated/compressed**  -> boxes + labels (JPEG, RECOMMENDED:
                  rosbridge sends JSON/base64 so the raw 921KB image is too heavy to display)
  - Raw Messages /yolo/detections   -> JSON [{class_name, confidence, bbox}]
  - (/yolo/image_annotated raw bgr8 is still available for other ROS consumers, NOT suitable via rosbridge)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_share = get_package_share_directory("rosbridge_server")

    astra_color = Node(
        package="astra_ros", executable="astra_color_node",
        name="astra_color_node", output="screen",
    )

    yolo = Node(
        package="yolo_ros", executable="yolo_node", name="yolo_node", output="screen",
        parameters=[{
            "target_only": LaunchConfiguration("target_only"),
            "conf": LaunchConfiguration("conf"),
        }],
    )

    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("target_only", default_value="true",
                              description="true: collectible objects only (TARGET_CLASSES) | false: all COCO classes"),
        DeclareLaunchArgument("conf", default_value="0.45",
                              description="YOLO confidence threshold"),
        astra_color, yolo, rosbridge,
    ])
