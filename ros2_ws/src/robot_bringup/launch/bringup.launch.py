"""
bringup.launch.py - REAL end-to-end STACK (NO Gazebo): TF + base + Astra + SLAM + NAV2.
=======================================================================================
Wiring:
  robot_state_publisher (robot.urdf.xacro) -> static TF tree base_link->{footprint,wheels,camera}
  fake_odom | cpp_package/wifi_node         -> /odom + tf odom->base_link
  astra_node + depthimage_to_laserscan      -> /scan (Astra depth, capped ~1m)
  slam_toolbox (our Astra config)           -> /map + tf map->odom
  NAV2 (planner/controller RPP/recoveries/bt + lifecycle, Astra config) -> navigate_to_pose
  explorer_node (WFD)                       -> self-exploration: frontier -> goal -> spinScan
  rosbridge :9090                           -> Foxglove (laptop/phone)

NOTE: the real URDF (robot_description/urdf/robot.urdf.xacro) matches the pipeline frames:
   base_link root, camera_depth_optical_frame - DIFFERENT from the Gazebo mecanum_robot.urdf.xacro at the root.
NOTE: use the robot_bringup ASTRA config (use_sim_time:False, max_laser_range 1.0, RPP),
   NOT the sim/TurtleBot3 config.

Args:
  odom:=wifi   (default) cpp_package/wifi_node - real base over WiFi (TCP server :2004)
  odom:=fake   base_bridge/fake_odom - test WITHOUT the base (stationary, still emits tf odom->base_link)
  sensor:=astra (default) enable Astra + depth->/scan  | none: only build the graph/TF (no camera plugged)
  nav:=true    enable NAV2 (planner/controller/...)     | false: SLAM only (teleop W5)
  autonomous:=true  enable explorer_node (WFD autonomy)  | false: manual nav via a Foxglove goal
  cmd_timeout:=0.5  STOP watchdog when /cmd_vel is silent (seconds)

End-to-end test recipe (bottom-up for a smooth bring-up - if a step fails, inspect that step):
  # B1 TF+wiring only, NO base/camera needed (check the graph comes up and the TF tree is unbroken):
  ros2 launch robot_bringup bringup.launch.py odom:=fake sensor:=none nav:=false autonomous:=false
  #   -> ros2 run tf2_tools view_frames  (map->odom->base_link->camera... continuous)
  # B2 add the real Astra -> /scan (camera required), map by teleop:
  ros2 launch robot_bringup bringup.launch.py odom:=fake nav:=false autonomous:=false
  #   + teleop:  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  # B3 real base + full autonomy (demo W6):
  ros2 launch robot_bringup bringup.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, LaunchConfigurationEquals
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
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")

    nav = LaunchConfiguration("nav")
    autonomous = LaunchConfiguration("autonomous")
    cmd_timeout = LaunchConfiguration("cmd_timeout")

    # Robot TF (robot_state_publisher from robot.urdf.xacro)
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, "launch", "description.launch.py")
        )
    )

    # Astra depth -> /scan (only when sensor:=astra)
    astra = Node(
        package="astra_ros", executable="astra_node", name="astra_node", output="screen",
        condition=LaunchConfigurationEquals("sensor", "astra"),
    )
    depth2scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        output="screen",
        parameters=[{
            "scan_height": 10, "scan_time": 0.066,
            "range_min": 0.20, "range_max": 0.90,   # below the Astra cap ~1.02m (T0)
            "output_frame": "camera_depth_optical_frame",
        }],
        remappings=[("depth", "/camera/depth/image_raw"),
                    ("depth_camera_info", "/camera/depth/camera_info")],
        condition=LaunchConfigurationEquals("sensor", "astra"),
    )

    # Odom source: real WiFi base (wifi_node) or fake_odom (test)
    wifi_node = Node(
        package="cpp_package", executable="wifi_node", name="socket_handler", output="screen",
        parameters=[{"cmd_timeout": cmd_timeout}],
        condition=LaunchConfigurationEquals("odom", "wifi"),
    )
    fake_odom = Node(
        package="base_bridge", executable="fake_odom", name="fake_odom", output="screen",
        condition=LaunchConfigurationEquals("odom", "fake"),
    )

    # SLAM
    slam = Node(
        package="slam_toolbox", executable="async_slam_toolbox_node",
        name="slam_toolbox", output="screen", parameters=[slam_params],
    )

    # NAV2 (Astra config; live-SLAM so NO map_server/amcl needed)
    nav2_nodes = [
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

    # WFD autonomy
    explorer = Node(
        package="cpp_package", executable="explorer_node", name="frontier_based_exploration",
        output="screen", condition=IfCondition(autonomous),
    )

    # Foxglove
    rosbridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(rosbridge_share, "launch", "rosbridge_websocket_launch.xml")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("odom", default_value="wifi",
                              description="wifi (real base) | fake (test without a base)"),
        DeclareLaunchArgument("sensor", default_value="astra",
                              description="astra (depth->/scan) | none (TF/graph only, no camera)"),
        DeclareLaunchArgument("nav", default_value="true",
                              description="true: enable NAV2 | false: SLAM only (teleop)"),
        DeclareLaunchArgument("autonomous", default_value="true",
                              description="true: WFD explorer autonomy | false: manual nav"),
        DeclareLaunchArgument("cmd_timeout", default_value="0.5",
                              description="STOP watchdog when /cmd_vel is silent (seconds)"),
        description, astra, depth2scan, wifi_node, fake_odom, slam,
        *nav2_nodes, explorer, rosbridge,
    ])
