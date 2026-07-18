"""
Launch robot_state_publisher to publish the static TF tree from the URDF.
========================================================================
Run:  ros2 launch robot_description description.launch.py
Check:
    ros2 run tf2_tools view_frames        # exports frames.pdf - the TF tree is unbroken
    ros2 topic echo /robot_description     # URDF as a string
Wheels are fixed so joint_state_publisher is NOT needed; set use_jsp:=true if the
wheels later become continuous joints.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("robot_description")
    xacro_path = os.path.join(pkg, "urdf", "robot.urdf.xacro")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_jsp = LaunchConfiguration("use_jsp")

    # Process xacro -> URDF string at launch time (needs the 'xacro' package).
    # Foxy: MUST wrap in ParameterValue(value_type=str), otherwise launch_ros tries to
    # parse the URDF string as YAML and breaks on the ':' inside a comment.
    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Whether to use the sim (Gazebo) /clock"),
        DeclareLaunchArgument("use_jsp", default_value="false",
                              description="Enable joint_state_publisher (only needed with non-fixed joints)"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=IfCondition(use_jsp),
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
