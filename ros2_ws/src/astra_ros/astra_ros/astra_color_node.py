"""
astra_color_node - publish the Astra COLOR image (bgr8) for YOLO / viewing in Foxglove.
=======================================================================================
The Astra over USB2 CANNOT stream depth + color simultaneously -> this node opens
mode="color" (do NOT run alongside the depth/SLAM astra_node). It is the source for yolo_node.

Run:  ros2 run astra_ros astra_color_node
Foxglove: Image panel /camera/color/image_raw.
"""
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

REPO_ROOT = os.environ.get("ROBOT_REPO_ROOT", "/home/asiclab/Robot_collecting_VLM_Model")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from layer1_vision.cameras.astra_openni import AstraCamera  # noqa: E402


class AstraColorNode(Node):
    def __init__(self):
        super().__init__("astra_color_node")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("topic", "/camera/color/image_raw")

        self.frame_id = self.get_parameter("frame_id").value
        rate = float(self.get_parameter("rate_hz").value)
        topic = self.get_parameter("topic").value

        # BEST_EFFORT: a 640x480x3 color image is ~921KB, no need for reliable (see astra_node).
        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)

        self.get_logger().info("Opening Astra (mode=color)...")
        self.cam = AstraCamera(mode="color")
        self.get_logger().info("Astra color OK -> %s @ %.0f Hz" % (topic, rate))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            bgr, _ = self.cam.read()
        except RuntimeError as e:
            self.get_logger().warn("Color read error: %s" % e)
            return
        if bgr is None:
            return
        bgr = np.ascontiguousarray(bgr)
        h, w = bgr.shape[:2]

        img = Image()
        img.header.stamp = self.get_clock().now().to_msg()
        img.header.frame_id = self.frame_id
        img.height = h
        img.width = w
        img.encoding = "bgr8"
        img.is_bigendian = 0
        img.step = w * 3
        img.data = bgr.tobytes()
        self.pub.publish(img)

    def destroy_node(self):
        try:
            self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AstraColorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
