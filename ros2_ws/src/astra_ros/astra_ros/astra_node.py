"""
astra_node - publish Astra depth as sensor_msgs/Image (16UC1, mm) + CameraInfo.
==============================================================================
Reuses the driver layer1_vision/cameras/astra_openni.py (mode="depth", ~30 FPS).
Frame = camera_depth_optical_frame (matches URDF + detection_log.py).
It is the source for depthimage_to_laserscan -> /scan -> slam_toolbox.

USB 2.0 note: this node opens depth ONLY. When RGB is needed (VLM) this node must be
stopped (no simultaneous depth+color streaming).
"""
import json
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# Insert the repo root to import the Astra driver (the driver finds tools/orbbec/openni2/ via __file__).
REPO_ROOT = os.environ.get("ROBOT_REPO_ROOT", "/home/asiclab/Robot_collecting_VLM_Model")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from layer1_vision.cameras.astra_openni import AstraCamera  # noqa: E402


def load_intrinsics():
    with open(os.path.join(REPO_ROOT, "config", "astra_intrinsics.json")) as f:
        return json.load(f)


class AstraNode(Node):
    def __init__(self):
        super().__init__("astra_node")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("frame_id", "camera_depth_optical_frame")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("info_topic", "/camera/depth/camera_info")
        # QoS: default BEST_EFFORT (SensorDataQoS) - a 614KB depth Image over RELIABLE gets
        # congested at ~7Hz on loopback DDS; depthimage_to_laserscan subscribes BEST_EFFORT
        # (verified via `ros2 topic info -v`), so this is the MATCHING + fast choice.
        # Set reliable=true if you need to view it directly with a RELIABLE subscriber tool.
        self.declare_parameter("reliable", False)

        self.frame_id = self.get_parameter("frame_id").value
        rate = float(self.get_parameter("rate_hz").value)
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("info_topic").value

        if bool(self.get_parameter("reliable").value):
            qos = QoSProfile(depth=5)
            qos.reliability = ReliabilityPolicy.RELIABLE
        else:
            qos = qos_profile_sensor_data  # BEST_EFFORT, KEEP_LAST 5

        self.intr = load_intrinsics()
        self.pub_depth = self.create_publisher(Image, depth_topic, qos)
        self.pub_info = self.create_publisher(CameraInfo, info_topic, qos)

        self.get_logger().info("Opening Astra (mode=depth)...")
        self.cam = AstraCamera(mode="depth")
        self.get_logger().info(
            "Astra OK -> %s + %s @ %.0f Hz, frame=%s" % (depth_topic, info_topic, rate, self.frame_id)
        )
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        try:
            _, depth_mm = self.cam.read()
        except RuntimeError as e:
            self.get_logger().warn("Depth read error: %s" % e)
            return

        # read() returns float32 (mm). astype(uint16) makes ONE new C-contiguous array -
        # dropping the redundant ascontiguousarray(float32) (previously copied twice ~1.2MB+614KB).
        depth_u16 = depth_mm.astype(np.uint16)
        h, w = depth_u16.shape
        stamp = self.get_clock().now().to_msg()

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = self.frame_id
        img.height = h
        img.width = w
        img.encoding = "16UC1"
        img.is_bigendian = 0
        img.step = w * 2
        img.data = depth_u16.tobytes()
        self.pub_depth.publish(img)

        fx = float(self.intr["fx"]); fy = float(self.intr["fy"])
        cx = float(self.intr["cx"]); cy = float(self.intr["cy"])
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.height = h
        info.width = w
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

        if not getattr(self, "_first_logged", False):
            self._first_logged = True
            self.get_logger().info("Published first frame: depth %dx%d (16UC1, mm)" % (w, h))

    def destroy_node(self):
        try:
            self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = AstraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
