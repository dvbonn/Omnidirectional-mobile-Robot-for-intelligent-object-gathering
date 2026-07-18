"""
camera_manager - a single node that OWNS the Astra camera, hot-switching mode via /camera_mode.
===============================================================================================
Astra over USB2 = one client at a time + NO simultaneous color+depth. This node is the sole
camera owner; it receives /camera_mode (std_msgs/String: "depth"|"color"|"both") -> close() +
reopen in the right mode, then publishes the matching topics. It replaces running astra_node
and astra_color_node separately (which would contend for the camera). It is the piece that lets
the T10 orchestrator switch the camera between phases.

  depth : /camera/depth/image_raw (16UC1 mm) + /camera/depth/camera_info     (~15Hz, for SLAM/scan)
  color : /camera/color/image_raw (bgr8)                                      (~15Hz, for YOLO)
  both  : all 3 topics above (driver toggles ~2Hz - slow, only for LOCATE/APPROACH phases)

Switch mode at runtime:  ros2 topic pub --once /camera_mode std_msgs/msg/String "{data: color}"
A switch costs ~1-2s (USB close/open) - normal. A single-threaded executor means the switch
(in the callback) does not run over read() (in the timer).
"""
import json
import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

REPO_ROOT = os.environ.get("ROBOT_REPO_ROOT", "/home/asiclab/Robot_collecting_VLM_Model")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
from layer1_vision.cameras.astra_openni import AstraCamera  # noqa: E402

VALID = ("depth", "color", "both")


def load_intrinsics():
    with open(os.path.join(REPO_ROOT, "config", "astra_intrinsics.json")) as f:
        return json.load(f)


class CameraManager(Node):
    def __init__(self):
        super().__init__("camera_manager")
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("mode", "depth")
        self.declare_parameter("depth_frame", "camera_depth_optical_frame")
        self.declare_parameter("color_frame", "camera_color_optical_frame")

        self.rate = float(self.get_parameter("rate_hz").value)
        self.depth_frame = self.get_parameter("depth_frame").value
        self.color_frame = self.get_parameter("color_frame").value
        self.intr = load_intrinsics()

        self.pub_depth = self.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.pub_info = self.create_publisher(CameraInfo, "/camera/depth/camera_info", qos_profile_sensor_data)
        self.pub_color = self.create_publisher(Image, "/camera/color/image_raw", qos_profile_sensor_data)
        self.sub_mode = self.create_subscription(String, "/camera_mode", self.on_mode, 10)

        self.submode = (self.get_parameter("mode").value or "depth").strip().lower()
        if self.submode not in VALID:
            self.submode = "depth"
        self._both_flip = False
        self.cam = None
        self._open_device()
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    # Open the device ONCE; changing mode = toggle stream (NO reopen)
    def _open_device(self):
        # Open the device in 'both' (create both streams, not started). Later mode switches only
        # stop/start a stream via cam.grab() - NO device close/reopen. The Astra over USB2 breaks
        # if oniShutdown()->oniInitialize() repeats (the reopen cycle) -> hangs, losing both depth
        # and color.
        try:
            self.cam = AstraCamera(mode="both")
            self.get_logger().info("Astra ready (device kept open; toggling streams, no reopen)")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error("Failed to open Astra: %s" % e)
            self.cam = None

    def on_mode(self, msg):
        req = (msg.data or "").strip().lower()
        if req not in VALID:
            self.get_logger().warn("invalid mode '%s' (depth|color|both)" % msg.data)
            return
        if req != self.submode:
            self.get_logger().info("Switching stream -> %s (stop/start, no device reopen)" % req)
            self.submode = req

    # Publish according to submode (grab: only toggles the running stream)
    def tick(self):
        if self.cam is None:
            return
        try:
            if self.submode == "color":
                bgr, depth_mm = self.cam.grab("color")
            elif self.submode == "both":
                # interleave depth/color (each ~half rate) - still only toggles the stream, no reopen
                self._both_flip = not self._both_flip
                bgr, depth_mm = self.cam.grab("color" if self._both_flip else "depth")
            else:  # "depth" (exploration default): stay on depth -> full ~7Hz, no toggle
                bgr, depth_mm = self.cam.grab("depth")
        except RuntimeError as e:
            self.get_logger().warn("Camera read error: %s" % e, throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        if depth_mm is not None:
            self._pub_depth(depth_mm, stamp)
        if bgr is not None:
            self._pub_color(bgr, stamp)

    def _pub_depth(self, depth_mm, stamp):
        d = depth_mm.astype(np.uint16)
        h, w = d.shape
        img = Image()
        img.header.stamp = stamp; img.header.frame_id = self.depth_frame
        img.height = h; img.width = w; img.encoding = "16UC1"; img.is_bigendian = 0
        img.step = w * 2; img.data = d.tobytes()
        self.pub_depth.publish(img)

        fx = float(self.intr["fx"]); fy = float(self.intr["fy"])
        cx = float(self.intr["cx"]); cy = float(self.intr["cy"])
        info = CameraInfo()
        info.header.stamp = stamp; info.header.frame_id = self.depth_frame
        info.height = h; info.width = w; info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.pub_info.publish(info)

    def _pub_color(self, bgr, stamp):
        bgr = np.ascontiguousarray(bgr)
        h, w = bgr.shape[:2]
        img = Image()
        img.header.stamp = stamp; img.header.frame_id = self.color_frame
        img.height = h; img.width = w; img.encoding = "bgr8"; img.is_bigendian = 0
        img.step = w * 3; img.data = bgr.tobytes()
        self.pub_color.publish(img)

    def destroy_node(self):
        if self.cam is not None:
            try:
                self.cam.close()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CameraManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
