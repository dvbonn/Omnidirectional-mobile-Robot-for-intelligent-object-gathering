"""
yolo_node - YOLOv8n as a ROS node: receive a color image -> detect -> publish annotated image + detections.
===========================================================================================================
Reuses the model + drawing functions from layer1_vision/vision_node.py (benchmarked in Chapter 4).
Pub:
  /yolo/image_annotated  (sensor_msgs/Image, bgr8)  -> Foxglove Image panel to view boxes+labels
  /yolo/detections       (std_msgs/String, JSON)    -> structured data [{class_name,confidence,bbox}]
Sub:
  /camera/color/image_raw (bgr8/rgb8)  <- astra_color_node

No cv_bridge/vision_msgs needed (convert the image manually, draw boxes with cv2).
Params: target_only (True=collectible objects only TARGET_CLASSES | False=all COCO classes),
         conf, model, image_topic.

Run:  ros2 run yolo_ros yolo_node --ros-args -p target_only:=false
"""
import json
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String

REPO_ROOT = os.environ.get("ROBOT_REPO_ROOT", "/home/asiclab/Robot_collecting_VLM_Model")
if REPO_ROOT not in sys.path:
    sys.path.insert(0, REPO_ROOT)
# Reuse the loader + drawer + target class list from the existing pipeline.
from layer1_vision.vision_node import (  # noqa: E402
    load_yolo_model, draw_detections, TARGET_CLASSES, CONFIDENCE_THRESHOLD,
)


class YoloNode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.declare_parameter("image_topic", "/camera/color/image_raw")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("conf", float(CONFIDENCE_THRESHOLD))
        self.declare_parameter("target_only", True)   # True: collectible objects only; False: all COCO classes

        image_topic = self.get_parameter("image_topic").value
        self.conf = float(self.get_parameter("conf").value)
        self.target_only = bool(self.get_parameter("target_only").value)

        self.get_logger().info("Loading YOLO (%s)..." % self.get_parameter("model").value)
        self.model = load_yolo_model(self.get_parameter("model").value)
        self.names = self.model.names  # COCO id->name

        # RELIABLE for the view topic: rosbridge 1.3.1 (Foxy) subscribes RELIABLE -> a BEST_EFFORT
        # publisher would NOT reach Foxglove. RELIABLE is compatible with both sides.
        view_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.pub_img = self.create_publisher(Image, "/yolo/image_annotated", view_qos)
        # JPEG-compressed image: rosbridge 1.3.1 (Foxy) sends msgs over JSON/base64 -> a raw
        # 921KB/frame image is too heavy for Foxglove. CompressedImage ~30-50KB -> smooth.
        # -> In Foxglove pick THIS topic for the Image panel.
        self.pub_comp = self.create_publisher(CompressedImage, "/yolo/image_annotated/compressed", view_qos)
        self.declare_parameter("jpeg_quality", 80)
        self._jpeg_q = int(self.get_parameter("jpeg_quality").value)
        self.pub_det = self.create_publisher(String, "/yolo/detections", 10)
        self.sub = self.create_subscription(
            Image, image_topic, self.on_image, qos_profile_sensor_data
        )
        self._warned = False
        self.get_logger().info(
            "YOLO ready. Sub %s -> /yolo/image_annotated + /yolo/detections (target_only=%s, conf=%.2f)"
            % (image_topic, self.target_only, self.conf)
        )

    def on_image(self, msg: Image):
        if msg.encoding not in ("bgr8", "rgb8"):
            if not self._warned:
                self.get_logger().warn("encoding %s not supported (need bgr8/rgb8)" % msg.encoding)
                self._warned = True
            return
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        bgr = arr if msg.encoding == "bgr8" else arr[:, :, ::-1]
        bgr = np.ascontiguousarray(bgr)   # YOLO + cv2 use BGR

        dets = self._infer(bgr)
        annotated = draw_detections(bgr.copy(), dets)   # draw boxes+labels (cv2)

        out = Image()
        out.header = msg.header
        out.height, out.width = annotated.shape[:2]
        out.encoding = "bgr8"
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = np.ascontiguousarray(annotated).tobytes()
        self.pub_img.publish(out)

        # CompressedImage JPEG for Foxglove via rosbridge (lightweight).
        ok, jpg = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_q])
        if ok:
            comp = CompressedImage()
            comp.header = msg.header
            comp.format = "jpeg"
            comp.data = jpg.tobytes()
            self.pub_comp.publish(comp)

        self.pub_det.publish(String(data=json.dumps(dets, ensure_ascii=False)))

    def _infer(self, bgr):
        results = self.model(bgr, verbose=False, conf=self.conf)
        out = []
        for r in results:
            if r.boxes is None:
                continue
            for b in r.boxes:
                cid = int(b.cls[0])
                if self.target_only and cid not in TARGET_CLASSES:
                    continue
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                out.append({
                    "class_id": cid,
                    "class_name": TARGET_CLASSES.get(cid, self.names.get(cid, str(cid))),
                    "confidence": round(float(b.conf[0]), 3),
                    "bbox": [int(x1), int(y1), int(x2 - x1), int(y2 - y1)],
                })
        return out


def main():
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
