"""
orchestrator_node - T10: DETECT (YOLO) -> drive to the object (Hybrid: NAV2 map-goal + final servo).
====================================================================================================
State machine:
  DETECT   : read /yolo/detections -> is the target object present? (true/false)
  LOCATE   : depth at the bbox center + intrinsics -> 3D point -> tf(camera->map) -> object pose on the map
  NAVIGATE : NavigateToPose(standoff goal) -> NAV2 plans a path + avoids -> gets close
  APPROACH : HOLONOMIC servo - center the bbox by strafing (vy) + advance to target_z (vx),
             NO turning (mecanum base) -> /cmd_vel
  ARRIVED  : stop (grasping = a later phase)

NOTE: Astra USB2 cannot do color+depth simultaneously -> this node publishes the REQUESTED camera
   mode to /camera_mode (color|depth|both); a "camera manager"/launch must satisfy it.
   Core math is in geometry.py (unit-tested). This is v1 - tune on the real base (W6/T10).
"""
import json
import os

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry  # noqa: F401  (ensures the dependency)
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import String

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from vlm_nav_orchestrator import geometry as geo

REPO_ROOT = os.environ.get("ROBOT_REPO_ROOT", "/home/asiclab/Robot_collecting_VLM_Model")


class State:
    DETECT = "DETECT"
    LOCATE = "LOCATE"
    NAVIGATE = "NAVIGATE"
    APPROACH = "APPROACH"
    ARRIVED = "ARRIVED"


class Orchestrator(Node):
    def __init__(self):
        super().__init__("vlm_nav_orchestrator")
        # Parameters
        self.declare_parameter("target_class", "")        # "" = highest-confidence object
        self.declare_parameter("standoff", 0.4)            # m: stop this far from the object
        self.declare_parameter("approach_target_z", 0.30)  # m: servo stop distance
        self.declare_parameter("approach_switch_dist", 0.9)  # m: below threshold -> switch to servo
        self.declare_parameter("target_lost_timeout", 1.5)   # s: keep last-good on a transient loss
        # DETECT duty-cycle (one Astra USB2 cannot reliably do color+depth at once):
        # spend most time in depth (keep /scan for SLAM/NAV2 exploration), blip to color for YOLO.
        self.declare_parameter("detect_scan_sec", 8.0)   # s: depth window (explore)
        self.declare_parameter("detect_look_sec", 2.5)   # s: color window (look for object)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("fx", 570.3422); self.declare_parameter("fy", 570.3422)
        self.declare_parameter("cx", 319.5);    self.declare_parameter("cy", 239.5)
        self.declare_parameter("image_width", 640)

        self.target_class = self.get_parameter("target_class").value
        self.standoff = float(self.get_parameter("standoff").value)
        self.target_z = float(self.get_parameter("approach_target_z").value)
        self.switch_dist = float(self.get_parameter("approach_switch_dist").value)
        self.lost_timeout = float(self.get_parameter("target_lost_timeout").value)
        self.detect_scan_sec = float(self.get_parameter("detect_scan_sec").value)
        self.detect_look_sec = float(self.get_parameter("detect_look_sec").value)
        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.fx = float(self.get_parameter("fx").value); self.fy = float(self.get_parameter("fy").value)
        self.cx = float(self.get_parameter("cx").value); self.cy = float(self.get_parameter("cy").value)
        self.img_w = int(self.get_parameter("image_width").value)

        # I/O
        self.sub_det = self.create_subscription(String, "/yolo/detections", self._on_det, 10)
        self.sub_depth = self.create_subscription(Image, "/camera/depth/image_raw",
                                                  self._on_depth, qos_profile_sensor_data)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_mode = self.create_publisher(String, "/camera_mode", 10)   # color|depth|both
        self.pub_state = self.create_publisher(String, "/orchestrator/state", 10)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # State
        self.state = State.DETECT
        self.latest_dets = []
        self.last_target = None      # most recent target detection (sticky)
        self.last_target_t = 0.0     # time (s) last_target was seen
        self.depth = None       # depth image in mm (np.uint16)
        self.nav_goal_handle = None
        self.nav_done = False
        self.nav_ok = False
        # DETECT starts in the depth window (explore). Continuous "both" is unreliable on the Astra
        # USB2 (toggling stop/start every frame tends to hang -> /scan dies); replaced by a depth<->color
        # duty-cycle in the DETECT tick.
        self._detect_look = False        # False=depth window (explore), True=color window (look)
        self._detect_phase_t = self.now_sec()
        self.request_mode("depth")
        self.create_timer(0.2, self.tick)   # 5 Hz
        self.get_logger().info("Orchestrator T10 (Hybrid) ready - state=DETECT, target='%s'"
                               % (self.target_class or "<highest confidence>"))

    # Callbacks
    def _on_det(self, msg):
        try:
            self.latest_dets = json.loads(msg.data)
        except (ValueError, TypeError):
            self.latest_dets = []

    def _on_depth(self, msg):
        if msg.encoding != "16UC1":
            return
        self.depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)

    # Helpers
    def request_mode(self, mode):
        self.pub_mode.publish(String(data=mode))

    def set_state(self, s):
        if s != self.state:
            self.state = s
            self.get_logger().info("-> state=%s" % s)
        self.pub_state.publish(String(data=s))

    def pick_target(self):
        """Pick the target detection: by target_class, or highest confidence."""
        cands = self.latest_dets
        if self.target_class:
            cands = [d for d in cands if d.get("class_name") == self.target_class]
        if not cands:
            return None
        return max(cands, key=lambda d: d.get("confidence", 0.0))

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sticky_target(self):
        """Like pick_target but KEEP last-good on a transient loss (<= lost_timeout).
        Needed for the Astra USB2: switching color<->both takes ~250ms, so detections flicker;
        without sticky, LOCATE/APPROACH fall back to DETECT immediately -> thrash, cannot localize."""
        det = self.pick_target()
        now = self.now_sec()
        if det is not None:
            self.last_target = det
            self.last_target_t = now
            return det
        if self.last_target is not None and (now - self.last_target_t) <= self.lost_timeout:
            return self.last_target   # transient loss -> keep the old object
        return None

    def depth_at(self, u, v, win=5):
        """Median depth (m) around (u,v); None if invalid."""
        if self.depth is None:
            return None
        h, w = self.depth.shape
        u, v = int(u), int(v)
        if not (0 <= u < w and 0 <= v < h):
            return None
        patch = self.depth[max(0, v - win):v + win + 1, max(0, u - win):u + win + 1]
        # Filter to the Astra's REAL range: drop close-range noise (<300mm - invalid pixels return
        # small nonzero values, e.g. 40mm making the servo think it arrived -> only turns) and >2000mm (beyond reliable range).
        valid = patch[(patch >= 300) & (patch <= 2000)]
        if valid.size == 0:
            return None
        return float(np.median(valid)) / 1000.0   # mm -> m

    def object_map_pose(self, det):
        """bbox + depth + tf -> (ox, oy) on the map; None if depth/tf missing."""
        u, v = geo.bbox_center(det["bbox"])
        z = self.depth_at(u, v)
        if z is None:
            return None
        X, Y, Z = geo.pixel_to_camera_point(u, v, z, self.fx, self.fy, self.cx, self.cy)
        pt = PointStamped()
        pt.header.frame_id = self.camera_frame
        pt.point.x, pt.point.y, pt.point.z = X, Y, Z
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.camera_frame, rclpy.time.Time())
            m = do_transform_point(pt, tf)
            return (m.point.x, m.point.y)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn("tf camera->map error: %s" % e, throttle_duration_sec=2.0)
            return None

    def robot_xy(self):
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return (tf.transform.translation.x, tf.transform.translation.y)
        except Exception:
            return None

    # State machine
    def tick(self):
        self.pub_state.publish(String(data=self.state))
        if self.state == State.DETECT:
            # Duty-cycle: depth window (keep /scan -> WFD/SLAM/NAV2 explore) <-> color window
            # (YOLO looks for the object). Switching mode costs ~1.5s so keep the color window long enough.
            elapsed = self.now_sec() - self._detect_phase_t
            if not self._detect_look:
                self.request_mode("depth")   # explore
                if elapsed >= self.detect_scan_sec:
                    self._detect_look = True
                    self._detect_phase_t = self.now_sec()
                    self.latest_dets = []     # drop old detections, only accept new color frames
                    self.get_logger().info("DETECT: blip to color to look for the object...", throttle_duration_sec=10.0)
            else:
                self.request_mode("color")   # look for the object
                if self.pick_target() is not None:
                    self.get_logger().info("Object detected -> LOCATE")
                    self.set_state(State.LOCATE)
                elif elapsed >= self.detect_look_sec:
                    self._detect_look = False
                    self._detect_phase_t = self.now_sec()   # color window over -> back to exploring

        elif self.state == State.LOCATE:
            self.request_mode("both")   # need color (bbox) + depth (Z)
            det = self.sticky_target()  # tolerate a transient loss during a mode switch
            if det is None:
                self.set_state(State.DETECT); return
            # Object close enough (depth Z <= switch_dist) -> SKIP NAV2, servo straight to it.
            # NAV2 for a short 0.4m hop is very fragile (the costmap lacks /scan when DETECT used
            # color, the goal lands in unmapped space -> abort -> loop DETECT<->NAVIGATE). The servo
            # only needs bbox+depth (available in both mode), no map/scan/plan -> much more robust.
            u, v = geo.bbox_center(det["bbox"])
            z_near = self.depth_at(u, v)
            if z_near is not None and z_near <= self.switch_dist:
                self.get_logger().info(
                    "Object close (z=%.2fm <= %.2fm) -> APPROACH (servo, skip NAV2)"
                    % (z_near, self.switch_dist))
                self.set_state(State.APPROACH); return
            obj = self.object_map_pose(det)
            rob = self.robot_xy()
            if obj is None or rob is None:
                return   # wait for depth/tf
            gx, gy, yaw = geo.standoff_goal(rob, obj, self.standoff)
            self.send_nav_goal(gx, gy, yaw)
            self.get_logger().info("Object @map (%.2f,%.2f) -> goal (%.2f,%.2f) yaw=%.2f" % (obj[0], obj[1], gx, gy, yaw))
            self.set_state(State.NAVIGATE)

        elif self.state == State.NAVIGATE:
            self.request_mode("depth")   # NAV2 needs /scan
            rob = self.robot_xy()
            det = self.pick_target()
            # close enough (per nav_done) -> switch to servo; or nav abort -> detect again
            if self.nav_done:
                if self.nav_ok:
                    self.get_logger().info("NAV2 reached the area -> APPROACH (servo)")
                    self.set_state(State.APPROACH)
                else:
                    self.get_logger().warn("NAV2 failed -> DETECT again")
                    self.set_state(State.DETECT)

        elif self.state == State.APPROACH:
            self.request_mode("both")   # servo needs bbox + depth
            det = self.sticky_target()  # keep the object when detections flicker during a mode switch
            if det is None:
                self.pub_cmd.publish(Twist())   # real loss (beyond lost_timeout) -> stop, wait
                return
            u, v = geo.bbox_center(det["bbox"])
            z = self.depth_at(u, v)
            vx, vy, arrived = geo.servo_cmd(u, self.img_w, z, self.target_z)
            # The Astra is blind up close (<~0.4-0.6m) -> depth=0 -> z=None. When CENTERED but z=None
            # it means the object is too close (in the blind zone), NOT a loss (det still present). Treat as
            # arrived, so the robot does not keep inching forward/sideways right in front of the object
            # because the servo cannot reach target_z inside the blind zone.
            centered = abs((u - self.img_w / 2.0) / (self.img_w / 2.0)) < 0.12
            if z is None and centered:
                arrived = True; vx = 0.0; vy = 0.0
            cmd = Twist(); cmd.linear.x = vx; cmd.linear.y = vy   # holonomic: advance/strafe, NO turning
            self.pub_cmd.publish(cmd)
            if arrived:
                self.pub_cmd.publish(Twist())
                self.get_logger().info("ARRIVED at the object (~%.2fm away). Ready to grasp/report." % self.target_z)
                self.set_state(State.ARRIVED)

        elif self.state == State.ARRIVED:
            self.pub_cmd.publish(Twist())   # hold stop

    # NAV2 action
    def send_nav_goal(self, x, y, yaw):
        self.nav_done = False; self.nav_ok = False
        if not self.nav_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("NavigateToPose server not ready -> NAV failed")
            self.nav_done = True; self.nav_ok = False
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.now_msg()
        goal.pose.pose.position.x = x; goal.pose.pose.position.y = y
        qx, qy, qz, qw = geo.yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx; goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz; goal.pose.pose.orientation.w = qw
        fut = self.nav_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_resp)

    def now_msg(self):
        return self.get_clock().now().to_msg()

    def _goal_resp(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().warn("NAV2 rejected the goal")
            self.nav_done = True; self.nav_ok = False
            return
        self.nav_goal_handle = gh
        gh.get_result_async().add_done_callback(self._goal_result)

    def _goal_result(self, fut):
        from action_msgs.msg import GoalStatus
        status = fut.result().status
        self.nav_done = True
        self.nav_ok = (status == GoalStatus.STATUS_SUCCEEDED)


def main():
    rclpy.init()
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub_cmd.publish(Twist())   # stop the base on shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
