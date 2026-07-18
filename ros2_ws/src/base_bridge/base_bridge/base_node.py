"""
base_node - ROS2 <-> ESP32 mecanum base bridge.
===============================================
  /cmd_vel (geometry_msgs/Twist)  --encode-->  SET_ROBOT_VELOCITY (base runs IK+PI)
  ROBOT_STATE telemetry (EKF pose) --decode-->  /odom (nav_msgs/Odometry)
                                              + tf odom->base_link
  Watchdog: /cmd_vel silent too long or shutdown -> send STOP_ROBOT.

Abstract transport (transport.py): TCP :2004 (default) or CAN (stub).
Byte-format logic is in protocol.py (unit-tested).

NOTE: not yet run against the real base (base not connected). When the base is ready:
    ros2 run base_bridge base_node --ros-args -p host:=<ip-or-mdns>
Until then use the same /cmd_vel that NAV2/visual-servoing publishes - no change to this node.
"""
import math
import threading

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from base_bridge import protocol as P
from base_bridge.transport import make_transport


def yaw_to_quat(yaw: float):
    """Quaternion (x,y,z,w) for a yaw rotation about the Z axis (2D mecanum base)."""
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class BaseNode(Node):
    def __init__(self):
        super().__init__("base_bridge")

        # Parameters
        self.declare_parameter("transport", "tcp")          # 'tcp' | 'can'
        self.declare_parameter("host", "mecanumbase.local")  # ESP32 IP/mDNS
        self.declare_parameter("port", 2004)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("cmd_timeout", 0.5)           # s: silence -> STOP
        self.declare_parameter("publish_tf", True)           # SLAM needs odom->base_link
        self.declare_parameter("reconnect_period", 2.0)      # s: retry connect

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)

        # Pub/Sub
        self.pub_odom = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.sub_cmd = self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        # Transport
        self.transport = make_transport(
            self.get_parameter("transport").value,
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
        )
        self._parser = P.FrameParser()
        self._tx_lock = threading.Lock()
        self._last_cmd_time = self.get_clock().now()
        self._sent_stop = False

        # Separate telemetry-read thread (recv blocking-timeout, does not block the ROS executor).
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

        # STOP watchdog when cmd_vel goes silent.
        self.create_timer(0.1, self._watchdog)

        self.get_logger().info(
            "base_bridge: transport=%s host=%s:%s -> /cmd_vel<->SET_ROBOT_VELOCITY, ROBOT_STATE->/odom%s"
            % (self.get_parameter("transport").value, self.get_parameter("host").value,
               self.get_parameter("port").value, " (+tf odom->base_link)" if self.publish_tf else "")
        )

    # /cmd_vel -> base
    def on_cmd_vel(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        self._sent_stop = False
        # Mecanum holonomic: vx (linear.x), vy (linear.y), vtheta (angular.z).
        self._safe_send(P.encode_set_velocity(msg.linear.x, msg.linear.y, msg.angular.z))

    def _watchdog(self):
        """No /cmd_vel for cmd_timeout -> STOP (safe when NAV2 hangs/loses connection)."""
        if self._sent_stop:
            return
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._safe_send(P.encode_stop())
            self._sent_stop = True

    def _safe_send(self, data: bytes):
        if not self.transport.connected:
            return
        try:
            with self._tx_lock:
                self.transport.send(data)
        except (OSError, ConnectionError) as e:
            self.get_logger().warn("Send to base failed: %s" % e)
            self._drop_connection()

    # base -> /odom + tf
    def _rx_loop(self):
        while self._running:
            if not self.transport.connected:
                self._try_connect()
                continue
            try:
                chunk = self.transport.recv(4096)
            except (OSError, ConnectionError) as e:
                self.get_logger().warn("Read from base failed: %s" % e)
                self._drop_connection()
                continue
            if not chunk:
                continue
            for payload in self._parser.feed(chunk):
                if payload and payload[0] == P.CMD_ROBOT_STATE:
                    try:
                        st = P.decode_robot_state_payload(payload)
                    except ValueError as e:
                        self.get_logger().warn("Corrupt ROBOT_STATE: %s" % e)
                        continue
                    self._publish_odom(st)
                # other telemetry cmds (BNO055/PMW3901...) ignored here.

    def _publish_odom(self, st: P.RobotState):
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quat(st.theta)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = st.x
        odom.pose.pose.position.y = st.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = st.vx
        odom.twist.twist.linear.y = st.vy
        odom.twist.twist.angular.z = st.vtheta
        self.pub_odom.publish(odom)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = st.x
            t.transform.translation.y = st.y
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # Connect/disconnect
    def _try_connect(self):
        try:
            self.transport.connect()
            self._parser = P.FrameParser()  # reset the frame buffer
            self.get_logger().info("Connected to the base.")
        except (OSError, ConnectionError, NotImplementedError) as e:
            self.get_logger().warn("Could not connect to base (%s) - retrying..." % e, throttle_duration_sec=5.0)
            self._sleep(float(self.get_parameter("reconnect_period").value))

    def _drop_connection(self):
        try:
            self.transport.close()
        except Exception:
            pass

    def _sleep(self, sec: float):
        # short sleep that does not block the executor (rx_loop is its own thread).
        import time
        time.sleep(sec)

    def destroy_node(self):
        self._running = False
        # Try to send one last STOP before closing (safety).
        try:
            if self.transport.connected:
                self.transport.send(P.encode_stop())
        except Exception:
            pass
        try:
            self.transport.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = BaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
