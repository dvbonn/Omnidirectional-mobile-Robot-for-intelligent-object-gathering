"""
fake_odom - a FAKE base for SLAM/Nav testing before the real base exists (replaces base_bridge).
================================================================================================
Publishes /odom + tf odom->base_link. Stationary by default (enough to smoke-test the
slam_toolbox wiring: it produces /map + map->odom). It can "drive" via the vx/vtheta params
to simulate motion (pose integration) when you want a richer map.

Does NOT replace the real base - it is only a stand-in to validate config/launch sim-free.
When the base exists: use base_bridge instead of this node (same /odom + tf, same frames).
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


class FakeOdom(Node):
    def __init__(self):
        super().__init__("fake_odom")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("vx", 0.0)        # m/s - 0 = stationary
        self.declare_parameter("vy", 0.0)
        self.declare_parameter("vtheta", 0.0)    # rad/s

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.vx = float(self.get_parameter("vx").value)
        self.vy = float(self.get_parameter("vy").value)
        self.vth = float(self.get_parameter("vtheta").value)
        rate = float(self.get_parameter("rate_hz").value)

        self.x = self.y = self.th = 0.0
        self.dt = 1.0 / rate
        self.pub = self.create_publisher(Odometry, "/odom", 10)
        self.tfb = TransformBroadcaster(self)
        self.create_timer(self.dt, self.tick)
        self.get_logger().info(
            "fake_odom: %s->%s @ %.0fHz, vx=%.2f vy=%.2f vtheta=%.2f %s"
            % (self.odom_frame, self.base_frame, rate, self.vx, self.vy, self.vth,
               "(stationary)" if (self.vx == self.vy == self.vth == 0.0) else "(simulated drive)")
        )

    def tick(self):
        # integrate pose from the body twist (simple, enough for testing)
        self.x += (self.vx * math.cos(self.th) - self.vy * math.sin(self.th)) * self.dt
        self.y += (self.vx * math.sin(self.th) + self.vy * math.cos(self.th)) * self.dt
        self.th += self.vth * self.dt
        qx, qy, qz, qw = yaw_to_quat(self.th)
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.vth
        self.pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tfb.sendTransform(t)


def main():
    rclpy.init()
    node = FakeOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
