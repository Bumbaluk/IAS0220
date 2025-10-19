from rclpy.node import Node
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from encoders_interfaces.msg import Counter
import numpy as np
import transforms3d.euler as euler


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry')

        # Subscribe and publish
        self.subscription = self.create_subscription(
            Counter,
            '/encoders_ticks',
            self.encoder_callback,
            10
        )
        self.publisher = self.create_publisher(Odometry, '/my_odom', 10)

        # Robot constants
        self.wheel_radius = 0.036
        self.wheel_base = 0.34
        self.ticks_per_rev = 508.8  # from encoders node, check...

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.last_ticks = [0, 0]

        # Initialize Odometry message with zeros
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0  # no rotation
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0

        # Publish once at startup
        self.publisher.publish(self.odom_msg)

    def encoder_callback(self, msg):
        # Get tick counts
        left_ticks = msg.count_left
        right_ticks = msg.count_right
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Compute distances
        d_left = left_ticks - self.last_ticks[0]
        d_right = right_ticks - self.last_ticks[1]

        dist_left = (
            2 * np.pi * self.wheel_radius * (d_left / self.ticks_per_rev))
        dist_right = (
            2 * np.pi * self.wheel_radius * (d_right / self.ticks_per_rev))

        # Velocities
        v = (dist_right + dist_left) / (2 * dt)
        omega = (dist_right - dist_left) / (self.wheel_base * dt)

        # Integrate pose
        self.theta += omega * dt
        self.x += v * np.cos(self.theta) * dt
        self.y += v * np.sin(self.theta) * dt

        # Update odometry message
        q = euler.euler2quat(0, 0, self.theta)
        quat = Quaternion()
        quat.x, quat.y, quat.z, quat.w = q[1], q[2], q[3], q[0]

        self.odom_msg.header.stamp = now.to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = quat
        self.odom_msg.twist.twist.linear.x = v
        self.odom_msg.twist.twist.angular.z = omega

        self.publisher.publish(self.odom_msg)

        # Save current state
        self.last_ticks = [left_ticks, right_ticks]
        self.last_time = now


def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
