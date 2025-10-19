import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from encoders_interfaces.msg import Counter
import numpy as np
import transforms3d.euler as euler


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry')
        self.subscription = self.create_subscription(
            Counter, '/encoders_ticks', self.encoder_callback, 10)
        self.publisher = self.create_publisher(Odometry, '/my_odom', 10)

        # Robot constants
        self.wheel_radius = 0.036
        self.wheel_base = 0.34
        self.ticks_per_rev = 508.8

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_ticks = None
        self.last_time = self.get_clock().now()

        # Initialize odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'
        self.odom_msg.pose.pose.orientation.w = 1.0

    def encoder_callback(self, msg: Counter):
        left_ticks = msg.count_left
        right_ticks = msg.count_right
        now = self.get_clock().now()

        if self.last_ticks is None:
            self.last_ticks = [left_ticks, right_ticks]
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # delta ticks
        delta_left = left_ticks - self.last_ticks[0]
        delta_right = right_ticks - self.last_ticks[1]

        if delta_left > self.ticks_per_rev / 2:
            delta_left -= self.ticks_per_rev
        elif delta_left < -self.ticks_per_rev / 2:
            delta_left += self.ticks_per_rev

        if delta_right > self.ticks_per_rev / 2:
            delta_right -= self.ticks_per_rev
        elif delta_right < -self.ticks_per_rev / 2:
            delta_right += self.ticks_per_rev

        # Convert ticks to meters THE - IS SUPER IMPORTANT
        d_left = (
            2 * np.pi * self.wheel_radius
              * (-delta_left) / self.ticks_per_rev)   # IMPORTANT -left
        d_right = (
            2 * np.pi * self.wheel_radius *
            delta_right / self.ticks_per_rev)

        # motion increments
        dl = (d_left + d_right) / 2.0       # forward displacement
        dtheta = (d_right - d_left) / self.wheel_base  # rotation

        dx = dl * np.cos(self.theta + dtheta/2)
        dy = dl * np.sin(self.theta + dtheta/2)

        self.x += dx
        self.y += dy
        self.theta += dtheta

        self.theta = (self.theta + np.pi) % (2*np.pi) - np.pi

        quat = euler.euler2quat(0, 0, self.theta)

        # Update odometry message
        self.odom_msg.header.stamp = now.to_msg()
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        self.odom_msg.pose.pose.orientation = Quaternion(
            x=quat[1],
            y=quat[2],
            z=quat[3],
            w=quat[0]
        )

        # Compute twist in base_link frame
        self.odom_msg.twist.twist.linear.x = dl / dt
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.angular.z = dtheta / dt

        # Publish odometry message
        self.publisher.publish(self.odom_msg)

        # Save state
        self.last_ticks = [left_ticks, right_ticks]
        self.last_time = now


def main():
    rclpy.init()
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
