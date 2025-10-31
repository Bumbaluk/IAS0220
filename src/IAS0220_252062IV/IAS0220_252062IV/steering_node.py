import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from transforms3d.euler import quat2euler


class SteeringNode(Node):
    """node for controlling robot based on sensor data"""
    def __init__(self):
        super().__init__('steering_node')

        # robot moves only if obstacle is CLOSER than this
        self.distance_threshold = 1.0

        # setting max speeds
        self.max_linear_speed = 1.0
        self.max_angular_speed = 1.0

        self.publish_frequency = 10.0  # DECIDE LATER, OBSERVE

        self.roll = 0.0
        self.pitch = 0.0
        self.distance = 0.0

        # Publisher, Subscribers
        self.cmd_pub = self.create_publisher(Twist,
                                             '/cmd_vel',
                                             10)
        self.imu_sub = self.create_subscription(Imu, '/imu',
                                                self.imu_callback, 10)
        self.dist_sub = self.create_subscription(Float32, '/distance',
                                                 self.distance_callback, 10)

        # Timer
        self.timer = self.create_timer(1.0 / self.publish_frequency,
                                       self.publish_velocity)

    def imu_callback(self, msg: Imu):
        """Convert IMU quaternion to RPY"""
        q = msg.orientation
        quat = [q.w, q.x, q.y, q.z]
        roll, pitch, yaw = quat2euler(quat)

        self.roll = roll
        self.pitch = pitch

    def distance_callback(self, msg: Float32):
        """Update distance from sensor."""
        self.distance = msg.data

    def publish_velocity(self):
        """Compute and publish Twist message based on IMU and distance data"""
        twist = Twist()
        # the robot completely stops(doesnt rotate) if nothing in front.good?
        if self.distance > self.distance_threshold:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            # Positive pitch: forward, negative: backward
            twist.linear.x = max(-self.max_linear_speed,
                                 min(self.max_linear_speed, self.pitch * 2))

            # Positive roll: turn left, negative: right
            twist.angular.z = max(-self.max_angular_speed,
                                  min(self.max_angular_speed, self.roll * 2))

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
