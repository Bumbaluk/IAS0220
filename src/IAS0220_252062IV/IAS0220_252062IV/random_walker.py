import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import random
import time


class WalkPublisher(Node):
    def __init__(self):
        super().__init__('random_walker')
        self.publisher_ = self.create_publisher(Vector3, 'velocity', 10)
        self.time_pub = self.create_publisher(String, '/name_and_time', 10)

        # Timer at 2Hz = every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # My ID
        self.student_id = "IAS0220_252062IV"

    def timer_callback(self):
        # publish velocity
        vel = Vector3(random.choices([-1, 0, 1]), random.choices(-1, 0, 1), 0)
        self.publisher_.publish(vel)

        # publish message with id time and velocity
        current_time = time.time()
        id_time_msg = f"{self.student_id},{current_time:.2f}"
        self.time_pub.publish(id_time_msg)

        # Loggers
        self.get_logger().info(f"My ID and the current time: {id_time_msg}")
        self.get_logger().info(f"Moving with this velocity for 0.5 seconds:\n"
                               f"x: {vel[0]}\ny: {vel[1]}\nz: {vel[2]}")


def main(args=None):
    rclpy.init(args=args)
    node = WalkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
