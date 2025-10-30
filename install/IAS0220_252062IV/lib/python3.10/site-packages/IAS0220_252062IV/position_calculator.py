import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Vector3, Pose


class PosCalculator(Node):
    def __init__(self):
        super().__init__('pos_calculator')
        self.vel_sub = self.create_subscription(
            Vector3, 'velocity', self.velocity_callback, 10)

        self.nam_t_sub = self.create_subscription(
            String, '/name_and_time', self.name_callback, 10)

        self.position = Pose()
        self.time_step = 0.5

    def name_callback(self, msg: String):
        try:
            student_id, timestamp = msg.data.strip().split(',')
            self.get_logger().info(
                f"Student {student_id} contacted, told me time is: {timestamp}"
            )
        except ValueError:
            self.get_logger().warn("Received malformed name_and_time message.")

    def velocity_callback(self, msg: Vector3):
        # Compute displacement = velocity * time
        dx = msg.x * self.time_step
        dy = msg.y * self.time_step

        # Update position
        self.position.position.x += dx
        self.position.position.y += dy
        self.position.position.z = 0.0  # Always 0

        # Log new position
        self.get_logger().info("The new position of the walker is :\n"
                               f"x = {self.position.position.x}\n"
                               f"y = {self.position.position.y}\n"
                               f"z = {self.position.position.z}")


def main(args=None):
    rclpy.init(args=args)
    node = PosCalculator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
