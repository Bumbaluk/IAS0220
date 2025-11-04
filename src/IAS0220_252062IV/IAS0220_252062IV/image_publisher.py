import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        # 2 HZ
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # initialize cvbridge
        self.bridge = CvBridge()

        package_name = 'IAS0220_252062IV'
        package_path = get_package_share_directory(package_name)
        self.image_folder = os.path.join(package_path, 'data', 'images')

        # load pictures
        self.image_files = [
            os.path.join(self.image_folder, f)
            for f in os.listdir(self.image_folder)
            if f.lower().endswith('.png')
        ]
        self.current_index = 0

    def timer_callback(self):
        if not self.image_files:
            return

        image_path = self.image_files[self.current_index]
        cv_image = cv2.imread(image_path)

        msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        msg.header.frame_id = "camera"

        self.publisher_.publish(msg)

        self.current_index = (self.current_index + 1) % len(self.image_files)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
