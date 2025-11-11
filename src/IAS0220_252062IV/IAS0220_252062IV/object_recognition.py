import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectRecognition(Node):
    def __init__(self):
        super().__init__('object_recognition')

        self.bridge = CvBridge()

        # Subscriber to camera feed
        self.subscription = self.create_subscription(
            Image, '/camera1/image_raw',
            self.image_callback, 10)

        self.publisher = self.create_publisher(Image, '/6_2_final_vid', 10)

        # numpy to ...
        # TODO: trial and error!!!
        self.lower_hsv = np.array([0, 120, 70])
        self.upper_hsv = np.array([10, 255, 255])

    def image_callback(self, msg):
        # TODO: implement!
        # WHAT DOES MASK COUNTOUR DO?

        # Convert ROS Image → OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert BGR → HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask for color range
        mask = cv2.inRange(hsv, self.lower_hsv, self.upper_hsv)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the biggest contour
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

            if radius > 10:  # filter out tiny detections
                # Draw a circle around the detected object
                cv2.circle(frame, (int(x), int(y)), int(radius),
                           (0, 255, 0), 3)
                cv2.putText(frame, f"Object at ({int(x)}, {int(y)})",
                            (int(x)-50, int(y)-radius-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # publish
        processed_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(processed_msg)
        pass


# TODO: is rcply importan? FINISH AND CHECK
def main(args=None):
    rclpy.init(args=args)
    node = ObjectRecognition()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object_recognition node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
