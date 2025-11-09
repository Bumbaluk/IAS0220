import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

# Constants
CHESSBOARD_SIZE = (7, 6)
SQUARE_SIZE = 0.108
NUM_IMAGES_TO_CALIBRATE = 37


class CameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')

        # Subscribers and Publishers
        self.image_sub = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.processed_pub = self.create_publisher(Image,
                                                   '/image_processed', 10)
        self.caminfo_pub = self.create_publisher(CameraInfo,
                                                 '/camera_info', 10)

        self.bridge = CvBridge()

        # calibration state
        self.state = "COLLECTING"

        # Calibration data
        self.objpoints = []  # 3D points
        self.imgpoints = []  # 2D points
        self.images_collected = 0
        self.image_size = None
        self.camera_matrix = None
        self.dist_coeffs = None

        # Prepare object points for chessboard
        self.objp = np.zeros((CHESSBOARD_SIZE[1]*CHESSBOARD_SIZE[0], 3),
                             np.float32)
        self.objp[:, :2] = (np.mgrid[0:CHESSBOARD_SIZE[0],
                            0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
                            * SQUARE_SIZE)

        # Termination criteria for cornerSubPix
        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    def publish_default_camera_info(self):
        """
        Publish a default or calibrated CameraInfo continuously.
        """
        cam_info_msg = CameraInfo()
        cam_info_msg.header.stamp = self.get_clock().now().to_msg()
        cam_info_msg.header.frame_id = "camera"

        if self.camera_matrix is not None and self.dist_coeffs is not None:
            # Use calibrated values
            cam_info_msg.height = self.image_size[1]
            cam_info_msg.width = self.image_size[0]
            cam_info_msg.distortion_model = "plumb_bob"
            cam_info_msg.d = self.dist_coeffs.flatten().tolist()
            cam_info_msg.k = self.camera_matrix.flatten().tolist()
            cam_info_msg.r = np.eye(3).flatten().tolist()
            P = np.zeros((3, 4))
            P[:3, :3] = self.camera_matrix
            cam_info_msg.p = P.flatten().tolist()
        else:
            # Default values before calibration
            cam_info_msg.height = 1080
            cam_info_msg.width = 1920
            cam_info_msg.distortion_model = "plumb_bob"
            cam_info_msg.d = [0, 0, 0, 0, 0]
            cam_info_msg.k = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            cam_info_msg.r = [1, 0, 0, 0, 1, 0, 0, 0, 1]
            cam_info_msg.p = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]

        self.caminfo_pub.publish(cam_info_msg)

    def image_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        current_time = self.get_clock().now().to_msg()

        if self.state == "COLLECTING":
            ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE,
                                                     None)
            if ret:
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11),
                                            (-1, -1), self.criteria)
                self.objpoints.append(self.objp)
                self.imgpoints.append(corners2)
                self.images_collected += 1
                self.image_size = gray.shape[::-1]
                cv_image = cv2.drawChessboardCorners(cv_image,
                                                     CHESSBOARD_SIZE,
                                                     corners2, ret)

            processed_msg = self.bridge.cv2_to_imgmsg(cv_image,
                                                      encoding='bgr8')
            processed_msg.header.stamp = current_time
            processed_msg.header.frame_id = "camera"
            self.processed_pub.publish(processed_msg)

            if self.images_collected >= NUM_IMAGES_TO_CALIBRATE:
                self.state = "CALIBRATING"

        elif self.state == "CALIBRATING":
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                self.objpoints, self.imgpoints, gray.shape[::-1], None, None
            )
            self.camera_matrix = mtx
            self.dist_coeffs = dist
            self.state = "PUBLISH_CAMERA_INFO"


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
