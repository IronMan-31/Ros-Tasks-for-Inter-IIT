import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class MultiCameraArucoPublisher(Node):
    def __init__(self):
        super().__init__('multi_camera_aruco_publisher')

        # Marker size in meters
        self.marker_length = 0.5

        # Camera intrinsics
        self.camera_matrix = np.array([[381.4631663987062, 0, 320.5],
                                       [0, 381.4631663987062, 240.5],
                                       [0, 0, 1]])
        self.dist_coeffs = np.zeros((5,))

        self.bridge = CvBridge()

        # Subscribe to three cameras
        self.sub_front = self.create_subscription(Image, '/camera1/image_raw', self.front_callback, 10)
        self.sub_right = self.create_subscription(Image, '/camera2/image_raw', self.left_callback, 10)
        self.sub_left  = self.create_subscription(Image, '/camera3/image_raw', self.right_callback, 10)

        # Publisher for fused marker position
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_pose', 10)

        # Store detected positions
        self.positions = []

        # Timer to fuse and publish
        self.create_timer(0.1, self.timer_callback)


    def rotate_tvec(self,tvec, yaw_deg):
        yaw = np.deg2rad(yaw_deg)
        Rz = np.array([
            [np.cos(yaw), -np.sin(yaw), 0],
            [np.sin(yaw),  np.cos(yaw), 0],
            [0,            0,           1]
        ])
        # tvec from camera frame (z-forward) to robot frame (x-forward)
        # First, swap axes to go from OpenCV camera (z-forward) → robot-like (x-forward)
        cam_to_robot_like = np.array([[0, 0, 1],
                                    [-1, 0, 0],
                                    [0, -1, 0]])
        return Rz @ (cam_to_robot_like @ tvec)


    def detect_marker(self, cv_image):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(cv_image, aruco_dict, parameters=parameters)
        if ids is not None and len(ids) > 0:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length,
                                                              self.camera_matrix, self.dist_coeffs)
            tvec = tvecs[0][0]  # First detected marker
            return tvec
        return None

    def fuse_positions(self):
        """Return the closest marker in front of the robot"""
        if len(self.positions) == 0:
            return None
        positions = np.array(self.positions)
        best_idx = np.argmin(positions[:, 0])  # x-forward distance
        return positions[best_idx]

    def front_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        tvec = self.detect_marker(cv_image)
        if tvec is not None:
            tvec_robot = self.rotate_tvec(tvec, 0)  # front cam aligned
            self.positions.append(tvec_robot)

    def right_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        tvec = self.detect_marker(cv_image)
        if tvec is not None:
            tvec_robot = self.rotate_tvec(tvec, 90)  # right cam rotated -90° about z
            self.positions.append(tvec_robot)

    def left_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        tvec = self.detect_marker(cv_image)
        if tvec is not None:
            tvec_robot = self.rotate_tvec(tvec, -90)  # left cam rotated +90° about z
            self.positions.append(tvec_robot)


    def timer_callback(self):
        marker_pos = self.fuse_positions()
        self.positions = []  # reset for next iteration

        if marker_pos is not None:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = float(marker_pos[0])
            pose_msg.pose.position.y = float(marker_pos[1])
            pose_msg.pose.position.z = float(marker_pos[2]) 
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraArucoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
