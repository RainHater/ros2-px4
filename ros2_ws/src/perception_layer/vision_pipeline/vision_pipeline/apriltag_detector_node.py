#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import apriltag

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge

import tf2_ros
import geometry_msgs.msg
from tf_transformations import quaternion_from_matrix

class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__('apriltag_detector')

        self.bridge = CvBridge()
        self.detector = apriltag.Detector()

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("tag_size", 0.16)  # meter

        self.tag_size = self.get_parameter("tag_size").value

        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value

        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/tag_detections", 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.fx = self.fy = self.cx = self.cy = None
        self.camera_frame = "camera_link"

        self.get_logger().info("AprilTag detector initialized.")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id
        self.get_logger().info("Camera intrinsic parameters received.")
        self.camera_info_sub.destroy()

    def image_callback(self, msg: Image):
        if None in [self.fx, self.fy, self.cx, self.cy]:
            self.get_logger().warn("Waiting for camera info...")
            return

        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

        detections = self.detector.detect(gray)
        if len(detections) == 0:
            return

        for det in detections:
            tag_id = det.tag_id
            corners = det.corners
            object_points = np.array([
                [-self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2, -self.tag_size/2, 0],
                [ self.tag_size/2,  self.tag_size/2, 0],
                [-self.tag_size/2,  self.tag_size/2, 0]
            ])
            image_points = np.array(corners, dtype=np.float32)

            camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])
            dist_coeffs = np.zeros(4)

            success, rvec, tvec = cv2.solvePnP(
                object_points, image_points, camera_matrix, dist_coeffs
            )

            if not success:
                continue

            R, _ = cv2.Rodrigues(rvec)
            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = tvec.flatten()
            q = quaternion_from_matrix(T)

            pose = PoseWithCovarianceStamped()
            pose.header = msg.header
            pose.pose.pose.position.x = tvec[0][0]
            pose.pose.pose.position.y = tvec[1][0]
            pose.pose.pose.position.z = tvec[2][0]
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]

            self.pose_pub.publish(pose)

            # TF broadcast
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.camera_frame
            t.child_frame_id = f"tag_{tag_id}"
            t.transform.translation.x = tvec[0][0]
            t.transform.translation.y = tvec[1][0]
            t.transform.translation.z = tvec[2][0]
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
