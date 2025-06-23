#!/usr/bin/env python3
import rclpy
import cv2
import math
import numpy as np
import tf_transformations

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class RectDetectorNode(Node):
    def __init__(self):
        super().__init__('rect_detect_node')
        self.bridge = CvBridge()
        self.image_raw_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_raw_callback,
            10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self._camera_info_callback,
            10)
        self.fx = 0
        self.fy = 0
        
    def get_rect(self, cv_image):
        height, width = cv_image.shape[:2]
        img_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        img_br = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)[1]
        kernel = np.ones((13, 13), np.uint8)
        img_br = cv2.morphologyEx(img_br, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(img_br, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if contours:
            max_n = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[0]
            rect = cv2.minAreaRect(max_n)
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            x_t, y_t = rect[0]
            x_c = width/2.0
            y_c = height/2.0
            fx = self.fx
            fy = self.fy

            #水平像素偏移（右为正）
            dx = x_t - x_c  
            #垂直像素偏移（下为正） 
            dy = y_t - y_c  
            #像素空间的欧氏距离 
            pixel_dist = math.hypot(dx, dy)  
            angle_x = math.degrees(math.atan2(dx, fx))
            angle_y = math.degrees(math.atan2(dy, fy))
            angle = math.degrees(math.atan2(dy/fy, dx/fx))

            cv2.circle(cv_image, (int(x_t), int(y_t)), 5, (0,255,0), -1) 
            cv2.circle(cv_image, (int(x_c), int(y_c)), 5, (255,0,0), -1) 
            cv2.polylines(cv_image, [box], True, (0,0,255), 2)
            
            self.get_logger().info(f"pixel_dist: {pixel_dist}, angle_x: {angle_x}, angle_y: {angle_y}, angle: {angle}")
            # self.get_logger().info(f"box: {box}, center: {rect[0]}")
            # self.get_logger().info(f"height: {height}, width: {width}")
        else:
            self.get_logger().warn("No contours found")
        return cv_image

    def _image_raw_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = self.get_rect(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)
    
    def _camera_info_callback(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]

        self.get_logger().info(f'fx={self.fx:.2f}, fy={self.fy:.2f}')
        self.destroy_subscription(self.camera_info_sub)
    
def main():
    rclpy.init()
    node = RectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
