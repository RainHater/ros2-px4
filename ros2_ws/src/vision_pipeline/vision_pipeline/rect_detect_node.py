#!/usr/bin/env python3
import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class RectDetectorNode(Node):
    def __init__(self):
        super().__init__('rect_detect_node')
        self.image_raw_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_raw_callback,
            10)
        self.bridge = CvBridge()

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
            center = rect[0]
            center_int = (int(center[0]), int(center[1]))
            cv2.circle(cv_image, center_int, 5, (0,255,0), -1) 
            cv2.polylines(cv_image, [box], True, (0,0,255), 2)
            self.get_logger().info(f"box: {box}, center: {center}")
            self.get_logger().info(f"height: {height}, width: {width}")
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


def main():
    rclpy.init()
    node = RectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
