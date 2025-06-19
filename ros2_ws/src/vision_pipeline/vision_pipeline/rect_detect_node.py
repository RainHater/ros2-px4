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

class PID:
    def __init__(self, Kp, Ki, Kd, dt):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, error):
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt
        out = self.Kp*error + self.Ki*self.integral + self.Kd*derivative
        self.prev_error = error
        return out

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
        dt = 1/30.0
        self.pid_yaw = PID(0.5, 0.0, 0.1, dt)
        self.pid_pitch = PID(0.3, 0.0, 0.05, dt)
        self.fx = 0.0
        self.fy = 0.0
        
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
            x_t = center[0]
            y_t = center[1]
            x_c = width//2
            y_c = height//2
            target_center_int = (int(x_t), int(y_t))        
            current_center_int = (int(x_c), int(y_c))  
            distance = math.hypot(x_t - x_c, y_t - y_c)
            cv2.circle(cv_image, target_center_int, 5, (0,255,0), -1) 
            cv2.circle(cv_image, current_center_int, 5, (255,0,0), -1) 
            cv2.polylines(cv_image, [box], True, (0,0,255), 2)
            
            self.get_logger().info(f"box: {box}, center: {center}, distance: {distance}")
            self.get_logger().info(f"height: {height}, width: {width}")

            self.servo_to_target(x_t, y_t, x_c, y_c, self.fx, self.fy)
        else:
            self.get_logger().warn("No contours found")
        return cv_image
    
    def servo_to_target(self, u_t, v_t, u_c, v_c, fx, fy):
        du = u_t - u_c
        dv = v_t - v_c
        # 简化：直接用像素 / 焦距 近似角度
        err_yaw = du / fx
        err_pitch = dv / fy

        # PID 计算输出
        cmd_yaw_rate   = self.pid_yaw.update(err_yaw)
        cmd_forward_vel = self.pid_pitch.update(err_pitch)

        twist = Twist()
        # 假设：正 pitch 误差（目标在图像上方）→ 应当后退；根据你的机体坐标系选正负
        twist.linear.x  = -cmd_forward_vel  
        twist.angular.z = -cmd_yaw_rate

        self.get_logger().info(f"twist: {twist}")
        # self.pub.publish(twist)

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
