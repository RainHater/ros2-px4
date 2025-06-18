import rclpy
from rclpy.node import Node
import cv2
import numpy as np

class RectDetectorNode(Node):
    def __init__(self):
        super().__init__('rect_detect_node')
        self.declare_parameter('image_path', 'test1.png')
        image_path = self.get_parameter('image_path').get_parameter_value().string_value
        self.get_logger().info(f'Image path: {image_path}')
        self.get_rect(image_path)

    def get_rect(self, image_path):
        image = cv2.imread(image_path)
        if image is None:
            self.get_logger().error(f"Failed to load image: {image_path}")
            return
        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_br = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)[1]
        kernel = np.ones((13, 13), np.uint8)
        img_br = cv2.morphologyEx(img_br, cv2.MORPH_CLOSE, kernel)
        contours = cv2.findContours(img_br, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        if contours:
            max_n = sorted(contours, key=lambda x: cv2.contourArea(x), reverse=True)[0]
            x, y, w, h = cv2.boundingRect(max_n)
            self.get_logger().info(f"Bounding rect: x={x}, y={y}, w={w}, h={h}")
            cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
        else:
            self.get_logger().warn("No contours found")
        
        cv2.imshow("QR Code Detection", image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = RectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
