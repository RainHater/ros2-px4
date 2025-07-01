import rclpy
from rclpy.node import Node
from common_msgs.msg import PidDebug
import matplotlib.pyplot as plt
import collections

class PidViewerNode(Node):
    def __init__(self):
        super().__init__('pid_viewer_node')

        self.subscription = self.create_subscription(
            PidDebug,
            '/debug/pid_viewer',
            self.listener_callback,
            10)

        self.window_size = 100
        self.time_data = collections.deque(maxlen=self.window_size)
        self.pixel_dist_data = collections.deque(maxlen=self.window_size)

        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], label='pixel_dist')
        self.ax.axhline(0, color='r', linestyle='--', label='target=0')
        self.ax.legend()
        self.ax.set_ylim(-5, 5)  # 根据实际pixel_dist范围调整

    def listener_callback(self, msg):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.time_data.append(t)
        self.pixel_dist_data.append(msg.pixel_dist)

        self.line.set_xdata(self.time_data)
        self.line.set_ydata(self.pixel_dist_data)

        self.ax.relim()
        self.ax.autoscale_view()

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = PidViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
