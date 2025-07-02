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

        self.time_data = []
        self.pixel_dist_data = []
        self.angle_x_data = []
        self.angle_y_data = []

        plt.ion()
        self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

        # pixel_dist plot
        self.line_pixel, = self.axs[0].plot([], [], label='pixel_dist')
        self.axs[0].axhline(0, color='r', linestyle='--', label='target=0')
        self.axs[0].set_ylabel("Pixel Dist")
        self.axs[0].legend()
        self.axs[0].grid()

        # angle_x plot
        self.line_ax, = self.axs[1].plot([], [], label='angle_x')
        self.axs[1].axhline(0, color='r', linestyle='--')
        self.axs[1].set_ylabel("Angle X (deg)")
        self.axs[1].legend()
        self.axs[1].grid()

        # angle_y plot
        self.line_ay, = self.axs[2].plot([], [], label='angle_y')
        self.axs[2].axhline(0, color='r', linestyle='--')
        self.axs[2].set_ylabel("Angle Y (deg)")
        self.axs[2].set_xlabel("Time (us)")  # timestamp 是 uint64，单位μs
        self.axs[2].legend()
        self.axs[2].grid()

    def listener_callback(self, msg):
        t = msg.timestamp
        self.time_data.append(t)
        self.pixel_dist_data.append(msg.pixel_dist)
        self.angle_x_data.append(msg.angle_x)
        self.angle_y_data.append(msg.angle_y)

        self.line_pixel.set_xdata(self.time_data)
        self.line_pixel.set_ydata(self.pixel_dist_data)

        self.line_ax.set_xdata(self.time_data)
        self.line_ax.set_ydata(self.angle_x_data)

        self.line_ay.set_xdata(self.time_data)
        self.line_ay.set_ydata(self.angle_y_data)

        for ax in self.axs:
            ax.relim()
            ax.autoscale_view()

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = PidViewerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
