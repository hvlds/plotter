import rclpy
from rclpy.node import Node

from irtag_msgs.msg import IRTagArray
import matplotlib.pyplot as plt 
import numpy as np

class Detection:
    def __init__(self, stamp, x, y, z):
        self.stamp = stamp
        self.x = x
        self.y = y
        self.z = z


class IRMarkerSubscriber(Node):
    def __init__(self):
        super().__init__('irmarker_subscriber')
        self.subscription = self.create_subscription(
            IRTagArray,
            '/irtracking/tag_detections',
            self.listener_callback,
            10)
        self.subscription
        self.all_detections = list()
        self.plot_detections = list()
        self.first_plot = True

        # Plot values
        self.times = list()
        self.x_values = list()
        self.z_values = list()
        self.y_values = list()

    def check_detections(self):
        if self.all_detections:
            first_stamp = self.all_detections[0].stamp
            last_stamp = self.all_detections[-1].stamp
            
            first_sec = first_stamp.sec + (first_stamp.nanosec / 10e9)
            last_sec = last_stamp.sec + (last_stamp.nanosec / 10e9)
            sec_diff = last_sec - first_sec
            
            if sec_diff <= 10:
                self.plot_detections.append(self.all_detections[-1])
                self.times.append(sec_diff)
                self.x_values.append(self.plot_detections[-1].x)
                self.y_values.append(self.plot_detections[-1].y)
                self.z_values.append(self.plot_detections[-1].z)

            elif self.first_plot:
                x = np.array(self.times)
                y = np.array(self.z_values)
                plt.title("IRMarker") 
                plt.xlabel("Time") 
                plt.ylabel("Z Position") 
                plt.plot(x,y) 
                plt.savefig("foo.png")
                self.first_plot = False


    def listener_callback(self, msg):
        stamp = msg.header.stamp
        for marker in msg.detections:
            x_pos = marker.pose.pose.position.x
            y_pos = marker.pose.pose.position.y
            z_pos = marker.pose.pose.position.z

            detection = Detection(stamp, x_pos, y_pos, z_pos)
            self.all_detections.append(detection)
            self.check_detections()

            # self.get_logger().info(str(marker.pose.pose.position.z))


def main(args=None):
    rclpy.init(args=args)

    irmarker_subscriber = IRMarkerSubscriber()

    rclpy.spin(irmarker_subscriber)

    irmarker_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
