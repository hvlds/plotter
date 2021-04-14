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

            if sec_diff <= 100:
                self.plot_detections.append(self.all_detections[-1])
                self.times.append(sec_diff)
                self.x_values.append(self.plot_detections[-1].x * 100)
                self.y_values.append(self.plot_detections[-1].y * 100)
                self.z_values.append(self.plot_detections[-1].z * 100)

            elif self.first_plot:
                self.save_csv("infrared_test.csv")
                self.get_logger().info("CSV is ready!")
                # self.save_graph(self.times, self.x_values,
                #                 "Time [s]", "X Position [cm]", "x_pos.png")
                # self.save_graph(self.times, self.y_values,
                #                 "Time [s]", "Y Position [cm]", "y_pos.png")
                # self.save_graph(self.times, self.z_values,
                #                 "Time [s]", "Z Position [cm]", "z_pos.png")
                self.first_plot = False

    def save_csv(self, output_file):
        time_diff = np.array(self.times)
        temp_x = np.array(self.x_values)
        temp_y = np.array(self.y_values)
        temp_z = np.array(self.z_values)
        np.savetxt(output_file, np.array(list(zip(time_diff, temp_x, temp_y, temp_z))),
            delimiter=',', header="time,x,y,z", comments="")

    def save_graph(self, x_values, y_values, x_label, y_label, output_file):
        x = np.array(x_values)
        y = np.array(y_values)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.plot(x, y, ".-")
        plt.savefig(output_file, bbox_inches="tight")
        plt.clf()

    def listener_callback(self, msg):
        stamp = msg.header.stamp
        for marker in msg.detections:
            x_pos = marker.pose.pose.position.x
            y_pos = marker.pose.pose.position.y
            z_pos = marker.pose.pose.position.z

            detection = Detection(stamp, x_pos, y_pos, z_pos)
            self.all_detections.append(detection)
            self.check_detections()


def main(args=None):
    rclpy.init(args=args)

    irmarker_subscriber = IRMarkerSubscriber()

    rclpy.spin(irmarker_subscriber)

    irmarker_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
