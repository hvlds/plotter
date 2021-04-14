import rclpy
from rclpy.node import Node

from apriltag_msgs.msg import AprilTagDetectionArray
import matplotlib.pyplot as plt
import numpy as np

from .detection import Detection
from .utils import save_graph


class ApriltagSubscriber(Node):
    def __init__(self):
        super().__init__('apriltag_subscriber')
        self.subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
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

            if sec_diff <= 60:
                self.plot_detections.append(self.all_detections[-1])
                self.times.append(sec_diff)
                self.x_values.append(self.plot_detections[-1].x * 100)
                self.y_values.append(self.plot_detections[-1].y * 100)
                self.z_values.append(self.plot_detections[-1].z * 100)

            elif self.first_plot:
                self.save_csv("apriltag_test.csv")
                self.get_logger().info("CSV is ready!")
                self.first_plot = False

    def save_csv(self, output_file):
        time_diff = np.array(self.times)
        temp_x = np.array(self.x_values)
        temp_y = np.array(self.y_values)
        temp_z = np.array(self.z_values)
        np.savetxt(output_file, np.array(list(zip(time_diff, temp_x, temp_y, temp_z))),
                delimiter=',', header="time,x,y,z", comments="")

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

    apriltag_subscriber = ApriltagSubscriber()

    rclpy.spin(apriltag_subscriber)

    apriltag_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
