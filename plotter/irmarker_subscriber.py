import rclpy
from rclpy.node import Node

from irtag_msgs.msg import IRTagArray, IRTag


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

    def check_detections(self):
        if self.detections:
            first_stamp = self.detections[0].stamp
            last_stamp = self.detections[-1].stamp
            
            if last_stamp.sec <= (first_stamp.sec + 60):
                self.plot_detections.append(self.check_detections[-1])
            elif self.first_plot:
                pass

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
