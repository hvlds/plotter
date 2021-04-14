import rclpy
from rclpy.node import Node

from irtag_msgs.msg import IRTagArray, IRTag

class IRMarkerSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            IRTagArray,
            '/irtracking/tag_detections',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        for marker in msg.detections:
            self.get_logger().info(str(marker.pose.pose.position.z))


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = IRMarkerSubscriber()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()