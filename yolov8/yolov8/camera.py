# !/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):

    def __init__(self):
        super().__init__('image_sub_node') #노드 이름 지정
        queue_size = 10
        self.subscriber = self.create_subscription(
            Image, '/skidbot/camera_sensor/image_raw', self.sub_callback, queue_size
        )
        self.bridge = CvBridge()
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        self.get_logger().info(f'asdf')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
            cv2.imshow("image", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"asdf: {e}")


def main(args=None):
    rclpy.init(args=args)

    camera_subscriber = CameraSubscriber()

    rclpy.spin(camera_subscriber)

    camera_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()