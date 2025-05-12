#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(String, '/light_state', qos_profile)
        self.subscription = self.create_subscription(Image, '/video_source/raw', self.image_callback, 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Rango más permisivo pero aún claro
        red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255)) | \
                   cv2.inRange(hsv, (170, 100, 100), (180, 255, 255))
        yellow_mask = cv2.inRange(hsv, (18, 100, 100), (30, 255, 255))
        green_mask = cv2.inRange(hsv, (40, 80, 80), (80, 255, 255))

        red_count = cv2.countNonZero(red_mask)
        yellow_count = cv2.countNonZero(yellow_mask)
        green_count = cv2.countNonZero(green_mask)

        state = "unknown"
        if green_count > 600:
            state = "green"
        elif red_count > 600:
            state = "red"
        elif yellow_count > 600:
            state = "yellow"

        self.publisher_.publish(String(data=state))
        self.get_logger().info(f"🔎 Luz detectada: {state} (R:{red_count}, Y:{yellow_count}, G:{green_count})")

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

