#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import time
import math

class PathGenerator(Node):
    def __init__(self):
        super().__init__('traffic_path_generator')
        self.publisher_ = self.create_publisher(Pose, '/goal', 10)
        self.goals = [
            [0.0, 0.5, 1.57],   
            [0.5, 0.5, 0.0],    
            [0.5, 0.0, -1.57],  
            [0.0, 0.0, 3.14]    
        ]

        self.timer = self.create_timer(1.0, self.publish_goals)
        self.sent = False

    def publish_goals(self):
        if self.sent:
            return
        self.sent = True

        for x, y, theta in self.goals:
            msg = Pose()
            msg.position.x = x
            msg.position.y = y
            msg.position.z = 0.0
            msg.orientation.z = math.sin(theta / 2.0)
            msg.orientation.w = math.cos(theta / 2.0)
            self.publisher_.publish(msg)
            self.get_logger().info(f'📤 Punto publicado: ({x}, {y}, θ={theta:.2f})')
            time.sleep(4.0)

        self.get_logger().info("✅ Todos los puntos fueron publicados.")
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
