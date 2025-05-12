#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time
import threading

class PathPIDController(Node):
    def __init__(self):
        super().__init__('traffic_pid_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/light_state', self.light_callback, 10)

        self.light_state = "unknown"
        self.green_granted = False
        self.goals = [
            [0.0, 0.5, 1.57],
            [0.5, 0.5, 0.0],
            [0.5, 0.0, -1.57],
            [0.0, 0.0, 3.14]
        ]
        self.goal_index = 0
        self.lock = threading.Lock()

        # Hilo principal de ejecución del path
        self.execution_thread = threading.Thread(target=self.run_path)
        self.execution_thread.start()

    def light_callback(self, msg):
        with self.lock:
            self.light_state = msg.data

    def get_light(self):
        with self.lock:
            return self.light_state

    def run_path(self):
        while rclpy.ok() and self.goal_index < len(self.goals):
            light = self.get_light()

            # Esperar luz verde solo al inicio
            while not self.green_granted:
                if light == "green":
                    self.green_granted = True
                    self.get_logger().info("🟢 Luz verde detectada, iniciando ruta.")
                    break
                else:
                    self.get_logger().info(f"🛑 Esperando luz verde para empezar... (actual: {light})")
                    self.stop()
                    time.sleep(0.5)
                    light = self.get_light()

            x, y, theta = self.goals[self.goal_index]
            self.get_logger().info(f"🚀 Ejecutando objetivo {self.goal_index+1}: ({x}, {y}, θ={theta:.2f})")
            self.move_straight(math.hypot(x, y))
            time.sleep(0.2)
            self.rotate(abs(theta), 0.3, math.copysign(1.0, theta))
            time.sleep(0.2)

            self.goal_index += 1
            self.get_logger().info("✅ Objetivo completado.\n")

        self.get_logger().info("🎉 Todos los objetivos completados.")
        self.stop()

    def move_straight(self, distance):
        dt = 0.1
        traveled = 0.0

        while traveled < distance and rclpy.ok():
            light = self.get_light()
            if light == "red":
                self.get_logger().info("🛑 Rojo detectado. Pausando...")
                self.stop()
                while self.get_light() != "green" and rclpy.ok():
                    time.sleep(0.1)
                self.get_logger().info("🟢 Verde detectado. Reanudando...")

            speed = 0.08 if light == "yellow" else 0.15
            twist = Twist()
            twist.linear.x = speed
            self.cmd_pub.publish(twist)

            time.sleep(dt)
            traveled += speed * dt

        self.stop()

    def rotate(self, angle_rad, angular_speed, direction):
        dt = 0.1
        rotated = 0.0

        while rotated < angle_rad and rclpy.ok():
            light = self.get_light()
            if light == "red":
                self.get_logger().info("🛑 Rojo durante rotación. Pausando...")
                self.stop()
                while self.get_light() != "green" and rclpy.ok():
                    time.sleep(0.1)
                self.get_logger().info("🟢 Verde detectado. Reanudando rotación...")

            twist = Twist()
            twist.angular.z = direction * angular_speed
            self.cmd_pub.publish(twist)

            time.sleep(dt)
            rotated += angular_speed * dt

        self.stop()

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PathPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
