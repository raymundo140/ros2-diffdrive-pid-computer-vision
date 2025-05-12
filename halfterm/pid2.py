#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PathPIDController(Node):
    def __init__(self):
        super().__init__('traffic_pid_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/light_state', self.light_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.light_state = "unknown"
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.goals = [
            [0.0, 0.5, 1.57],
            [0.5, 0.5, 0.0],
            [0.5, 0.0, -1.57],
            [0.0, 0.0, 3.14]
        ]
        self.goal_index = 0
        self.state = "WAITING"  # WAITING, MOVING, ROTATING, DONE

        self.timer = self.create_timer(0.1, self.control_loop)

    def light_callback(self, msg):
        self.light_state = msg.data

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.goal_index >= len(self.goals):
            if self.state != "DONE":
                self.get_logger().info("🎉 Todos los objetivos completados.")
                self.stop()
                self.state = "DONE"
            return

        if self.state == "WAITING":
            if self.light_state != "green":
                self.get_logger().info(f"🛑 Esperando luz verde... (actual: {self.light_state})")
                self.stop()
                return

            self.get_logger().info(f"🚀 Ejecutando objetivo {self.goal_index + 1}")
            self.state = "MOVING"

        elif self.state == "MOVING":
            goal_x, goal_y, _ = self.goals[self.goal_index]
            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            dist = math.hypot(dx, dy)

            self.get_logger().info(f"🔍 t2 Distancia al objetivo: {dist:.2f} m")

            if self.light_state == "red":
                self.get_logger().info("🛑 Rojo detectado. Pausando...")
                self.stop()
                return

            if dist < 0.08:
                self.stop()
                self.state = "ROTATING"
                return

            speed = 0.08 if self.light_state == "yellow" else 0.15
            twist = Twist()
            twist.linear.x = speed  # 👈 INVERSIÓN DE DIRECCIÓN AQUÍ
            self.cmd_pub.publish(twist)

        elif self.state == "ROTATING":
            _, _, goal_theta = self.goals[self.goal_index]
            error = self.normalize_angle(goal_theta - self.current_theta)

            self.get_logger().info(f"🌀 Rotando: error = {math.degrees(error):.1f}°")

            if self.light_state == "red":
                self.get_logger().info("🛑 Rojo durante rotación. Pausando...")
                self.stop()
                return

            if abs(error) < 0.05:
                self.stop()
                self.get_logger().info("✅ Objetivo completado.")
                self.goal_index += 1
                self.state = "WAITING"
                return

            twist = Twist()
            twist.angular.z = 0.3 * math.copysign(1.0, error)
            self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

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