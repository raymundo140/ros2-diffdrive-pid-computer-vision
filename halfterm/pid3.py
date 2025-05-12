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
        self.state = "WAITING"

        # PID parameters for linear motion
        self.kp_lin = 1.2
        self.ki_lin = 0.0
        self.kd_lin = 0.5
        self.error_lin_sum = 0.0
        self.prev_error_lin = 0.0

        # PID parameters for rotation
        self.kp_ang = 1.5
        self.ki_ang = 0.0
        self.kd_ang = 0.3
        self.error_ang_sum = 0.0
        self.prev_error_ang = 0.0

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

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

            if self.light_state == "red":
                self.get_logger().info("🛑 Rojo detectado. Pausando...")
                self.stop()
                return

            if dist < 0.08:
                self.stop()
                self.state = "ROTATING"
                self.error_lin_sum = 0.0
                self.prev_error_lin = 0.0
                return

            # PID control for linear distance
            error = dist
            self.error_lin_sum += error * self.dt
            d_error = (error - self.prev_error_lin) / self.dt
            output = self.kp_lin * error + self.ki_lin * self.error_lin_sum + self.kd_lin * d_error
            self.prev_error_lin = error

            twist = Twist()
            twist.linear.x = min(output, 0.2)  # Cap max speed
            self.cmd_pub.publish(twist)

        elif self.state == "ROTATING":
            _, _, goal_theta = self.goals[self.goal_index]
            error = self.normalize_angle(goal_theta - self.current_theta)

            if self.light_state == "red":
                self.get_logger().info("🛑 Rojo durante rotación. Pausando...")
                self.stop()
                return

            if abs(error) < 0.05:
                self.stop()
                self.get_logger().info("✅ Objetivo completado.")
                self.goal_index += 1
                self.state = "WAITING"
                self.error_ang_sum = 0.0
                self.prev_error_ang = 0.0
                return

            # PID control for rotation
            self.error_ang_sum += error * self.dt
            d_error = (error - self.prev_error_ang) / self.dt
            output = self.kp_ang * error + self.ki_ang * self.error_ang_sum + self.kd_ang * d_error
            self.prev_error_ang = error

            twist = Twist()
            twist.angular.z = max(min(output, 0.8), -0.8)  # Limit angular speed
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
