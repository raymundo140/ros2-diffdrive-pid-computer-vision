!/usr/bin/env python3
import rclpy
import math
import signal
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.qos import qos_profile_sensor_data

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')

        self.L = 0.18  # Distancia entre ruedas [m]
        self.R = 0.05  # Radio de la rueda [m]
        self.dt = 0.05 # Intervalo de tiempo [s]

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wr = 0.0
        self.wl = 0.0

        self.sub_r = self.create_subscription(Float32, '/VelocityEncR', self.encR_callback, qos_profile_sensor_data)
        self.sub_l = self.create_subscription(Float32, '/VelocityEncL', self.encL_callback, qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(self.dt, self.update_odometry)

        self.get_logger().info("🧭 Nodo de Dead Reckoning iniciado.")

    def encR_callback(self, msg):
        self.wr = -msg.data  # ❗️INVERSIÓN DE SIGNO para corregir dirección
        self.get_logger().info(f"🔁 wr recibido (ajustado): {self.wr:.2f}")

    def encL_callback(self, msg):
        self.wl = msg.data  # Mantener sin invertir
        self.get_logger().info(f"🔁 wl recibido: {self.wl:.2f}")

    def update_odometry(self):
        v_r = self.R * self.wr
        v_l = self.R * self.wl
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.L

        dx = v * math.cos(self.theta) * self.dt
        dy = v * math.sin(self.theta) * self.dt
        dtheta = omega * self.dt

        self.x += dx
        self.y += dy
        self.theta += dtheta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))  # Normalize angle

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = self.quaternion_from_yaw(self.theta)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        self.get_logger().info(
            f"[ODOM] x={self.x:.2f} y={self.y:.2f} θ={self.theta:.2f} v={v:.2f} ω={omega:.2f}"
        )

    def quaternion_from_yaw(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.z = math.sin(yaw / 2.0)
        return q

    def stop_handler(self, signum, frame):
        self.get_logger().info("🛑 Interrupción recibida. Cerrando nodo...")
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    signal.signal(signal.SIGINT, node.stop_handler)
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('Shutdown completo.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
