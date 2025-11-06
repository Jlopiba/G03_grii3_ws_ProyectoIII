#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import math
import time

class DrawAndAvoid(Node):
    def __init__(self):
        super().__init__('draw_and_avoid_node')

        # Publicadores y suscriptores
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # Timers
        self.timer_marker = self.create_timer(1.0, self.publish_marker)
        self.timer_motion = self.create_timer(0.1, self.move_pattern)

        # Parámetros de movimiento
        self.safe_distance = 0.45     # distancia mínima segura
        self.forward_speed = 0.15
        self.turn_speed = 0.5
        self.obstacle_detected = False
        self.state = 0                # controla la secuencia del "3"
        self.state_start_time = time.time()

        self.get_logger().info("Nodo draw_and_avoid iniciado correctamente ✅")

    # --- Lectura del LIDAR ---
    def scan_callback(self, msg):
        front_ranges = msg.ranges[len(msg.ranges)//2 - 10 : len(msg.ranges)//2 + 10]
        front_distance = min(front_ranges) if front_ranges else float('inf')

        self.obstacle_detected = front_distance < self.safe_distance
        if self.obstacle_detected:
            self.get_logger().info(f"⚠️ Obstáculo detectado a {front_distance:.2f} m — deteniendo.")

    # --- Movimiento del robot ---
    def move_pattern(self):
        twist = Twist()

        # Si hay obstáculo, parar
        if self.obstacle_detected:
            self.cmd_pub.publish(twist)
            return

        # Movimiento para "dibujar" el número 3 (por tiempo)
        elapsed = time.time() - self.state_start_time

        # Secuencia simple de trazos que forman un “3”
        if self.state == 0:   # trazo superior
            twist.linear.x = self.forward_speed
            if elapsed > 2.0:
                self.state = 1
                self.state_start_time = time.time()
        elif self.state == 1: # primer giro (parte superior)
            twist.angular.z = -self.turn_speed
            if elapsed > 1.3:
                self.state = 2
                self.state_start_time = time.time()
        elif self.state == 2: # trazo medio
            twist.linear.x = self.forward_speed
            if elapsed > 2.0:
                self.state = 3
                self.state_start_time = time.time()
        elif self.state == 3: # segundo giro (parte inferior)
            twist.angular.z = -self.turn_speed
            if elapsed > 1.3:
                self.state = 4
                self.state_start_time = time.time()
        elif self.state == 4: # trazo inferior
            twist.linear.x = self.forward_speed
            if elapsed > 2.0:
                self.state = 5
                self.state_start_time = time.time()
        elif self.state == 5: # detener al finalizar el "3"
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("✅ Figura '3' completada.")
            self.cmd_pub.publish(twist)
            return  # mantiene el robot quieto

        # Publicar movimiento
        self.cmd_pub.publish(twist)

    # --- Publicar marcador "3" en RViz ---
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"  # visible desde RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.position.x = 1.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.8
        marker.text = "3"
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DrawAndAvoid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

