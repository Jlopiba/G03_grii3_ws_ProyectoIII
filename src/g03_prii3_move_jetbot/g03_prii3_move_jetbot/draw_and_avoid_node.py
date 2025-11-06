#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker

DEG = math.pi/180.0

class DrawAndAvoid(Node):
    def __init__(self):
        super().__init__('draw_and_avoid_node')

        # ===== Parámetros ajustables por launch =====
        self.declare_parameter('safe_distance', 0.55)
        self.declare_parameter('v_line', 0.15)   # m/s
        self.declare_parameter('v_arc', 0.12)    # m/s (para giros)
        self.declare_parameter('w_turn', 0.6)    # rad/s (módulo) giros

        # Distancias (m) para las líneas de tu secuencia 1,3,5,7:
        self.declare_parameter('d1', 0.60)
        self.declare_parameter('d3', 0.40)
        self.declare_parameter('d5', 0.60)
        self.declare_parameter('d7', 0.40)

        self.safe_distance = float(self.get_parameter('safe_distance').value)
        self.v_line = float(self.get_parameter('v_line').value)
        self.v_arc = float(self.get_parameter('v_arc').value)
        self.w_turn = float(self.get_parameter('w_turn').value)

        self.d1 = float(self.get_parameter('d1').value)
        self.d3 = float(self.get_parameter('d3').value)
        self.d5 = float(self.get_parameter('d5').value)
        self.d7 = float(self.get_parameter('d7').value)

        # ===== ROS Pub/Sub =====
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # ===== Timers =====
        self.ctrl_dt = 0.05  # 20 Hz
        self.ctrl_timer = self.create_timer(self.ctrl_dt, self.control_loop)
        self.marker_timer = self.create_timer(1.0, self.publish_marker)

        # ===== Estado =====
        self.obstacle = False
        self.last_obstacle = None
        self.paused_for_obstacle = False

        # Utilizamos tiempos = distancia / velocidad para líneas,
        # y tiempos de giro = ángulo / |w_turn|
        def line(d):  # devuelve (mode, dur, v, w)
            return ('line', max(0.01, d / max(1e-6, self.v_line)), self.v_line, 0.0)

        def turn_deg(deg):  # deg>0 izquierda, deg<0 derecha
            w = self.w_turn if deg > 0 else -self.w_turn
            t = abs(deg)*DEG / self.w_turn
            return ('turn', t, 0.0, w)

    
        self.phases = [
            line(self.d1),           
            turn_deg(-90),           
            line(self.d3),           
            turn_deg(-90),           
            line(self.d5),           
            turn_deg(+180),          
            line(self.d5),           
            turn_deg(-90), 
            line(self.d3),  
            turn_deg(-90), 
            line(self.d1),         
            ('stop', 0.0, 0.0, 0.0)
        ]

        self.phase = 0
        self.phase_t0 = time.time()
        self.get_logger().info('Nodo draw_and_avoid iniciado con patrón 1-8 ')

    # ========== LIDAR ==========
    def scan_cb(self, msg: LaserScan):
        try:
            center_idx = int(round((0.0 - msg.angle_min) / msg.angle_increment))
        except Exception:
            center_idx = len(msg.ranges) // 2

        half = max(1, int(round((15.0*DEG)/msg.angle_increment)))
        i0 = max(0, center_idx - half)
        i1 = min(len(msg.ranges)-1, center_idx + half)

        vals = []
        for r in msg.ranges[i0:i1+1]:
            if math.isfinite(r) and r > 0.05:
                vals.append(r)
        dmin = min(vals) if vals else float('inf')

        self.obstacle = (dmin < self.safe_distance)
        if self.last_obstacle is None or self.obstacle != self.last_obstacle:
            self.last_obstacle = self.obstacle
            if self.obstacle:
                self.get_logger().info(f' Obstáculo a {dmin:.2f} m — PARO')
            else:
                self.get_logger().info(' Libre — reanudo trayectoria')

    # ========== CONTROL ==========
    def control_loop(self):
        now = time.time()

        # Si ya terminamos: mantener stop a 20 Hz
        if self.phase >= len(self.phases):
            self.publish_cmd(0.0, 0.0)
            return

        # Obstáculo: PAUSA dura (publica 0,0 continuamente)
        if self.obstacle:
            self.paused_for_obstacle = True
            self.publish_cmd(0.0, 0.0)
            return

        # Reanudación tras obstáculo: no pierdas progreso de fase
        if self.paused_for_obstacle:
            self.paused_for_obstacle = False
            self.phase_t0 = now

        mode, dur, v_set, w_set = self.phases[self.phase]
        elapsed = now - self.phase_t0

        if mode == 'stop':
            self.publish_cmd(0.0, 0.0)
            self.phase = len(self.phases)
            self.get_logger().info(' Figura "3" completada.')
            return

        # Publica el comando deseado (20 Hz)
        self.publish_cmd(v_set, w_set)

        # Avance de fase por tiempo
        if elapsed >= dur:
            self.phase += 1
            self.phase_t0 = now
            if self.phase < len(self.phases):
                nxt = self.phases[self.phase][0]
                self.get_logger().info(f'Fase {self.phase+1}: {nxt}')
            else:
                self.get_logger().info('Secuencia completada')

    def publish_cmd(self, v, w):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.cmd_pub.publish(t)

    # ========== Marker en RViz ==========
    def publish_marker(self):
        m = Marker()
        m.header.frame_id = 'odom'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'grupo3'
        m.id = 3
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.scale.z = 1.0
        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.pose.position.x = 1.0
        m.pose.position.y = 0.0
        m.pose.position.z = 0.8
        m.text = '3'
        self.marker_pub.publish(m)

def main(args=None):
    rclpy.init(args=args)
    node = DrawAndAvoid()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

